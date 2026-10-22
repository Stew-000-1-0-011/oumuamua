"""oumuamua の起動一式。

/scan (urg_node2) -> sotoba_node -> TF field -> base_link -> tracker_node -> /cmd_vel
-> chassis_node -> /wheel<i>/target_velocity -> mini_shirasu_node (wheel<i>, i = 0..wheel_count-1)
-> /robomas_can_tx -> robomas_bridge (USB-CAN) -> mini-shirasu

tracker_node の目標 (/tracker_node/reference) を出すノードはここには含まない。

把持機構 (昇降 + 吸引) は gripper_count 個 (既定 2) 立てる。機構 i ごとに
  gripper<i> (gripper_node): ~/move_lift (アクション) で昇降、~/set_suction で吸引のオン/オフ
  -> /robomas_target<motor> -> robomas_bridge -> ロボマス
  -> /suction<i>/target_current -> suction<i> (mini_shirasu_node、電流制御) -> mini-shirasu
sim:=true なら gripper<i> だけを立てる (昇降は指令をそのまま高さとみなす)。

task_commander は ~/run (task_commander/action/RunTask) で作業 (ブロックの回収、スラロームの往復、
棚への配置、各地点への移動) を 1 つずつ受け、path_follower の ~/follow と gripper<i> を動かす。

estimator:=true (既定) なら state_estimator が自己位置と速度を推定し、sotoba_node には事前分布を、
tracker_node には推定 (~/odom) を渡す。速度の入力は velocity_source で選ぶ:
  wheels  (既定): 車輪オドメトリ (chassis_node の ~/body_velocity、sim:=true なら robot_sim の ~/body_velocity)
  cmd_vel        : tracker_node の指令 (下位の速度制御の遅れを cmd_tau で与える)
localization で自己位置推定のやり方を選ぶ:
  sotoba (既定): sotoba_node (ICP)。下の estimator と組み合わせる
  amcl         : nav2_amcl (パーティクルフィルタ) と map_server (config/maps/field.yaml)。
                 body_odometry が機体速度を積分して TF odom -> base_link を出す。
                 estimator:=false なら AMCL が TF field -> odom を出し、
                 estimator:=true なら AMCL の推定 (/amcl_pose) を state_estimator の観測にし、state_estimator が field -> odom を出す
  emcl         : emcl2 (膨張リセット付きの MCL)。止まっていても更新する。TF field -> odom を必ず出すので estimator:=false で使う
  gicp         : gicp_localizer (small_gicp で点群地図 config/maps/field.pcd に位置合わせ)。
                 odom とTF・estimator の扱いは amcl と同じ (推定は /gicp_localizer/pose)
estimator:=false (localization:=sotoba) なら sotoba_ros の persistence_predictor (持続予測) が事前分布を出し、sotoba_node が TF field -> base_link を出す。

navigation で cmd_vel より前 (経路の追従) のやり方を選ぶ:
  tracker (既定): path_follower + tracker_node (holonomic_tracker)
  nav2          : Nav2 (controller_server (MPPI)・planner_server・bt_navigator・velocity_smoother など、config/nav2.yaml)。
                  path_follower の代わりに nav2_path_driver が同じ経路ファイルを Nav2 に渡す (nav2_mode)。
                  odom フレームが要るので localization:=amcl か gicp と組み合わせる

sim:=true なら、/cmd_vel から先 (足回り・モータドライバ・USB-CAN・LiDAR) の代わりに
oumuamua_sim の robot_sim を立てる。robot_sim は /cmd_vel で動き、/scan を出す。
"""

import tempfile

import yaml
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def config(package, name):
    return PathJoinSubstitution([FindPackageShare(package), 'config', name])


class NodeParams(Substitution):
    """パラメータの上書きを、ノード名をキーにした一時ファイルに書いてそのパスを返す。

    launch_ros の辞書のパラメータは "/**" として書かれ、パッケージのパラメータファイルに
    ノード名で書かれた値に負ける (ros_arguments の -p はさらにパラメータファイルより前に並ぶ)。
    上書きしたい値は、これを parameters の最後に置いて渡す。
    値はサブスティテューションでもよく、展開した文字列を YAML として読む ('true' -> bool など)。
    辞書を入れ子にすると、入れ子のパラメータ (initial_pose.x など) になる。
    """

    def __init__(self, node_name, params):
        super().__init__()
        self.node_name = node_name
        self.params = params

    def perform(self, context):
        def resolve(value):
            if isinstance(value, dict):
                return {k: resolve(v) for k, v in value.items()}
            if isinstance(value, (bool, int, float)):
                return value
            return yaml.safe_load(perform_substitutions(context, normalize_to_list_of_substitutions(value)))

        values = resolve(self.params)
        with tempfile.NamedTemporaryFile('w', prefix=f'{self.node_name}_', suffix='.yaml', delete=False) as f:
            yaml.safe_dump({self.node_name: {'ros__parameters': values}}, f)
            return f.name


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    lidar_connection = LaunchConfiguration('lidar_connection')
    bridge = LaunchConfiguration('bridge')
    rviz = LaunchConfiguration('rviz')
    sim = LaunchConfiguration('sim')
    estimator = LaunchConfiguration('estimator')
    velocity_source = LaunchConfiguration('velocity_source')
    localization = LaunchConfiguration('localization')
    navigation = LaunchConfiguration('navigation')
    use_tracker = IfCondition(PythonExpression(["'", navigation, "' == 'tracker'"]))
    use_sotoba = IfCondition(PythonExpression(["'", localization, "' == 'sotoba'"]))
    use_amcl = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))
    # 機体 (base_link) の初期姿勢。sotoba 以外の自己位置推定に渡す
    # (パラメータは double なので、'0' のような整数の書き方でも float にする)
    start_pose = {k: PythonExpression(['float(', LaunchConfiguration(f'start_{k}'), ')']) for k in ('x', 'y', 'yaw')}
    # 文字列 'true' / 'false' を、パラメータとして bool で渡すための式
    not_estimator = PythonExpression(["'", estimator, "' != 'true'"])
    # 実機でだけ立てるもの
    real = UnlessCondition(sim)
    odometry_only = LaunchConfiguration('odometry_only')
    lidar_real = IfCondition(PythonExpression(["'", lidar, "' == 'true' and '", sim, "' != 'true'"]))

    # --- LiDAR (urg_node2 はライフサイクルノード。起動したら configure -> activate まで進める) ---
    urg = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        namespace='',
        remappings=[('scan', '/scan')],
        parameters=[PathJoinSubstitution([
            FindPackageShare('oumuamua_bringup'), 'config', ['urg_', lidar_connection, '.yaml'],
        ])],
        output='screen',
        condition=lidar_real,
    )
    urg_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=urg,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(urg),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        ),
        condition=lidar_real,
    )
    urg_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=urg,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(urg),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
        condition=lidar_real,
    )

    # --- base_link -> laser (LiDAR の取付)。逆さ付けなら x 軸まわりに 180 度 ---
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '--x', LaunchConfiguration('lidar_x'),
            '--y', LaunchConfiguration('lidar_y'),
            '--z', LaunchConfiguration('lidar_z'),
            '--yaw', LaunchConfiguration('lidar_yaw'),
            '--roll', PythonExpression(
                ["'3.141592653589793' if '", LaunchConfiguration('lidar_upside_down'), "' == 'true' else '0.0'"]),
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        output='screen',
    )

    # --- 自己位置 (sotoba_ros) ---
    sotoba = Node(
        package='sotoba_ros',
        executable='sotoba_node',
        name='sotoba_node',
        parameters=[
            config('sotoba_ros', 'sotoba_node.yaml'),
            config('oumuamua_bringup', 'sotoba_node.yaml'),
            NodeParams('sotoba_node', {
                'lidar_height': LaunchConfiguration('lidar_z'),
                'lidar_upside_down': LaunchConfiguration('lidar_upside_down'),
                'publish_tf': not_estimator,
            }),
            LaunchConfiguration('overrides'),
        ],
        output='screen',
        condition=use_sotoba,
    )
    # sotoba_node は予測をしないので、事前分布を出すノードが要る。
    # estimator:=true なら state_estimator、false なら sotoba_ros の持続予測 (機体の動きは知らない)
    persistence_predictor = Node(
        package='sotoba_ros',
        executable='persistence_predictor',
        name='persistence_predictor',
        parameters=[
            config('sotoba_ros', 'sotoba_node.yaml'),
            config('oumuamua_bringup', 'sotoba_node.yaml'),
        ],
        remappings=[
            ('posterior_beliefs', '/sotoba_node/posterior_beliefs'),
            ('prior_beliefs', '/sotoba_node/prior_beliefs'),
            ('initial_beliefs', '/sotoba_node/initial_beliefs'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", estimator, "' != 'true' and '", localization, "' == 'sotoba'"])),
    )

    # --- localization:=amcl / gicp (どちらも body_odometry の odom -> base_link の上に field -> odom を出す) ---
    def body_velocity_topic(context):
        return '/robot_sim/body_velocity' if sim.perform(context) == 'true' else '/chassis_node/body_velocity'

    def make_amcl(context):
        mode = localization.perform(context)
        if mode not in ('amcl', 'gicp', 'emcl'):
            return []
        if mode == 'emcl' and estimator.perform(context) == 'true':
            raise RuntimeError('localization:=emcl publishes field -> odom itself; use estimator:=false')
        odometry = Node(
            package='body_odometry',
            executable='body_odometry_node',
            name='body_odometry',
            remappings=[('~/body_velocity', body_velocity_topic(context)), ('~/odom', '/odom')],
            output='screen',
        )
        if mode == 'emcl':
            return [
                odometry,
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    parameters=[
                        config('oumuamua_bringup', 'emcl2.yaml'),
                        NodeParams('map_server', {'yaml_filename': LaunchConfiguration('map')}),
                    ],
                    output='screen',
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    parameters=[config('oumuamua_bringup', 'emcl2.yaml')],
                    output='screen',
                ),
                Node(
                    package='emcl2',
                    executable='emcl2_node',
                    name='emcl2',
                    parameters=[
                        config('oumuamua_bringup', 'emcl2.yaml'),
                        NodeParams('emcl2', {
                            'initial_pose_x': start_pose['x'],
                            'initial_pose_y': start_pose['y'],
                            'initial_pose_a': start_pose['yaw'],
                        }),
                        LaunchConfiguration('overrides'),
                    ],
                    remappings=[('scan', '/scan'), ('map', '/map')],
                    output='screen',
                ),
            ]
        if mode == 'gicp':
            return [odometry, Node(
                package='gicp_localizer',
                executable='gicp_localizer',
                name='gicp_localizer',
                parameters=[
                    config('gicp_localizer', 'gicp_localizer.yaml'),
                    NodeParams('gicp_localizer', {
                        'map_file': LaunchConfiguration('point_map'),
                        'initial_pose': PythonExpression([
                            '[float(', LaunchConfiguration('start_x'), '), float(', LaunchConfiguration('start_y'),
                            '), float(', LaunchConfiguration('start_yaw'), ')]']),
                        'publish_tf': not_estimator,
                    }),
                    LaunchConfiguration('overrides'),
                ],
                output='screen',
            )]
        return [
            odometry,
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    config('oumuamua_bringup', 'amcl.yaml'),
                    NodeParams('map_server', {'yaml_filename': LaunchConfiguration('map')}),
                ],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[
                    config('oumuamua_bringup', 'amcl.yaml'),
                    NodeParams('amcl', {
                        'initial_pose': {**start_pose, 'z': 0.0},
                        'tf_broadcast': not_estimator,
                    }),
                    LaunchConfiguration('overrides'),
                ],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                parameters=[config('oumuamua_bringup', 'amcl.yaml')],
                output='screen',
            ),
        ]
    amcl = OpaqueFunction(function=make_amcl)

    # --- 状態推定 (state_estimator) ---
    def make_estimator(context):
        if estimator.perform(context) != 'true':
            return []
        source = velocity_source.perform(context)
        params = [
            config('state_estimator', 'state_estimator.yaml'),
            config('oumuamua_bringup', 'state_estimator.yaml'),
        ]
        if source == 'cmd_vel':
            velocity_topic = '/cmd_vel'
            tau = float(LaunchConfiguration('cmd_tau').perform(context))
            params.append(NodeParams('state_estimator', {'filter.tau_linear': tau, 'filter.tau_angular': tau}))
        elif source == 'wheels':
            velocity_topic = body_velocity_topic(context)
        else:
            raise RuntimeError(f"velocity_source must be 'wheels' or 'cmd_vel': {source}")
        remappings = [('~/body_velocity', velocity_topic)]
        if localization.perform(context) in ('amcl', 'gicp'):
            # AMCL / gicp_localizer の推定を観測にする。TF は body_odometry の odom -> base_link の上に field -> odom を出す
            params.append(NodeParams('state_estimator', {
                'measurement_source': 'pose',
                'odom_frame': 'odom',
                'initial_pose': PythonExpression([
                    '[float(', LaunchConfiguration('start_x'), '), float(', LaunchConfiguration('start_y'),
                    '), float(', LaunchConfiguration('start_yaw'), ')]']),
            }))
            remappings.append(('~/pose_measurement',
                               '/amcl_pose' if localization.perform(context) == 'amcl' else '/gicp_localizer/pose'))
        if odometry_only.perform(context) == 'true':
            params.append(NodeParams('state_estimator', {'dead_reckoning': True}))
        params.append(LaunchConfiguration('overrides'))
        return [Node(
            package='state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            parameters=params,
            remappings=remappings,
            output='screen',
        )]
    state_estimator = OpaqueFunction(function=make_estimator)

    # --- 軌道追従 (holonomic_tracker) ---
    tracker = Node(
        package='holonomic_tracker',
        executable='tracker_node',
        name='tracker_node',
        parameters=[
            config('holonomic_tracker', 'tracker_node.yaml'),
            config('oumuamua_bringup', 'tracker_node.yaml'),
            NodeParams('tracker_node', {
                'pose_source': PythonExpression(["'odom' if '", estimator, "' == 'true' else 'tf'"]),
            }),
        ],
        output='screen',
        condition=use_tracker,
    )

    # --- 経路 (path_follower)。起動しただけでは走らない。~/start で経路の始点から走り出す ---
    path_follower = Node(
        package='path_follower',
        executable='path_follower_node',
        name='path_follower',
        parameters=[
            config('path_follower', 'path_follower.yaml'),
            config('oumuamua_bringup', 'path_follower.yaml'),
            NodeParams('path_follower', {
                'path_dir': LaunchConfiguration('path_dir'),
                'odom_topic': PythonExpression(
                    ["'/state_estimator/odom' if '", estimator, "' == 'true' else '/tracker_node/odom'"]),
            }),
        ],
        output='screen',
        condition=use_tracker,
    )

    # --- navigation:=nav2 ---
    def make_nav2(context):
        if navigation.perform(context) != 'nav2':
            return []
        if localization.perform(context) not in ('amcl', 'gicp'):
            raise RuntimeError('navigation:=nav2 needs the odom frame: use localization:=amcl or gicp')
        nav2_params = config('oumuamua_bringup', 'nav2.yaml')

        def server(package, executable, name=None, remappings=()):
            return Node(
                package=package,
                executable=executable,
                name=name or executable,
                parameters=[nav2_params],
                remappings=list(remappings),
                output='screen',
            )
        return [
            server('nav2_controller', 'controller_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
            server('nav2_planner', 'planner_server'),
            server('nav2_behaviors', 'behavior_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
            server('nav2_bt_navigator', 'bt_navigator'),
            server('nav2_velocity_smoother', 'velocity_smoother',
                   remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),
            server('nav2_lifecycle_manager', 'lifecycle_manager', name='lifecycle_manager_navigation'),
            # path_follower と同じ名前・サービスで、経路ファイルを Nav2 に渡す
            Node(
                package='oumuamua_bringup',
                executable='nav2_path_driver.py',
                name='path_follower',
                parameters=[
                    config('path_follower', 'path_follower.yaml'),
                    config('oumuamua_bringup', 'path_follower.yaml'),
                    NodeParams('path_follower', {
                        'path_dir': LaunchConfiguration('path_dir'),
                        'mode': LaunchConfiguration('nav2_mode'),
                    }),
                ],
                output='screen',
            ),
        ]
    nav2 = OpaqueFunction(function=make_nav2)

    # --- 足回り (omni_chassis) ---
    chassis = Node(
        package='omni_chassis',
        executable='chassis_node',
        name='chassis_node',
        parameters=[LaunchConfiguration('chassis_params'), NodeParams('chassis_node', {'cmd_vel_stamped': True})],
        output='screen',
        condition=real,
    )

    # --- モータドライバ (mini-shirasu を車輪ごとに 1 枚。CAN は robomas_bridge 越し) ---
    def make_wheels(context):
        if sim.perform(context) == 'true':
            return []
        count = int(LaunchConfiguration('wheel_count').perform(context))
        return [
            Node(
                package='mini_shirasu_ros',
                executable='mini_shirasu_node',
                name=f'wheel{i}',
                parameters=[LaunchConfiguration('mini_shirasu_params')],
                output='screen',
            )
            for i in range(count)
        ]
    wheels = OpaqueFunction(function=make_wheels)

    # --- 把持機構 (gripper<i>: 昇降と吸引のオン/オフ、suction<i>: 吸引のモータの mini-shirasu) ---
    def make_grippers(context):
        count = int(LaunchConfiguration('gripper_count').perform(context))
        nodes = [
            Node(
                package='gripper',
                executable='gripper_node',
                name=f'gripper{i}',
                parameters=[LaunchConfiguration('gripper_params')],
                output='screen',
            )
            for i in range(count)
        ]
        if sim.perform(context) != 'true':
            nodes += [
                Node(
                    package='mini_shirasu_ros',
                    executable='mini_shirasu_node',
                    name=f'suction{i}',
                    parameters=[LaunchConfiguration('suction_params')],
                    output='screen',
                )
                for i in range(count)
            ]
        return nodes
    grippers = OpaqueFunction(function=make_grippers)

    # --- 作業の指示 (task_commander: ~/run で作業を 1 つずつ受け、path_follower と gripper<i> を動かす) ---
    task_commander = Node(
        package='task_commander',
        executable='task_commander_node',
        name='task_commander',
        parameters=[
            config('oumuamua_bringup', 'task_commander.yaml'),
            NodeParams('task_commander', {
                'odom_topic': PythonExpression(
                    ["'/state_estimator/odom' if '", estimator, "' == 'true' else '/tracker_node/odom'"]),
            }),
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gripper_count'), "' != '0'"])),
    )

    # --- USB-CAN (robomas_plugins) ---
    robomas = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='robomas_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='robomas_plugins',
                plugin='robomas_bridge::RobomasBridge',
                name='robomas_bridge',
            ),
        ],
        # output='none',
        condition=IfCondition(PythonExpression(["'", bridge, "' == 'true' and '", sim, "' != 'true'"])),
    )

    # --- シミュレータ (sim:=true) ---
    # フィールドの寸法・初期姿勢・LiDAR の高さと上下は sotoba_node と同じ値を読ませる
    robot_sim = Node(
        package='oumuamua_sim',
        executable='robot_sim',
        name='robot_sim',
        parameters=[
            config('sotoba_ros', 'sotoba_node.yaml'),
            config('oumuamua_bringup', 'sotoba_node.yaml'),
            config('oumuamua_sim', 'robot_sim.yaml'),
            LaunchConfiguration('sim_params'),
            NodeParams('robot_sim', {
                'cmd_vel_stamped': True,
                'lidar_height': LaunchConfiguration('lidar_z'),
                'lidar_upside_down': LaunchConfiguration('lidar_upside_down'),
                'lidar_x': LaunchConfiguration('lidar_x'),
                'lidar_y': LaunchConfiguration('lidar_y'),
                'lidar_yaw': LaunchConfiguration('lidar_yaw'),
            }),
        ],
        output='screen',
        condition=IfCondition(sim),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config('sotoba_ros', 'sotoba.rviz')],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='urg_node2 を起動する'),
        DeclareLaunchArgument(
            'lidar_connection', default_value='serial', choices=['ether', 'serial'],
            description='LiDAR の接続 (config/urg_<ether|serial>.yaml を使う)'),
        DeclareLaunchArgument(
            'lidar_x', default_value='0.28',
            description='base_link から見た LiDAR の位置 [m]。正面の車輪 0 (中心から 0.20 m) の 8 cm 先'),
        DeclareLaunchArgument('lidar_y', default_value='0.0', description='base_link から見た LiDAR の位置 [m]'),
        DeclareLaunchArgument(
            'lidar_z', default_value='0.14',
            description='LiDAR の走査面の床からの高さ [m] (sotoba_node の lidar_height にも渡す)'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0', description='LiDAR の向き [rad]'),
        DeclareLaunchArgument(
            'lidar_upside_down', default_value='true',
            description='LiDAR が上下逆さに付いているか (sotoba_node にも渡す)'),
        DeclareLaunchArgument(
            'chassis_params', default_value=config('oumuamua_bringup', 'chassis_node.yaml'),
            description='chassis_node のパラメータファイル'),
        DeclareLaunchArgument(
            'wheel_count', default_value='3',
            description='mini_shirasu_node (wheel0..) の数。chassis_node.yaml の車輪数と合わせる'),
        DeclareLaunchArgument(
            'mini_shirasu_params', default_value=config('oumuamua_bringup', 'mini_shirasu.yaml'),
            description='mini_shirasu_node (wheel0..) のパラメータファイル'),
        DeclareLaunchArgument(
            'gripper_count', default_value='2',
            description='把持機構 (gripper<i> と suction<i>) の数。0 なら立てない'),
        DeclareLaunchArgument(
            'gripper_params', default_value=config('oumuamua_bringup', 'gripper.yaml'),
            description='gripper_node (gripper0..) のパラメータファイル'),
        DeclareLaunchArgument(
            'suction_params', default_value=config('oumuamua_bringup', 'suction.yaml'),
            description='吸引の mini_shirasu_node (suction0..) のパラメータファイル'),
        DeclareLaunchArgument('bridge', default_value='true', description='robomas_bridge (USB-CAN) を起動する'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'estimator', default_value='true',
            description='state_estimator で自己位置と速度を推定する。false なら sotoba_ros の持続予測で事前を出す'),
        DeclareLaunchArgument(
            'localization', default_value='sotoba', choices=['sotoba', 'amcl', 'emcl', 'gicp'],
            description='自己位置推定のやり方。sotoba: sotoba_node (ICP)、amcl: nav2_amcl、emcl: emcl2、gicp: gicp_localizer (small_gicp)'),
        DeclareLaunchArgument(
            'overrides', default_value=config('oumuamua_bringup', 'overrides_empty.yaml'),
            description='自己位置推定のノード (sotoba_node・amcl・gicp_localizer・state_estimator) に最後に読ませる'
                        'パラメータファイル。oumuamua_bench が計算負荷のつまみを振るのに使う'),
        DeclareLaunchArgument(
            'navigation', default_value='tracker', choices=['tracker', 'nav2'],
            description='経路の追従のやり方。tracker: path_follower + tracker_node、nav2: Nav2 (localization:=amcl / gicp と組み合わせる)'),
        DeclareLaunchArgument(
            'nav2_mode', default_value='follow_path', choices=['follow_path', 'through_poses'],
            description='navigation:=nav2 で経路を渡すやり方。follow_path: 経路の点をそのまま FollowPath に、'
                        'through_poses: 間引いた姿勢を NavigateThroughPoses に (経路は Nav2 が引き直す)'),
        DeclareLaunchArgument(
            'point_map', default_value=config('oumuamua_bringup', 'maps/field.pcd'),
            description='localization:=gicp の点群地図 (PCD)'),
        DeclareLaunchArgument(
            'map', default_value=config('oumuamua_bringup', 'maps/field.yaml'),
            description='localization:=amcl の地図 (nav2_map_server の yaml)'),
        DeclareLaunchArgument(
            'start_x', default_value='-2.2',
            description='機体 (base_link) の初期位置 [m]。sotoba 以外の自己位置推定に渡す。'
                        'sotoba_node.yaml の start_* (LiDAR の姿勢) と同じ置き場所にすること'),
        DeclareLaunchArgument('start_y', default_value='1.3', description='同上'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='同上 [rad]'),
        DeclareLaunchArgument(
            'velocity_source', default_value='wheels', choices=['wheels', 'cmd_vel'],
            description='state_estimator の速度入力。wheels: 車輪オドメトリ、cmd_vel: 指令値'),
        DeclareLaunchArgument(
            'cmd_tau', default_value='0.18',
            description='velocity_source:=cmd_vel のときの、指令への追従の時定数 [s]。実機の記録 (wkp 1.6) で 0.18 s'),
        DeclareLaunchArgument(
            'path_dir',
            default_value=PathJoinSubstitution([FindPackageShare('oumuamua_bringup'), 'config', 'paths']),
            description='path_follower の経路ファイル (make_path.py で作った CSV) を置いた場所。どれを読むかは path_follower.yaml の paths'),
        DeclareLaunchArgument(
            'odometry_only', default_value='false',
            description='初期位置だけ LiDAR で合わせ、動き出したら車輪オドメトリだけで自己位置を出す (state_estimator の dead_reckoning)'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='/cmd_vel から先を oumuamua_sim で模擬する (LiDAR・足回り・ドライバ・USB-CAN は立てない)'),
        DeclareLaunchArgument(
            'sim_params', default_value=config('oumuamua_sim', 'robot_sim.yaml'),
            description='robot_sim に追加で読ませるパラメータファイル (robot_sim.yaml の後に読む。乱数の種・傾き・障害物など)'),
        urg_configure,
        urg_activate,
        urg,
        lidar_tf,
        sotoba,
        persistence_predictor,
        amcl,
        state_estimator,
        tracker,
        path_follower,
        nav2,
        chassis,
        wheels,
        grippers,
        task_commander,
        robomas,
        robot_sim,
        rviz_node,
    ])
