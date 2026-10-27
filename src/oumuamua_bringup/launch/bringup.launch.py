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
estimator:=false なら sotoba_ros の persistence_predictor (持続予測) が事前分布を出し、sotoba_node が TF field -> base_link を出す。

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
    """

    def __init__(self, node_name, params):
        super().__init__()
        self.node_name = node_name
        self.params = params

    def perform(self, context):
        values = {}
        for key, value in self.params.items():
            if isinstance(value, (bool, int, float)):
                values[key] = value
            else:
                text = perform_substitutions(context, normalize_to_list_of_substitutions(value))
                values[key] = yaml.safe_load(text)
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

    # # --- 自己位置 (sotoba_ros) ---
    # sotoba = Node(
    #     package='sotoba_ros',
    #     executable='sotoba_node',
    #     name='sotoba_node',
    #     parameters=[
    #         config('sotoba_ros', 'sotoba_node.yaml'),
    #         config('oumuamua_bringup', 'sotoba_node.yaml'),
    #         NodeParams('sotoba_node', {
    #             'lidar_height': LaunchConfiguration('lidar_z'),
    #             'lidar_upside_down': LaunchConfiguration('lidar_upside_down'),
    #             'publish_tf': not_estimator,
    #         }),
    #     ],
    #     output='screen',
    # )
    # # sotoba_node は予測をしないので、事前分布を出すノードが要る。
    # # estimator:=true なら state_estimator、false なら sotoba_ros の持続予測 (機体の動きは知らない)
    # persistence_predictor = Node(
    #     package='sotoba_ros',
    #     executable='persistence_predictor',
    #     name='persistence_predictor',
    #     parameters=[
    #         config('sotoba_ros', 'sotoba_node.yaml'),
    #         config('oumuamua_bringup', 'sotoba_node.yaml'),
    #     ],
    #     remappings=[
    #         ('posterior_beliefs', '/sotoba_node/posterior_beliefs'),
    #         ('prior_beliefs', '/sotoba_node/prior_beliefs'),
    #         ('initial_beliefs', '/sotoba_node/initial_beliefs'),
    #     ],
    #     output='screen',
    #     condition=IfCondition(not_estimator),
    # )

    # # --- 状態推定 (state_estimator) ---
    # def make_estimator(context):
    #     if estimator.perform(context) != 'true':
    #         return []
    #     source = velocity_source.perform(context)
    #     params = [
    #         config('state_estimator', 'state_estimator.yaml'),
    #         config('oumuamua_bringup', 'state_estimator.yaml'),
    #     ]
    #     if source == 'cmd_vel':
    #         velocity_topic = '/cmd_vel'
    #         tau = float(LaunchConfiguration('cmd_tau').perform(context))
    #         params.append(NodeParams('state_estimator', {'filter.tau_linear': tau, 'filter.tau_angular': tau}))
    #     elif source == 'wheels':
    #         velocity_topic = '/robot_sim/body_velocity' if sim.perform(context) == 'true' else '/chassis_node/body_velocity'
    #     else:
    #         raise RuntimeError(f"velocity_source must be 'wheels' or 'cmd_vel': {source}")
    #     if odometry_only.perform(context) == 'true':
    #         params.append(NodeParams('state_estimator', {'dead_reckoning': True}))
    #     return [Node(
    #         package='state_estimator',
    #         executable='state_estimator_node',
    #         name='state_estimator',
    #         parameters=params,
    #         remappings=[('~/body_velocity', velocity_topic)],
    #         output='screen',
    #     )]
    # state_estimator = OpaqueFunction(function=make_estimator)

    # # --- 軌道追従 (holonomic_tracker) ---
    # tracker = Node(
    #     package='holonomic_tracker',
    #     executable='tracker_node',
    #     name='tracker_node',
    #     parameters=[
    #         config('holonomic_tracker', 'tracker_node.yaml'),
    #         config('oumuamua_bringup', 'tracker_node.yaml'),
    #         NodeParams('tracker_node', {
    #             'pose_source': PythonExpression(["'odom' if '", estimator, "' == 'true' else 'tf'"]),
    #         }),
    #     ],
    #     output='screen',
    # )

    # # --- 経路 (path_follower)。起動しただけでは走らない。~/start で経路の始点から走り出す ---
    # path_follower = Node(
    #     package='path_follower',
    #     executable='path_follower_node',
    #     name='path_follower',
    #     parameters=[
    #         config('path_follower', 'path_follower.yaml'),
    #         config('oumuamua_bringup', 'path_follower.yaml'),
    #         NodeParams('path_follower', {
    #             'path_dir': LaunchConfiguration('path_dir'),
    #             'odom_topic': PythonExpression(
    #                 ["'/state_estimator/odom' if '", estimator, "' == 'true' else '/tracker_node/odom'"]),
    #         }),
    #     ],
    #     output='screen',
    # )

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
        urg_configure,
        urg_activate,
        urg,
        lidar_tf,
        # sotoba,
        # persistence_predictor,
        # state_estimator,
        # tracker,
        # path_follower,
        chassis,
        wheels,
        grippers,
        task_commander,
        robomas,
        robot_sim,
        rviz_node,
    ])
