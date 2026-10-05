"""oumuamua の起動一式。

/scan (urg_node2) -> sotoba_node -> TF field -> base_link -> tracker_node -> /cmd_vel
-> chassis_node -> /wheel<i>/target_velocity -> mini_shirasu_node (wheel<i>, i = 0..wheel_count-1)
-> /robomas_can_tx -> robomas_bridge (USB-CAN) -> mini-shirasu

tracker_node の目標 (/tracker_node/reference) を出すノードはここには含まない。

estimator:=true (既定) なら state_estimator が自己位置と速度を推定し、sotoba_node には事前分布を、
tracker_node には推定 (~/odom) を渡す。速度の入力は velocity_source で選ぶ:
  wheels  (既定): 車輪オドメトリ (chassis_node の ~/body_velocity、sim:=true なら robot_sim の ~/body_velocity)
  cmd_vel        : tracker_node の指令 (下位の速度制御の遅れを cmd_tau で与える)
estimator:=false なら sotoba_node が持続予測で単体で動き、TF field -> base_link を出す。

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
                'prior_source': PythonExpression(["'external' if '", estimator, "' == 'true' else 'internal'"]),
                'publish_tf': not_estimator,
            }),
        ],
        output='screen',
    )

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
            velocity_topic = '/robot_sim/body_velocity' if sim.perform(context) == 'true' else '/chassis_node/body_velocity'
        else:
            raise RuntimeError(f"velocity_source must be 'wheels' or 'cmd_vel': {source}")
        return [Node(
            package='state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            parameters=params,
            remappings=[('~/body_velocity', velocity_topic)],
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
    )

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
        output='screen',
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
            'lidar_connection', default_value='ether', choices=['ether', 'serial'],
            description='LiDAR の接続 (config/urg_<ether|serial>.yaml を使う)'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='base_link から見た LiDAR の位置 [m]'),
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
        DeclareLaunchArgument('bridge', default_value='true', description='robomas_bridge (USB-CAN) を起動する'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'estimator', default_value='true',
            description='state_estimator で自己位置と速度を推定する。false なら sotoba_node 単体 (持続予測)'),
        DeclareLaunchArgument(
            'velocity_source', default_value='wheels', choices=['wheels', 'cmd_vel'],
            description='state_estimator の速度入力。wheels: 車輪オドメトリ、cmd_vel: 指令値'),
        DeclareLaunchArgument(
            'cmd_tau', default_value='0.1',
            description='velocity_source:=cmd_vel のときの、指令への追従の時定数 [s]'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='/cmd_vel から先を oumuamua_sim で模擬する (LiDAR・足回り・ドライバ・USB-CAN は立てない)'),
        urg_configure,
        urg_activate,
        urg,
        lidar_tf,
        sotoba,
        state_estimator,
        tracker,
        chassis,
        wheels,
        robomas,
        robot_sim,
        rviz_node,
    ])
