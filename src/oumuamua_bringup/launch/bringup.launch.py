"""oumuamua の起動一式。

/scan (urg_node2) -> sotoba_node -> TF field -> base_link -> tracker_node -> /cmd_vel
-> chassis_node -> /wheel<i>/target_velocity -> mini_shirasu_node (wheel<i>, i = 0..wheel_count-1)
-> /robomas_can_tx -> robomas_bridge (USB-CAN) -> mini-shirasu

tracker_node の目標 (/tracker_node/reference) を出すノードはここには含まない。

sim:=true なら、/cmd_vel から先 (足回り・モータドライバ・USB-CAN・LiDAR) の代わりに
oumuamua_sim の robot_sim を立てる。robot_sim は /cmd_vel で動き、/scan を出す。
"""

from launch import LaunchDescription
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
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def config(package, name):
    return PathJoinSubstitution([FindPackageShare(package), 'config', name])


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    lidar_connection = LaunchConfiguration('lidar_connection')
    bridge = LaunchConfiguration('bridge')
    rviz = LaunchConfiguration('rviz')
    sim = LaunchConfiguration('sim')
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
            {
                'lidar_height': LaunchConfiguration('lidar_z'),
                'lidar_upside_down': LaunchConfiguration('lidar_upside_down'),
            },
        ],
        output='screen',
    )

    # --- 軌道追従 (holonomic_tracker) ---
    tracker = Node(
        package='holonomic_tracker',
        executable='tracker_node',
        name='tracker_node',
        parameters=[
            config('holonomic_tracker', 'tracker_node.yaml'),
            config('oumuamua_bringup', 'tracker_node.yaml'),
        ],
        output='screen',
    )

    # --- 足回り (omni_chassis) ---
    chassis = Node(
        package='omni_chassis',
        executable='chassis_node',
        name='chassis_node',
        parameters=[LaunchConfiguration('chassis_params')],
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
            {
                'lidar_height': LaunchConfiguration('lidar_z'),
                'lidar_upside_down': LaunchConfiguration('lidar_upside_down'),
                'lidar_x': LaunchConfiguration('lidar_x'),
                'lidar_y': LaunchConfiguration('lidar_y'),
                'lidar_yaw': LaunchConfiguration('lidar_yaw'),
            },
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
            'sim', default_value='false',
            description='/cmd_vel から先を oumuamua_sim で模擬する (LiDAR・足回り・ドライバ・USB-CAN は立てない)'),
        urg_configure,
        urg_activate,
        urg,
        lidar_tf,
        sotoba,
        tracker,
        chassis,
        wheels,
        robomas,
        robot_sim,
        rviz_node,
    ])
