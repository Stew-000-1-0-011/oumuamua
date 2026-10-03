"""mini-shirasu 1 枚と CAN で話せるかだけを確かめる。モータは回さない。

robomas_bridge (USB-CAN) と mini_shirasu_node を 1 つ立て、無効のまま設定だけを送る。
うまくいけば mini_shirasu_node のログに `configured (disabled)` が出て、
`/wheel<board>/status` に Status が 50Hz で届く。

    ros2 launch oumuamua_bringup can_test.launch.py board:=0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def make_node(context):
        board = int(LaunchConfiguration('board').perform(context))
        return [Node(
            package='mini_shirasu_ros',
            executable='mini_shirasu_node',
            name=f'wheel{board}',
            parameters=[
                LaunchConfiguration('mini_shirasu_params'),
                # 設定だけして、出力は無効のまま
                {'enable_on_start': False},
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('node')),
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'board', default_value='0',
            description='試す基板。mini_shirasu.yaml の wheel<board> の CAN ID を使う'),
        DeclareLaunchArgument(
            'node', default_value='true',
            description='false ならブリッジだけ立てる (フレームを手で送って試すとき)'),
        DeclareLaunchArgument(
            'mini_shirasu_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('oumuamua_bringup'), 'config', 'mini_shirasu.yaml']),
            description='mini_shirasu_node のパラメータファイル'),
        ComposableNodeContainer(
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
        ),
        OpaqueFunction(function=make_node),
    ])
