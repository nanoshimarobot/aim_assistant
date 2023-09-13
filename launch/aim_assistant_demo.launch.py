import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

robot_ns = "ER"
# rviz_config = "sakamoto.rviz"
rviz_config = "shooter_dbg.rviz"
# test_rviz_config = "sim_test.rviz"

def generate_launch_description():
    container = ComposableNodeContainer(
        name='er_ros_container',
        namespace='ER',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # getShooterTrackingControllerComponent(),
            # getAimAssistantVLP16Component(),
            getAimAssistantComponent(),
            # getChassisDriveComponent(),
            # getNodeObserverComponent(),
            # getControllerComponent()
        ],
        output='screen'
    )

    list = [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', os.path.join(get_package_share_directory("aim_assistant"), 'demo_bag', 'moving_with_livox.mcap')
            , '--remap', '/livox/lidar:=/livox/mid_70'],
            output='screen'
        ),
        container,
        # rviz
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # output=
            arguments=['-d' + os.path.join(get_package_share_directory("aim_assistant"), "rviz", "demo.rviz")],
        ),
    ]

    return LaunchDescription(list)

def composition(pkg_name, plugin_name, comp_name, parameter_file, parameter_node_name):
    # params = None
    if parameter_file is not None:
        with open(parameter_file, 'r') as f:
            params = yaml.safe_load(f)[parameter_node_name]['ros__parameters']
        component = ComposableNode(
            package=pkg_name,
            plugin=plugin_name,
            # namespace='/perception',
            # remappings=[
            #     ('points', 'points_concatenate_node/output'),
            #     ('polygon', 'scan_segmentation_node/polygon')
            # ],
            name=comp_name,
            namespace=robot_ns,
            parameters=[params])
    else:
        component = ComposableNode(
            package=pkg_name,
            plugin=plugin_name,
            # namespace='/perception',
            # remappings=[
            #     ('points', 'points_concatenate_node/output'),
            #     ('polygon', 'scan_segmentation_node/polygon')
            # ],
            name=comp_name,
            namespace=robot_ns)
    return component

def getAimAssistantComponent():
    return composition(
        'aim_assistant',
        'abu2023::AimAssistant',
        'aim_assistant',
        os.path.join(get_package_share_directory("aim_assistant"), "config", "aim_assistant_param.yaml"),
        '/ER/aim_assistant_node')