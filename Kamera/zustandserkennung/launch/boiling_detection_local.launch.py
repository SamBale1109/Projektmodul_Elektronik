from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

pkg_dir = get_package_share_directory("zustandserkennung")

ARGUMENTS = [
	DeclareLaunchArgument('yolo_model',default_value=os.path.join(pkg_dir,"models","boil_detection.pt"),
        description='Full path to the object detection model that should be used'),


    DeclareLaunchArgument('launch_rviz',default_value='false',
        description='set true to start rviz2'
    ),
]

def generate_launch_description():

    camera_driver_node = Node(
        package="zustandserkennung",
        executable="camera_publisher",
        output="screen",
        parameters=[{"camera_idx": 3}],
        remappings=[("camera/color/image_raw", "usb_cam/image_raw")]
    )
    
    detection_node = Node(
        package="zustandserkennung",
        executable="boiling_detection_node",
        output="screen",
        parameters=[{"confidance" : 0.7,
                    "show_img" : True,
                    "yolo_model": LaunchConfiguration("yolo_model") }],
        remappings=[("/camera/color/image_raw","usb_cam/image_raw")]
    )
    
    rviz_config_file = os.path.join(pkg_dir,'config','image_display.rviz')
    launch_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition = IfCondition(LaunchConfiguration('launch_rviz'))
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(camera_driver_node)
    ld.add_action(detection_node)
    ld.add_action(launch_rviz2)

    return ld

