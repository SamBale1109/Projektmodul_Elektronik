from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    camera_driver_node = Node(
        package="zustandserkennung",
        executable="camera_publisher",
        output="screen",
        remappings=[("camera/color/image_raw", "usb_cam/image_raw")]
    )
    
    detection_node = Node(
        package="zustandserkennung",
        executable="boiling_detection_node",
        output="screen",
        parameters=[{"confidance" : 0.7,
                    "show_img" : True}],
        remappings=[("/camera/color/image_raw","usb_cam/image_raw")]
    )

    ld = LaunchDescription()

    ld.add_action(camera_driver_node)
    ld.add_action(detection_node)

    return ld

