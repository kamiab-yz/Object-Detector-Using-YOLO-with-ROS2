from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    image_publisher_node = Node(
        package="vision_pkg",
        executable="image_publisher"

    )

    obj_detector_node = Node(
        package="vision_pkg",
        executable="obj_place_detector",
        parameters=[
            {"object_name": "bottle"},
            {"place_name": "chair"}
        ]
    )

    ld.add_action(image_publisher_node)
    ld.add_action(obj_detector_node)
    return ld