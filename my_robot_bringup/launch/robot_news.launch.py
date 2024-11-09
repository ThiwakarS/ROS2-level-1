from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_names = ["giskard", "bb8", "daneel", "jander", "c3po"]
    
    for i in robot_names:
        my_robot_node = Node(
            package = "my_py_pkg",
            executable = "robot_news_station",
            name = "robot_news_station" + i,
            parameters = [{"news" : i}]
        )
        
        ld.add_action(my_robot_node)
    
    my_smartphone_node = Node(
        package = "my_py_pkg",
        executable = "smartphone"
    )
    ld.add_action(my_smartphone_node)
    return ld