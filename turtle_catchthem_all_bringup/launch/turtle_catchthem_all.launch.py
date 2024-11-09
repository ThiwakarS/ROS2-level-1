from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    turtle_sim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_controller_node = Node(
        package="turtle_catchthem_all",
        executable="turtle_controller",
    )

    turtle_killer_node = Node(
        package="turtle_catchthem_all",
        executable="turtle_killer",
    )

    turtle_spawner_node = Node(
        package="turtle_catchthem_all",
        executable="turtle_spawner",
    )
    
    # Event handler to stop all nodes if any node fails
    on_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle_sim_node,  # You can also target other nodes
            on_exit=[Shutdown(reason="A node has exited.")]
        )
    )

    # Add nodes to the LaunchDescription
    ld.add_action(turtle_sim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_killer_node)
    ld.add_action(turtle_spawner_node)
    
    # Add the event handler to stop all nodes on failure
    ld.add_action(on_exit_handler)

    return ld
