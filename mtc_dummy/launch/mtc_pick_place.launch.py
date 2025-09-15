from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
 
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(robot_name='dummy2-gripperv2',package_name = "moveit_dummy2_config").to_dict()
 
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_dummy",
        executable="mtc_dummy",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
 
    return LaunchDescription([pick_place_demo])