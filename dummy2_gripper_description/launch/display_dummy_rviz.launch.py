import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'dummy2_gripper_description' # 包名
    pkg_share = get_package_share_directory(package_name)
    
    xacro_file = os.path.join(pkg_share,'urdf','dummy2-gripperv2.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    urdf_model_path = robot_description_config.toxml()
    
    rviz_config_path = os.path.join(pkg_share, 'config', 'dummy2.rviz')  # 替换为实际配置文件名  

    # 功能：根据关节状态发布机器人模型的状态，用于可视化
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_model_path}] # 传递 URDF 内容
    )

    # 功能：通过图形界面发布机器人关节状态
    joint_state_publisher_gui = Node(   
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': urdf_model_path}]   # 传递 URDF 内容
    )

    # 启动 RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path] # 通过 -d 参数指定配置文件
    )
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
