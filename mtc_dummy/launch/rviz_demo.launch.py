import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt 配置加载
    moveit_config = (MoveItConfigsBuilder("dummy2-gripperv2", package_name="moveit_dummy2_config")
        .robot_description(file_path="config/dummy2-gripperv2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy2-gripperv2.srdf")  
        .to_moveit_configs()
    )
    rviz_config_file = get_package_share_directory("mtc_dummy") + "/config/mtc.rviz"
    ros2_controllers_path = os.path.join(get_package_share_directory("moveit_dummy2_config"),"config","ros2_controllers.yaml")
    move_group_capabilities = { "capabilities": "move_group/ExecuteTaskSolutionCapability" }
    
    # 1.MoveGroup 节点
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), move_group_capabilities], # 启用任务解决方案执行能力
    )

    # 2.RViz 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.robot_description,            # 传递机器人模型描述
                    moveit_config.robot_description_semantic],  # 传递语义描述（如规划组定义）。
    )

    # 3.静态 TF 发布
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 4.机器人状态发布
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    # 5.ROS2 控制节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
    )
    # 6.控制器加载
    load_controllers = [
        ExecuteProcess(
            cmd=["ros2 run controller_manager spawner {}".format(controller)],
            shell=True,
        )
        for controller in [
            "dummy_arm_controller",
            "dummy_hand_controller",
            "joint_state_broadcaster",
        ]
    ]
    return LaunchDescription([  #列表扩展符 (+) 将 load_controllers 中的元素展开后合并到主列表中
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers # 它也是个列表
    )