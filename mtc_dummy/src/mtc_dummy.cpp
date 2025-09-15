#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
 
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dummy");
namespace mtc = moveit::task_constructor;
 
class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions &options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    void doTask();
    void setupPlanningScene();
 
private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};
 
// 获取节点的基础接口：让外部代码访问 ROS 2 节点的 最小功能接口（NodeBaseInterface），而不暴露完整的 Node 对象。
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}
 
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options) : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}
 
// 规划场景设置
void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};
 
    geometry_msgs::msg::Pose pose;
    pose.position.x = -0.3;
    pose.position.y = 0.2;
    pose.orientation.w = 1.0;
    object.pose = pose;
 
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}
 
// 任务执行流程
void MTCTaskNode::doTask()
{
    task_ = createTask(); // 1.创建任务
 
    try
    {
        task_.init(); // 2.初始化任务
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }
 
    if (!task_.plan(5))
    { // 3.规划任务，超时5秒
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front()); // 4.发布可视化结果
 
    auto result = task_.execute(*task_.solutions().front()); // 5.执行规划结果
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }
 
    return;
}
 
// 任务构造
mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_); // 加载机器人模型
 
    const auto &arm_group_name = "dummy_arm";
    const auto &hand_group_name = "dummy_hand";
    const auto &hand_frame = "dummy_hand";
 
    // 设置任务属性
    task.setProperty("group", arm_group_name); // 控制机械臂组
    task.setProperty("eef", hand_group_name);  // 末端执行器组
    task.setProperty("ik_frame", hand_frame);  // 逆运动学参考框架
 
// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop
 
    // 添加任务阶段
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get(); // 保存指针供后续使用
    task.add(std::move(stage_state_current));      // 添加当前状态阶段
 
    // 求解器
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);           // 使用 MoveIt 的规划管道，通常默认为 OMPL

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); // 是一个简单的规划器，它在 Start 和 Goal Joint 状态之间进行插值
 
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>(); // 笛卡尔路径规划器
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);
 
    // 添加“打开手爪”阶段
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner); // 创建移动任务阶段，并指定关节空间插值规划器
    stage_open_hand->setGroup(hand_group_name);                                                       // 设置目标运动组 如 "hand" 或 "gripper"
    stage_open_hand->setGoal("open");                                                                 // 设置目标为命名位形（来自SRDF） 需预定义
    task.add(std::move(stage_open_hand));                                                             // 将阶段添加到任务流水线
 
    // Pick stages
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",     // Connect会尝试连接前一个阶段的结束状态和下一个阶段的开始状态
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}}); // 规划器和运动组配置
    stage_move_to_pick->setTimeout(5.0);                                                                                                            // 超时时间设为 5 秒
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);                                                                         // 继承父阶段属性
    task.add(std::move(stage_move_to_pick));                                                                                                        // 添加到任务
 
    mtc::Stage *attach_object_stage = nullptr; // 创建一个指向MoveIt Task Constructor阶段对象的指针，并将其设置为nullptr，稍后将用于保存阶段。
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});           // 将父任务（task）的指定属性 动态绑定 到子容器（grasp）的属性
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"}); // 初始化属性继承
 
        { // 1.创建一个MoveRelative阶段，用于接近物体
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner); // MTC 中的相对运动阶段，沿指定方向移动一定距离
            stage->properties().set("marker_ns", "approach_object");                                        // 设置可视化标记命名空间
            stage->properties().set("link", hand_frame);                                                    // 指定运动参考坐标系
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});                           // 从父容器（grasp）继承 group 属性
            stage->setMinMaxDistance(0.1, 0.15);                                                            // 规划器会在此范围内生成运动路径（例如：沿方向移动 10~15 厘米）
 
            geometry_msgs::msg::Vector3Stamped vec; // Set hand forward direction
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage)); // 将阶段添加到容器
        }
 
        { // 2.创建一个阶段来生成抓取姿态
            // 生成围绕目标物体（object）的多个候选抓取位姿，考虑物体几何和抓取方向
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT); // 继承父容器属性
            stage->properties().set("marker_ns", "grasp_pose");        // RViz中显示的标记命名空间
            stage->setPreGraspPose("open");                            // 预抓取姿态（来自SRDF）
            stage->setObject("object");                                // 目标物体的ID（需与PlanningScene中一致）
            stage->setAngleDelta(M_PI / 12);                           // 旋转步长（15度）
            stage->setMonitoredStage(current_state_ptr);               // 监听当前状态
 
            // 抓取坐标系变换 (Eigen::Isometry3d)
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.1; // = 0.1 表示抓取点沿手爪Z轴偏移10cm（补偿物体中心到手爪的距离）
 
            // 将生成的抓取位姿通过逆运动学（IK）转换为机械臂关节空间解
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);                                                   // 最大IK解数量
            wrapper->setMinSolutionDistance(1.0);                                            // 解之间的最小关节空间距离
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);                          // 设置IK参考坐标系
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});   // 继承属性
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"}); // 从接口继承目标位姿
            grasp->insert(std::move(wrapper));                                               // 是外层的SerialContainer,此阶段会作为抓取任务的一个子阶段插入
        }
 
        { // 3.修改规划场景(ModifyPlanningScene) 的阶段，专门用于在抓取过程中允许机械手(hand)和目标物体(object)之间的碰撞
            // 创建了一个名为"allow collision (hand,object)"的ModifyPlanningScene阶段
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("object", //目标物体指定
                                             // 获取机械手组(hand_group_name)中所有具有碰撞几何体的连杆名称
                                    task.getRobotModel() ->getJointModelGroup(hand_group_name) ->getLinkModelNamesWithCollisionGeometry(),
                                    true);   // 参数true表示允许这些连杆与物体之间的碰撞
            grasp->insert(std::move(stage)); // 将这个阶段添加到抓取容器(grasp)中
        }
 
        { // 4.关闭手部
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }
 
        { // 5.再次使用ModifyPlanningScene阶段，这次是将物体附加到手部，使用attachObject
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
 
        { // 6.创建了一个 MoveRelative 阶段，用于在抓取物体后将其垂直提起。这是抓取操作中常见的"提起"步骤，确保物体安全离开支撑表面
            // 创建一个相对运动阶段，命名为 "lift object"，使用 cartesian_planner（笛卡尔空间规划器）
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" }); // 从父容器（grasp）继承 group 属性
            stage->setMinMaxDistance(0.1, 0.3); // 沿指定方向移动 10-30厘米，规划器会在此范围内寻找无碰撞的最优距离
            stage->setIKFrame(hand_frame); // 指定逆运动学计算的参考坐标系（末端执行器坐标系）如 "panda_hand"
            stage->properties().set("marker_ns", "lift_object"); // RViz 可视化标记
            
            // 运动方向定义
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";  // 参考坐标系设为世界坐标系
            vec.vector.z = 1.0;             // 沿世界坐标系的Z轴正方向（垂直向上）
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        task.add(std::move(grasp));
    }
 
    // Place stages
    { // 1.创建了一个 Connect 阶段，用于规划机械臂和手爪从当前位置到放置位置的过渡运动
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},    // 机械臂运动组
                                                     {hand_group_name, sampling_planner}}); // 手爪运动组
        stage_move_to_place->setTimeout(5.0);  // 5秒超时
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT); // 从父任务继承 eef、group 等属性
        task.add(std::move(stage_move_to_place));
    }
 
    { // 2.创建了一个 串行容器（SerialContainer） 用于组织放置物体的多个子阶段，并通过属性配置确保其继承父任务的参数
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"}); // 将父任务（task）的 eef（末端执行器）、group（运动组）、ik_frame（逆运动学参考帧）属性 动态绑定 到子容器。
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});// 从父任务 复制初始值 到子容器（仅初始化时生效，后续父任务修改不影响子容器）
        
        { // 3.创建了一个 放置位姿生成与逆运动学求解 的复合阶段，用于计算机械臂放置物体时的目标位姿和可行关节解
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose"); // 生成放置位姿阶段
            stage->properties().configureInitFrom(mtc::Stage::PARENT);  // 继承父容器属性
            stage->properties().set("marker_ns", "place_pose");         // RViz可视化标记的命名空间
            stage->setObject("object");                                 // 要放置的物体ID
 
            // 放置位姿设置
            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "object";     // 相对于物体坐标系
            // target_pose_msg.pose.position.y = 0.5;          // 沿物体Y轴偏移0.5米
            target_pose_msg.pose.position.x = 0.5;          // 沿物体Y轴偏移0.5米
            target_pose_msg.pose.orientation.w = 1.0;       // 无旋转（四元数单位态）
            stage->setPose(target_pose_msg);                // 设置相对位姿
            stage->setMonitoredStage(attach_object_stage);  // 确保物体仍附着在机械手上
 
            // 逆运动学求解阶段
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage)); // 将放置位姿转换为机械臂关节空间解
            wrapper->setMaxIKSolutions(2);          // 最多计算2个IK解
            wrapper->setMinSolutionDistance(1.0);   // 解之间的最小关节空间距离
            wrapper->setIKFrame("object");          // 以物体坐标系为IK参考
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});  // 继承属性
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});// 从接口继承目标位姿
            place->insert(std::move(wrapper));      // 加入容器
        }
 
        { // 4.创建了一个 MoveTo 阶段，用于在放置物体后控制手爪（夹爪）张开到预定义的"open"位置
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }
 
        { // 5.创建了一个 ModifyPlanningScene 阶段，用于在放置操作完成后重新禁止机械手与物体之间的碰撞检测
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision"); // 创建一个修改规划场景的阶段，命名为 "forbid collision"
            stage->allowCollisions("object", // 目标物体ID
                                // 获取机械手所有带碰撞几何的连杆名称
                                   task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                                   false);  // 禁止碰撞
            place->insert(std::move(stage));// 加入容器
        }
 
        { // 6.创建了一个 ModifyPlanningScene 阶段，专门用于将已抓取的物体从机械手上分离（detach）
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("object", hand_frame); // 物体分离操作
            place->insert(std::move(stage));// 加入容器
        }
 
        { // 7.创建了一个 MoveRelative 阶段，用于在放置物体后控制机械臂沿世界坐标系X轴负方向后退一定距离，实现安全撤离
            // 创建一个相对运动阶段，命名为 "retreat"（安全撤离）规划器：使用 cartesian_planner（笛卡尔空间规划器）
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"}); // 从父容器（place）继承 group 属性（如 "panda_arm"）
            stage->setMinMaxDistance(0.1, 0.3); // 沿指定方向移动 10-30厘米
            stage->setIKFrame(hand_frame);      // 逆运动学参考坐标系
            stage->properties().set("marker_ns", "retreat"); //  RViz可视化标记
 
            // 运动方向定义
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";  // 参考坐标系设为世界坐标系 为什么用世界坐标系？ 确保撤离方向固定（通常为机械臂基座的后方），不受机械手当前姿态影响
            vec.vector.x = -0.5;            // 沿世界坐标系X轴负方向
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
 
        task.add(std::move(place));
    }
 
    { // 8.创建了一个 MoveTo 阶段，用于控制机械臂返回预定义的"ready"（就绪）位置，通常用于任务结束后的复位操作
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setGoal("home");
        task.add(std::move(stage));
    }
 
    return task;
}
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 
    // 作用：启用节点参数的自动声明机制，允许通过构造函数参数或命令行参数直接覆盖参数值，而无需预先声明
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
 
    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
 
    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                     {
                                                         executor.add_node(mtc_task_node->getNodeBaseInterface());    // 添加节点到执行器
                                                         executor.spin();                                             // 启动执行器
                                                         executor.remove_node(mtc_task_node->getNodeBaseInterface()); // 清理节点
                                                     });
 
    mtc_task_node->setupPlanningScene(); // 设置场景
    mtc_task_node->doTask();             // 执行任务
 
    spin_thread->join(); // 等待 spin_thread 线程结束
    rclcpp::shutdown();  // 关闭 ROS 2 节点
    return 0;
}