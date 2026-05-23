/*
@brief This ROS 2 node implements a MoveIt Task Constructor for detecting and removing weeds.

@description The node calls the perception service, receives pose of weed and subsequently removes it.

@author Ziga Breznikar
@date 11.04.2026

*/

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace mtc = moveit::task_constructor; // for quality of life

namespace remove_weed
{
    class MTCTaskNode
    {
    public:
        MTCTaskNode(const rclcpp::NodeOptions& options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        void doTask();
        void setupPlanningScene();

    private:
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped object_pose_; // object pose
    }; // MTCTaskNode

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("remove_weed_mtc", options)}
    {

    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    void MTCTaskNode::setupPlanningScene()
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "base_link";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 };

        geometry_msgs::msg::Pose pose;
        pose.position.x = -0.4;
        pose.position.y = 0.15;
        pose.position.z = 0.4;
        pose.orientation.z = 1.0;
        pose.orientation.w = 0.0; // Force this to 0
        object.pose = pose;

        // setting up object_pose_ 
        object_pose_.header.frame_id = object.header.frame_id;
        object_pose_.pose = object.pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);

    }

    void MTCTaskNode::doTask()
    {
        task_ = createTask();

        try
        {
            task_.init();
            RCLCPP_INFO(node_->get_logger(), "Starting removal task!");
        }
        catch(mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e);
            return;
        }

        if(!task_.plan(5))
        {
            RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
            return;
        }

        // publish the planned solutions for visualization
        task_.introspection().publishSolution(*task_.solutions().front());
        RCLCPP_INFO(node_->get_logger(), "Published task");
        // execute the planned task
        auto results = task_.execute(*task_.solutions().front());
        RCLCPP_INFO(node_->get_logger(), "Executed task!");
        if (results.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Task execution failed with error code: %d", results.val);
            return;
        }

        return;
    }

    
    mtc::Task MTCTaskNode::createTask()
    {
    mtc::Task task;
    task.stages()->setName("removal task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "me6_group";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "dummy_tcp";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // add dumping robot pose defined in srdf file read from yaml file here. 

    // Initialize planners for different types of motions
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    mtc::Stage* current_state_ptr = nullptr;

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("closed");
    task.add(std::move(stage_open_hand));

    return task;
    }

} // remove_weed

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<remove_weed::MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}