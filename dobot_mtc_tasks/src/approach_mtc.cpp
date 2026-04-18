/*
@brief This ROS 2 node implements a MoveIt Task Constructor for approaching an object using tester EE.

@description This program serves for testing purposes, more specifically generating MoveIt planning scenes from point cloud data
and approaching an object with tester EE.

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

namespace approach
{
    class MTCTaskNode
    {
    public:
        // constructor
        MTCTaskNode(const rclcpp::NodeOptions& options);
        // getter function for node base interface
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        // interface with MTC task object
        void doTask();
        // sets up planning scene
        void setupPlanningScene();

    private:
        // Compose an MTC task from a series of stages.
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped object_pose_; // object pose
    }; // Approach

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("approach_mtc", options) }
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
        object.primitives[0].dimensions = { 0.5, 0.005 };

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

    /*
    @brief Plan and execute approach task.
    */

    void MTCTaskNode::doTask()
    {
        task_ = createTask();

        try
        {
            task_.init();
            RCLCPP_INFO(node_->get_logger(), "Starting approach task!");
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
        // execute the planned task
        auto results = task_.execute(*task_.solutions().front());
        if (results.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Task execution failed with error code: %d", results.val);
            return;
        }

        return;
    }

    /*
    @brief Create the MTC task with all the necessary stages.
    @return Created MTC task
    */

    mtc::Task MTCTaskNode::createTask()
    {
        mtc::Task task;
        task.stages()->setName("approach task");
        task.loadRobotModel(node_);

        const auto& arm_group_name = "me6_group"; // main planning group
        const auto& hand_group_name = "tester"; // end effector group
        const auto& hand_frame = "dummy_end_effector"; // frame for inverse kinematics

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // Here we can add parameters from config file later, for now hardcoded.
        std::string me6_group_home_pose = "home";

        // Initialize planners for different types of motions
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);

        /****************************************************
         *                                                  *
         *               Current State                      *
         *                                                  *
         ***************************************************/
        // pointer to store the current state
        mtc::Stage* current_state_ptr = nullptr;

        // add a stage to capture the current state
        {
            auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
            current_state_ptr = stage_state_current.get();
            task.add(std::move(stage_state_current));
        }
        /****************************************************
         *                                                  *
         *               Move To Approach                   *
         *                                                  *
         ***************************************************/
        // create a stage to move the arm to a pre-approach position
        {
            auto stage_move_to_approach = std::make_unique<mtc::stages::Connect>(
                "move to approach",
                mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } }
            );
            stage_move_to_approach->setTimeout(5.0);
            stage_move_to_approach->properties().configureInitFrom(mtc::Stage::PARENT); // inherit properties like group, eef etc
            task.add(std::move(stage_move_to_approach));
        }
        /****************************************************
         *                                                  *
         *               Approach Container                 *
         *                                                  *
         ***************************************************/

        {
            auto approach_container = std::make_unique<mtc::SerialContainer>("approach object");
            approach_container->properties().set("group", arm_group_name);
            approach_container->properties().set("eef", hand_group_name);
            approach_container->properties().set("ik_frame", hand_frame);

            /****************************************
            * 1. Generate approach poses from cylinder => Changed to GenerateRandomPose
            ****************************************/ 
            auto generate_approach = std::make_unique<mtc::stages::GenerateRandomPose>("generate approach pose");
            generate_approach->properties().configureInitFrom(mtc::Stage::PARENT);
            generate_approach->setPose(object_pose_);
            generate_approach->setMonitoredStage(current_state_ptr); 
            generate_approach->setMaxSolutions(50); 
            // Extrinsic ZYX rotation <-> Intrinsic XYZ
            // Extrinsic ZYX: Rotate about fixed axes in order Z->Y->X
            // Radians
            generate_approach->sampleDimension<std::uniform_real_distribution>(mtc::stages::GenerateRandomPose::ROLL, 0.17);
            generate_approach->sampleDimension<std::uniform_real_distribution>(mtc::stages::GenerateRandomPose::PITCH, 0.17);
            generate_approach->sampleDimension<std::uniform_real_distribution>(mtc::stages::GenerateRandomPose::YAW, M_PI_4);


            // define EE pose
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.05;

            /****************************************
            * 2. Solve IK for generated poses (wrapper pattern)
            ****************************************/ 
            auto approach_ik = std::make_unique<mtc::stages::ComputeIK>("approach IK", std::move(generate_approach));
            approach_ik->setMaxIKSolutions(8);          // try up to 8 solutions 
            approach_ik->setMinSolutionDistance(0.01);
            approach_ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            approach_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            approach_ik->setIKFrame(grasp_frame_transform, hand_frame);
            

            approach_container->insert(std::move(approach_ik));

            /****************************************
            * 3. Cartesian approach
            ****************************************/
           /*
           {
            auto stage_approach_object = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage_approach_object->properties().set("marker_ns", "approach_object");
            stage_approach_object->setIKFrame(hand_frame);
            stage_approach_object->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage_approach_object->setMinMaxDistance(0.001, 0.15);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;                     // move forward along dummy_end_effector Z axis
            stage_approach_object->setDirection(vec);

            approach_container->insert(std::move(stage_approach_object));    
           }

           {
            auto stage_retreat_object = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage_retreat_object->properties().set("marker_ns", "approach_object");
            stage_retreat_object->setIKFrame(hand_frame);
            stage_retreat_object->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage_retreat_object->setMinMaxDistance(0.001, 0.15);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = -1.0;
            stage_retreat_object->setDirection(vec);

            approach_container->insert(std::move(stage_retreat_object));        
           }
           */
           task.add(std::move(approach_container));

        }
        
        // fixes maybe
        //{
        //auto stage_current_after = std::make_unique<mtc::stages::CurrentState>("post-approach state");
        //task.add(std::move(stage_current_after));
        //}

        /****************************************************
         *                                                  *
         *                 Connect To Home                  *
         *                                                  *
         ***************************************************/
        /*
        // create a connect stage to home
        auto stage_connect_to_home = std::make_unique<mtc::stages::Connect>(
            "connect to home", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } }
        );

        stage_connect_to_home->setTimeout(5.0);
        stage_connect_to_home->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_connect_to_home));
        */

        /****************************************************
         *                                                  *
         *               Move To Home                       *
         *                                                  *
         ***************************************************/
        // create a stage that moves to home position
        auto stage_move_home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage_move_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage_move_home->setGoal(me6_group_home_pose);
        task.add(std::move(stage_move_home));

        return task;

    }   

} // namespace approach

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<approach::MTCTaskNode>(options);
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