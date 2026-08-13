/*
@brief This ROS 2 node is responsible for running weed removal task in a loop, based on 4 specified poses.

TODO: add comments about method functionalities

@author Ziga Breznikar
@date 12.08.2026
*/

#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <chrono>

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

namespace move_loop
{
    class MTCTaskNode
    {
        public:
            MTCTaskNode(const rclcpp::NodeOptions& options);
            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
            bool setupPlanningScene();
            void loopTask();
            void doTaskOnce();
        private:
            bool doTask();
            mtc::Task createTask();
            bool getPoses(); // this can be replaced with service for perception
            unsigned short num_of_weeds_;
            mtc::Task task_;
            rclcpp::Node::SharedPtr node_;
            std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;

    }; // MTCTaskNode

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("move_loop_mtc", options)}
    {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    bool MTCTaskNode::getPoses()
    {
        num_of_weeds_ = static_cast<unsigned short>(
            node_->get_parameter("num_of_poses").as_int()
        );

        goal_poses_.clear();
        goal_poses_.resize(num_of_weeds_);

        for(unsigned short i = 0; i < num_of_weeds_; i++){
            std::stringstream ss;
            ss << "poses.pose_" << i;
            std::string s = ss.str();

            std::vector<double> pose_array = node_->get_parameter(s).as_double_array();

            if(pose_array.size() != 7){
                RCLCPP_ERROR(node_->get_logger(), "Invalid pose size");
                return false;
            }

            geometry_msgs::msg::PoseStamped pose;

            pose.header.frame_id = "base_link";
            pose.pose.position.x = pose_array[0];
            pose.pose.position.y = pose_array[1];
            pose.pose.position.z = pose_array[2];

            pose.pose.orientation.x = pose_array[3];
            pose.pose.orientation.y = pose_array[4];
            pose.pose.orientation.z = pose_array[5];
            pose.pose.orientation.w = pose_array[6];

            goal_poses_.push_back(pose);


        }

        return true;

    }

    bool MTCTaskNode::setupPlanningScene()
    {
        if(!getPoses()){
            RCLCPP_ERROR(node_->get_logger(), "Could not extract poses");
            return false;
        }

        moveit::planning_interface::PlanningSceneInterface psi;

        // add try catch for debugging purposes

        // create weed_objects CollisionObject vector
        std::vector<moveit_msgs::msg::CollisionObject> weed_objects;
        weed_objects.clear();
        weed_objects.resize(num_of_weeds_);

        //moveit_msgs::msg::CollisionObject weed_objects;
        //weed_objects.id = "weed_objects";
        //weed_objects.header.frame_id = "base_link";
        //weed_objects.primitives.resize(num_of_weeds_);
        //weed_objects.primitive_poses.resize(num_of_weeds_);

        float height = 0.005;
        float width = 0.02;

        for(unsigned short i = 0; i < num_of_weeds_; i++){
            std::string weed_id = "weed_" + std::to_string(i);
            weed_objects[i].id = weed_id;
            weed_objects[i].header.frame_id = "base_link";
            weed_objects[i].primitives.resize(1);
            weed_objects[i].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            weed_objects[i].primitives[0].dimensions = {height,width};
            weed_objects[i].pose = goal_poses_[i].pose;

            psi.applyCollisionObject(weed_objects[i]);

            //weed_objects.primitives[i].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            //weed_objects.primitives[i].dimensions = {height,width};
            //weed_objects.primitives_poses[i] = goal_poses_[i].pose;
        }


        //psi.applyCollisionObject(weed_objects);

        return true;

    }

    mtc::Task MTCTaskNode::createTask()
    {
        mtc::Task task;
        task.stages()->setName("removal demonstration task");
        task.loadRobotModel(node_);

        const auto& arm_group_name = "me6_group";
        const auto& hand_group_name = "gripper";
        const auto& hand_frame = "dummy_tcp";

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // initialize planners
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.03); // desired step size in m => change if cartesian planner does not want to compute solutions

        mtc::Stage* current_state_ptr = nullptr;

        /****************************************************
        *                                                   *
        *                   Current State                   *
        *                                                   *
        ****************************************************/

        //std::stringstream ss;
        //ss << "current_" << i;
        //std::string s = ss.str();

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current_state");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        for(unsigned short i = 0; i < num_of_weeds_; i++){
            //std::string weed_id = "weed_" + std::to_string(i);

            /****************************************************
            *                                                   *
            *                   Current State                   *
            *                                                   *
            ****************************************************/

            //std::stringstream ss;
            //ss << "current_" << i;
            //std::string s = ss.str();

            //auto stage_state_current = std::make_unique<mtc::stages::CurrentState>(s);
            //current_state_ptr = stage_state_current.get();
            //task.add(std::move(stage_state_current));

            std::string weed_id = "weed_" + std::to_string(i);
            

            /****************************************************
            *                                                   *
            *                Move to Weed                       *
            *                                                   *
            ****************************************************/

            std::stringstream ss;
            //ss.str(""); // empty it
            ss << "move_to_weed_" << i;
            std::string s = ss.str();
            //s = ss.str();

            auto stage_move_to_weed = std::make_unique<mtc::stages::Connect>(
                s,
                mtc::stages::Connect::GroupPlannerVector{ {arm_group_name, sampling_planner} });
            stage_move_to_weed->setTimeout(5.0);
            stage_move_to_weed->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage_move_to_weed));

            /****************************************************
            *                                                   *
            *         Pull Weed Serial Container                *
            *                                                   *
            ****************************************************/

            {
                std::stringstream ss;
                ss << "pull_weed_" << i;
                std::string s = ss.str();

                auto stage_pull_weed = std::make_unique<mtc::SerialContainer>(s);
                task.properties().exposeTo(stage_pull_weed->properties(), { "eef", "group", "ik_frame" }); // declare properties from parent task
                stage_pull_weed->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" }); // initialize properties from parent task

                {
                    /****************************************
                    *        Allow collision hand-weed      *
                    ****************************************/ 

                    auto stage =
                        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, weed)");
                    stage->allowCollisions(
                        weed_id,
                        task.getRobotModel()->getLinkModelNamesWithCollisionGeometry(),
                        true);
                    
                    stage_pull_weed->insert(std::move(stage));

                }

                {
                    /****************************************
                    *             Move relative             *
                    ****************************************/

                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("cartesian approach weed", cartesian_planner);
                    stage->properties().set("marker_ns", "approach_object");
                    stage->properties().set("link", hand_frame);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.01, 0.3); // min max distance

                    // set movement direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame; // move w.r.t. dummy_tcp
                    vec.vector.z = 1.0; // move into positive z direction
                    stage->setDirection(vec);
                    stage_pull_weed->insert(std::move(stage));

                }

                {
                    /****************************************
                    *          Generate grasp pose          *
                    ****************************************/          
                   
                    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate removal pose");
                    stage->properties().configureInitFrom(mtc::Stage::PARENT);
                    stage->properties().set("marker_ns", "grasp_pose");
                    stage->setPreGraspPose("open");
                    stage->setObject(weed_id); // does this work if weed_objects is a vector?
                    stage->setAngleDelta(M_PI / 12);
                    stage->setMonitoredStage(current_state_ptr);

                    Eigen::Isometry3d grasp_frame_transform;
                    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
                    grasp_frame_transform.linear() = q.matrix();
                    grasp_frame_transform.translation().z() = -0.010;

                    auto wrapper =
                    std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
                    wrapper->setMaxIKSolutions(8);
                    wrapper->setMinSolutionDistance(1.0);
                    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
                    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                    stage_pull_weed->insert(std::move(wrapper));

                }

                {
                    /****************************************
                    *              Close gripper            *
                    ****************************************/  
                    auto stage =
                        std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
                    stage->setGroup(hand_group_name);
                    stage->setGoal("closed");
                    stage_pull_weed->insert(std::move(stage));  
                }

                {
                    /****************************************
                     *           Attach weed                 *
                     ****************************************/ 
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach weed");
                    stage->attachObject(weed_id, hand_frame); // again how does this work with vector weed_objects
                    stage_pull_weed->insert(std::move(stage));

                }

                {
                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.01, 0.3); // edit this for pulling the weed out
                    stage->setIKFrame(hand_frame);
                    stage->properties().set("marker_ns", "lift_object");

                    // Set upward direction w.r.t. world
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = "world";
                    vec.vector.y = -1.0;
                    stage->setDirection(vec);
                    stage_pull_weed->insert(std::move(stage));
                }

                task.add(std::move(stage_pull_weed));

            }

            /****************************************************
            *                                                   *
            *                 Move To Dump                      *
            *                                                   *
            ****************************************************/ 

            ss.str("");
            //std::stringstream ss;
            ss << "move_to_dump_" << i;
            s = ss.str();

            auto stage_dump = std::make_unique<mtc::stages::MoveTo>(s,sampling_planner);
            stage_dump->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage_dump->setGoal("drop");
            task.add(std::move(stage_dump)); 

            /****************************************************
            *                                                   *
            *            Drop weed Serial Container             * 
            *                                                   *
            ****************************************************/

            {
                std::stringstream ss;
                ss << "dump_weed_" << i;
                auto stage_drop_weed = std::make_unique<mtc::SerialContainer>(s);
                task.properties().exposeTo(stage_drop_weed->properties(), { "eef", "group", "ik_frame" });
                stage_drop_weed->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

                {
                    /****************************************
                    *           Open gripper                *
                    ****************************************/ 
                    auto stage = 
                        std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
                    stage->setGroup(hand_group_name);
                    stage->setGoal("open");
                    stage_drop_weed->insert(std::move(stage));   
                }

                {
                    /****************************************
                    *           Forbid collision            *
                    ****************************************/ 
                    auto stage =
                        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                    stage->allowCollisions(weed_id, // again how does this work with vector
                                        task.getRobotModel()
                                            ->getJointModelGroup(hand_group_name)
                                            ->getLinkModelNamesWithCollisionGeometry(),
                                        false);
                    stage_drop_weed->insert(std::move(stage));
                }

                {

                    /****************************************
                    *           Detach object               *
                    ****************************************/ 
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                    stage->detachObject(weed_id, hand_frame); // again how does this work with vector
                    stage_drop_weed->insert(std::move(stage));

                }

                {

                    /****************************************
                    *           Retreat                     *
                    ****************************************/ 

                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.0, 0.5); // No need to move
                    stage->setIKFrame(hand_frame);
                    stage->properties().set("marker_ns", "retreat");

                    // retreat direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = "world";
                    vec.vector.z = 1.0;
                    stage->setDirection(vec);
                    stage_drop_weed->insert(std::move(stage));

                }

                task.add(std::move(stage_drop_weed));

            }

        }

        return task;

    }

    bool MTCTaskNode::doTask()
    {
        try
        {
            // plan a new solution
            if(!task_.plan(5)){
                RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
                return false;
            }
        }
        catch(const std::exception& e) // general exception
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Exception when planning task: %s",
                e.what()
            );
            return false;
        }

        if(task_.solutions().empty()){
            RCLCPP_ERROR(node_->get_logger(), "No task solutions found");
            return false;
        }

        task_.introspection().publishSolution(*task_.solutions().front()); // visualize
        RCLCPP_INFO(node_->get_logger(), "Executing task");

        //execute, blocks...
        auto results = task_.execute(*task_.solutions().front());

        if (results.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
            RCLCPP_ERROR(
                node_->get_logger(),
                "Task execution failed with error code: %d",
                results.val);

            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Task execution succeeded");

        return true;
        
    }

    void MTCTaskNode::loopTask()
    {

        
        if(!setupPlanningScene()){
            RCLCPP_ERROR(node_->get_logger(), "Could not setup PlanningScene");
            return;
        }

        task_ = createTask();

        try
        {
            task_.init();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Exception when initializing task: %s",
                e.what());
            return;
        }

        while (rclcpp::ok())
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Starting demonstration cycle");

            if (!doTask())
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Task failed, stopping demonstration");

                break;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Demonstration cycle completed");

            // Pause between cycles
            rclcpp::sleep_for(
                std::chrono::seconds(3));
        }
        
    }

    void MTCTaskNode::doTaskOnce()
    {
        if(!setupPlanningScene()){
            RCLCPP_ERROR(node_->get_logger(), "Could not setup PlanningScene");
            return;
        }

        task_ = createTask();

        try
        {
            task_.init();
        }
        catch(mtc::InitStageException& e) // issues with connecting stages
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e);
            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Starting demonstration cycle");

        if (!doTask())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Task failed, stopping demonstration");
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Demonstration cycle completed");

        return;

    }


} // move_loop

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<move_loop::MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // call mtc_task_node related functions here
  //mtc_task_node->setupPlanningScene();
  //mtc_task_node->loopTask();
  mtc_task_node->doTaskOnce();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
