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

#include "dobot_msgs_fb/srv/approach_object.hpp"

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
        bool getObjectFromService(int object_id); // fetch object pose

    private:
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped object_pose_; // object pose
        geometry_msgs::msg::PoseStamped dumping_pose_; // pose for dumping weed
        geometry_msgs::msg::PoseStamped ground_plane_pose_; // pose of ground collision plane
        rclcpp::Client<dobot_msgs_fb::srv::ApproachObject>::SharedPtr approach_client_;
        float height;
        float radius;
    }; // MTCTaskNode

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("remove_weed_mtc", options)}
    {
        approach_client_ = 
            node_->create_client<dobot_msgs_fb::srv::ApproachObject>("/approach_object");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    bool MTCTaskNode::getObjectFromService(int object_id)
    {
        if(!approach_client_->wait_for_service(std::chrono::seconds(2))){ // if not responding in 2s
            RCLCPP_ERROR(node_->get_logger(), "Service not available!");
            return false;
        }

        auto request = std::make_shared<dobot_msgs_fb::srv::ApproachObject::Request>();
        request->object_id = object_id;

        auto future = approach_client_->async_send_request(request);

        if(future.wait_for(std::chrono::seconds(2)) != std::future_status::ready){ // attempting to call service
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return false;
        }

        auto response = future.get();

        RCLCPP_INFO(node_->get_logger(),
                    "Got object coordinates: x=%.3f, y=%.3f, z=%.3f, radius=%.3f, height=%.3f",
                    response->x, response->y, response->z, response->radius, response->height);
        
        object_pose_.header.frame_id = "base_link";

        object_pose_.pose.position.x = response->x;
        object_pose_.pose.position.y = response->y;
        object_pose_.pose.position.z = response->z;

        object_pose_.pose.orientation.z = 0.0;
        object_pose_.pose.orientation.w = 1.0;

        radius = response->radius;
        //height = response->height;
        height = 0.05; // quick fix

        return true;

    }

    void MTCTaskNode::setupPlanningScene()
    {
        // ground collision plane
        moveit_msgs::msg::CollisionObject ground_plane;
        ground_plane.id = "ground_plane";
        ground_plane.header.frame_id = "base_link";
        ground_plane.primitives.resize(1);
        ground_plane.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        ground_plane.primitives[0].dimensions = {0.005,1.5,1.5}; // change this if needed

        ground_plane_pose_.header.frame_id = "base_link";

        ground_plane_pose_.pose.position.x = 0.0;
        ground_plane_pose_.pose.position.y = 0.23;
        ground_plane_pose_.pose.position.z = 0.0;

        ground_plane_pose_.pose.orientation.x = 0.7071;
        ground_plane_pose_.pose.orientation.y = 0.7071;;
        ground_plane_pose_.pose.orientation.z = 0.0;
        ground_plane_pose_.pose.orientation.w = 0.0;

        ground_plane.pose = ground_plane_pose_.pose;


        // creating object
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "base_link";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;


        // Call service here to dynamically get object cartesian coordinates
        if(!getObjectFromService(1)){
            RCLCPP_ERROR(node_->get_logger(), "Using fallback pose!");

            object.primitives[0].dimensions = { 0.1, 0.02 };

            object_pose_.header.frame_id = "base_link";

            object_pose_.pose.position.x = -0.2;
            object_pose_.pose.position.y = -0.2;
            object_pose_.pose.position.z = -0.2;

            object_pose_.pose.orientation.z = 0.0;
            object_pose_.pose.orientation.w = 1.0;

        }
        else{
            object.primitives[0].dimensions = { height, radius};
        }


        // creating object
        //moveit_msgs::msg::CollisionObject object;
        //object.id = "object";
        //object.header.frame_id = "base_link";
        //object.primitives.resize(1);
        //object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        //object.primitives[0].dimensions = { 0.1, 0.02 };

        //geometry_msgs::msg::Pose pose;
        //pose.position.x = -0.2;
        //pose.position.y = 0.15;
        //pose.position.z = 0.1;
        //pose.orientation.z = 0.0;
        //pose.orientation.w = 1.0; 


        object.pose = object_pose_.pose; // save to pose to object

        // setting up object_pose_ 
        //object_pose_.header.frame_id = object.header.frame_id;
        //object_pose_.pose = object.pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);
        psi.applyCollisionObject(ground_plane);

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
    task.stages()->setName("weed removal task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "me6_group";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "dummy_tcp";

    // Set task properties
    task.setProperty("group", arm_group_name); // manipulator group
    task.setProperty("eef", hand_group_name); // gripper group
    task.setProperty("ik_frame", hand_frame); // TCP of gripper

    // Defining dumping pose, read from yaml
    dumping_pose_.header.frame_id = node_->get_parameter("dumping_pose.header.frame_id").as_string();

    dumping_pose_.pose.position.x = node_->get_parameter("dumping_pose.position.x").as_double();
    dumping_pose_.pose.position.y = node_->get_parameter("dumping_pose.position.y").as_double();
    dumping_pose_.pose.position.z = node_->get_parameter("dumping_pose.position.z").as_double();

    dumping_pose_.pose.orientation.x = node_->get_parameter("dumping_pose.orientation.x").as_double();
    dumping_pose_.pose.orientation.y = node_->get_parameter("dumping_pose.orientation.y").as_double();
    dumping_pose_.pose.orientation.z = node_->get_parameter("dumping_pose.orientation.z").as_double();
    dumping_pose_.pose.orientation.w = node_->get_parameter("dumping_pose.orientation.w").as_double();


    // Initialize planners for different types of motions
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.03); // desired step size in m => change if cartesian planner does not want to compute solutions

    mtc::Stage* current_state_ptr = nullptr;
    mtc::Stage* attach_weed_ptr = nullptr;

    /****************************************************
     *                                                  *
     *               Current State                      *
     *                                                  *
     ***************************************************/    

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    /****************************************************
     *                                                  *
     *                Move to Weed                      *
     *                                                  *
     ***************************************************/ 

    auto stage_move_to_weed = std::make_unique<mtc::stages::Connect>(
            "move_to_weed",
            mtc::stages::Connect::GroupPlannerVector{ {arm_group_name, sampling_planner} });
    stage_move_to_weed->setTimeout(5.0);
    stage_move_to_weed->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_weed));

    /****************************************************
     *                                                  *
     *         Pull Weed Serial Container               *
     *                                                  *
     ***************************************************/  
    
    {
        auto stage_pull_weed = std::make_unique<mtc::SerialContainer>("pull weed");
        // Declare the task properties from the parent task in the serial container
        task.properties().exposeTo(stage_pull_weed->properties(), { "eef", "group", "ik_frame" });
        // Initialize properties so contained stages can access them
        stage_pull_weed->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

        {

            /****************************************
            *           Move Relative               *
            ****************************************/ 

            auto stage =
            std::make_unique<mtc::stages::MoveRelative>("cartesian approach weed", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.01, 0.3);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            stage_pull_weed->insert(std::move(stage));
        }

        {
            /****************************************
            *           Generate Grasp Poses        *
            ****************************************/ 
            /*
            This part is a bit confusing so descriptive comment follows:
                First section is responsible for generating IK poses
                Second section: grasp_frame_transform
                    It rotates and translated the hand frame
                    Tries to align the transformed frame (generate_frame_transform) with IK poses
            */
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate removal pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);

            // extrinsically rotate around x for 180
            // Extrinsic ZYX rotation <-> Intrinsic XYZ
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.05;

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
            *           Allow Collision             *
            ****************************************/ 

            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, weed)");
                stage->allowCollisions("object",
                                    task.getRobotModel()
                                        //->getJointModelGroup(hand_group_name) // disabled since it reports collision (it should not)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
            stage_pull_weed->insert(std::move(stage));

        }

        {
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
            stage->attachObject("object", hand_frame);
            attach_weed_ptr = stage.get();
            stage_pull_weed->insert(std::move(stage));

        }

        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.01, 0.3); // edit this for pulling the weed out
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            stage_pull_weed->insert(std::move(stage));
        }

        task.add(std::move(stage_pull_weed));

    }

    /****************************************************
     *                                                  *
     *         Move To Dump                             *
     *                                                  *
     ***************************************************/ 

    auto stage_move_to_dump = std::make_unique<mtc::stages::Connect>(
            "move_to_dump",
            mtc::stages::Connect::GroupPlannerVector{ {arm_group_name, sampling_planner} });
    stage_move_to_dump->setTimeout(5.0);
    stage_move_to_dump->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_dump));    

    /****************************************************
     *                                                  *
     *         Drop Weed Serial Container               *
     *                                                  *
     ***************************************************/ 

    {
        auto stage_drop_weed = std::make_unique<mtc::SerialContainer>("dump weed");
        task.properties().exposeTo(stage_drop_weed->properties(), { "eef", "group", "ik_frame" });
        stage_drop_weed->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });


        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate dumping pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("object");
            stage->setPose(dumping_pose_);

            stage->setMonitoredStage(attach_weed_ptr);

            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("dump pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("object");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    
            stage_drop_weed->insert(std::move(wrapper));

        }


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
            stage->allowCollisions("object",
                                task.getRobotModel()
                                    ->getJointModelGroup(hand_group_name)
                                    ->getLinkModelNamesWithCollisionGeometry(),
                                false);
            stage_drop_weed->insert(std::move(stage));
        }
            /****************************************
            *           Detach object               *
            ****************************************/ 
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("object", hand_frame);
            stage_drop_weed->insert(std::move(stage));

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

    /****************************************************
     *                                                  *
     *         Move To Home                             *
     *                                                  *
     ***************************************************/ 

    auto stage_move_to_home =
        std::make_unique<mtc::stages::MoveTo>("move to home", interpolation_planner);
    stage_move_to_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_move_to_home->setGoal("home");
    task.add(std::move(stage_move_to_home));   

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