/*
@brief This ROS 2 node upgrades the existing framework for removing weeds with detecting and subsequent
avoidance of asparagus.

@description TODO

@author Ziga Breznikar
@date 14.06.2026

*/

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <vector>

#include "asparagus.hpp"
#include "dobot_msgs_fb/srv/remove_weed.hpp"


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

namespace avoid_remove_weed
{
    class MTCTaskNode
    {
    public:
        MTCTaskNode(const rclcpp::NodeOptions& options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        void doTask();
        void setupPlanningScene();
        bool getSceneFromService(); // calls perception service, data used in setupPlanningScene


    private:
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped weed_pose_;
        std::vector<Asparagus> asparagus_; // mnozina -> when calling service we create asparagus_ objects in a vector
        geometry_msgs::msg::PoseStamped ground_plane_pose_;
        geometry_msgs::msg::PoseStamped farmbeast_plane_pose_; 
        rclcpp::Client<dobot_msgs_fb::srv::RemoveWeed>::SharedPtr service_client_;
    }; // MTCTaskNode


    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("avoid_remove_weed_mtc", options)}
    {
      service_client_ = 
        node_->create_client<dobot_msgs_fb::srv::RemoveWeed>("/remove_weed");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

    bool MTCTaskNode::getSceneFromService() // add later
    {
    if(!service_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(node_->get_logger(), "Service not available");
        return false;
    }

    auto request = std::make_shared<dobot_msgs_fb::srv::RemoveWeed::Request>(); // create empty request, and send it
    auto future = service_client_->async_send_request(request); // send empty request

    if(future.wait_for(std::chrono::seconds(2)) != std::future_status::ready){
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
        return false;
    }

    auto response = future.get();

    if(!response->response){ // if response == false
        RCLCPP_ERROR(node_->get_logger(), "Did not detect weeds");
        return false;
    }

    // defining weed_pose_ attribute
    weed_pose_.header.frame_id = "base_link";

    weed_pose_.pose.position.x = response->weed_x;
    weed_pose_.pose.position.y = response->weed_y;
    weed_pose_.pose.position.z = response->weed_z;

    weed_pose_.pose.orientation.x = 0.707;
    weed_pose_.pose.orientation.w = 0.707;

    // creating Asparagus objects
    auto asparagus_flat = response->asparagus;
    
    // filling asparagus_ vector
    for (size_t i = 0; i < asparagus_flat.size(); i += 5){

        geometry_msgs::msg::PoseStamped asparagus_pose;

        asparagus_pose.pose.position.x = asparagus_flat[i];
        asparagus_pose.pose.position.y = asparagus_flat[i+1];
        asparagus_pose.pose.position.z = asparagus_flat[i+2];

        auto asparagus_height = asparagus_flat[i+3];
        auto asparagus_radius = asparagus_flat[i+4];

        asparagus_.push_back(Asparagus(asparagus_pose, asparagus_height, asparagus_radius));
    }

    return true;
      
    }

    void MTCTaskNode::setupPlanningScene()
    {
      // defining collision planes

      // ground plane collision object
      moveit_msgs::msg::CollisionObject ground_plane;
      ground_plane.id = node_->get_parameter("collision_planes.ground_plane.id").as_string();
      ground_plane.header.frame_id = node_->get_parameter("collision_planes.ground_plane.frame_id").as_string();
      ground_plane.primitives.resize(1);
      ground_plane.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      ground_plane.primitives[0].dimensions = {
        node_->get_parameter("collision_planes.ground_plane.dimensions.x").as_double(),
        node_->get_parameter("collision_planes.ground_plane.dimensions.y").as_double(),
        node_->get_parameter("collision_planes.ground_plane.dimensions.z").as_double()
      };

      // ground_plane_pose_ attribute
      ground_plane_pose_.header.frame_id = ground_plane.header.frame_id;

      ground_plane_pose_.pose.position.x = node_->get_parameter("collision_planes.ground_plane.pose.position.x").as_double();
      ground_plane_pose_.pose.position.y = node_->get_parameter("collision_planes.ground_plane.pose.position.y").as_double();
      ground_plane_pose_.pose.position.z = node_->get_parameter("collision_planes.ground_plane.pose.position.z").as_double();

      ground_plane_pose_.pose.orientation.x = node_->get_parameter("collision_planes.ground_plane.pose.orientation.x").as_double();
      ground_plane_pose_.pose.orientation.y = node_->get_parameter("collision_planes.ground_plane.pose.orientation.y").as_double();
      ground_plane_pose_.pose.orientation.z = node_->get_parameter("collision_planes.ground_plane.pose.orientation.z").as_double();
      ground_plane_pose_.pose.orientation.w = node_->get_parameter("collision_planes.ground_plane.pose.orientation.w").as_double();

      ground_plane.pose = ground_plane_pose_.pose;

      /********************************************************************************/

      // farmbeast plane collision object
      moveit_msgs::msg::CollisionObject farmbeast_plane;
      farmbeast_plane.id = node_->get_parameter("collision_planes.farmbeast_plane.id").as_string();
      farmbeast_plane.header.frame_id = node_->get_parameter("collision_planes.farmbeast_plane.frame_id").as_string();
      farmbeast_plane.primitives.resize(1);
      farmbeast_plane.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      farmbeast_plane.primitives[0].dimensions = {
        node_->get_parameter("collision_planes.farmbeast_plane.dimensions.x").as_double(),
        node_->get_parameter("collision_planes.farmbeast_plane.dimensions.y").as_double(),
        node_->get_parameter("collision_planes.farmbeast_plane.dimensions.z").as_double(),
      };

      // farmbeast_plane_pose_ attribute
      farmbeast_plane_pose_.header.frame_id = farmbeast_plane.header.frame_id;

      farmbeast_plane_pose_.pose.position.x = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.x").as_double();
      farmbeast_plane_pose_.pose.position.y = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.y").as_double();
      farmbeast_plane_pose_.pose.position.z = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.z").as_double();

      farmbeast_plane_pose_.pose.orientation.x = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.x").as_double();
      farmbeast_plane_pose_.pose.orientation.y = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.y").as_double();
      farmbeast_plane_pose_.pose.orientation.z = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.z").as_double();
      farmbeast_plane_pose_.pose.orientation.w = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.w").as_double();


      // defining weed object
      moveit_msgs::msg::CollisionObject weed_object;
      weed_object.id = "weed_object";
      weed_object.header.frame_id = "base_link";
      weed_object.primitives.resize(1);
      weed_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;

      // defining fallback poses, meant for testing
      if(!getSceneFromService()){ // if getSceneFromService returns false
        RCLCPP_ERROR(node_->get_logger(), "Call to service failed...using fallback poses!");

        // weed_pose_
        weed_object.primitives[0].dimensions = { 0.005, 0.02 };

        weed_pose_.header.frame_id = "base_link";
        weed_pose_.pose.position.x = -0.2;
        weed_pose_.pose.position.y = 0.19;
        weed_pose_.pose.position.z = 0.20;

        weed_pose_.pose.orientation.x = 0.707;
        weed_pose_.pose.orientation.w = 0.707;

        weed_object.pose = weed_pose_.pose;

        // random asparagus poses (max 3 for testing)
        moveit_msgs::msg::CollisionObject asparagus_objects; // creates one object
        asparagus_objects.id = "asparagus_objects";
        asparagus_objects.header.frame_id = "base_link";
        asparagus_objects.primitives.resize(3); // three asparagus
        asparagus_objects.primitive_poses.resize(3);

        float height = 0.20;
        float radius = 0.005;

        geometry_msgs::msg::PoseStamped pose_1;
        geometry_msgs::msg::PoseStamped pose_2;
        geometry_msgs::msg::PoseStamped pose_3;

        float orientation_x = 0.707;
        float orientation_w = 0.707;

        // first asparagus
        pose_1.header.frame_id = "base_link";
        pose_1.pose.position.x = -0.5;
        pose_1.pose.position.y = height / 2;
        pose_1.pose.position.z = 0.1;
        pose_1.pose.orientation.x = orientation_x;
        pose_1.pose.orientation.w = orientation_w;

        // second asparagus
        pose_2.header.frame_id = "base_link";
        pose_2.pose.position.x = -0.4;
        pose_2.pose.position.y = height / 2;
        pose_2.pose.position.z = 0.3;
        pose_2.pose.orientation.x = orientation_x;
        pose_2.pose.orientation.w = orientation_w; 
        
        // third asparagus
        pose_3.header.frame_id = "base_link";
        pose_3.pose.position.x = -0.3;
        pose_3.pose.position.y = height / 2;
        pose_3.pose.position.z = 0.3;
        pose_3.pose.orientation.x = orientation_x;
        pose_3.pose.orientation.w = orientation_w;

        asparagus_.push_back(Asparagus(pose_1, height, radius));
        asparagus_.push_back(Asparagus(pose_2, height, radius));
        asparagus_.push_back(Asparagus(pose_3, height, radius));

        
        // defining asparagus objects
        for(int i=0; i < asparagus_.size(); i++){
          asparagus_objects.primitives[i].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          asparagus_objects.primitives[i].dimensions = {asparagus_[i].getHeight(),asparagus_[i].getRadius()};
          asparagus_objects.primitive_poses[i] = asparagus_[i].getPose().pose; // single object
        }

        // color weed
        moveit_msgs::msg::ObjectColor color;
        color.id = "weed_object";
        color.color.r = 1.0;
        color.color.g = 0.0;
        color.color.b = 0.0;
        color.color.a = 1.0;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(ground_plane);
        psi.applyCollisionObject(farmbeast_plane);
        psi.applyCollisionObject(weed_object);
        psi.applyCollisionObject(asparagus_objects);

        return;

      }
      else{ // if getSceneService returns true

        // weed_object
        weed_object.primitives[0].dimensions = { 0.005, 0.02 };
        weed_object.pose = weed_pose_.pose;

        // asparagus_objects
        moveit_msgs::msg::CollisionObject asparagus_objects; // creates one object
        asparagus_objects.id = "asparagus_objects";
        asparagus_objects.header.frame_id = "base_link";
        asparagus_objects.primitives.resize(asparagus_.size()); // n asparagus
        asparagus_objects.primitive_poses.resize(asparagus_.size());

        // defining asparagus objects
        for(int i=0; i < asparagus_.size(); i++){
          asparagus_objects.primitives[i].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          asparagus_objects.primitives[i].dimensions = {asparagus_[i].getHeight(),asparagus_[i].getRadius()};
          auto pose_temp = asparagus_[i].getPose();
          pose_temp.pose.orientation.x = 0.707;
          pose_temp.pose.orientation.w = 0.707;
          asparagus_objects.primitive_poses[i] = pose_temp.pose; // single object
        }

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(ground_plane);
        psi.applyCollisionObject(farmbeast_plane);
        psi.applyCollisionObject(weed_object);
        psi.applyCollisionObject(asparagus_objects);

        return;
      }

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

      // Set task properties
      const auto& arm_group_name = "me6_group";
      const auto& hand_group_name = "gripper";
      const auto& hand_frame = "dummy_tcp";

      task.setProperty("group", arm_group_name); // manipulator group
      task.setProperty("eef", hand_group_name); // gripper group
      task.setProperty("ik_frame", hand_frame); // TCP of gripper

      // initialize planners
      auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);
      cartesian_planner->setStepSize(.03); // desired step size in m => change if cartesian planner does not want to compute solutions

      mtc::Stage* current_state_ptr = nullptr;

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
              stage->setMinMaxDistance(0.01, 0.3); // min max distance

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
              stage->setObject("weed_object");
              stage->setAngleDelta(M_PI / 12);
              stage->setMonitoredStage(current_state_ptr);

              // extrinsically rotate around x for 180
              // Extrinsic ZYX rotation <-> Intrinsic XYZ
              Eigen::Isometry3d grasp_frame_transform;
              Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
              grasp_frame_transform.linear() = q.matrix();
              grasp_frame_transform.translation().z() = 0.00; // changed 0.05 before, and 0.0

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
              *       Allow Collision hand-weed       * To res lahka gor fuknes, assuma da se v approach stage-u ne pride v kolizijo
              ****************************************/ 

              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, weed)");
                  stage->allowCollisions("weed_object",
                                      task.getRobotModel()
                                          //->getJointModelGroup(hand_group_name) // disabled since it reports collision (it should not)
                                          ->getLinkModelNamesWithCollisionGeometry(),
                                      true);
              stage_pull_weed->insert(std::move(stage));

          }

          {
            /****************************************
            *   Allow Collision gripper-ground      *
            ****************************************/
            auto stage = 
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper, ground)");
                stage->allowCollisions(node_->get_parameter("collision_planes.ground_plane.id").as_string(),
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
              stage->attachObject("weed_object", hand_frame);
              //attach_weed_ptr = stage.get(); // redundant
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

      auto stage_dump = std::make_unique<mtc::stages::MoveTo>("move to dump", sampling_planner);
      stage_dump->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage_dump->setGoal("drop");
      task.add(std::move(stage_dump)); 

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
              stage->allowCollisions("weed_object",
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
              stage->detachObject("weed_object", hand_frame);
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

} // avoid_remove_weed




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<avoid_remove_weed::MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // call mtc_task_node related functions here, useful for testing
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}