/*
@brief 

TODO: add comments about method functionalities

@author Ziga Breznikar
@date 19.08.2026
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
#include <map>

#include "asparagus.hpp"
#include "dobot_msgs_fb/srv/remove_weeds.hpp"

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

namespace mtc = moveit::task_constructor;

namespace pick_up_weeds
{
    class MTCTaskNode
    {
    public:
        MTCTaskNode(const rclcpp::NodeOptions& options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        bool doTask();
        bool getSceneFromService(); // call once
        bool getSceneFromYaml();
    private:
        bool setupPlanningScene();
        mtc::Task createTask();
        rclcpp::Client<dobot_msgs_fb::srv::RemoveWeeds>::SharedPtr service_client_;
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        std::map<std::string, Asparagus> asparagus_;
        std::map<std::string, geometry_msgs::msg::PoseStamped> weeds_;
        std::string selected_weed_id; // current weed
        unsigned short max_attempts_;
    }; // MTCTaskNode

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("pick_up_weeds_mtc", options)}
    {
        service_client_ =
            node_->create_client<dobot_msgs_fb::srv::RemoveWeeds>("/remove_weeds");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

    bool MTCTaskNode::getSceneFromYaml()
    {

        // defining weeds_
        auto weeds_flat = node_->get_parameter("fake_scene.weeds_flat").as_double_array();
        unsigned short id = 0;

        // check if weeds_ is initially empty
        if(!weeds_.empty()){
            RCLCPP_ERROR(node_->get_logger(), "weeds_ not initially empty");
            return false;
        }

        for(size_t i = 0; i < weeds_flat.size(); i+=3){
            std::string weed_id = "weed_" + std::to_string(id);

            geometry_msgs::msg::PoseStamped weed_pose;

            weed_pose.pose.position.x = weeds_flat[i];
            weed_pose.pose.position.y = weeds_flat[i+1];
            weed_pose.pose.position.z = weeds_flat[i+2];

            weed_pose.pose.orientation.x = 0.707;
            weed_pose.pose.orientation.w = 0.707;

            weeds_.emplace(
                weed_id,
                weed_pose
            );

            //weeds_[weed_id] = weed_pose;

            id++;
        }

        // debugger
        for(auto weed : weeds_){
            RCLCPP_INFO(node_->get_logger(), "%s", weed.first.c_str());
        }

        // defining asparagus objects (poses and geometry)
        auto asparagus_flat = node_->get_parameter("fake_scene.asparagus_flat").as_double_array();
        id = 0;

        for(size_t i = 0; i < asparagus_flat.size(); i+=3){

            std::string asparagus_id = "asparagus_" + std::to_string(id);

            geometry_msgs::msg::PoseStamped asparagus_pose;

            asparagus_pose.pose.position.x = asparagus_flat[i];
            asparagus_pose.pose.position.y = asparagus_flat[i+1] - 0.1; // height/2
            asparagus_pose.pose.position.z = asparagus_flat[i+2];

            asparagus_pose.pose.orientation.x = 0.707;
            asparagus_pose.pose.orientation.w = 0.707;

            float asparagus_height = 0.15;
            float asparagus_radius = 0.005;

            asparagus_.emplace(
                asparagus_id,
                Asparagus(asparagus_pose, asparagus_height, asparagus_radius)
            );

            id++;

        }

        return true;
  
    }

    /*
    @brief Function fills up asparagus_ and weeds_ with relevant data
    */

    bool MTCTaskNode::getSceneFromService()
    {
        // check if service available
        if(!service_client_->wait_for_service(std::chrono::seconds(5))){
            RCLCPP_ERROR(node_->get_logger(), "Service not available, timed out...");
            return false;
        }

        auto request = std::make_shared<dobot_msgs_fb::srv::RemoveWeeds::Request>(); // create empty service request
        auto future = service_client_->async_send_request(request); // send request

        if(future.wait_for(std::chrono::seconds(2)) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Service not responding when called");
            return false;
        }

        auto response = future.get();

        if(!response->response){ // if response == false => did not detect weeds
            RCLCPP_ERROR(node_->get_logger(), "Did not detect weeds");
            return false;
        }

        // defining weeds_
        auto weeds_flat = response->weeds;
        unsigned short id = 0;

        // check if weeds_ is initially empty
        if(!weeds_.empty()){
            RCLCPP_ERROR(node_->get_logger(), "weeds_ not initially empty");
            return false;
        }

        for(size_t i = 0; i < weeds_flat.size(); i+=3){
            std::string weed_id = "weed_" + std::to_string(id);

            geometry_msgs::msg::PoseStamped weed_pose;

            weed_pose.pose.position.x = weeds_flat[i];
            weed_pose.pose.position.y = weeds_flat[i+1];
            weed_pose.pose.position.z = weeds_flat[i+2];

            weed_pose.pose.orientation.x = 0.707;
            weed_pose.pose.orientation.w = 0.707;

            weeds_.emplace(
                weed_id,
                weed_pose
            );

            //weeds_[weed_id] = weed_pose;

            id++;
        }

        // debugger
        for(auto weed : weeds_){
            RCLCPP_INFO(node_->get_logger(), "%s", weed.first.c_str());
        }

        // defining asparagus objects (poses and geometry)
        auto asparagus_flat = response->asparagus;
        id = 0;

        for(size_t i = 0; i < asparagus_flat.size(); i+=5){

            std::string asparagus_id = "asparagus_" + std::to_string(id);

            geometry_msgs::msg::PoseStamped asparagus_pose;

            asparagus_pose.pose.position.x = asparagus_flat[i];
            asparagus_pose.pose.position.y = asparagus_flat[i+1] - 0.1; // height/2
            asparagus_pose.pose.position.z = asparagus_flat[i+2];

            asparagus_pose.pose.orientation.x = 0.707;
            asparagus_pose.pose.orientation.w = 0.707;

            auto asparagus_height = asparagus_flat[i+3];
            auto asparagus_radius = asparagus_flat[i+4];

            //asparagus_[asparagus_id] = Asparagus(asparagus_pose, asparagus_height, asparagus_radius);

            asparagus_.emplace(
                asparagus_id,
                Asparagus(asparagus_pose, asparagus_height, asparagus_radius)
            );

            id++;

        }

        return true;

    }

    bool MTCTaskNode::setupPlanningScene()
    {
        try
        {
            moveit::planning_interface::PlanningSceneInterface psi;

            auto object_ids = psi.getKnownObjectNames();
            psi.removeCollisionObjects(object_ids); // fix?

            /*****************************************Collision Planes*******************************************/

            // ground plane
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

            ground_plane.pose.position.x = node_->get_parameter("collision_planes.ground_plane.pose.position.x").as_double();
            ground_plane.pose.position.y = node_->get_parameter("collision_planes.ground_plane.pose.position.y").as_double();
            ground_plane.pose.position.z = node_->get_parameter("collision_planes.ground_plane.pose.position.z").as_double();

            ground_plane.pose.orientation.x = node_->get_parameter("collision_planes.ground_plane.pose.orientation.x").as_double();
            ground_plane.pose.orientation.y = node_->get_parameter("collision_planes.ground_plane.pose.orientation.y").as_double();
            ground_plane.pose.orientation.z = node_->get_parameter("collision_planes.ground_plane.pose.orientation.z").as_double();
            ground_plane.pose.orientation.w = node_->get_parameter("collision_planes.ground_plane.pose.orientation.w").as_double();

            psi.applyCollisionObject(ground_plane);

            // farmbeast plane
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

            farmbeast_plane.pose.position.x = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.x").as_double();
            farmbeast_plane.pose.position.y = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.y").as_double();
            farmbeast_plane.pose.position.z = node_->get_parameter("collision_planes.farmbeast_plane.pose.position.z").as_double();

            farmbeast_plane.pose.orientation.x = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.x").as_double();
            farmbeast_plane.pose.orientation.y = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.y").as_double();
            farmbeast_plane.pose.orientation.z = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.z").as_double();
            farmbeast_plane.pose.orientation.w = node_->get_parameter("collision_planes.farmbeast_plane.pose.orientation.w").as_double();

            psi.applyCollisionObject(farmbeast_plane);
            /*****************************************Collision Planes*******************************************/

            // weed objects
            std::vector<moveit_msgs::msg::CollisionObject> weed_objects;
            weed_objects.resize(weeds_.size()); // resize

            float height = 0.005;
            float width = 0.002;

            unsigned short id = 0;

            for(auto weed : weeds_){

                weed_objects[id].id = weed.first;
                weed_objects[id].header.frame_id = "base_link";
                weed_objects[id].primitives.resize(1);
                weed_objects[id].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                weed_objects[id].primitives[0].dimensions = {height,width};
                weed_objects[id].pose = weed.second.pose;

                psi.applyCollisionObject(weed_objects[id]);

                id++;

            }

            // asparagus objects

            std::vector<moveit_msgs::msg::CollisionObject> asparagus_objects;
            asparagus_objects.resize(asparagus_.size()); // resize

            id = 0;

            for(auto asparagus : asparagus_){

                asparagus_objects[id].id = asparagus.first;
                asparagus_objects[id].header.frame_id = "base_link";
                asparagus_objects[id].primitives.resize(1);
                asparagus_objects[id].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                asparagus_objects[id].primitives[0].dimensions = {asparagus.second.getHeight(),asparagus.second.getRadius()};
                asparagus_objects[id].pose = asparagus.second.getPose().pose;

                psi.applyCollisionObject(asparagus_objects[id]);

                id++;

            }


        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Exception when setting up scene: %s",
                e.what()
            );
        }

        return true;

    }

    mtc::Task MTCTaskNode::createTask()
    {

        std::string task_name = selected_weed_id + " removal task";

        mtc::Task task;
        task.stages()->setName(task_name);
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
        cartesian_planner->setMaxVelocityScalingFactor(0.3);
        cartesian_planner->setMaxAccelerationScalingFactor(0.2);
        cartesian_planner->setStepSize(.03); // desired step size in m => change if cartesian planner does not want to compute solutions

        mtc::Stage* current_state_ptr = nullptr;

        /****************************************************
        *                                                   *
        *                   Current State                   *
        *                                                   *
        ****************************************************/

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current_state");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        /****************************************************
        *                                                   *
        *                Move to Weed                       *
        *                                                   *
        ****************************************************/

        auto stage_move_to_weed = std::make_unique<mtc::stages::Connect>(
            "move_to_weed",
            mtc::stages::Connect::GroupPlannerVector{ {arm_group_name, sampling_planner} });
        stage_move_to_weed->setTimeout(30.0);
        stage_move_to_weed->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_weed));

        /****************************************************
        *                                                   *
        *         Pull Weed Serial Container                *
        *                                                   *
        ****************************************************/

        {

            auto stage_pull_weed = std::make_unique<mtc::SerialContainer>("pull_weed_container");
            task.properties().exposeTo(stage_pull_weed->properties(), { "eef", "group", "ik_frame" }); // declare properties from parent task
            stage_pull_weed->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" }); // initialize properties from parent task

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
                *        Allow collision hand-weed      *
                ****************************************/

                auto stage =
                    std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, weed)");
                stage->allowCollisions(
                    selected_weed_id,
                    task.getRobotModel()
                        ->getJointModelGroup(hand_group_name)
                        ->getLinkModelNamesWithCollisionGeometry(),
                    true);
                
                stage_pull_weed->insert(std::move(stage));

            }
            /*
            {
                // gripper ground

                auto stage = 
                    std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper, ground)");
                    stage->allowCollisions(node_->get_parameter("collision_planes.ground_plane.id").as_string(),
                                                            task.getRobotModel()
                                                                //->getJointModelGroup(hand_group_name) // disabled since it reports collision (it should not)
                                                                ->getLinkModelNamesWithCollisionGeometry(),
                                                            true);
                stage_pull_weed->insert(std::move(stage));


            }
            */
            {
                /****************************************
                *          Generate grasp pose          *
                ****************************************/          
                
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate removal pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject(selected_weed_id);
                stage->setAngleDelta(M_PI / 12);
                stage->setMonitoredStage(current_state_ptr);

                Eigen::Isometry3d grasp_frame_transform;
                Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
                grasp_frame_transform.linear() = q.matrix();
                grasp_frame_transform.translation().z() = -0.010; //-0.01

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
                stage->attachObject(selected_weed_id, hand_frame);
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



            {
                /****************************************
                *   Disallow Collision gripper-ground      *
                ****************************************/

                auto stage = 
                    std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper, ground)");
                    stage->allowCollisions(node_->get_parameter("collision_planes.ground_plane.id").as_string(),
                                                            task.getRobotModel()
                                                                //->getJointModelGroup(hand_group_name) // disabled since it reports collision (it should not)
                                                                ->getLinkModelNamesWithCollisionGeometry(),
                                                            false);
                stage_pull_weed->insert(std::move(stage));


            }

            task.add(std::move(stage_pull_weed));

        }

        /****************************************************
        *                                                   *
        *                 Move To Dump                      *
        *                                                   *
        ****************************************************/ 

        auto stage_dump = std::make_unique<mtc::stages::MoveTo>("move_to_dump",sampling_planner);
        stage_dump->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage_dump->setGoal("drop");
        task.add(std::move(stage_dump)); 

        /****************************************************
        *                                                   *
        *            Drop weed Serial Container             * 
        *                                                   *
        ****************************************************/

        {
            auto stage_drop_weed = std::make_unique<mtc::SerialContainer>("dump_weed_serial_container");
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
                stage->allowCollisions(selected_weed_id,
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
                stage->detachObject(selected_weed_id, hand_frame);
                stage_drop_weed->insert(std::move(stage));

            }

            {
                /****************************************
                *           Remove object               *
                ****************************************/ 

                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
                stage->removeObject(selected_weed_id);
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
        *                                                   *
        *                  Move to home                     *
        *                                                   *
        ****************************************************/
        

        auto stage_move_to_home =
            std::make_unique<mtc::stages::MoveTo>("move_to_home", interpolation_planner);
        stage_move_to_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage_move_to_home->setGoal("home");
        task.add(std::move(stage_move_to_home));  
        

        return task;

    }

    bool MTCTaskNode::doTask()
    {

        //unsigned short attempts = 0;
        max_attempts_ = weeds_.size() * 2;

        while(!weeds_.empty()){

            bool made_progress = false;

            for(auto it = weeds_.begin(); it != weeds_.end();){ // main loop

                const auto& weed_id = it->first;
                selected_weed_id = weed_id;

                RCLCPP_INFO(
                    node_->get_logger(),
                        "planning for %s",
                        weed_id.c_str());

                if(!setupPlanningScene()){
                    RCLCPP_ERROR(node_->get_logger(), "Could not setup PlanningScene");
                    return false;
                }

                task_ = createTask(); // create a task for selected_weed_id

                try
                {
                    task_.init();
                    RCLCPP_INFO(node_->get_logger(), "Starting removal task!");
                }
                catch(mtc::InitStageException& e)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
                    return false;
                }

                // if planning successful
                if(task_.plan(5)){

                    task_.introspection().publishSolution(*task_.solutions().front());
                    auto results = task_.execute(*task_.solutions().front()); // blocking function

                    if(results.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS){

                        RCLCPP_INFO(node_->get_logger(),
                                        "successfully removed %s",
                                        weed_id.c_str());


                        it = weeds_.erase(it);

                        made_progress = true;

                        RCLCPP_INFO(node_->get_logger(),
                                        "weeds_remaining: %zu",
                                        weeds_.size());

                        continue;

                    }
                    else{
                        RCLCPP_WARN(node_->get_logger(),
                                    "Execution failed for %s",
                                    weed_id.c_str());
                    }
                }
                else{
                    RCLCPP_WARN(node_->get_logger(),
                    "Could not plan solution for: %s",
                    weed_id.c_str());
                }

                // planning execution failed, leave the weed in the map and move on
                it++;
                //attempts++;

            }

            if(!made_progress){
                RCLCPP_WARN(node_->get_logger(),
                            "Could not remove all weeds, reposition");
                return false;
            }

        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Removed all weeds"
        );

        return true;
        
    }


} // pick_up_weeds

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<pick_up_weeds::MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // call mtc_task_node related functions here
  //mtc_task_node->doTaskOnce();
  mtc_task_node->getSceneFromService(); // get asparagus and weeds 
  //mtc_task_node->getSceneFromYaml();
  mtc_task_node->doTask(); 

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}