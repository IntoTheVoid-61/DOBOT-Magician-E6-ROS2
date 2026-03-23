/*
@brief This ROS 2 node implements a move_to_home action server. It orchestrates the movement of the robotic arm into 
predefined, hard coded home position.

@author: Ziga Breznikar
@date: 21.03.2026


*/

#include <memory>
#include <thread>
#include <cmath>

#include "dobot_msgs_fb/action/move_to_home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>

using MoveToHome = dobot_msgs_fb::action::MoveToHome;
using GoalHandleMoveToHome = rclcpp_action::ServerGoalHandle<MoveToHome>;

namespace dobot_system_tests
{
class MoveToHomeActionServer : public rclcpp::Node
{
public:
    // constructor
    MoveToHomeActionServer() : Node("move_to_home_action_server_cpp")
    {
        using namespace std::placeholders;

        // initialize action server
        action_server_ = rclcpp_action::create_server<MoveToHome>(
            this,
            "move_to_home", // action name
            std::bind(&MoveToHomeActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToHomeActionServer::handle_cancel, this, _1),
            std::bind(&MoveToHomeActionServer::handle_accepted, this, _1)
        );

        // create_wall_timer(delay,callback). Defining callback as a lambda function. TLDR, wait 1s than initialize move_group_interface_
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){
                move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "me6_group"
                );
                timer_->cancel();
            }
        );

        // declare parameters
        this->declare_parameter<double>("velocity_scaling");
        this->declare_parameter<double>("acceleration_scaling");
        this->declare_parameter<std::vector<double>>("home_pose");
        this->declare_parameter<std::string>("end_effector_link");

        // initialize default parameters from yaml file
        velocity_scaling_param = this->get_parameter("velocity_scaling").as_double();
        acceleration_scaling_param = this->get_parameter("acceleration_scaling").as_double();
        pose_array_param = this->get_parameter("home_pose").as_double_array();
        end_effector_link_param = this->get_parameter("end_effector_link").as_string();

    } // constructor

private:
    // class member variables
    rclcpp_action::Server<MoveToHome>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
    double velocity_scaling_param;
    double acceleration_scaling_param;
    std::vector<double> pose_array_param; // pose should contain 7 elements (3 for point and 4 for orientation)
    std::string end_effector_link_param;

    /*
    @brief Callback function for handling new goal requests

    Function is called when a new goal is received from an action client.
    Function logs the information about receiving a goal and accepts-executes

    @param uuid Unique identifier for goal request.
    @param goal The goal request message containing velocity scaling and acceleration scaling.
    */

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToHome::Goal> goal
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for move to home action!");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    /*
    @brief Callback function for handling cancel requests.

    Function is called when a client request the cancellation of the ongoing goal.

    @param goal_handle The goal handle associated with the goal to be canceled.
    
    */

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToHome> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal!");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    /*
    @brief Callback function for handling accepted goals.

    Function is called when a goal has been accepted and needs to be executed.
    Spawns a new thread to execute the goal, avoiding blocking the main thread.

    @param goal_handle The goal handle associated with the accepted goal.

    */

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToHome> goal_handle)
    {
        // create a new thread with proper bind syntax
        // This needs to be returned quickly to avoid blocking the executor, spin a new thread
        // void lambda function
        std::thread{
            [this, goal_handle](){
                this->execute(goal_handle);
            }
        }.detach();
    };

    /*
    @brief Main executor function for move_to_home action

    Function runs in a separate thread and performs the following:
    1. Clears potentially cached targets
    2. Sets max velocity scalling factor (there are default values provided)
    3. Sets max acceleration scalling factor (there are default values provided)
    4. Sets and executes the predefined home pose.

    Through out the action execution provides feedback, finally provides status of goal
    completion.
    
    */

    void execute(const std::shared_ptr<GoalHandleMoveToHome> goal_handle)
    {
        // remove this, its redundant
        //while (!move_group_interface_) {
        //RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface...");
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //}

        // log start
        RCLCPP_INFO(this->get_logger(), "Executing goal...moving to home position!");

        // get goal msg
        const auto goal = goal_handle->get_goal();

        // create results and feedback msg
        auto result = std::make_shared<MoveToHome::Result>();
        auto feedback = std::make_shared<MoveToHome::Feedback>();

        // send initial feedback
        feedback->status = "GOAL_RECEIVED";
        goal_handle->publish_feedback(feedback);

        // clear cached pose
        move_group_interface_->clearPoseTargets();

        // get goal values
        double velocity_scaling_goal = goal->velocity_scaling;
        double acceleration_scaling_goal = goal->acceleration_scaling;
        geometry_msgs::msg::Pose home_pose_goal = goal->home_pose;
        std::string end_effector_link_goal = goal->end_effector_link;

        // Use goal if valid, otherwise fallback to parameters
        double velocity_scaling =
        (velocity_scaling_goal > 0.0) ? velocity_scaling_goal : velocity_scaling_param;
        double acceleration_scaling = 
        (acceleration_scaling_goal > 0.0) ? acceleration_scaling_goal : acceleration_scaling_param;
        std::string end_effector_link =
            end_effector_link_goal.empty() ? end_effector_link_param : end_effector_link_goal;
        
        // if home_pose_goal empty construct pose from array
        geometry_msgs::msg::Pose home_pose;
        if (home_pose_goal.position.x == 0 && home_pose_goal.position.y == 0 && home_pose_goal.position.z ==0){
            home_pose.position.x = pose_array_param[0];
            home_pose.position.y = pose_array_param[1];
            home_pose.position.z = pose_array_param[2];
            home_pose.orientation.x = pose_array_param[3];
            home_pose.orientation.y = pose_array_param[4];
            home_pose.orientation.z = pose_array_param[5];
            home_pose.orientation.w = pose_array_param[6];
        }
        else{
            home_pose = home_pose_goal;
        }

        // velocity and acceleration scaling
        move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling);
        move_group_interface_->setMaxAccelerationScalingFactor(acceleration_scaling);

        // explicit declaration of end-effector link
        move_group_interface_->setEndEffectorLink(end_effector_link);

        // set goal pose
        move_group_interface_->setPoseTarget(home_pose, end_effector_link);

        // evaluating
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // get current time
        auto start_time = this->now();

        // execute if evaluation returns true
        if(success){
            move_group_interface_->asyncExecute(plan);
        }
        else{
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            return;
        }

        // main loop
        while(rclcpp::ok()){
            // check for cancellation
            if (goal_handle->is_canceling()){
                move_group_interface_->stop();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                feedback->status = "GOAL_CANCELED";
                goal_handle->publish_feedback(feedback);
                return;
            }

            // calculate elapsed time
            auto elapsed_time = (this->now() - start_time).seconds();
            feedback->elapsed_time = elapsed_time;
            feedback->status = "EXECUTING MOVEMENT...";
            goal_handle->publish_feedback(feedback);

            // check completion
            auto current_pose = move_group_interface_->getCurrentPose().pose;
            // calculate differentials of position
            double dx = current_pose.position.x - goal->home_pose.position.x;
            double dy = current_pose.position.y - goal->home_pose.position.y;
            double dz = current_pose.position.z - goal->home_pose.position.z;
            // calculate differentials of orientation (quarts)
            double dx_orient = current_pose.orientation.x - goal->home_pose.orientation.x;
            double dy_orient = current_pose.orientation.y - goal->home_pose.orientation.y;
            double dz_orient = current_pose.orientation.z - goal->home_pose.orientation.z;
            double dw_orient = current_pose.orientation.w - goal->home_pose.orientation.w;

            double positional_error = sqrt(dx*dx + dy*dy + dz*dz + dx_orient*dx_orient + dy_orient*dy_orient + dz_orient*dz_orient + dw_orient*dw_orient);

            if (positional_error < 0.01){
                break;
            }

            // sleep ofr 500ms 
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } // main loop

    // set results and send
    result->success = true;
    goal_handle->succeed(result);
    feedback->status = "GOAL_SUCCEEDED";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");

    } // execute

}; // MoveToHomeActionServer
} // namespace dobot_system_tests


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<dobot_system_tests::MoveToHomeActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}