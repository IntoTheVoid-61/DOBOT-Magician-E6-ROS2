/*
@brief This ROS 2 node implements a move_to_pose action server. It orchestrates the movement of the robotic arm into a specified pose.

This action serves as a test program which can be later generalized and implemented as the main movement segment into the entire framework.
Server is implemented based on dobot_msgs_fb/action/MoveToPose.action

@ author: Ziga Breznikar
@ date: 17.03.2026

*/

#include <memory>
#include <thread>
#include <cmath>

#include "dobot_msgs_fb/action/move_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;
using MoveToPose = dobot_msgs_fb::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;


namespace dobot_system_tests
{
class MoveToPoseActionServer : public rclcpp::Node
{
public:
    // constructor
    MoveToPoseActionServer() : Node("move_to_pose_action_server_cpp")
    {
        using namespace std::placeholders;

        // Initialize action_server
        action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, _1));
        
        // Initialize move_group_interface with 1s delay
        timer_ = this->create_wall_timer(
            1s,
            [this](){
                move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "me6_group"
                );
                timer_->cancel();
            }
        );

        
    }

private:
    // class member variables
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    @brief Callback function for handling new goal requests

    Function is called when a new goal is received from an action client.

    @param uuid Unique identifier for the goal request.
    @param goal The goal request message containing velocity_scaling, acceleration_scaling, target_pose.
    */

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToPose::Goal> goal
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for move to pose action!");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    /*
    @brief Callback function for handling cancel requests.

    Function is called when a client request the cancellation of the ongoing goal.
    
    @param goal_handle The goal handle associated with the goal to be canceled.
    */

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle
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

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        // create a new thread with proper bind syntax
        std::thread{
            [this, goal_handle]() {
                this->execute(goal_handle);
            }
        }.detach();
    };

    /*
    @brief Executes the move_to_pose action

    Function runs in a separate thread and performs the following:
    1. Clears potentially cached targets 
    2. Sets max velocity scaling factor
    3. Sets max acceleration scaling factor
    4. Sets and executes the desired Pose

    */

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle){

        RCLCPP_INFO(this->get_logger(), "Executing goal...moving to specified pose!");

        // get goal msg
        const auto goal = goal_handle->get_goal();

        // create results and feedback msg
        auto result = std::make_shared<MoveToPose::Result>();
        auto feedback = std::make_shared<MoveToPose::Feedback>();

        // send initial feedback
        feedback->status = "GOAL_RECEIVED";
        goal_handle->publish_feedback(feedback);

        // clear cached pose
        move_group_interface_->clearPoseTargets();

        // velocity and acceleration scaling
        move_group_interface_->setMaxVelocityScalingFactor(goal->velocity_scaling);
        move_group_interface_->setMaxAccelerationScalingFactor(goal->acceleration_scaling);

        // explicit declaration of end-effector link and setting to specified pose
        move_group_interface_->setEndEffectorLink("Link6");
        move_group_interface_->setPoseTarget(goal->target_pose, "Link6");

        // Evaluating 
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // get current time
        auto start_time = this->now();

        // Execute if evaluation returns true
        if (success){
            // blocks until motion is finished, we need a separate thread!!!!!!
            move_group_interface_->asyncExecute(plan);
        }
        else{
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            return;
        }

        // main execution loop
        while(rclcpp::ok()){
            // check for cancellation
            if (goal_handle->is_canceling()){

                move_group_interface_->stop(); // stop movement

                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                feedback->status = "GOAL_CANCELED";
                goal_handle->publish_feedback(feedback);

                return;

            }

            // calculate and publish elapsed time
            auto elapsed_time = (this->now() - start_time).seconds();
            feedback->elapsed_time = elapsed_time;
            feedback->status = "EXECUTING MOVEMENT...";
            goal_handle->publish_feedback(feedback);

            // Check completion
            auto current_pose = move_group_interface_->getCurrentPose().pose;
            // calculate differentials of positions (could be too minimal since no orientation)
            double dx = current_pose.position.x - goal->target_pose.position.x;
            double dy = current_pose.position.y - goal->target_pose.position.y;
            double dz = current_pose.position.z - goal->target_pose.position.z;

            double positional_error = sqrt(dx*dx + dy*dy + dz*dz);

            // Break if tolerance reached
            if (positional_error < 0.01){
                break;
            }


            std::this_thread::sleep_for(500ms);


        }

        // Set and send results
        result->success = true;

        // Mark the goal as succeeded
        goal_handle->succeed(result);

        // Send final feedback
        feedback->status = "GOAL_SUCCEEDED";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}; // MoveToPoseActionServer

} // namespace dobot_system_tests   

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<dobot_system_tests::MoveToPoseActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}