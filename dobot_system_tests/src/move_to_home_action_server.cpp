/*
@brief This ROS 2 node implements a move_to_home action server. It orchestrates the movement of the robotic arm into yaml file
predefined home position joint state.

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
        this->declare_parameter<std::vector<double>>("joint_vector");

        // initialize default parameters from yaml file
        velocity_scaling_param = this->get_parameter("velocity_scaling").as_double();
        acceleration_scaling_param = this->get_parameter("acceleration_scaling").as_double();
        joint_vector_param = this->get_parameter("joint_vector").as_double_array();

    } // constructor

private:
    // class member variables
    rclcpp_action::Server<MoveToHome>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
    double velocity_scaling_param;
    double acceleration_scaling_param;
    std::vector<double> joint_vector_param; // robot has 6 joints

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

        // clear cached pose, i think this is redundant
        //move_group_interface_->clearPoseTargets();

        // get goal values
        double velocity_scaling_goal = goal->velocity_scaling;
        double acceleration_scaling_goal = goal->acceleration_scaling;
        std::vector<double> joint_vector_goal = goal->joint_vector;

        // Use goal if valid, otherwise fallback to parameters
        double velocity_scaling =
        (velocity_scaling_goal > 0.0) ? velocity_scaling_goal : velocity_scaling_param;
        double acceleration_scaling = 
        (acceleration_scaling_goal > 0.0) ? acceleration_scaling_goal : acceleration_scaling_param;
        std::vector<double> joint_vector =
        joint_vector_goal.empty() ? joint_vector_param : joint_vector_goal;
        

        // velocity and acceleration scaling
        move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling);
        move_group_interface_->setMaxAccelerationScalingFactor(acceleration_scaling);

        // set goal pose
        move_group_interface_->setJointValueTarget(joint_vector);

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
            auto current_joint_values = move_group_interface_->getCurrentJointValues();
            double diff = 0;
            // calculate differentials of joint_values
            for (int i = 0; i < 6; i++){
                diff += std::pow(current_joint_values[i] - joint_vector[i],2);
            }
            diff = std::sqrt(diff);
            if (diff < 0.01){
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