/*
@brief This ROS 2 node implements a move_to_pose action client for sending movement goals.

This action client is responsible for sending the desired pose and velocity-acceleration scaling commands
to the action server.

Action client:
    - move_to_pose (dobot_msgs_fb/action/MoveToPose)
    Sends goals to move_to_pose and receives feedback and results
        - Goals: velocity_scaling, acceleration_scaling, target_pose
        - Feedback: status, elapsed_time
        - Results: success

@ author: Ziga Breznikar
@ date: 18.03.2026


*/

#include <memory>
#include <thread>

#include "dobot_msgs_fb/action/move_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.h"

using MoveToPose = dobot_msgs_fb::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

namespace dobot_system_tests
{
class MoveToPoseActionClient : public rclcpp::Node
{
public:
    /*
    @brief Constructor for MoveToPoseActionClient
    @param options ROS 2 node options
    */
    explicit MoveToPoseActionClient(const rclcpp::NodeOptions& options)
    : Node("move_to_pose_action_client_cpp", options)
    {
        // create action_client
        action_client_ = rclcpp_action::create_client<MoveToPose>(
            this,
            "move_to_pose"
        );

        this->declare_parameter("velocity_scaling", 0.5);
        this->declare_parameter("acceleration_scaling", 0.5);
        this->declare_parameter("target_pose", std::vector<double>{0.3,0.0,0.2,0.0,0.0,0.0,1.0});

        // add timer for calling send_goal
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this](){
                send_goal();
                timer_->cancel();
            }
        );
    }

private:
    // class member variables
    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
    GoalHandleMoveToPose::SharedPtr goal_handle_;
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    @brief Callback function for goal response.
    @param goal_handle The goal handle returned by the action server.
    */

    void goal_response_callback(const GoalHandleMoveToPose::SharedPtr& goal_handle){

        if(!goal_handle){
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
            return;
        }

        goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Goal was accepted by the server!");
        return;

    }

    /*
    @brief Callback function for feedback
    @param goal_handle The goal handle associated with the feedback.
    * @param feedback The feedback message received from the action server.
    */

    void feedback_callback(
        const GoalHandleMoveToPose::SharedPtr& goal_handle,
        const std::shared_ptr<const MoveToPose::Feedback>& feedback
    ){
        (void) goal_handle; // unused parameter
        // Log feedback
        RCLCPP_INFO(get_logger(), 
                    "Feedback received - elapsed time: %.2f seconds, Status: %s",
                    feedback->elapsed_time,
                    feedback->status.c_str()
                    );
    }


    /*
    @brief Callback function for getting results
    @param result The result message received from the action server.
    */

    void get_result_callback(const GoalHandleMoveToPose::WrappedResult& result){

        switch (result.code)
        {
        // Not necessarily currently but we could add the pose error
        // If succeeded
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!");
            break;
        // if aborted
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
        // if canceled
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal was canceled");
            break;
        // if unknown
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }

        return;

    }

    /*
    @brief Cancel the current goal
    Attempts to cancel the currently executing goal. Checks for a valid goal handle
    and waits for the cancellation response from the server.
    */

    void cancel_goal(){

        RCLCPP_INFO(get_logger(), "Requesting to cancel goal!");
        if(!goal_handle_)
        {
            RCLCPP_ERROR(get_logger(), "Goal handle is null!");
            return;
        }

        auto future = action_client_->async_cancel_goal(goal_handle_);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)

        {
        RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
        return;
        }

        RCLCPP_INFO(get_logger(), "Goal cancellation request sent");
        return;

    }

    /*
    @brief Send a goal to the action server.
    
    */

    void send_goal(){

        // wait for action server
        if(!action_client_->wait_for_action_server(std::chrono::seconds(5))){
            RCLCPP_ERROR(get_logger(), "Action server not available!");
            return;
        }

        // Get parameters for the goal
        auto velocity_scaling = this->get_parameter("velocity_scaling").as_double();
        auto acceleration_scaling = this->get_parameter("acceleration_scaling").as_double();
        auto pose_array = this->get_parameter("target_pose").as_double_array();

        // create Pose object and assign values from pose vector
        geometry_msgs::msg::Pose pose;

        pose.position.x = pose_array[0];
        pose.position.y = pose_array[1];
        pose.position.z = pose_array[2];
        pose.orientation.x = pose_array[3];
        pose.orientation.y = pose_array[4];
        pose.orientation.z = pose_array[5];
        pose.orientation.w = pose_array[6];

        auto goal_msg = MoveToPose::Goal();
        goal_msg.velocity_scaling = velocity_scaling;
        goal_msg.acceleration_scaling = acceleration_scaling;
        goal_msg.target_pose = pose;

        RCLCPP_INFO(get_logger(), 
                    "Sending goal: v=%.2f, a=%.2f, pos=(%.2f, %.2f, %.2f)",
                    velocity_scaling, 
                    acceleration_scaling, 
                    pose.position.x,
                    pose.position.y,
                    pose.position.z);

        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MoveToPoseActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&MoveToPoseActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&MoveToPoseActionClient::get_result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

}; // MoveToPoseActionClient

} // namespace dobot_system_tests

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<dobot_system_tests::MoveToPoseActionClient>(rclcpp::NodeOptions());
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}