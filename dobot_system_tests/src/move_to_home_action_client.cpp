/*
@brief This ROS 2 implements the move_to_home action client. Pairs with move_to_home action server

@author: Ziga Breznikar
@date: 25.03.2026

*/

#include <memory>
#include <thread>

#include "dobot_msgs_fb/action/move_to_home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.h"

using MoveToHome = dobot_msgs_fb::action::MoveToHome;
using GoalHandleMoveToHome = rclcpp_action::ClientGoalHandle<MoveToHome>;

namespace dobot_system_tests
{
class MoveToHomeActionClient : public rclcpp::Node
{
public:
    /*
    @brief Constructor for MoveToPoseActionClient
    @param options ROS 2 node options
    */

    explicit MoveToHomeActionClient(const rclcpp::NodeOptions& options)
    : Node("move_to_home_action_client_cpp", options)
    {
        // create action_client
        action_client_ = rclcpp_action::create_client<MoveToHome>(
            this,
            "move_to_home"
        );

        // declare parameters (default values if values are like this yaml file will be read)
        this->declare_parameter("velocity_scaling", 0.0);
        this->declare_parameter("acceleration_scaling", 0.0);
        this->declare_parameter("joint_vector", std::vector<double>{});

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
    rclcpp_action::Client<MoveToHome>::SharedPtr action_client_;
    GoalHandleMoveToHome::SharedPtr goal_handle_;
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    @brief Callback function for goal response.
    @param goal_handle The goal handle returned by the action server.
    */

    void goal_response_callback(const GoalHandleMoveToHome::SharedPtr& goal_handle){

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
        const GoalHandleMoveToHome::SharedPtr& goal_handle,
        const std::shared_ptr<const MoveToHome::Feedback>& feedback
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

    void get_result_callback(const GoalHandleMoveToHome::WrappedResult& result){

        switch (result.code)
        {
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


        // get parameters for the goal
        auto velocity_scaling = this->get_parameter("velocity_scaling").as_double();
        auto acceleration_scaling = this->get_parameter("acceleration_scaling").as_double();
        auto joint_vector = this->get_parameter("joint_vector").as_double_array();

        auto goal_msg = MoveToHome::Goal();
        goal_msg.velocity_scaling = velocity_scaling;
        goal_msg.acceleration_scaling = acceleration_scaling;
        goal_msg.joint_vector = joint_vector;
    
        RCLCPP_INFO(get_logger(),
                    "Sending to home!");

        auto send_goal_options = rclcpp_action::Client<MoveToHome>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MoveToHomeActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&MoveToHomeActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&MoveToHomeActionClient::get_result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);

    }

}; // MoveToHomeActionClient

} // namespace dobot_system_tests

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<dobot_system_tests::MoveToHomeActionClient>(rclcpp::NodeOptions());
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}