#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>

#include <moveit/kinematic_constraints/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <chrono>





namespace move_loop
{
    class MoveLoop
    {
    public:
        using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
        using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

        MoveLoop(const rclcpp::NodeOptions& options); // constructor
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        bool setPoses();
        void createMotionSequence();
        void executeMotionSequence();

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<MoveGroupSequence>::SharedPtr action_client_;

        std::vector<geometry_msgs::msg::PoseStamped> poses_;
        std::vector<moveit_msgs::msg::MotionSequenceItem> items_;
        unsigned short num_of_poses_;

    }; // MoveLoop

    MoveLoop::MoveLoop(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("move_loop", options)}
    {
        // initialize action client
        action_client_ =
            rclcpp_action::create_client<MoveGroupSequence>(
                node_,
                "/sequence_move_group"
            );
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MoveLoop::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

    bool MoveLoop::setPoses()
    {
        num_of_poses_ = 
            static_cast<unsigned short>(
                node_->get_parameter("num_of_poses").as_int());
            
        poses_.clear();
        poses_.reserve(num_of_poses_);
        

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link"; 

        for(int i = 0; i < num_of_poses_; i++){
            std::stringstream ss;
            ss << "poses.pose_" << i;
            std::string s = ss.str();

            std::vector<double> pose_array = node_->get_parameter(s).as_double_array();

            if (pose_array.size() != 7){
                RCLCPP_ERROR(node_->get_logger(), "Pose must contain 7 values!");
                return false;
            }

            pose.pose.position.x = pose_array[0];
            pose.pose.position.y = pose_array[1];
            pose.pose.position.z = pose_array[2];

            pose.pose.orientation.x = pose_array[3];
            pose.pose.orientation.y = pose_array[4];
            pose.pose.orientation.z = pose_array[5];
            pose.pose.orientation.w = pose_array[6];

            poses_.push_back(pose);

        }

        return true;

    }

    void MoveLoop::createMotionSequence() // add a try catch here, make function return type bool
    {
        const auto arm_group_name = "me6_group";
        const auto hand_group_frame = "gripper";
        const auto hand_frame = "dummy_tcp";

        for(int i = 0; i < num_of_poses_; i++){
            moveit_msgs::msg::MotionSequenceItem item; // create motion sequence item
            item.blend_radius = 0.01; // blend radius between this and next trajectory
            item.req.group_name = arm_group_name;
            item.req.planner_id = "PTP";
            item.req.allowed_planning_time = 5.0;
            item.req.max_velocity_scaling_factor = 0.1;
            item.req.max_acceleration_scaling_factor = 0.1;

            item.req.goal_constraints.push_back(
                kinematic_constraints::constructGoalConstraints(hand_frame,poses_[i])
            );

            items_.push_back(item);

        }

    }

    void MoveLoop::executeMotionSequence() // for looping add the return type here as rclcpp_action::ResultCode...
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))){ // check if server not responding
            RCLCPP_ERROR(node_->get_logger(), "Server not responding");
            return;
        }

        moveit_msgs::msg::MotionSequenceRequest sequence_request;

        // add items_ to sequence request object
        for (const auto & item : items_){
            sequence_request.items.push_back(item);
        }

        // create action goal
        auto goal_msg = MoveGroupSequence::Goal();
        goal_msg.request = sequence_request;


        auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();

        // response callback
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle)
            {
            if (!goal_handle)
            {
                RCLCPP_ERROR(
                node_->get_logger(),
                "Sequence goal was rejected");
            }
            else
            {
                RCLCPP_INFO(
                node_->get_logger(),
                "Sequence goal accepted");
            }
            };

        send_goal_options.result_callback =
            [this](
            const GoalHandleMoveGroupSequence::WrappedResult & result)
            {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Motion sequence succeeded");
                break;

                case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Motion sequence aborted. Error code: %d",
                    result.result->response.error_code.val);
                break;

                case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Motion sequence canceled");
                break;

                default:
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Unknown result code");
                break;
            }
            };

        // send goal via action client    
        action_client_->async_send_goal(
            goal_msg, send_goal_options
        );

        return;
    
    }


} // move_loop





int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true); // for yaml file
    auto move_loop_node = std::make_shared<move_loop::MoveLoop>(options); // call constructor
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(move_loop_node->getNodeBaseInterface());

    if (!move_loop_node->setPoses()){

        return 1;
        }

    move_loop_node->createMotionSequence();
    move_loop_node->executeMotionSequence();

    executor.spin();
    rclcpp::shutdown();

    return 0;
}