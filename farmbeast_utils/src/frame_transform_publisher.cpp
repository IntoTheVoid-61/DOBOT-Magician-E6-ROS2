/*
@brief This ROS 2 node implements a publisher to topic definition dobot_msgs_fb/msg/FrameTransform topic.
       It is configurable by a yaml file in which you define the source_frame and target frame, topic therefore includes the pose of target_frame relative to source_frame.

@author Ziga Breznikar
@date 09.05.2026

*/

#include <memory>
#include <chrono>

#include "dobot_msgs_fb/msg/frame_transform.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/string.hpp"



namespace farmbeast_utils
{
class FrameTransformPublisher : public rclcpp::Node
{
public:
    /*
    @brief A constructor that orchestrates:
        - Initialize node,
        - Declare parameters,
        - Read parameters,
        - Create publisher,
        - Initialize TF listener,
        - Create timer
    */
    // constructor
    FrameTransformPublisher() : Node("frame_transform_node"){

    // declare parameters
    this->declare_parameter<std::string>("source_frame", "base_link");
    this->declare_parameter<std::string>("target_frame", "Link6");
    this->declare_parameter<double>("publish_rate", 10.0);

    // Initialize parameters from yaml
    source_frame_param_ = this->get_parameter("source_frame").as_string();
    target_frame_param_ = this->get_parameter("target_frame").as_string();
    publish_rate_param_ = this->get_parameter("publish_rate").as_double();
    
    // Create publisher with custom msg definition
    publisher_ = this->create_publisher<dobot_msgs_fb::msg::FrameTransform>("frame_transform", 10);

    // TF buffer -> Stores transform data
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(
            this->get_clock());

    // Subscribes to /tf /tf_static
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);

    // Timer
    auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / publish_rate_param_)
    );

    timer_ = this->create_wall_timer(
        period,
        std::bind(
            &FrameTransformPublisher::timer_callback,
            this
        )
    );

    // callback function to send msg

    } // constructor

private:

    rclcpp::Publisher<dobot_msgs_fb::msg::FrameTransform>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string source_frame_param_;
    std::string target_frame_param_;
    double publish_rate_param_;


    /*
    @brief Publishes the msg to topic
    */
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                target_frame_param_,
                source_frame_param_,
                tf2::TimePointZero
            );

            // convert from Quaternion to RPY
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // fill custom msg
            dobot_msgs_fb::msg::FrameTransform msg;

            msg.time_stamp = transform.header.stamp;
            msg.source_frame = source_frame_param_;
            msg.target_frame = target_frame_param_;
            
            msg.translation.x = transform.transform.translation.x;
            msg.translation.y = transform.transform.translation.y;
            msg.translation.z = transform.transform.translation.z;

            msg.rpy.x = roll;
            msg.rpy.y = pitch;
            msg.rpy.z = yaw;

            msg.quaternion = transform.transform.rotation;

            publisher_->publish(msg);
        }
        catch(tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "%s",
                ex.what());

            return;            
        }

    }

}; // FrameTransformPublisher
} // namespace farmbeast_utils

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<farmbeast_utils::FrameTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}