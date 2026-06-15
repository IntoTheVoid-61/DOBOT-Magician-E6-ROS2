/*
@brief Class for asparagus 
*/


#include <geometry_msgs/msg/pose_stamped.hpp>

class Asparagus
{
public:
    Asparagus(const geometry_msgs::msg::PoseStamped pose, const float height, const float radius)
    {
        pose_ = pose;
        height_ = height;
        radius_ = radius;
    }
    
    geometry_msgs::msg::PoseStamped getPose() const{
        return pose_;
    }

    float getHeight() const{
        return height_;
    }

    float getRadius() const{
        return radius_;
    }

private:
geometry_msgs::msg::PoseStamped pose_;
float height_;
float radius_;

};