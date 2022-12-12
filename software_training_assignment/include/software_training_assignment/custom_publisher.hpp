#ifndef SOFTWARE_TRAINING_ASSIGNMENT__CUSTOM_PUBLISHER_HPP_
#define SOFTWARE_TRAINING_ASSIGNMENT__CUSTOM_PUBLISHER_HPP_


#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <software_training_assignment/msg/software.hpp>
#include <turtlesim/msg/pose.hpp>


namespace composition
{

class Custom_publisher : public rclcpp::Node
{

public:
    SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
    explicit Custom_publisher(rclcpp::NodeOptions &options);

private:
    //subscribers for position
    rclcpp::Subscription<turtelsim::msg::Pose>::SharedPtr stationary_turtle_sub_;
    rclcpp::Subscription<turtelsim::msg::Pose>::SharedPtr moving_turtle_sub_;

    //publisher for turtle with custom memssage
    rclcpp::Publisher<software_training_assignment::msg::Software>::SharedPtr pub_;

    //timer callback function for the publisher
    rclcpp::TimerBase::SharedPtr timer_;

    //callback groups 
    rclcpp::CallbackGroup::SharedPtr callbacks_;

    //x, y distances for stationary turtle
    double x_stationary_turtle_;
    double y_stationary_turtle_;

    //x,y distances for moving turtle
    //use doubles since they have more precision than floats
    double x_moving_turtle_;
    double y_moving_turtle_;

    //distance between the two turtles
    double total_distance_;

    static const unsigned int QUEUE{10};

};

} // namespace composition

#endif SOFTWARE_TRAINING_ASSIGNMENT__CUSTOM_PUBLISHER_HPP_

