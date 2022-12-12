#ifndef SOFTWARE_TRAINING_ASSIGNMENT__CLEAR_TURTLES_CLIENT_HPP_
#define SOFTWARE_TRAINING_ASSIGNMENT__CLEAR_TURTLES_CLIENT_HPP_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <software_training_assignment/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>

namespace composition{

class Turtle_client_node : public rclcpp::Node {
public: 
    SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC 
    explicit Turtle_client_node(const rclcpp::NodeOptions & options);

protected:
    //function to kill all existing turtles
    void kill();

private:
    //client node
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    //vector to store all the different turtles
    std::vector<std::string> turtle_names = {"first_turtle", "moving_turtle"
    ,"stationary_turtle"};

};

} //namespace composition

#endif //SOFTWARE_TRAINING_ASSIGNMENT__CLEAR_TURTLES_CLIENT_HPP