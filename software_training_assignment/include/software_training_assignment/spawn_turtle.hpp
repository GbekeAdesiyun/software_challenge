#ifndef SOFTWARE_TRAINING_ASSIGNMENT__SPAWN_TURTLE_HPP_
#define SOFTWARE_TRAINING_ASSIGNMENT__SPAWN_TURTLE_HPP_

#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <software_training_assignment/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>


namespace composition
{

class Spawn_turtle : public rclcpp::Node 
{

public:
    SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
    explicit Spawn_turtle(const rclcpp::NodeOptions &options);

    void spawn();

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
    static const unsigned int Number_of_turtles[2] = {};

    typedef struct turtleInfo
    {
        float x_pos = 0.0;
        float y_pos = 0.0;
        float theta;
    } turtle_info;

    std::vector<std::string> turtle_names{"stationary_turtle", "moving_turtle"};
    std::vector<turtle_info> turtle_bio{{.x_pos = 5, .y_pos = 5, .theta = 0},
                                        {.x_pos 25, .y_pos = 10, .theta = 0}};

    //map of the turtle names to turtle information
    std::map<std::string, turtle_info> turtle_description;
};

} //namespace composition

#endif  //SOFTWARE_TRAINING_ASSIGNMENT__SPAWN_TURTLE_HPP_   
