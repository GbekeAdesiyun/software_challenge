#ifndef SOFTWARE_TRAINING_ASSIGNMENT__CIRCULAR_MOVE_PUBLISHER_HPP_ 
#define SOFTWARE_TRAINING_ASSIGNMENT__CIRCULAR_MOVE_PUBLISHER_HPP_

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include <software_training_assignment/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition 
{

class Circular_move_publisher : public rclcpp::Node 
{

public:
    SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
    //parameterized constructor using NodeOptions which specifies default values
    explicit Circular_move_publisher(const rclcpp::NodeOptions &options);

protected:
    //callback function where messages will actually be published
    void timer_callback();

private:
    //Publisher - (why is this"SharedPtr and not shared_ptr)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    //callback for the timer
    rclcpp::TimerBase::SharedPtr my_timer;

    /*Constexpr is used because the initalization needs to be done 
    at compile time and the initialization is done by the constructor*/
    static constexpr unsigned int my_queue[10] = {};

    //struct to contain coordinates of turtle
    typedef struct point {

        //struct to hold linear coordinates
        typedef struct linearPoint{
            static constexpr float x = 12;
            static constexpr float y = 12;
            static constexpr float z = 12;
        } linear;

        //struct to hold angular coordinates
        /*struct tag name is lowercase and struct type name 
        is uppercase as that follows common programming practice*/
        typedef struct angularPoint{
            static constexpr float x = 1.41;
            static constexpr float y = 1.41;
            static constexpr float z = 1.41;
        } angular;
    } coordinates;


}; //end of class declaration

} // composition namespace  

#endif //SOFTWARE_TRAINING_ASSIGNMENT__CIRCULAR_MOVE_PUBLISHER_HPP_


