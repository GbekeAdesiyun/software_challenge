#include <software_training_assignment/circular_move_publisher.hpp>

using namespace std::chrono_literals;

namespace composition 
{
//default constructor
// Circular_move_publisher::Circular_move_publisher() : 

Circular_move_publisher::Circular_move_publisher(
    const rclcpp::NodeOptions &options ) 
    : Node("circular_move_publisher", options) , my_queue(0) coordinates.linear.x(0),
    coordinates.linear.y(0), coordinates.linear.z(0), coordinates.angular.x(0),
    coordinates.angular.y(0), coordinates.angular.z(0)
    {
        //Creating the publisher
        publisher = 
        create_publisher<geometry_msgs::msg::Twist>(
            "/circular_motion_turtle/velocity", rclcpp::Qos(my_queue));

        //my_timer makes the timer_callback to be called every 10ms
        my_timer = this->create_wall_timer(10ms,timer_callback);
    }

void Circular_move_publisher::timer_callback()
{
    //use unique pointer to store messages
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = Circular_move_publisher::coordinates.linear.x;
    message->linear.y = Circular_move_publisher::coordinates.linear.y;
    message->linear.z = Circular_move_publisher::coordinates.linear.z;

    message->angular.x = Circular_move_publisher::coordinates.angular.x;
    message->angular.y = Circular_move_publisher::coordinates.angular.y;
    message->angular.z = Circular_move_publisher::coordinates.angular.z;

    publisher->publish(std::move(message));
}

} //namespace composition

#include <rclcpp_components/register_node_macro.hpp>

//Register the component with the class circular_move_publisher 
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Circular_move_publisher)

