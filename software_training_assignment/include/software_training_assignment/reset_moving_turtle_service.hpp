#ifndef SOFTWARE_TRAINING_ASSIGNMENT__RESET_MOVING_TURTLE_SERVICE_HPP_
#define SOFTWARE_TRAINING_ASSIGNMENT__RESET_MOVING_TURTLE_SERVICE_HPP_

#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/srv/software.hpp>
#include <software_training_assignment/visibility.h>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition
{

class Reset_moving_turtle_service : public rclcpp::Node
{
    public:
        SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
        explicit Reset_moving_turtle_service(const rclcpp::NodeOptions &options);
    
    private:
        //create service that resets "movinf_turtle" to starting position
        rclcpp::Service<software_training_assignment::srv::Software>::SharedPtr service_;

        //create client
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;

        //callback for server
        SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
        void service_callback
        (
            const std::shared_ptr<software_training_assignment::srv::Software::Request> request,
            std::shared_ptr<software_training_assignment::srv::Software::Response> response
        );

        typedef struct reset{
            constexpr static float x = 5.44;
            constexpr static float y = 5.44;
            constexpr static float theta = 0;
        } reset_coordinates;
};


} // namespace composition

#endif // SOFTWARE_TRAINING_ASSIGNMENT__RESET_MOVING_TURTLE_SERVICE_HPP_