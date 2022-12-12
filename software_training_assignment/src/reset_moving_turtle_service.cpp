#include <chrono>
#include <software_training_assignment/reset_moving_turtle_service.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;


namespace composition
{
    Reset_moving_turtle_service::Reset_moving_turtle_service(
        const rclcpp::NodeOptions &options) 
        : Node("Reset_moving_turtle_service", options)
        {

            //create client for node
            client_ = create_client<turtlesim::srv::TeleportAbsolute>(
                "moving_turtle/teleport_absoloute");
            
            service_ = create_service<software_training_assignment::srv::Software>(
                "reset_moving_turtle", 
                std::bind(&reset_moving_turtle_service::service_callback, this, _1, _2));
        
        }

    //Question: why are two callbacks needed for service 
    //  and why isn't the create_wall_timer() needed?
    void Reset_moving_turtle_service::service_callback(
        const std::shared_ptr<software_training_assignment::srv::Software::Request> request,
        std::shared_ptr<software_training_assignment::srv::Software::Response> response)
    {
        (void)request; //explicit type cast to void because request is not needed

        RCLCPP_INFO(this->get_logger(), "[Reset_moving_turtle_service]: Starting process ...");

        //wait for service to become ready
        if(!client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(),"Interrupted while waiting for the service. Exiting now");
                response->success = false;
                return;
            }
                RCLCPP_INFO(this->get_logger(), "Service is not avilable currently, waiting again...");
                response->success = false;
                return;
        }

        auto client_request = 
        std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

        //populate request data
        client_request->x = Reset_moving_turtle_service::reset_coordinates.x;
        client_request->y = Reset_moving_turtle_service::reset_coordinates.y;
        client_request->theta = Reset_moving_turtle_service::reset_coordinates.theta;

        //Callback function for response
        auto responseCallback =
        [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future_response)
        ->void 
        {
            void(future_response); //it is not needed
            RCLCPP_INFO(this->get_logger(), 
            "[Reset_moving_turtle_service]: moving_turtle has moved");
            //rclcpp::shutdown() isn't included so this allows the turtle to move 
            //      and be reset to its starting position
        };

        //client request
        auto result = client_->async_send_request(client_request, responseCallback);

        RCLCPP_INFO(this->get_logger(), "moving_turtle has now been reset");

        response->success = true;

    }

} //namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Reset_moving_turtle_service)
