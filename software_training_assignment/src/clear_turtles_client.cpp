#include "software_training_assignment/clear_turtles_client.hpp"

using namespace std::chrono_literals;

namespace composition 
{

Turtle_client_node::Turtle_client_node(const rclcpp::NodeOptions &options) 
: Node("Turtle_client_node", options) 
{
    //client node
    client_ = create_client<turtlesim::srv::kill>("kill");

    //callback function
    timer_ = create_wall_timer(10s, std::bind(&Turtle_client_node::kill, this));
}

void Turtle_client_node::kill()
{
    //search for client node every 2s
    if(!client_->wait_for_service(2s))
    {
        //if client is cancelled(by entering ctrl+C in terminal for example)
        //  return an error log message
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), 
            "Interrupted while waiting for the service. Now exiting.");
            return;
        }
        //continue waiting if no client nodes have been found
        RCLCPP_INFO(this->get_logger(),
        "Service not available, waiting for it to become available...");
        return;
    }

    //loop to delete all the existing turtles
    for(std::string &name : turtle_names) 
    {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
    
        using FutureServiceResponse = 
        rclcpp::Client<turtlesim::srv::Kill>::SharedFuture;
        //explanation needed for this callback function
        auto callback = [this](FutureServiceResponse response)->void
        {
            (void)response; //explicit type cast to void because response is not needed
            RCLCPP_INFO(this->get_logger(),"The turtle has been killed");
            rclcpp::shutdown();
        };

        auto result = client_->async_send_request(request,callback);
    }

}

} //namespace composition 


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Turtle_client_node)


