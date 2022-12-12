#include <software_training_assignment/spawn_turtle.hpp>

using namespace std::chrono_literals;

namespace composition
{

Spawn_turtle::Spawn_turtle(const rclcpp::NodeOptions &options) : Node("Spawn_turtle",options)
{
    //creation of client that requests the /Spawn service
    client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

    //callback for client
    timer_ = this->create_wall_timer(10s, std::bind(&Spawn_turtle::spawn, this));

    //populate map with contents
    for(size_t i{0}; i < Number_of_turtles; i++)
    {
        turtle_description.insert({turtle_names[i],turtle_bio[i]});
    }
}

void Spawn_turtle::spawn()
{
    if(!client_->wait_for_service(2s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Service was interrupted. Exiting now");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(),"Waiting for service");
        return;
    }

    for(const std::string &name : turtle_names)
    {
        //request
        std::unique_ptr<turtlesim::srv::Spawn::Request> request =
            std::make_unique<turtlesim::srv::Request>();


        //Response
        request->name = name;
        request->x = turtle_description[name].x_pos;
        request->y = turtle_description[name].y_pos;
        request->theta = turtle_description[name].theta;

        using FutureServiceResponse = 
        rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        //callback because 'spin()' is unavailable
        auto callback = 
        [this](FutureServiceResponse response)-> void 
        {
            RCLCPP_INFO(this->get_logger(),"The Turtle created is the: %s",
            response.get()->name.c_str());
            rclcpp::shutdown();
        };

        //send the request
        auto result = client_->async_send_request(std::move(request), callback);
        
    }
}


} //namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Spawn_turtle)