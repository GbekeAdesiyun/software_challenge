#include <cmath>
#include <iostream>
#include <string>
#include <software_training_assignment/custom_publisher.hpp>
#include <string>

using namespace std::chrono_literals;

namespace composition
{

Custom_publisher::Custom_publisher(const rclcpp::NodeOptions &options) 
: Node("custom_publisher", options)
{

    //callback function for stationary turtle position
    auto stationary_turtle_callback =
    [this](const turtlesim::msg::Pose::SharedPtr msg) -> void
    {
        this->x_stationary_turtle_ = msg->x;
        this->y_stationary_turtle_ = msg->y;
    };

    //callback function for moving turtle position
    auto moving_turtle_callback = 
    [this](const turtlesim::msg::Pose::SharedPtr msg) -> void
    {
        this->x_moving_turtle_ = msg->x;
        this->y_stationary_turtle_ = msg->y;
    };

    //callback for publisher
    auto publisherCallback = [this](void)-> void 
    {
        //absolute difference in the different turtle's coordinates
        double position_x{abs(this->x_stationary_turtle_ - this->x_moving_turtle_)};
        double position_y{abs(this->y_stationary_turtle_ - this->y_moving_turtle_)};


        //message to publish
        auto msg = 
        std::make_unique<software_training_assignment::msg::Software>();
        msg->x_pos = position_x;
        msg->y_pos = position_y;

        //distance using trigonometry
        msg->distance = sqrt((position_x * position_x) + (position_y * position_y));

        //publish the message
        this->pub_->publish(std::move(msg));
    };

    //create callback group
    callbacks_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    //create callback thread
    auto callback_option= rclcpp::SubscriptionOptions();
    callback_option.callback_group = callbacks_;

    //set ros2 topic statistics
    callback_option.topic_stats_option.state = 
    rclcpp::TopicStatisticsState::Enable;

    //set the custom name for this topic statistic
    std::string s_name("/statistics");
    std::string node_name(this->get_name());

    //the topic name will be '/statistics/custom_publisher'
    std::string stat_name = s_name + node_name;
    callback_option.topic_stats_option.publish_topic = stat_name.c_str();

    //instantiate subscribers
    stationary_turtle_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/stationary_turtle/pose", QUEUE, stationary_turtle_callback, 
        callback_option);
    
    moving_turtle_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/moving_turtle/pose", QUEUE, moving_turtle_callback, 
        callback_option);

    //instantiate publisher
    pub_ = this->create_publisher<software_training_assignment::msg::Software>(
        "/difference", QUEUE);

    //set timer for publisher
    timer_ = this->create_wall_timer(3s, publisherCallback, callbacks_);


}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(components::Custom_publisher)