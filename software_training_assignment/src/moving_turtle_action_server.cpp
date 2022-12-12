#include <cmath>
#include <memory>
#include <chrono>

#include <software_training_assignment/moving_turtle_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace composition
{

Moving_turtle_action_server::Moving_turtle_action_server(
    const rclcpp::NodeOptions &options) : Node("moving_turtle_action_server", options)
    {
        //create publisher
        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/moving_turtle/cmd_vel", rclcpp::Qos(QUEUE));



        //Enable topic statistics for subscriber now
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        auto subscriberCallback =
        [this](const ::msg::Pose::SharedPtr msg)->void
        {

            this->Moving_turtle_action_server::x_ = msg->x;
            this->Moving_turtle_action_server::y_ = msg->y;
            this->Moving_turtle_action_server::theta = msg->theta;
            this->Moving_turtle_action_server::linear_velocity_= msg->linear_velocity;
            this->Moving_turtle_action_server::angular_velocity_ = msg->angular_velocity;
        
        };

        //subscriber
        this->subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/moving_turtle/pose", QUEUE, subscriberCallback);

        //action server
        this->action_server_ = 
        rclcpp_action::create_server<software_training_assignment::action::Software>(
            this,
            "moving_turtle_action_server",
            std::bind(&Moving_turtle_action_server::handle_goal, this, _1, _2),
            std::bind(&Moving_turtle_action_server::handle_cancel, this, _1),
            std::bind(&Moving_turtle_action_server::handle_accepted, this, _1));

    }

//callback function for handling goals
rclcpp_action::GoalResponse Moving_turtle_action_server::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const software_training_assignmnent::action::Software::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),"Received goal request");
    RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f Z:%f", goal->linear_pos.x,
    goal->linear_pos.y, goal->linear_pos.z);
    RCLCPP_INFO(this->get_logger(), "angular X:%f Y:%f Z:%f" goal->angular_pos.x,
    goal->angular_pos.y, angular_pos.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//callback function that tells cleint cancellation request has been accepted
rclcpp_action::CancelResponse Moving_turtle_action_server::handle_cancel(
    const std::shared_ptr<GoalHandleActionServer> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(),"Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

//callback function that uses a thread to handle the execution 
//      and return handle_accepted quickly because 
//          the execution processs takes a long time
void Moving_turtle_action_server::handle_accepted(
    const std::shared_ptr<GoalHandleActionServer> goal_handle)
{
    std::thread{std::bind(&Moving_turtle_action_server::execute, this, _1),
    goal_handle}.detach();

}

//execute function does all the remaining processing needed
void Moving_turtle_action_server::execute(
    const std::shared_ptr<GoalHandleActionServer> goal_handle)
{

    rclcpp::Time start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    //get the goal that needs to be executed
    const auto goal = goal_handle->get_goal();

    //feedback
    std::unique_ptr<software_training_assignment::action::Software::Feedback> feedback =
    std::make_unique<software_training_assignment::action::Software::Feedback>();

    //result 
    std::unique_ptr<software_training_assignment::action::Software::Result> result =
    std::make_unique<software_training_assignment::action::Software::Result>();

    //reference to feedback to make it easier to use
    float &curr_x = feedback->x_pos;
    float &curr_y = feedback->y_pos;
    float &curr_theta = feedback->theta_pos;

    //to keep track of linear feedback
    float lin_x{0};
    float lin_y{0};
    float lin_z{0};

    // to keep track of angular feedback
    float ang_x{0};
    float ang_y{0};
    float ang_z{0};

    while(rclcpp::ok() &&
          (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y ||
           lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x ||
           ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z))
    {
        //check if there is a cancel request
        if(goal_handle->is_cancelling())
        {
            //get the time taken so far and update result
            rclcpp::Time curr_time = this->now();
            rclcpp::Duration time = curr_time - start_time;
            long int duration{time.nanosecond()};
            result->duration = duration;
            goal_handle->canceled(std::move(result));
            RCLCPP_INFO(this->get_logger(), "Goal Cancelled");
            return;
        }
        
        //message
        auto message_cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

        //fill the message content
        //increment linear or angular  x y z values only if 
        //      they are less than desired values

        message_cmd_vel->linear.x = (lin_x < goal->linear_pos.x) ? lin_x++ : lin_x;
        message_cmd_vel->linear.y = (lin_x < goal->linear_pos.y) ? lin_y++ : lin_y;
        message_cmd_vel->linear.z = (lin_x < goal->linear_pos.z) ? lin_z++ : lin_z;

        message_cmd_vel->angular.x = 
        (ang_x < goal->angular_pos.x) ? ang_x++ : ang_x;
        message_cmd_vel->angular.y = 
        (ang_x < goal->angular_pos.y) ? ang_y++ : ang_y;
        message_cmd_vel->angular.z = 
        (ang_x < goal->angular_pos.z) ? ang_z++ : ang_z;

        //publish message
        this->publisher_->publish(std::move(message_cmd_vel));

        //compute feedback
        curr_x = this->Moving_turtle_action_server::x_ - lin_x;
        curr_y = this->Moving_turtle_action_server::y_ - lin_y;

        float theta{0};

        // namespace computation
        // {
        float x1{lin_x} , x2{lin_y} , x3{lin_z};

        float magnitude{static_cast<float>(sqrt((x1 * x1) + (x1 * x1) + (x3 * x3)))};

        theta = acos(x3/magnitude);

        // }

        curr_theta = this->Moving_turtle_action_server::theta_ - theta;

        //publish feedback
        goal_handle->publish_feedback(std::move(feedback));
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
        loop_rate.sleep();

    }

    //Check if the goal is done
    if(rclcpp::ok())
    {
        rclcpp::Time end_time = this->now();
        rclcpp::Duration duration = end_time = start_time;
        long int result_time{duration.nanoseconds()};

        //fill in the results
        result->duration = result_time;

        goal_handle->succeed(std::move(result));
        RCLCPP_INFO(this->get_logger(), "Goal execution is complete");
    }
    
}



} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Moving_turtle_action_server)
