#ifndef SOFTWARE_TRAINING_ASSIGNMENT__MOVING_TURTLE_ACTION_HPP_
#define SOFTWARE_TRAINING_ASSIGNMENT__MOVING_TURTLE_ACTION_HPP_

#include <memory>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>          // for ros2 time 
#include <rclcpp_action/rclcpp_action.hpp>      //for ros2 action
#include <sofwtare_training_assignment/action/software.hpp>
#include <software_training_assignmnent/visibility.h>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

namespace composition
{

class Moving_turtle_action_server : public rclcpp::Node 
{
    public:
        SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
        explicit Moving_turtle_action_server(const rclcpp::NodeOptions &options);

        using GoalHandleActionServer =
        rclcpp_action::ServerGoalHandle<software_training_assignment::action::Software>;

        private:
        //Question: why didn't you use create_server() for this?
        rclcpp_action::Server<software_training_assignment::action::Software>::SharedPtr action_server_;

        //publisher to publish moving_turtle commands
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        //subscriber to get moving_turtle position
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;

        //callback function for the goal
        SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const software_training_assignment::action::Software::Goal> goal);
        
        //cancel goal callback function
        SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
        //Question: Is this supposed to be ClientGoalHandle or ServerGoalHandle?
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleActionServer> goal_handle);

        //handle accepted callback function
        SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
        void handle_accepted(const shared_ptr<GoalHandleActionServer> goal_handle);

        //callback function to execute 
        void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);

        //needed for the subscriber
        static float x_;
        static float y_;
        static float theta_;
        static float linear_velocity_;
        static float angular_velocity_;

        static constexpr unsigned int QUEUE[10] = {};
};

} // namespace composition

#endif //SOFTWARE_TRAINING_ASSIGNMENT__MOVING_TURTLE_ACTION_HPP_