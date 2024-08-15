#include "ros2_behavior_tree_example/plugins/setNaviGoal_bt_node.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace polymath
{
namespace bt_ros_example
{
    setNaviGoal::setNaviGoal(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    goal_succeeded_(false),
    goal_sent_(false)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // Setup subscription on running this node
        // Subscription will run when lifecycle node executor is called 

        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

        return;
    }

    setNaviGoal::~setNaviGoal()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN PONG RECEIVED NODE");
    }

    BT::NodeStatus setNaviGoal::tick()
    {

        auto input_x = getInput<double>("x");
        auto input_y = getInput<double>("y");
        auto input_w = getInput<double>("w");

        if (!input_x || !input_y || !input_w) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get x or y or w from input ports");
            return BT::NodeStatus::FAILURE;
        }


        if (!goal_sent_)
        {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
                RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
                return BT::NodeStatus::FAILURE;
            }

            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = node_->now();
            // goal_msg.pose.pose.position.x = 1.0;  // Set desired goal position
            // goal_msg.pose.pose.position.y = 1.0;
            // goal_msg.pose.pose.orientation.w = 1.0;

            goal_msg.pose.pose.position.x = input_x.value();  // Set desired goal position
            goal_msg.pose.pose.position.y = input_y.value();
            goal_msg.pose.pose.orientation.w = input_w.value();

            // Send the goal
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&setNaviGoal::result_callback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(node_->get_logger(), "Goal Sent as: x=%f, y=%f, w=%f !", input_x.value(), input_y.value(), input_w.value());
            goal_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        // Check if goal succeeded
        if (goal_succeeded_) {
            goal_sent_ = false;  // Reset for the next tick
            RCLCPP_INFO(node_->get_logger(), "Goal Reached!");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }



    void setNaviGoal::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal was successful");
                goal_succeeded_ = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                goal_succeeded_ = false;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                goal_succeeded_ = false;
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                goal_succeeded_ = false;
                break;
        }
    }

}
}
