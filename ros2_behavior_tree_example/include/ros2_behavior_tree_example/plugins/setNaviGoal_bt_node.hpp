#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Condition Node that tells us whether a pong has been received
    ///
    ///
    class setNaviGoal : public BT::ConditionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a pong message has been received
        /// @param condition_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        setNaviGoal(const std::string & condition_name, const BT::NodeConfig & conf);
        setNaviGoal() = delete;
        ~setNaviGoal();

        ///
        ///@brief Creates list of BT ports
        ///@return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<double>("x"),
                BT::InputPort<double>("y"),
                BT::InputPort<double>("w"),
            };
        }

        ///
        /// @brief The main behavior call when this node is run
        /// @return NodeStatus Success or Failiure
        ///
        BT::NodeStatus tick() override;

        // virtual BT::NodeStatus onResultReceived(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) = 0;

        ///
        /// @brief Record the last received pong
        ///

      //  void pong_callback(std_msgs::msg::String::SharedPtr msg);

       void result_callback (const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

    private:
        ///
        /// @brief Subscription to the pong message from a tertiary node
        ///
        std::string sub_topic_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        int32_t pong_id_received_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
        std::atomic<bool> goal_succeeded_;
        std::atomic<bool> goal_sent_;
    };
}
}
