#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include <std_srvs/srv/trigger.hpp>

namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Condition Node that tells us whether a pong has been received
    ///
    ///
    class callStdSrvTrigger : public BT::ConditionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a pong message has been received
        /// @param condition_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        callStdSrvTrigger(const std::string & condition_name, const BT::NodeConfig & conf);
        callStdSrvTrigger() = delete;
        ~callStdSrvTrigger() override;

        ///
        ///@brief Creates list of BT ports
        ///@return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("service_name"),
                // BT::OutputPort<int32_t>("last_pong_id"),
            };
        }

        ///
        /// @brief The main behavior call when this node is run
        /// @return NodeStatus Success or Failiure
        ///
        BT::NodeStatus tick() override;

        ///
        /// @brief Record the last received pong
        ///

    private:

        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_result_;
        std::atomic<bool> service_called_;
    };
}
}
