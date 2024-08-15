#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include <std_srvs/srv/empty.hpp>

#include <cob_srvs/srv/set_string.hpp>

namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Condition Node that tells us whether a pong has been received
    ///
    ///
    class callCobSrvSetString : public BT::ConditionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a pong message has been received
        /// @param condition_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        callCobSrvSetString(const std::string & condition_name, const BT::NodeConfig & conf);
        callCobSrvSetString() = delete;
        ~callCobSrvSetString() override;

        ///
        ///@brief Creates list of BT ports
        ///@return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("service_name"),
                BT::InputPort<std::string>("input_data"),
                // BT::OutputPort<int32_t>("last_pong_id"),
            };
        }

        ///
        /// @brief The main behavior call when this node is run
        /// @return NodeStatus Success or Failiure
        ///
        BT::NodeStatus tick() override;

        void service_response_callback(rclcpp::Client<cob_srvs::srv::SetString>::SharedFuture future);

        ///
        /// @brief Record the last received pong
        ///

    private:

        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        rclcpp::Client<cob_srvs::srv::SetString>::SharedPtr client_;
        rclcpp::Client<cob_srvs::srv::SetString>::SharedFuture future_result_;

        std::atomic<bool> service_called_;
        std::atomic<bool> service_succeeded_;
    };
}
}
