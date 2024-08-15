#include "ros2_behavior_tree_example/plugins/sub_bt_node.hpp"

namespace polymath
{
namespace bt_ros_example
{
    subNode::subNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    sub_topic_("/chatter")
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // Create a callback group and executor for this node specifically
        callback_group_ = node_->create_callback_group(
                                rclcpp::CallbackGroupType::MutuallyExclusive,
                                false);
        executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        // Setup subscription on running this node
        // Subscription will run when lifecycle node executor is called 
        sub_ = node_->create_subscription<std_msgs::msg::String>(sub_topic_,
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&subNode::pong_callback, this, std::placeholders::_1),
                                                        options);

        return;
    }

    subNode::~subNode()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN PONG RECEIVED NODE");

        // cancel stops the executor if it's spinning
        executor_.cancel();

        callback_group_.reset();
        sub_.reset();
    }

    BT::NodeStatus subNode::tick()
    {
        // Spin the executor for all work available on the tick
        // After a pause this means that this node will take slightly longer to process all messages
        // Depending on the QOS settings for the subscription

        // However if this is an older version, it will keep doing work and can hang so we may want to add a timeout.
        executor_.spin_some();

        RCLCPP_INFO(node_->get_logger(), "TICK::PONG_NODE");
        
        // record last pong id received so that other nodes could use it


        // reset pong id since we've received it
        return BT::NodeStatus::SUCCESS;
    }

    void subNode::pong_callback(std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "RECEIVED PONG WITH ID %s", msg->data.c_str());
        return;
    }
}
}
