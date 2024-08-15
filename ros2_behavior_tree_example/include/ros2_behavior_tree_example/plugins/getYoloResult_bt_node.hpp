#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include <yolov8_msgs/msg/inference_result.hpp>
#include <yolov8_msgs/msg/yolov8_inference.hpp>

namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Condition Node that tells us whether a pong has been received
    ///
    ///
    class getYoloResultNode : public BT::ConditionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a pong message has been received
        /// @param condition_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        getYoloResultNode(const std::string & condition_name, const BT::NodeConfig & conf);
        getYoloResultNode() = delete;
        ~getYoloResultNode();

        ///
        ///@brief Creates list of BT ports
        ///@return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_name")
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
       void yolo_callback(yolov8_msgs::msg::Yolov8Inference::SharedPtr last_msg);

    private:
        ///
        /// @brief Subscription to the pong message from a tertiary node
        ///
        std::string sub_topic_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        // int32_t pong_id_received_;
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr sub_;

        std::atomic<bool> object_found_;
        std::atomic<bool> move_on_;
    };
}
}
