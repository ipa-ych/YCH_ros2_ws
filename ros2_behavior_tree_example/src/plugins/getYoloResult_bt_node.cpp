#include "ros2_behavior_tree_example/plugins/getYoloResult_bt_node.hpp"

namespace polymath
{
namespace bt_ros_example
{
    getYoloResultNode::getYoloResultNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    sub_topic_("/Yolov8_Inference"),
    object_found_(false),
    move_on_(false)
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
        sub_ = node_->create_subscription<yolov8_msgs::msg::Yolov8Inference>(sub_topic_,
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&getYoloResultNode::yolo_callback, this, std::placeholders::_1),
                                                        options);

        return;
    }

    getYoloResultNode::~getYoloResultNode()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN YoloInference RECEIVED NODE");

        // cancel stops the executor if it's spinning
        executor_.cancel();

        callback_group_.reset();
        sub_.reset();
    }

    BT::NodeStatus getYoloResultNode::tick()
    {
        // Spin the executor for all work available on the tick
        // After a pause this means that this node will take slightly longer to process all messages
        // Depending on the QOS settings for the subscription

        // However if this is an older version, it will keep doing work and can hang so we may want to add a timeout.
        executor_.spin_some();

        RCLCPP_INFO(node_->get_logger(), "TICK::GET_YOLO_RESULT_NODE");
        
        // record last pong id received so that other nodes could use it

        if(!move_on_)
        {   
            RCLCPP_INFO(node_->get_logger(), "Object not found");
            if (object_found_)
            {
                RCLCPP_INFO(node_->get_logger(), "Object found");
                move_on_ = true;
                // return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (move_on_) {
            RCLCPP_INFO(node_->get_logger(), "Object found");
            return BT::NodeStatus::SUCCESS;
        }
        

        // reset pong id since we've received it
        return BT::NodeStatus::RUNNING;
    }

    void getYoloResultNode::yolo_callback(yolov8_msgs::msg::Yolov8Inference::SharedPtr last_msg)
        {
            auto object_class = getInput<std::string>("object_name");
            if (last_msg && !last_msg->yolov8_inference.empty())
                {
                    for (const auto& inference : last_msg->yolov8_inference)
                    {   
                        // RCLCPP_INFO(logger(), "Wait for %s", object_class);
                        RCLCPP_INFO(node_->get_logger(), "Wait for %s", object_class->c_str());
                        RCLCPP_INFO(node_->get_logger(), "Detected class_name: %s", inference.class_name.c_str());
                        if (inference.class_name == object_class)
                        {
                            object_found_ = true;
                            RCLCPP_INFO(node_->get_logger(), "%s detected! Robot return to Starting Point", object_class->c_str());
                            break;
                        }
                        else 
                            object_found_ = false;
                    }
                }
        }
    }
}

