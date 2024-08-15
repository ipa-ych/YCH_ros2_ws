#include "ros2_behavior_tree_example/plugins/callStdSrvsTrigger_bt_node.hpp"
#include "rclcpp/service.hpp"

namespace polymath
{
namespace bt_ros_example
{
    callStdSrvTrigger::callStdSrvTrigger(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    service_called_(false)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // callback_group_ = node_->create_callback_group(
        //                         rclcpp::CallbackGroupType::MutuallyExclusive,
        //                         false);
        // executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        // rclcpp::SubscriptionOptions options;
        // options.callback_group = callback_group_;

        // sub_ = node_->create_subscription<std_msgs::msg::String>(sub_topic_,
        //                                                 rclcpp::SystemDefaultsQoS(),
        //                                                 std::bind(&callStdSrvTrigger::pong_callback, this, std::placeholders::_1),
        //                                                 options);

        // client_ = node_->create_client<std_srvs::srv::Empty>("/clear");

        // return;
    }

    callStdSrvTrigger::~callStdSrvTrigger()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN CLEAN NODE");

        // cancel stops the executor if it's spinning
        // executor_.cancel();

        // callback_group_.reset();
        // sub_.reset();
    }

    BT::NodeStatus callStdSrvTrigger::tick()
    {
        

        // auto service_name_ = getInput<std::string>("service_name")->c_str();
        auto service_name_ = getInput<std::string>("service_name").value();
        client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_);

        RCLCPP_INFO(node_->get_logger(), "tick %s service ", service_name_.c_str());


        if (!service_called_)
        {
            if (!client_->wait_for_service(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(node_->get_logger(), "Service %s not available after waiting", service_name_.c_str());
                return BT::NodeStatus::FAILURE;
            }

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            future_result_ = client_->async_send_request(request);
            service_called_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (service_called_)
        {
            auto status = future_result_.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready)
            {
                if (future_result_.get())
                {
                    RCLCPP_INFO(node_->get_logger(), "Service %s called successfully", service_name_.c_str());
                    service_called_ = false;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", service_name_.c_str());
                    service_called_ = false;
                    return BT::NodeStatus::FAILURE;
                }
            }
        }

        return BT::NodeStatus::RUNNING;

        // if (!client_->wait_for_service(std::chrono::seconds(10)))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Service /clear not available after waiting");
        //     return BT::NodeStatus::FAILURE;
        // }

        // RCLCPP_INFO(node_->get_logger(), "tick /clear service 2");

        // auto request = std::make_shared<std_srvs::srv::Empty::Request>();

        // RCLCPP_INFO(node_->get_logger(), "tick /clear service 3");

        // auto result = client_->async_send_request(request);

        // RCLCPP_INFO(node_->get_logger(), "tick /clear service 4");

        // if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "Service /clear called successfully");
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service /clear");
        //     return BT::NodeStatus::FAILURE;
        // }
        // return BT::NodeStatus::RUNNING;
    }

}
}
