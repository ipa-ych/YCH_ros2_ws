#include "ros2_behavior_tree_example/plugins/callCobSrvsSetString_bt_node.hpp"
#include "rclcpp/service.hpp"
#include "cob_srvs/srv/set_string.hpp" // Include the header for SetString service

namespace polymath
{
namespace bt_ros_example
{
    callCobSrvSetString::callCobSrvSetString(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    service_called_(false),
    service_succeeded_(false)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

        executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    }

    callCobSrvSetString::~callCobSrvSetString()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN CLEAN NODE");
        executor_.cancel();

        callback_group_.reset();
        client_.reset();
    }

    void callCobSrvSetString::service_response_callback(rclcpp::Client<cob_srvs::srv::SetString>::SharedFuture future)
    {
        RCLCPP_INFO(node_->get_logger(), "in callback");
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Service called successfully: %s", response->message.c_str());
            service_succeeded_ = true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", response->message.c_str());
            service_succeeded_ = false;
        }
        service_called_ = false; // Reset service_called_ to allow for subsequent service calls
    }

    BT::NodeStatus callCobSrvSetString::tick()
    {
        auto service_name_ = getInput<std::string>("service_name").value();
        auto input_data_ = getInput<std::string>("input_data").value();

        client_ = node_->create_client<cob_srvs::srv::SetString>(service_name_); // Use SetString service

        RCLCPP_INFO(node_->get_logger(), "tick %s service ", service_name_.c_str());

        if (!service_called_)
        {
            if (!client_->wait_for_service(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(node_->get_logger(), "Service %s not available after waiting", service_name_.c_str());
                return BT::NodeStatus::FAILURE;
            }

            auto request = std::make_shared<cob_srvs::srv::SetString::Request>();
            request->data = input_data_; // Set the request data

            // auto future_result_ = client_->async_send_request(request, std::bind(&callCobSrvSetString::service_response_callback, this, std::placeholders::_1));
            future_result_ = client_->async_send_request(request);
            service_called_ = true;
            executor_.spin_some();
            RCLCPP_INFO(node_->get_logger(), " executor spin ");

            return BT::NodeStatus::RUNNING;
        }

        if (service_called_)
        {
            RCLCPP_INFO(node_->get_logger(), " service called ");
            auto status = future_result_.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready)
            {
                if (future_result_.valid())
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


        if (service_succeeded_)
        {
            service_succeeded_ = false; // Reset the flag for the next tick
            return BT::NodeStatus::SUCCESS;
        }
        executor_.spin_some();
        return BT::NodeStatus::RUNNING;
    }

}
}
