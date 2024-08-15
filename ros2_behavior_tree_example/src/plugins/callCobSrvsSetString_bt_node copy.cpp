#include "ros2_behavior_tree_example/plugins/callCobSrvsSetString_bt_node.hpp"
#include "rclcpp/service.hpp"

namespace polymath
{
namespace bt_ros_example
{
    callCobSrvSetString::callCobSrvSetString(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    service_called_(false)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node"); 
        // callback_group_ = node_->create_callback_group(
        //                         rclcpp::CallbackGroupType::MutuallyExclusive,
        //                         false);
        // executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    }

    callCobSrvSetString::~callCobSrvSetString()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN CLEAN NODE");
    }

    BT::NodeStatus callCobSrvSetString::tick()
    {
        

        // auto service_name_ = getInput<std::string>("service_name")->c_str();
        auto service_name_ = getInput<std::string>("service_name").value();
        auto input_data_ = getInput<std::string>("input_data").value();
        client_ = node_->create_client<cob_srvs::srv::SetString>(service_name_);

        RCLCPP_INFO(node_->get_logger(), "tick %s service ", service_name_.c_str());


        if (!service_called_)
        {
            if (!client_->wait_for_service(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(node_->get_logger(), "Service %s not available after waiting", service_name_.c_str());
                return BT::NodeStatus::FAILURE;
            }

            auto request = std::make_shared<cob_srvs::srv::SetString::Request>();
            request->data = input_data_.c_str();

            future_result_ = client_->async_send_request(request);
            service_called_ = true;
            RCLCPP_INFO(node_->get_logger(), "service_called_ is set to true ");
            return BT::NodeStatus::RUNNING;
        }

        if (service_called_)
        {
            RCLCPP_INFO(node_->get_logger(), "service_called_ = true 1");
            auto status = future_result_.wait_for(std::chrono::milliseconds(1));

            RCLCPP_INFO(node_->get_logger(), "status is set");

            if (status == std::future_status::ready)
            {
                RCLCPP_INFO(node_->get_logger(), "status is ready");


                RCLCPP_INFO(node_->get_logger(), "flag");
                // auto response_success = future_result_.get()->success;
                // RCLCPP_INFO(node_->get_logger(), "respones_success: %s",response_success);
                RCLCPP_INFO(node_->get_logger(), "result %d",future_result_.get());

                
                if (future_result_.get()->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Service %s called successfully: %s", service_name_.c_str());
                    service_called_ = false;
                    future_result_ = {};
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s: %s", service_name_.c_str());
                    service_called_ = false;
                    future_result_ = {};
                    return BT::NodeStatus::FAILURE;
                }

            }
            
        }

        return BT::NodeStatus::RUNNING;

    }

}
}
