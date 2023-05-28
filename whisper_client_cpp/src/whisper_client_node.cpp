#include "whisper_client_cpp/whisper_client_node.hpp"

using std::placeholders::_1;

namespace whisper_client
{

  WhisperClientNode::WhisperClientNode(const rclcpp::NodeOptions &options) : rclcpp::Node("whisper_client_node", options)
  {
    whisper_res_pub_ptr = this->create_publisher<std_msgs::msg::String>("whisper_inference_results", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&WhisperClientNode::call_whisper_server, this));
    message = std::make_shared<std_msgs::msg::String>();

    client_ = this->create_client<whisper_interfaces::srv::WhisperResponse>("whisper_client");

  }

  void WhisperClientNode::call_whisper_server()
  {

    auto request = std::make_shared<whisper_interfaces::srv::WhisperResponse::Request>();
    request->record_time = 2;
    using ServiceResponseFuture =
      rclcpp::Client<whisper_interfaces::srv::WhisperResponse>::SharedFuture;
    RCLCPP_INFO(this->get_logger(), "sending request");

    auto result = client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = result.get();
      RCLCPP_INFO(this->get_logger(), "transcribed results: %s" , res->result.c_str());

    } else {
      RCLCPP_INFO(this->get_logger(), "calling failed");
    }
  }


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_client::WhisperClientNode)
