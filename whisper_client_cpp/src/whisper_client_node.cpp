#include "whisper_client_cpp/whisper_client_node.hpp"

using std::placeholders::_1;

namespace whisper_client
{

  WhisperClientNode::WhisperClientNode(const rclcpp::NodeOptions &options) : rclcpp::Node("whisper_client_node", options)
  {
    whisper_res_pub_ptr = this->create_publisher<std_msgs::msg::String>("whisper_inference_results", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&WhisperClientNode::call_whisper_server, this));
    message = std::make_shared<std_msgs::msg::String>();

    client_ = create_client<whisper_interfaces::srv::WhisperResponse>("whisper_client");

    
  }

  void WhisperClientNode::call_whisper_server()
  {
    timer_->cancel();
    message->data = "2";
    whisper_res_pub_ptr->publish(std::move(*message));
  }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_client::WhisperClientNode)
