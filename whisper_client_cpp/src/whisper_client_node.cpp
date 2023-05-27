#include "whisper_client_cpp/whisper_client_node.hpp"

using std::placeholders::_1;

namespace whisper_client
{
WhisperClientNode::WhisperClientNode(const rclcpp::NodeOptions & options) : rclcpp::Node("whisper_client_node", options)
{
  pub_posecov_stamp_ptr = this->create_publisher<std_msgs::msg::String>("whisper_inference_results", rclcpp::QoS{10});
  timer_ = this->create_wall_timer(
      3000ms, std::bind(&WhisperClientNode::call_whisper_server, this));

}

void WhisperClientNode::call_whisper_server()
{
  auto message = std_msgs::msg::String();
  message.data = "2";
  pub_posecov_stamp_ptr->publish(message);

}


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_client::WhisperClientNode)
