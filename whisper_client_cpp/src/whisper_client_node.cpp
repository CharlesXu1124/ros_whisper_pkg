#include "whisper_client_cpp/whisper_client_node.hpp"

using std::placeholders::_1;

namespace whisper_client
{
WhisperClientNode::WhisperClientNode(const rclcpp::NodeOptions & options) : rclcpp::Node("whisper_client_node", options)
{
  pub_posecov_stamp_ptr = this->create_publisher<std_msgs::msg::String>("whisper_inference_results", rclcpp::QoS{10});


}

void WhisperClientNode::call_whisper_server()
{

}


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_client::WhisperClientNode)
