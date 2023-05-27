#ifndef WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_
#define WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace whisper_client
{
class WhisperClientNode : public rclcpp::Node
{
public:
  explicit WhisperClientNode(const rclcpp::NodeOptions & options);

  void call_whisper_server();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_posecov_stamp_ptr;
};

}


#endif  // WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_