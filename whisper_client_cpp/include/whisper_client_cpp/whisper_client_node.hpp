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

#include "whisper_interfaces/srv/whisper_response.hpp"


namespace whisper_client
{
class WhisperClientNode : public rclcpp::Node
{
public:
  explicit WhisperClientNode(const rclcpp::NodeOptions & options);
  void call_whisper_server();

private:

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr whisper_res_pub_ptr;
  std::shared_ptr<std_msgs::msg::String> message;
  rclcpp::Client<whisper_interfaces::srv::WhisperResponse>::SharedPtr client_;
};

}


#endif  // WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_