#ifndef WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_
#define WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "whisper_interfaces/action/whisper_response.hpp"

namespace whisper_client
{
  class WhisperClientNode : public rclcpp::Node
  {
  public:
    using WhisperReponse = whisper_interfaces::action::WhisperResponse;
    using GoalHandleWhisper = rclcpp_action::ClientGoalHandle<WhisperReponse>;

    explicit WhisperClientNode(const rclcpp::NodeOptions &options);
    void goal_response_callback(const GoalHandleWhisper::SharedPtr &goal_handle);
    void feedback_callback(
        GoalHandleWhisper::SharedPtr,
        const std::shared_ptr<const WhisperReponse::Feedback> feedback);
    void result_callback(const GoalHandleWhisper::WrappedResult &result);

    void call_whisper_server();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr whisper_res_pub_ptr;
    std::shared_ptr<std_msgs::msg::String> message;
    rclcpp_action::Client<WhisperReponse>::SharedPtr client_ptr_;
  };

}

#endif // WHISPER_CLIENT_NODE__WHISPER_CLIENT_NODE_HPP_