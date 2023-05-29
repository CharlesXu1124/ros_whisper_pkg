#include "whisper_client_cpp/whisper_client_node.hpp"

using std::placeholders::_1;

namespace whisper_client
{

  WhisperClientNode::WhisperClientNode(const rclcpp::NodeOptions &options) : rclcpp::Node("whisper_response_client", options)
  {
    whisper_res_pub_ptr = this->create_publisher<std_msgs::msg::String>("whisper_inference_results", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&WhisperClientNode::call_whisper_server, this));
    message = std::make_shared<std_msgs::msg::String>();

    client_ptr_ = rclcpp_action::create_client<WhisperReponse>(this, "whisper_transcribe");
  }

  void WhisperClientNode::goal_response_callback(const GoalHandleWhisper::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void WhisperClientNode::feedback_callback(
      GoalHandleWhisper::SharedPtr,
      const std::shared_ptr<const WhisperReponse::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "-------task status------";
    ss << feedback->log << std::endl;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void WhisperClientNode::result_callback(const GoalHandleWhisper::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;
    ss << "Result received: ";
    ss << result.result->transcription << std::endl;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  }

  void WhisperClientNode::call_whisper_server()
  {
    // this->timer_->cancel();

    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = WhisperReponse::Goal();

    goal_msg.record_and_transcribe = "2";

    RCLCPP_INFO(this->get_logger(), "Sending transcribe task");

    auto send_goal_options = rclcpp_action::Client<WhisperReponse>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WhisperClientNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&WhisperClientNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&WhisperClientNode::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
  RCLCPP_COMPONENTS_REGISTER_NODE(whisper_client::WhisperClientNode)
