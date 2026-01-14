#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace unity_nitros_bridge
{

class CameraInfoHeaderRewriterNode : public rclcpp::Node
{
public:
  explicit CameraInfoHeaderRewriterNode(const rclcpp::NodeOptions & options)
  : Node("camera_info_header_rewriter", options)
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/kevin/stereo_camera/left/camera_info_resize");
    this->declare_parameter<std::string>("output_topic", "/kevin/stereo_camera/left/camera_info_optical");
    this->declare_parameter<std::string>("target_frame_id", "stereo_left_optical_frame");
    
    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    target_frame_id_ = this->get_parameter("target_frame_id").as_string();
    
    // Create subscriber and publisher
    sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      input_topic, 10,
      std::bind(&CameraInfoHeaderRewriterNode::callback, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(output_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "CameraInfo Header Rewriter initialized:");
    RCLCPP_INFO(this->get_logger(), "  Input: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target frame_id: %s", target_frame_id_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create new message with modified frame_id
    auto output_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
    output_msg->header.frame_id = target_frame_id_;
    
    pub_->publish(*output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  std::string target_frame_id_;
};

}  // namespace unity_nitros_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(unity_nitros_bridge::CameraInfoHeaderRewriterNode)
