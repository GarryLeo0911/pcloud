// ROS 2 median filter node (simple image -> image)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <cstring>

class MedianFilterNode : public rclcpp::Node {
public:
  MedianFilterNode() : rclcpp::Node("median_filter") {
    input_topic_ = this->declare_parameter<std::string>("input_image_topic", "image");
    output_topic_ = this->declare_parameter<std::string>("output_image_topic", "image_filtered");
    kernel_size_ = this->declare_parameter<int>("kernel_size", 3);
    iterations_ = this->declare_parameter<int>("iterations", 1);

    // subscription & publisher
    sub_ = image_transport::create_subscription(
        this, input_topic_,
        std::bind(&MedianFilterNode::image_cb, this, std::placeholders::_1),
        "raw");
    pub_ = image_transport::create_publisher(this, output_topic_);

    // dynamic params
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MedianFilterNode::on_params, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_params(
      const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == "kernel_size" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int v = p.as_int();
        if (v < 1) v = 1;
        if (v % 2 == 0) v += 1; // kernel must be odd
        kernel_size_ = v;
      } else if (p.get_name() == "iterations" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int v = p.as_int();
        if (v < 1) v = 1;
        iterations_ = v;
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    const auto &enc = msg->encoding;

    int cv_type = 0;
    bool needs_bgr_to_rgb = false;
    if (enc == "mono8") {
      cv_type = CV_8UC1;
    } else if (enc == "bgr8") {
      cv_type = CV_8UC3;
    } else if (enc == "rgb8") {
      cv_type = CV_8UC3;
      needs_bgr_to_rgb = true; // we'll convert to BGR for processing and back to RGB for output
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported encoding '%s'", enc.c_str());
      return;
    }

    // Wrap input data (do not modify in place)
    cv::Mat input(static_cast<int>(msg->height), static_cast<int>(msg->width), cv_type,
                  const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
    cv::Mat work = input.clone();

    // Convert RGB->BGR if needed for OpenCV default
    if (needs_bgr_to_rgb) {
      cv::cvtColor(work, work, cv::COLOR_RGB2BGR);
    }

    // Apply median filter
    for (int i = 0; i < iterations_; ++i) {
      cv::medianBlur(work, work, kernel_size_);
    }

    // Convert back to RGB if needed
    if (needs_bgr_to_rgb) {
      cv::cvtColor(work, work, cv::COLOR_BGR2RGB);
    }

    // Build output message
    auto out_msg = std::make_shared<sensor_msgs::msg::Image>();
    out_msg->header = msg->header;
    out_msg->height = static_cast<uint32_t>(work.rows);
    out_msg->width = static_cast<uint32_t>(work.cols);
    out_msg->encoding = enc;
    out_msg->is_bigendian = false;
    out_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(work.step);
    out_msg->data.resize(work.total() * work.elemSize());
    std::memcpy(out_msg->data.data(), work.data, out_msg->data.size());
    pub_.publish(out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  int kernel_size_;
  int iterations_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MedianFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
