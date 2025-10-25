// ROS 2 preview publisher using DepthAI pipeline

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <oakd_pcloud/stereo_pipeline.hpp>

class PreviewPublisherNode : public rclcpp::Node {
public:
  PreviewPublisherNode()
      : rclcpp::Node("preview_publisher"), it_(std::shared_ptr<rclcpp::Node>(this, [](auto *){})) {
    image_pub_ = image_transport::create_publisher(this, "preview/image");

    // initialize depthai device and pipeline
    stereo_.initDepthaiDev();

    auto streams = stereo_.getExposedImageStreams();
    if (streams.empty()) {
      RCLCPP_ERROR(get_logger(), "No streams available from DepthAI device");
      throw std::runtime_error("No DepthAI streams");
    }
    // Use the last stream which is configured as 'preview' in the pipeline
    preview_queue_ = streams.back();

    // parameters
    this->declare_parameter<std::string>(
        "frame_id", std::string("oakd_frame"));
    int period_ms = this->declare_parameter<int>("publish_period_ms", 33); // ~30Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&PreviewPublisherNode::publish_once, this));
  }

private:
  void publish_once() {
    if (!preview_queue_) return;
    auto frame = preview_queue_->tryGet<dai::ImgFrame>();
    if (!frame) return;

    // get cv::Mat from depthai frame
    cv::Mat cv_frame = frame->getCvFrame();
    if (cv_frame.empty()) return;

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", cv_frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = this->get_parameter("frame_id").as_string();
    image_pub_.publish(msg);
  }

  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  StereoExampe stereo_;
  std::shared_ptr<dai::DataOutputQueue> preview_queue_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreviewPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
