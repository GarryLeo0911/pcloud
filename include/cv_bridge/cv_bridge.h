#pragma once

// Minimal cv_bridge stub to allow building without system cv_bridge.
// This is intentionally small and only implements the pieces used by this repo.

#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace cv_bridge {

struct CvImage {
  std_msgs::msg::Header header;
  std::string encoding;
  cv::Mat image;

  CvImage() = default;
  CvImage(const std_msgs::msg::Header &h, const std::string &enc, const cv::Mat &img)
  : header(h), encoding(enc), image(img) {}

  void toImageMsg(sensor_msgs::msg::Image &out) const {
    out.header = header;
    out.height = image.rows;
    out.width = image.cols;
    out.encoding = encoding;
    out.is_bigendian = 0;
    out.step = static_cast<decltype(out.step)>(image.step);
    size_t bytes = image.total() * image.elemSize();
    out.data.resize(bytes);
    if (bytes) memcpy(out.data.data(), image.data, bytes);
  }
};

using CvImagePtr = std::shared_ptr<CvImage>;

inline CvImagePtr toCvCopy(const sensor_msgs::msg::Image::SharedPtr &msg, const std::string &encoding) {
  auto p = std::make_shared<CvImage>();
  p->header = msg->header;
  p->encoding = encoding;
  // Attempt a best-effort conversion based on encoding
  int type = CV_8UC1;
  if (encoding.find("rgb") != std::string::npos || encoding.find("8UC3") != std::string::npos) type = CV_8UC3;
  if (msg->height > 0 && msg->width > 0 && !msg->data.empty()) {
    // create cv::Mat that copies the data
    p->image = cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), type);
    size_t bytes = p->image.total() * p->image.elemSize();
    memcpy(p->image.data, msg->data.data(), std::min(bytes, msg->data.size()));
  }
  return p;
}

} // namespace cv_bridge
