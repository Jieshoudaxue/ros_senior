#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <zbar.h>

namespace qr_detector {

struct QrTag {
  std::string qr_msg;
  std::vector<cv::Point> square_apex_vector;
};

using QrTags = std::vector<QrTag>;

class QrDetector {
public:
  QrDetector() {
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  QrTags detect(const cv::Mat& image) {
    cv::Mat gray_img;
    cv::cvtColor(image, gray_img, CV_BGR2GRAY);

    const uint64_t width = image.cols;
    const uint64_t height = image.rows;
    zbar::Image img(width, height, "Y800", gray_img.data, width*height);
    scanner_.scan(img);

    QrTags tags;
    for (auto s = img.symbol_begin(); s != img.symbol_end(); ++s) {
      QrTag tag;
      tag.qr_msg = s->get_data();
      for (int i = 0; i < s->get_location_size(); i++) {
        tag.square_apex_vector.push_back(cv::Point(s->get_location_x(i), s->get_location_y(i)));
      }

      tags.push_back(tag);
    }

    return tags;
  }

private:
  zbar::ImageScanner scanner_;
};

class QrDetectorNodelet : public nodelet::Nodelet {
public:
  QrDetectorNodelet();
  virtual ~QrDetectorNodelet();

  void onInit() override;

private:
  void DownstreamConnectCb();
  void DownstreamDisconnectCb();
  void ImageCb(const sensor_msgs::ImageConstPtr& image);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Publisher qr_pub_;
  QrDetector detector_;
};

}







