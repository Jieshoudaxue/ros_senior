#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include "qr_detector/detector.h"

namespace qr_detector {

class QrDetectorNodelet : public nodelet::Nodelet {
 public:
  QrDetectorNodelet();
  virtual ~QrDetectorNodelet();

private:
  void onInit() override;
  void connectCallback();
  void disconnectCallback();
  void imageCallback(const sensor_msgs::ImageConstPtr& image);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_subscriber_;
  ros::Publisher tags_publisher_;
  Detector detector_;
};

}
