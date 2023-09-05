#include "qr_detector/qr_detector_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"

PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

namespace qr_detector {

QrDetectorNodelet::QrDetectorNodelet()
    : it_(nh_)
{ }

QrDetectorNodelet::~QrDetectorNodelet()
{
  img_subscriber_.shutdown();
}

void QrDetectorNodelet::onInit()
{
  nh_ = getNodeHandle();

  tags_publisher_ = nh_.advertise<std_msgs::String>("qr_codes", 10,
                                                  std::bind(&QrDetectorNodelet::connectCallback, this),
                                                  std::bind(&QrDetectorNodelet::disconnectCallback, this));

  NODELET_INFO_STREAM("Initializing nodelet... [" << nh_.getNamespace() << "]");
}

void QrDetectorNodelet::connectCallback()
{
  if (!img_subscriber_ && tags_publisher_.getNumSubscribers() > 0)
  {
    NODELET_INFO("Connecting to image topic.");
    img_subscriber_ = it_.subscribe("image", 1, &QrDetectorNodelet::imageCallback, this);
  }
}

void QrDetectorNodelet::disconnectCallback()
{
  if (tags_publisher_.getNumSubscribers() == 0)
  {
    NODELET_INFO("Unsubscribing from image topic.");
    img_subscriber_.shutdown();
  }
}

void QrDetectorNodelet::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
  cv_bridge::CvImageConstPtr cv_image;

  try {
    cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  auto tags = detector_.detect(cv_image->image, 10);
  for (auto& tag : tags)
  {
    std_msgs::String msg;
    msg.data = tag.message;
    tags_publisher_.publish(msg);
  }
}

}
