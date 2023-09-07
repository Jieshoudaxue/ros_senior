#include "qr_detector/qr_detector_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

namespace qr_detector {

QrDetectorNodelet::QrDetectorNodelet() : it_(nh_) {
    ROS_INFO("QrDetectorNodelet Constructor");
}

QrDetectorNodelet::~QrDetectorNodelet() {
    img_sub_.shutdown();
    ROS_INFO("QrDetectorNodelet Destructor");
}

void QrDetectorNodelet::onInit() {
    nh_ = getNodeHandle();
    qr_pub_ = nh_.advertise<std_msgs::String>("/qr_codes", 10, 
                                            std::bind(&QrDetectorNodelet::DownstreamConnectCb, this),
                                            std::bind(&QrDetectorNodelet::DownstreamDisconnectCb, this));
    ROS_INFO("init qr detector nodelet, nodehand name %s", nh_.getNamespace().c_str());
}

void QrDetectorNodelet::DownstreamConnectCb() {
    if (!img_sub_ && qr_pub_.getNumSubscribers() > 0) {
        img_sub_ = it_.subscribe("image", 1, &QrDetectorNodelet::ImageCb, this);
        ROS_INFO("subscribe image topic from usb_cam");
    }
}

void QrDetectorNodelet::DownstreamDisconnectCb() {
    if (qr_pub_.getNumSubscribers() == 0) {
        img_sub_.shutdown();
        ROS_INFO("unsubscibe image topic");
    }
}

void QrDetectorNodelet::ImageCb(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImageConstPtr cv_image;

    try {
        cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception %s", e.what());
        return;
    }

    QrTags qr_tags = detector_.detect(cv_image->image);
    for (auto& qr_tag : qr_tags) {
        std_msgs::String msg;
        msg.data = qr_tag.qr_msg;

        qr_pub_.publish(msg);
    }
}

}