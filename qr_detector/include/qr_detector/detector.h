#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "zbar.h"

namespace qr_detector {

struct Tag {
  std::string message;
  std::vector<cv::Point> polygon;
};

using Tags = std::vector<Tag>;

/**
 * Main QR codes detector class
 */
class Detector {
 public:
  Detector();
  ~Detector() = default;

  /**
   * Detects tags in image.
   *
   * @param[in] image - image with qr tags
   * @param[in] timeout - max time for tags detection in ms
   */
  Tags detect(const cv::Mat& image, size_t timeout = 100);

 protected:
  zbar::ImageScanner scanner_;
};

}
