/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include "ar_track_alvar/Alvar.h"
#include "ar_track_alvar/Camera.h"
#include "ar_track_alvar/FileFormatUtils.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace std;

namespace alvar
{
using namespace std;

void ProjPoints::Reset()
{
  object_points.clear();
  image_points.clear();
  point_counts.clear();
}

// TODO: Does it matter what the etalon_square_size is???
bool ProjPoints::AddPointsUsingChessboard(const cv::Mat& image,
                                          double etalon_square_size,
                                          int etalon_rows, int etalon_columns,
                                          bool visualize)
{
  if (image.cols == 0)
    return false;
  cv::Mat gray = cv::Mat(image.rows, image.cols, CV_8UC1);
  std::vector<cv::Point2f> corners;
  if (image.channels() == 1)
    image.copyTo(gray);
  else
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);

  width = image.cols;
  height = image.rows;

  std::size_t point_count = 0;

  int pattern_was_found = cv::findChessboardCorners(
      gray, cv::Size(etalon_rows, etalon_columns), corners);
  point_count = !pattern_was_found ? 0 : corners.size();
  if (point_count > 0)
  {
    cv::cornerSubPix(
        gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 10,
                         0.01f));
    for (int i = 0; i < point_count; i++)
    {
      cv::Point3d po;
      cv::Point2d pi;
      po.x = etalon_square_size * (i % etalon_rows);
      po.y = etalon_square_size * (i / etalon_rows);
      po.z = 0;
      pi.x = corners[i].x;
      pi.y = corners[i].y;
      object_points.push_back(po);
      image_points.push_back(pi);
    }
    point_counts.push_back(point_count);
  }
  if (visualize)
  {
    cv::drawChessboardCorners(image, cv::Size(etalon_rows, etalon_columns),
                              corners, false);
  }
  corners.clear();
  gray.release();
  if (point_count > 0)
    return true;
  return false;
}

bool ProjPoints::AddPointsUsingMarkers(vector<PointDouble>& marker_corners,
                                       vector<PointDouble>& marker_corners_img,
                                       cv::Mat& image)
{
  width = image.cols;
  height = image.rows;
  if ((marker_corners.size() == marker_corners_img.size()) &&
      (marker_corners.size() == 4))
  {
    for (size_t p = 0; p < marker_corners.size(); p++)
    {
      cv::Point3d po;
      cv::Point2d pi;
      po.x = marker_corners[p].x;
      po.y = marker_corners[p].y;
      po.z = 0;
      pi.x = marker_corners_img[p].x;
      pi.y = marker_corners_img[p].y;
      object_points.push_back(po);
      image_points.push_back(pi);
    }
    point_counts.push_back(marker_corners.size());
  }

  return true;
}

Camera::Camera()
{
  calib_K = cv::Mat(3, 3, CV_64F, calib_K_data);
  calib_D = cv::Mat(4, 1, CV_64F, calib_D_data);
  memset(calib_K_data, 0, sizeof(double) * 3 * 3);
  memset(calib_D_data, 0, sizeof(double) * 4);
  calib_K_data[0][0] = 550;  // Just some focal length by default
  calib_K_data[1][1] = 550;  // Just some focal length by default
  calib_K_data[0][2] = 320;
  calib_K_data[1][2] = 240;
  calib_K_data[2][2] = 1;
  calib_x_res = 640;
  calib_y_res = 480;
  x_res = 640;
  y_res = 480;
}

Camera::Camera(ros::NodeHandle& n, std::string cam_info_topic) : n_(n)
{
  calib_K = cv::Mat(3, 3, CV_64F, calib_K_data);
  calib_D = cv::Mat(4, 1, CV_64F, calib_D_data);
  memset(calib_K_data, 0, sizeof(double) * 3 * 3);
  memset(calib_D_data, 0, sizeof(double) * 4);
  calib_K_data[0][0] = 550;  // Just some focal length by default
  calib_K_data[1][1] = 550;  // Just some focal length by default
  calib_K_data[0][2] = 320;
  calib_K_data[1][2] = 240;
  calib_K_data[2][2] = 1;
  calib_x_res = 640;
  calib_y_res = 480;
  x_res = 640;
  y_res = 480;
  cameraInfoTopic_ = cam_info_topic;
  ROS_INFO("Subscribing to info topic");
  sub_ = n_.subscribe(cameraInfoTopic_, 1, &Camera::camInfoCallback, this);
  getCamInfo_ = false;
}

//
// Camera::Camera(int w, int h) {
//	calib_K = cvMat(3, 3, CV_64F, calib_K_data);
//	calib_D = cvMat(4, 1, CV_64F, calib_D_data);
//	memset(calib_K_data,0,sizeof(double)*3*3);
//	memset(calib_D_data,0,sizeof(double)*4);
//	calib_K_data[0][0] = w/2;
//	calib_K_data[1][1] = w/2;
//	calib_K_data[0][2] = w/2;
//	calib_K_data[1][2] = h/2;
//	calib_K_data[2][2] = 1;
//	calib_x_res = w;
//	calib_y_res = h;
//	x_res = w;
//	y_res = h;
//}

void Camera::SetSimpleCalib(int _x_res, int _y_res, double f_fac)
{
  memset(calib_K_data, 0, sizeof(double) * 3 * 3);
  memset(calib_D_data, 0, sizeof(double) * 4);
  calib_K_data[0][0] = _x_res * f_fac;  // Just some focal length by default
  calib_K_data[1][1] = _x_res * f_fac;  // Just some focal length by default
  calib_K_data[0][2] = _x_res / 2;
  calib_K_data[1][2] = _y_res / 2;
  calib_K_data[2][2] = 1;
  calib_x_res = _x_res;
  calib_y_res = _y_res;
}

bool Camera::LoadCalibXML(const char* calibfile)
{
  TiXmlDocument document;
  if (!document.LoadFile(calibfile))
    return false;
  TiXmlElement* xml_root = document.RootElement();

  return xml_root->QueryIntAttribute("width", &calib_x_res) == TIXML_SUCCESS &&
         xml_root->QueryIntAttribute("height", &calib_y_res) == TIXML_SUCCESS &&
         FileFormatUtils::parseXMLMatrix(xml_root->FirstChildElement("intrinsic"
                                                                     "_matrix"),
                                         calib_K) &&
         FileFormatUtils::parseXMLMatrix(xml_root->FirstChildElement("distortio"
                                                                     "n"),
                                         calib_D);
}

bool Camera::LoadCalibOpenCV(const char* calibfile)
{
  cv::FileStorage fs;
  bool success = fs.open(calibfile, cv::FileStorage::READ);

  if (success)
  {
    // K Intrinsic
    cv::FileNode intrinsic_mat_node = fs["intrinsic_matrix"];
    cv::Mat intrinsic_mat;
    cv::read(intrinsic_mat_node, intrinsic_mat);
    calib_K.at<double>(0, 0) = intrinsic_mat.at<double>(0, 0);
    calib_K.at<double>(0, 1) = intrinsic_mat.at<double>(0, 1);
    calib_K.at<double>(0, 2) = intrinsic_mat.at<double>(0, 2);
    calib_K.at<double>(1, 0) = intrinsic_mat.at<double>(1, 0);
    calib_K.at<double>(1, 1) = intrinsic_mat.at<double>(1, 1);
    calib_K.at<double>(1, 2) = intrinsic_mat.at<double>(1, 2);
    calib_K.at<double>(2, 0) = intrinsic_mat.at<double>(2, 0);
    calib_K.at<double>(2, 1) = intrinsic_mat.at<double>(2, 1);
    calib_K.at<double>(2, 2) = intrinsic_mat.at<double>(2, 2);

    // D Distortion
    cv::FileNode dist_mat_node = fs["distortion"];
    cv::Mat dist_mat;
    cv::read(dist_mat_node, dist_mat);
    calib_D.at<double>(0, 0) = dist_mat.at<double>(0, 0);
    calib_D.at<double>(1, 0) = dist_mat.at<double>(1, 0);
    calib_D.at<double>(2, 0) = dist_mat.at<double>(2, 0);
    calib_D.at<double>(3, 0) = dist_mat.at<double>(3, 0);

    // Resolution
    cv::FileNode width_node = fs["width"];
    cv::FileNode height_node = fs["height"];
    cv::read(width_node, calib_x_res, 0);
    cv::read(height_node, calib_y_res, 0);
    fs.release();
    return true;
  }
  return false;
}

void Camera::SetCameraInfo(const sensor_msgs::CameraInfo& camInfo)
{
  cam_info_ = camInfo;

  calib_x_res = cam_info_.width;
  calib_y_res = cam_info_.height;
  x_res = calib_x_res;
  y_res = calib_y_res;

  calib_K.at<double>(0, 0) = cam_info_.K[0];
  calib_K.at<double>(0, 1) = cam_info_.K[1];
  calib_K.at<double>(0, 2) = cam_info_.K[2];
  calib_K.at<double>(1, 0) = cam_info_.K[3];
  calib_K.at<double>(1, 1) = cam_info_.K[4];
  calib_K.at<double>(1, 2) = cam_info_.K[5];
  calib_K.at<double>(2, 0) = cam_info_.K[6];
  calib_K.at<double>(2, 1) = cam_info_.K[7];
  calib_K.at<double>(2, 2) = cam_info_.K[8];

  if (cam_info_.D.size() >= 4)
  {
    calib_D.at<double>(0, 0) = cam_info_.D[0];
    calib_D.at<double>(1, 0) = cam_info_.D[1];
    calib_D.at<double>(2, 0) = cam_info_.D[2];
    calib_D.at<double>(3, 0) = cam_info_.D[3];
  }
  else
  {
    calib_D.at<double>(0, 0) = 0;
    calib_D.at<double>(1, 0) = 0;
    calib_D.at<double>(2, 0) = 0;
    calib_D.at<double>(3, 0) = 0;
  }
}

void Camera::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if (!getCamInfo_)
  {
    SetCameraInfo(*cam_info);
    getCamInfo_ = true;
    sub_.shutdown();
  }
}

bool Camera::SetCalib(const char* calibfile, int _x_res, int _y_res,
                      FILE_FORMAT format)
{
  x_res = _x_res;
  y_res = _y_res;
  if (!calibfile)
    return false;

  bool success = false;
  switch (format)
  {
    case FILE_FORMAT_XML:
      success = LoadCalibXML(calibfile);
      break;
    case FILE_FORMAT_OPENCV:
    case FILE_FORMAT_DEFAULT:
      success = LoadCalibOpenCV(calibfile);
      break;
    default:
      // TODO: throw exception?
      break;
  };

  if (success)
  {
    // Scale matrix in case of different resolution calibration.
    // The OpenCV documentation says:
    // - If an image from camera is up-sampled/down-sampled by some factor, all
    // intrinsic camera parameters
    //   (fx, fy, cx and cy) should be scaled (multiplied/divided, respectively)
    //   by the same factor.
    // - The distortion coefficients remain the same regardless of the captured
    // image resolution.
    if ((calib_x_res != x_res) || (calib_y_res != y_res))
    {
      calib_K_data[0][0] *= (double(x_res) / double(calib_x_res));
      calib_K_data[0][2] *= (double(x_res) / double(calib_x_res));
      calib_K_data[1][1] *= (double(y_res) / double(calib_y_res));
      calib_K_data[1][2] *= (double(y_res) / double(calib_y_res));
    }
  }
  return success;
}

bool Camera::SaveCalibXML(const char* calibfile)
{
  TiXmlDocument document;
  document.LinkEndChild(new TiXmlDeclaration("1.0", "UTF-8", "no"));
  document.LinkEndChild(new TiXmlElement("camera"));
  TiXmlElement* xml_root = document.RootElement();
  xml_root->SetAttribute("width", calib_x_res);
  xml_root->SetAttribute("height", calib_y_res);
  xml_root->LinkEndChild(
      FileFormatUtils::createXMLMatrix("intrinsic_matrix", calib_K));
  xml_root->LinkEndChild(
      FileFormatUtils::createXMLMatrix("distortion", calib_D));
  return document.SaveFile(calibfile);
}

bool Camera::SaveCalibOpenCV(const char* calibfile)
{
  cv::FileStorage fs;
  bool success = fs.open(calibfile, cv::FileStorage::WRITE);
  if (success)
  {
    cv::write(fs, "intrinsic_matrix", calib_K);
    cv::write(fs, "distortion", calib_D);
    cv::write(fs, "width", calib_x_res);
    cv::write(fs, "height", calib_y_res);
    fs.release();
    return true;
  }
  return false;
}

bool Camera::SaveCalib(const char* calibfile, FILE_FORMAT format)
{
  if (!calibfile)
    return false;

  switch (format)
  {
    case FILE_FORMAT_XML:
      return SaveCalibXML(calibfile);
    case FILE_FORMAT_OPENCV:
    case FILE_FORMAT_DEFAULT:
      return SaveCalibOpenCV(calibfile);
    default:
      return false;
  };
}

void Camera::Calibrate(ProjPoints& pp)
{
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  object_points.emplace_back();
  for (const auto& point : pp.object_points)
  {
    object_points[0].push_back(point);
  }
  image_points.emplace_back();
  for (const auto& point : pp.image_points)
  {
    image_points[0].push_back(point);
  }
  cv::calibrateCamera(object_points, image_points,
                      cv::Size(pp.width, pp.height), calib_K, calib_D,
                      cv::Mat(), cv::Mat(), cv::CALIB_USE_INTRINSIC_GUESS);

  calib_x_res = pp.width;
  calib_y_res = pp.height;
}

void Camera::SetRes(int _x_res, int _y_res)
{
  x_res = _x_res;
  y_res = _y_res;
  // Scale matrix in case of different resolution calibration.
  // The OpenCV documentation says:
  // - If an image from camera is up-sampled/down-sampled by some factor, all
  // intrinsic camera parameters
  //   (fx, fy, cx and cy) should be scaled (multiplied/divided, respectively)
  //   by the same factor.
  // - The distortion coefficients remain the same regardless of the captured
  // image resolution.
  if ((calib_x_res != x_res) || (calib_y_res != y_res))
  {
    calib_K_data[0][0] *= (double(x_res) / double(calib_x_res));
    calib_K_data[0][2] *= (double(x_res) / double(calib_x_res));
    calib_K_data[1][1] *= (double(y_res) / double(calib_y_res));
    calib_K_data[1][2] *= (double(y_res) / double(calib_y_res));
  }
}

// TODO: Better approach for this...
// Note, the proj_matrix[8] is now negated. This is due to the fact
// that with OpenCV and OpenGL projection matrices both y and z
// should be mirrored. All other components are
void Camera::GetOpenglProjectionMatrix(double proj_matrix[16], const int width,
                                       const int height,
                                       const float far_clip /*= 1000.0f*/,
                                       const float near_clip /*= 0.1f*/)
{
  proj_matrix[0] = 2.0f * calib_K_data[0][0] / float(width);
  proj_matrix[1] = 0;
  proj_matrix[2] = 0;
  proj_matrix[3] = 0;
  proj_matrix[4] = 2.0f * calib_K_data[0][1] / float(width);  // skew
  proj_matrix[5] = 2.0f * calib_K_data[1][1] / float(height);
  proj_matrix[6] = 0;
  proj_matrix[7] = 0;
  // proj_matrix[8]	= (2.0f * calib_K_data[0][2] / float(width)) - 1.0f;
  proj_matrix[8] = -(2.0f * calib_K_data[0][2] / float(width)) + 1.0f;
  proj_matrix[9] = (2.0f * calib_K_data[1][2] / float(height)) - 1.0f;
  proj_matrix[10] = -(far_clip + near_clip) / (far_clip - near_clip);
  proj_matrix[11] = -1.0f;
  proj_matrix[12] = 0;
  proj_matrix[13] = 0;
  proj_matrix[14] = -2.0f * far_clip * near_clip / (far_clip - near_clip);
  proj_matrix[15] = 0;
}

void Camera::SetOpenglProjectionMatrix(double proj_matrix[16], const int width,
                                       const int height)
{
  x_res = width;
  y_res = height;
  calib_x_res = width;
  calib_y_res = height;
  calib_K_data[0][0] = proj_matrix[0] * float(width) / 2.0f;
  calib_K_data[0][1] = proj_matrix[4] * float(width) / 2.0f;
  calib_K_data[1][1] = proj_matrix[5] * float(height) / 2.0f;
  // calib_K_data[0][2] = (proj_matrix[8] + 1.0f) * float(width) / 2.0f;
  calib_K_data[0][2] =
      (-proj_matrix[8] + 1.0f) * float(width) / 2.0f;  // Is this ok?
  calib_K_data[1][2] = (proj_matrix[9] + 1.0f) * float(height) / 2.0f;
  calib_K_data[2][2] = 1;
}

void Camera::Undistort(PointDouble& point)
{
  /*
    // focal length
    double ifx = 1./cvmGet(&calib_K, 0, 0);
    double ify = 1./cvmGet(&calib_K, 1, 1);

    // principal point
    double cx = cvmGet(&calib_K, 0, 2);
    double cy = cvmGet(&calib_K, 1, 2);

    // distortion coeffs
    double* k = calib_D.data.db;

    // compensate distortion iteratively
    double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
    for(int j = 0; j < 5; j++){
      double r2 = x*x + y*y;
      double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
      double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
      double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
      x = (x0 - delta_x)*icdist;
      y = (y0 - delta_y)*icdist;
    }
    // apply compensation
    point.x = x/ifx + cx;
    point.y = y/ify + cy;
  */
}

void Camera::Undistort(vector<PointDouble>& points)
{
  /*
    // focal length
    double ifx = 1./cvmGet(&calib_K, 0, 0);
    double ify = 1./cvmGet(&calib_K, 1, 1);

    // principal point
    double cx = cvmGet(&calib_K, 0, 2);
    double cy = cvmGet(&calib_K, 1, 2);

    // distortion coeffs
    double* k = calib_D.data.db;

    for(unsigned int i = 0; i < points.size(); i++)
    {
      // compensate distortion iteratively
      double x = (points[i].x - cx)*ifx, y = (points[i].y - cy)*ify, x0 = x, y0
    = y; for(int j = 0; j < 5; j++){ double r2 = x*x + y*y; double icdist
    = 1./(1 + k[0]*r2 + k[1]*r2*r2); double delta_x = 2*k[2]*x*y + k[3]*(r2 +
    2*x*x); double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y; x = (x0 -
    delta_x)*icdist; y = (y0 - delta_y)*icdist;
      }
      // apply compensation
      points[i].x = x/ifx + cx;
      points[i].y = y/ify + cy;
    }
  */
}

void Camera::Undistort(cv::Point2f& point)
{
  /*
    // focal length
    double ifx = 1./cvmGet(&calib_K, 0, 0);
    double ify = 1./cvmGet(&calib_K, 1, 1);

    // principal point
    double cx = cvmGet(&calib_K, 0, 2);
    double cy = cvmGet(&calib_K, 1, 2);

    // distortion coeffs
    double* k = calib_D.data.db;


    // compensate distortion iteratively
    double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
    for(int j = 0; j < 5; j++){
      double r2 = x*x + y*y;
      double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
      double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
      double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
      x = (x0 - delta_x)*icdist;
      y = (y0 - delta_y)*icdist;
    }
    // apply compensation
    point.x = float(x/ifx + cx);
    point.y = float(y/ify + cy);
  */
}

/*
  template<class PointType>
  void Undistort(PointType& point) {
    // focal length
    double ifx = 1./cvmGet(&calib_K, 0, 0);
    double ify = 1./cvmGet(&calib_K, 1, 1);

    // principal point
    double cx = cvmGet(&calib_K, 0, 2);
    double cy = cvmGet(&calib_K, 1, 2);

    // distortion coeffs
    double* k = calib_D.data.db;

    // compensate distortion iteratively
    double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
    for(int j = 0; j < 5; j++){
      double r2 = x*x + y*y;
      double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
      double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
      double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
      x = (x0 - delta_x)*icdist;
      y = (y0 - delta_y)*icdist;
    }
    // apply compensation
    point.x = x/ifx + cx;
    point.y = y/ify + cy;
  }
*/

void Camera::Distort(vector<PointDouble>& points)
{
  /*
    double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
    double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
    double _fx = 1./fx, _fy = 1./fy;
    double* k = calib_D.data.db;

    double k1 = k[0], k2 = k[1];
    double p1 = k[2], p2 = k[3];

    for(unsigned int i = 0; i < points.size(); i++)
    {
      // Distort
      double y = (points[i].y - v0)*_fy;
      double y2 = y*y;
      double _2p1y = 2*p1*y;
      double _3p1y2 = 3*p1*y2;
      double p2y2 = p2*y2;

      double x = (points[i].x - u0)*_fx;
      double x2 = x*x;
      double r2 = x2 + y2;
      double d = 1 + (k1 + k2*r2)*r2;

      points[i].x = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
      points[i].y = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
    }
  */
}

void Camera::Distort(PointDouble& point)
{
  /*
    double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
    double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
    double _fx = 1./fx, _fy = 1./fy;
    double* k = calib_D.data.db;

    double k1 = k[0], k2 = k[1];
    double p1 = k[2], p2 = k[3];

    // Distort
    double y = (point.y - v0)*_fy;
    double y2 = y*y;
    double _2p1y = 2*p1*y;
    double _3p1y2 = 3*p1*y2;
    double p2y2 = p2*y2;

    double x = (point.x - u0)*_fx;
    double x2 = x*x;
    double r2 = x2 + y2;
    double d = 1 + (k1 + k2*r2)*r2;

    point.x = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
    point.y = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
  */
}

void Camera::Distort(cv::Point2f& point)
{
  /*
    double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
    double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
    double _fx = 1./fx, _fy = 1./fy;
    double* k = calib_D.data.db;

    double k1 = k[0], k2 = k[1];
    double p1 = k[2], p2 = k[3];

    // Distort
    double y = (point.y - v0)*_fy;
    double y2 = y*y;
    double _2p1y = 2*p1*y;
    double _3p1y2 = 3*p1*y2;
    double p2y2 = p2*y2;

    double x = (point.x - u0)*_fx;
    double x2 = x*x;
    double r2 = x2 + y2;
    double d = 1 + (k1 + k2*r2)*r2;

    point.x = float(fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0);
    point.y = float(fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0);
  */
}

void Camera::CalcExteriorOrientation(const vector<cv::Point3d>& pw,
                                     const vector<PointDouble>& pi,
                                     cv::Mat& rodriques, cv::Mat& tra) const
{
  vector<cv::Point2d> pi2;
  for (const auto& point : pi)
  {
    pi2.push_back(point);
  }

  tra.setTo(cv::Scalar::all(0));
  rodriques.setTo(cv::Scalar::all(0));
  cv::solvePnP(pw, pi2, calib_K, cv::Mat(), rodriques, tra, false,
               cv::SOLVEPNP_ITERATIVE);
}

void Camera::CalcExteriorOrientation(const vector<PointDouble>& pw,
                                     const vector<PointDouble>& pi,
                                     cv::Mat& rodriques, cv::Mat& tra) const
{
  vector<cv::Point3d> pw3;
  for (const auto& point : pw)
  {
    pw3.emplace_back(cv::Point3d(point.x, point.y, 0));
  }

  CalcExteriorOrientation(pw3, pi, rodriques, tra);
}

void Camera::CalcExteriorOrientation(const vector<PointDouble>& pw,
                                     const vector<PointDouble>& pi,
                                     Pose* pose) const
{
  cv::Mat ext_rodriques_mat = cv::Mat(3, 1, CV_64F);
  cv::Mat ext_translate_mat = cv::Mat(3, 1, CV_64F);
  CalcExteriorOrientation(pw, pi, ext_rodriques_mat, ext_translate_mat);
  pose->SetRodriques(ext_rodriques_mat);
  pose->SetTranslation(ext_translate_mat);
}

void Camera::ProjectPoints(vector<cv::Point3d>& pw, Pose* pose,
                           vector<cv::Point2d>& pi) const
{
  cv::Mat ext_rodriques_mat = cv::Mat(3, 1, CV_64F);
  cv::Mat ext_translate_mat = cv::Mat(3, 1, CV_64F);
  pose->GetRodriques(ext_rodriques_mat);
  pose->GetTranslation(ext_translate_mat);
  cv::Mat object_points = cv::Mat((int)pw.size(), 1, CV_32FC3);
  cv::Mat image_points = cv::Mat((int)pi.size(), 1, CV_32FC2);
  for (size_t i = 0; i < pw.size(); i++)
  {
    object_points.at<float>(i * 3 + 0) = (float)pw[i].x;
    object_points.at<float>(i * 3 + 1) = (float)pw[i].y;
    object_points.at<float>(i * 3 + 2) = (float)pw[i].z;
  }
  cv::projectPoints(object_points, ext_translate_mat, ext_translate_mat,
                    calib_K, calib_D, image_points);
  for (size_t i = 0; i < pw.size(); i++)
  {
    pi[i].x = image_points.at<float>(i * 2 + 0);
    pi[i].y = image_points.at<float>(i * 2 + 1);
  }
  object_points.release();
  image_points.release();
}

void Camera::ProjectPoints(const cv::Mat& object_points,
                           const cv::Mat& rotation_vector,
                           const cv::Mat& translation_vector,
                           cv::Mat& image_points) const
{
  // Project points
  cv::projectPoints(object_points, rotation_vector, translation_vector, calib_K,
                    calib_D, image_points);
}

void Camera::ProjectPoints(const cv::Mat& object_points, const Pose* pose,
                           cv::Mat& image_points) const
{
  cv::Mat ext_rodriques_mat = cv::Mat(3, 1, CV_64F);
  cv::Mat ext_translate_mat = cv::Mat(3, 1, CV_64F);
  pose->GetRodriques(ext_rodriques_mat);
  pose->GetTranslation(ext_translate_mat);
  cv::projectPoints(object_points, ext_rodriques_mat, ext_translate_mat,
                    calib_K, calib_D, image_points);
}

void Camera::ProjectPoints(const cv::Mat& object_points, double gl[16],
                           cv::Mat& image_points) const
{
  double glm[4][4] = {
    gl[0], gl[4], gl[8],  gl[12], gl[1], gl[5], gl[9],  gl[13],
    gl[2], gl[6], gl[10], gl[14], gl[3], gl[7], gl[11], gl[15],
  };
  cv::Mat glm_mat = cv::Mat(4, 4, CV_64F, glm);

  // For some reason we need to mirror both y and z ???
  double cv_mul_data[4][4];
  cv::Mat cv_mul = cv::Mat(4, 4, CV_64F, cv_mul_data);
  cv::setIdentity(cv_mul);
  cv_mul.at<double>(1, 1) = -1;
  cv_mul.at<double>(2, 2) = -1;
  glm_mat = cv_mul * glm_mat;

  // Rotation
  Rotation r;
  r.SetMatrix(glm_mat);
  double rod[3];
  cv::Mat rod_mat = cv::Mat(3, 1, CV_64F, rod);
  r.GetRodriques(rod_mat);
  // Translation
  double tra[3] = { glm[0][3], glm[1][3], glm[2][3] };
  cv::Mat tra_mat = cv::Mat(3, 1, CV_64F, tra);
  // Project points
  ProjectPoints(object_points, rod_mat, tra_mat, image_points);
}

void Camera::ProjectPoint(const cv::Point3d& pw, const Pose* pose,
                          cv::Point2d& pi) const
{
  float object_points_data[3] = { (float)pw.x, (float)pw.y, (float)pw.z };
  float image_points_data[2] = { 0 };
  cv::Mat object_points = cv::Mat(1, 1, CV_32FC3, object_points_data);
  cv::Mat image_points = cv::Mat(1, 1, CV_32FC2, image_points_data);
  ProjectPoints(object_points, pose, image_points);
  pi.x = image_points.at<float>(0);
  pi.y = image_points.at<float>(1);
}

void Camera::ProjectPoint(const cv::Point3f& pw, const Pose* pose,
                          cv::Point2f& pi) const
{
  float object_points_data[3] = { (float)pw.x, (float)pw.y, (float)pw.z };
  float image_points_data[2] = { 0 };
  cv::Mat object_points = cv::Mat(1, 1, CV_32FC3, object_points_data);
  cv::Mat image_points = cv::Mat(1, 1, CV_32FC2, image_points_data);
  ProjectPoints(object_points, pose, image_points);
  pi.x = image_points.at<float>(0);
  pi.y = image_points.at<float>(1);
}

Homography::Homography()
{
  H = cv::Mat(3, 3, CV_64F);
}

void Homography::Find(const vector<PointDouble>& pw,
                      const vector<PointDouble>& pi)
{
  assert(pw.size() == pi.size());
  int size = (int)pi.size();

  cv::Point2d srcp[size];
  cv::Point2d dstp[size];

  for (int i = 0; i < size; ++i)
  {
    srcp[i].x = pw[i].x;
    srcp[i].y = pw[i].y;

    dstp[i].x = pi[i].x;
    dstp[i].y = pi[i].y;
  }

  cv::Mat src_pts, dst_pts;
  dst_pts = cv::Mat(1, size, CV_64FC2, dstp);
  src_pts = cv::Mat(1, size, CV_64FC2, srcp);

  cv::Mat tmp = cv::findHomography(src_pts, dst_pts);
  if (tmp.elemSize() > 0)
  {
    H = tmp;
  }
}

void Homography::ProjectPoints(const vector<PointDouble>& from,
                               vector<PointDouble>& to) const
{
  int size = (int)from.size();

  cv::Point3d srcp[size];

  for (int i = 0; i < size; ++i)
  {
    srcp[i].x = from[i].x;
    srcp[i].y = from[i].y;
    srcp[i].z = 1;
  }

  cv::Point3d dstp[size];

  cv::Mat src_pts, dst_pts;
  src_pts = cv::Mat(1, size, CV_64FC3, srcp);
  dst_pts = cv::Mat(1, size, CV_64FC3, dstp);

  if (! H.empty()) {
    cv::transform(src_pts, dst_pts, H);

    to.clear();
    for (int i = 0; i < size; ++i)
    {
      PointDouble pt;
      pt.x = dstp[i].x / dstp[i].z;
      pt.y = dstp[i].y / dstp[i].z;

      to.push_back(pt);
    }
  }
}

}  // namespace alvar
