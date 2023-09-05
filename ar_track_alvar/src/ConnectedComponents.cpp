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

#include "ar_track_alvar/ConnectedComponents.h"
#include "ar_track_alvar/Draw.h"
#include <cassert>

using namespace std;

namespace alvar
{
using namespace std;

Labeling::Labeling()
{
  cam = 0;
  thresh_param1 = 31;
  thresh_param2 = 5;
}

Labeling::~Labeling()
{
  gray.release();
  bw.release();
}

bool Labeling::CheckBorder(const std::vector<cv::Point>& contour, int width,
                           int height)
{
  bool ret = true;
  for (const auto& pt : contour)
  {
    if ((pt.x <= 1) || (pt.x >= width - 2) || (pt.y <= 1) ||
        (pt.y >= height - 2))
      ret = false;
  }
  return ret;
}

LabelingCvSeq::LabelingCvSeq() : _n_blobs(0), _min_edge(20), _min_area(25)
{
  SetOptions();
}

LabelingCvSeq::~LabelingCvSeq()
{
}

void LabelingCvSeq::SetOptions(bool _detect_pose_grayscale)
{
  detect_pose_grayscale = _detect_pose_grayscale;
}

void LabelingCvSeq::LabelSquares(cv::Mat& image, bool visualize)
{
  if (!gray.empty() && ((gray.cols != image.cols) || (gray.rows != image.rows)))
  {
    gray.release();
    bw.release();
  }
  if (gray.empty())
  {
    gray = cv::Mat(image.rows, image.cols, CV_8UC1);
    bw = cv::Mat(image.rows, image.cols, CV_8UC1);
  }

  // Convert grayscale and threshold
  if (image.channels() == 4)
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  else if (image.channels() == 3)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 1)
    image.copyTo(gray);
  else
  {
    cerr << "Unsupported image format" << endl;
  }

  cv::adaptiveThreshold(gray, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY_INV, thresh_param1, thresh_param2);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> squares;
  std::vector<std::vector<cv::Point>> square_contours;

  cv::findContours(bw, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE,
                   cv::Point(0, 0));

  for (const auto& contour : contours)
  {
    if (contour.size() < _min_edge)
    {
      continue;
    }
    std::vector<cv::Point> result;
    cv::approxPolyDP(contour, result, cv::arcLength(contour, false) * 0.035,
                     true);  // TODO: Parameters?
    if (result.size() == 4 && CheckBorder(result, image.cols, image.rows) &&
        cv::contourArea(result) > _min_area &&  // TODO check limits
        cv::isContourConvex(result))  // ttehop: Changed to 'contours' instead
                                      // of 'result'
    {
      squares.push_back(result);
      square_contours.push_back(contour);
    }
  }

  _n_blobs = squares.size();
  blob_corners.resize(_n_blobs);

  // For every detected 4-corner blob
  for (int i = 0; i < _n_blobs; ++i)
  {
    vector<Line> fitted_lines(4);
    blob_corners[i].resize(4);
    const auto& square = squares.at(i);
    const auto& square_contour = square_contours.at(i);

    for (int j = 0; j < 4; ++j)
    {
      const auto& pt0 = square.at(j);
      const auto& pt1 = square.at((j + 1) % 4);
      int k0 = -1, k1 = -1;
      for (int k = 0; k < square_contour.size(); k++)
      {
        const auto& pt2 = square_contour.at(k);
        if ((pt0.x == pt2.x) && (pt0.y == pt2.y))
          k0 = k;
        if ((pt1.x == pt2.x) && (pt1.y == pt2.y))
          k1 = k;
      }
      int len;
      if (k1 >= k0)
        len = k1 - k0 - 1;  // neither k0 nor k1 are included
      else
        len = square_contour.size() - k0 + k1 - 1;
      if (len == 0)
        len = 1;

      cv::Mat line_data = cv::Mat(1, len, CV_32FC2);
      for (int l = 0; l < len; l++)
      {
        int ll = (k0 + l + 1) % square_contour.size();
        const auto& p = square_contour.at(ll);
        cv::Point2f pp;
        pp.x = float(p.x);
        pp.y = float(p.y);

        // Undistort
        if (cam)
          cam->Undistort(pp);

        line_data.at<cv::Point2f>(0, l) = pp;
      }

      // Fit edge and put to vector of edges
      cv::Vec4f params;

      // TODO: The detect_pose_grayscale is still under work...
      /*
      if (detect_pose_grayscale &&
          (pt0->x > 3) && (pt0->y > 3) &&
          (pt0->x < (gray->width-4)) &&
          (pt0->y < (gray->height-4)))
      {
          // ttehop: Grayscale experiment
          FitLineGray(line_data, params, gray);
      }
      */
      cv::fitLine(line_data, params, cv::DIST_L2, 0, 0.01, 0.01);

      // cvFitLine(line_data, CV_DIST_L2, 0, 0.01, 0.01, params);
      ////cvFitLine(line_data, CV_DIST_HUBER, 0, 0.01, 0.01, params);
      Line line = Line(params);
      if (visualize)
        DrawLine(image, line);
      fitted_lines[j] = line;

      line_data.release();
    }

    // Calculated four intersection points
    for (size_t j = 0; j < 4; ++j)
    {
      PointDouble intc =
          Intersection(fitted_lines[j], fitted_lines[(j + 1) % 4]);

      // TODO: Instead, test OpenCV find corner in sub-pix...
      // CvPoint2D32f pt = cvPoint2D32f(intc.x, intc.y);
      // cvFindCornerSubPix(gray, &pt,
      //                   1, cvSize(3,3), cvSize(-1,-1),
      //                   cvTermCriteria(
      //                   CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,10,1e-4));

      // TODO: Now there is a wierd systematic 0.5 pixel error that is fixed
      // here...
      // intc.x += 0.5;
      // intc.y += 0.5;

      if (cam)
        cam->Distort(intc);

      // TODO: Should we make this always counter-clockwise or clockwise?
      /*
      if (image->origin && j == 1) blob_corners[i][3] = intc;
      else if (image->origin && j == 3) blob_corners[i][1] = intc;
      else blob_corners[i][j] = intc;
      */
      blob_corners[i][j] = intc;
    }
    if (visualize)
    {
      for (size_t j = 0; j < 4; ++j)
      {
        PointDouble& intc = blob_corners[i][j];
        if (j == 0)
          cv::circle(image, cv::Point(int(intc.x), int(intc.y)), 5,
                     CV_RGB(255, 255, 255));
        if (j == 1)
          cv::circle(image, cv::Point(int(intc.x), int(intc.y)), 5,
                     CV_RGB(255, 0, 0));
        if (j == 2)
          cv::circle(image, cv::Point(int(intc.x), int(intc.y)), 5,
                     CV_RGB(0, 255, 0));
        if (j == 3)
          cv::circle(image, cv::Point(int(intc.x), int(intc.y)), 5,
                     CV_RGB(0, 0, 255));
      }
    }
  }
}

std::vector<std::vector<cv::Point>> LabelingCvSeq::LabelImage(cv::Mat& image,
                                                              int min_size,
                                                              bool approx)
{
  if (!gray.empty() && ((gray.cols != image.cols) || (gray.rows != image.rows)))
  {
    gray.release();
    bw.release();
  }
  if (gray.empty())
  {
    gray = cv::Mat(image.rows, image.cols, CV_8UC1);
    bw = cv::Mat(image.rows, image.cols, CV_8UC1);
  }

  // Convert grayscale and threshold
  if (image.channels() == 4)
    cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
  else if (image.channels() == 3)
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  else if (image.channels() == 1)
    image.copyTo(gray);
  else
  {
    cerr << "Unsupported image format" << endl;
  }

  cv::adaptiveThreshold(gray, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY_INV, thresh_param1, thresh_param2);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> squares;

  cv::findContours(bw, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE,
                   cv::Point(0, 0));

  for (const auto& contour : contours)
  {
    if (approx)
    {
      std::vector<cv::Point> result;
      cv::approxPolyDP(contour, result, cv::arcLength(contour, false) * 0.02,
                       false);  // TODO: Parameters?
      if (cv::isContourConvex(result))
      {
        squares.push_back(result);
      }
    }
    else
      squares.push_back(contour);
  }

  return squares;
}

inline int round(double x)
{
  return (x) >= 0 ? (int)((x) + 0.5) : (int)((x)-0.5);
}

template <class T>
inline T absdiff(T c1, T c2)
{
  return (c2 > c1 ? c2 - c1 : c1 - c2);
}

//#define SHOW_DEBUG
#ifdef SHOW_DEBUG
#include "highgui.h"
#endif

// TODO: This should be in LabelingCvSeq ???
void FitLineGray(cv::Mat& line_data, float params[4], cv::Mat& gray)
{
  // this very simple approach works...
  /*
  float *cx = &(params[2]);
  float *cy = &(params[3]);
  float *sx = &(params[0]);
  float *sy = &(params[1]);
  CvPoint2D32f *p1 = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, 0,
  sizeof(CvPoint2D32f)); CvPoint2D32f *p2 =
  (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, line_data->cols-1,
  sizeof(CvPoint2D32f)); *cx = p1->x; *cy = p1->y; *sx = p2->x - p1->x; *sy =
  p2->y - p1->y; return;
  */

#ifdef SHOW_DEBUG
  IplImage* tmp =
      cvCreateImage(cvSize(gray->width, gray->height), IPL_DEPTH_8U, 3);
  IplImage* tmp2 =
      cvCreateImage(cvSize(gray->width * 5, gray->height * 5), IPL_DEPTH_8U, 3);
  cvCvtColor(gray, tmp, CV_GRAY2RGB);
  cvResize(tmp, tmp2, CV_INTER_NN);
#endif

  // Discover 1st the line normal direction
  auto p1 = line_data.ptr<cv::Point2f>(0, 0);
  auto p2 = line_data.ptr<cv::Point2f>(0, line_data.cols - 1);
  double dx = +(p2->y - p1->y);
  double dy = -(p2->x - p1->x);
  if ((dx == 0) && (dy == 0))
    return;
  else if (dx == 0)
  {
    dy /= dy;
  }
  else if (dy == 0)
  {
    dx /= dx;
  }
  else if (abs(dx) > abs(dy))
  {
    dy /= dx;
    dx /= dx;
  }
  else
  {
    dx /= dy;
    dy /= dy;
  }

  // Build normal search table
  const int win_size = 5;
  const int win_mid = win_size / 2;
  const int diff_win_size = win_size - 1;
  double xx[win_size], yy[win_size];
  double dxx[diff_win_size], dyy[diff_win_size];
  xx[win_mid] = 0;
  yy[win_mid] = 0;
  for (int i = 1; i <= win_size / 2; i++)
  {
    xx[win_mid + i] = round(i * dx);
    xx[win_mid - i] = -xx[win_mid + i];
    yy[win_mid + i] = round(i * dy);
    yy[win_mid - i] = -yy[win_mid + i];
  }
  for (int i = 0; i < diff_win_size; i++)
  {
    dxx[i] = (xx[i] + xx[i + 1]) / 2;
    dyy[i] = (yy[i] + yy[i + 1]) / 2;
  }

  // Adjust the points
  for (int l = 0; l < line_data.cols; l++)
  {
    auto p = line_data.ptr<cv::Point2f>(0, l);

    double dx = 0, dy = 0, ww = 0;
    for (int i = 0; i < diff_win_size; i++)
    {
      unsigned char c1 = (unsigned char)gray.at<uchar>(
          int((p->y + yy[i]) * gray.cols + (p->x + xx[i])));
      unsigned char c2 = (unsigned char)gray.at<uchar>(
          int((p->y + yy[i + 1]) * gray.cols + (p->x + xx[i + 1])));
#ifdef SHOW_DEBUG
      cvCircle(tmp2, cv::Point((p->x + xx[i]) * 5 + 2, (p->y + yy[i]) * 5 + 2),
               0, CV_RGB(0, 0, 255));
      cvCircle(
          tmp2,
          cv::Point((p->x + xx[i + 1]) * 5 + 2, (p->y + yy[i + 1]) * 5 + 2), 0,
          CV_RGB(0, 0, 255));
#endif
      double w = absdiff(c1, c2);
      dx += dxx[i] * w;
      dy += dyy[i] * w;
      ww += w;
    }
    if (ww > 0)
    {
      dx /= ww;
      dy /= ww;
    }
#ifdef SHOW_DEBUG
    cvLine(tmp2, cv::Point(p->x * 5 + 2, p->y * 5 + 2),
           cv::Point((p->x + dx) * 5 + 2, (p->y + dy) * 5 + 2),
           CV_RGB(0, 255, 0));
    p->x += float(dx);
    p->y += float(dy);
    cvCircle(tmp2, cv::Point(p->x * 5 + 2, p->y * 5 + 2), 0, CV_RGB(255, 0, 0));
#else
    p->x += float(dx);
    p->y += float(dy);
#endif
  }

#ifdef SHOW_DEBUG
  cvNamedWindow("tmp");
  cvShowImage("tmp", tmp2);
  cvWaitKey(0);
  cvReleaseImage(&tmp);
  cvReleaseImage(&tmp2);
#endif
}

}  // namespace alvar
