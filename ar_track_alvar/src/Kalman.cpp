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

#include <iostream>
#include <algorithm>  // for std::max
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ar_track_alvar/Kalman.h"
#include "ar_track_alvar/Util.h"
#include "ar_track_alvar/Alvar.h"

namespace alvar
{
KalmanSensorCore::KalmanSensorCore(const KalmanSensorCore& k)
{
  m = k.m;
  n = k.n;
  z = k.z.clone();
  H = k.H.clone();
  H_trans = k.H_trans.clone();
  K = k.K.clone();
  z_pred = k.z_pred.clone();
  z_residual = k.z_residual.clone();
  x_gain = k.x_gain.clone();
}

KalmanSensorCore::KalmanSensorCore(int _n, int _m)
{
  n = _n;
  m = _m;
  z = cv::Mat::zeros(m, 1, CV_64FC1);
  H = cv::Mat::zeros(m, n, CV_64FC1);
  H_trans = cv::Mat::zeros(n, m, CV_64FC1);
  K = cv::Mat::zeros(n, m, CV_64FC1);
  z_pred = cv::Mat::zeros(m, 1, CV_64FC1);
  z_residual = cv::Mat::zeros(m, 1, CV_64FC1);
  x_gain = cv::Mat::zeros(n, 1, CV_64FC1);
}

KalmanSensorCore::~KalmanSensorCore()
{
  z.release();
  H.release();
  H_trans.release();
  K.release();
  z_pred.release();
  z_residual.release();
  x_gain.release();
}

void KalmanSensorCore::update_x(const cv::Mat& x_pred, cv::Mat& x)
{
  // x = x_pred + K * (z - H*x_pred)
  z_pred = H * x_pred;
  z_residual = x_pred * -1 + z;
  x_gain = K * z_residual;
  x = x_pred * 1 + x_gain;
}

void KalmanCore::predict_x(unsigned long tick)
{
  // x_pred = F * x;
  x_pred = F * x;
}

KalmanCore::KalmanCore(const KalmanCore& s)
{
  n = s.n;
  x = s.x.clone();
  F = s.F.clone();
  x_pred = s.x_pred.clone();
  F_trans = s.F_trans.clone();
}

KalmanCore::KalmanCore(int _n)
{
  n = _n;
  x = cv::Mat::zeros(n, 1, CV_64FC1);
  F = cv::Mat::eye(n, n, CV_64FC1);
  F_trans = cv::Mat::eye(n, n, CV_64FC1);
  x_pred = cv::Mat::zeros(n, 1, CV_64FC1);
}

KalmanCore::~KalmanCore()
{
  x.release();
  F.release();
  F_trans.release();
  x_pred.release();
}

cv::Mat& KalmanCore::predict()
{
  predict_x(0);
  return x_pred;
}

cv::Mat& KalmanCore::predict_update(KalmanSensorCore* sensor)
{
  predict();
  sensor->update_x(x_pred, x);
  return x;
}

KalmanSensor::KalmanSensor(const KalmanSensor& k) : KalmanSensorCore(k)
{
  R = k.R.clone();
  R_tmp = k.R_tmp.clone();
  P_tmp = k.P_tmp.clone();
}

KalmanSensor::KalmanSensor(int n, int _m) : KalmanSensorCore(n, _m)
{
  R = cv::Mat::zeros(m, m, CV_64FC1);
  R_tmp = cv::Mat::zeros(m, m, CV_64FC1);
  P_tmp = cv::Mat::zeros(n, n, CV_64FC1);
}

KalmanSensor::~KalmanSensor()
{
  R.release();
  R_tmp.release();
  P_tmp.release();
}

void KalmanSensor::update_K(const cv::Mat& P_pred)
{
  // K = P * trans(H) * inv(H*P*trans(H) + R)
  H_trans = H.t();
  K = P_pred * H_trans;
  R_tmp = H * K;
  R_tmp = R_tmp * 1 + R;
  R_tmp = R_tmp.inv();
  R_tmp = R_tmp.inv();
  K = H_trans * R_tmp;
  K = P_pred * K;
}

void KalmanSensor::update_P(const cv::Mat& P_pred, cv::Mat& P)
{
  // P = (I - K*H) * P_pred
  P_tmp = K * H;
  cv::setIdentity(P);
  P = P_tmp * -1 + P;
  P = P * P_pred;
}

void Kalman::predict_P()
{
  // P_pred = F*P*trans(F) + Q
  F_trans = F.t();
  P_pred = P * F_trans;
  P_pred = F * P_pred;
  P_pred = P_pred * 1 + Q;
}

Kalman::Kalman(int _n) : KalmanCore(_n)
{
  prev_tick = 0;
  Q = cv::Mat::zeros(n, n, CV_64FC1);
  P = cv::Mat::zeros(n, n, CV_64FC1);
  P_pred = cv::Mat::zeros(n, n, CV_64FC1);
}

Kalman::~Kalman()
{
  Q.release();
  P.release();
  P_pred.release();
}

void Kalman::update_F(unsigned long tick)
{
  // cvSetIdentity(F);
}

cv::Mat& Kalman::predict(unsigned long tick)
{
  update_F(tick);
  predict_x(tick);
  predict_P();
  return x_pred;
}

cv::Mat& Kalman::predict_update(KalmanSensor* sensor, unsigned long tick)
{
  predict(tick);
  sensor->update_H(x_pred);
  sensor->update_K(P_pred);
  sensor->update_x(x_pred, x);
  sensor->update_P(P_pred, P);
  prev_tick = tick;
  return x;
}

double Kalman::seconds_since_update(unsigned long tick)
{
  unsigned long tick_diff = (prev_tick ? tick - prev_tick : 0);
  return ((double)tick_diff / 1000.0);
}

void KalmanSensorEkf::update_H(const cv::Mat& x_pred)
{
  // By default we update the H by calculating Jacobian numerically
  const double step = 0.000001;
  H.setTo(cv::Scalar::all(0));
  for (int i = 0; i < n; i++)
  {
    cv::Mat H_column;
    H_column = H.col(i);

    delta.setTo(cv::Scalar::all(0));
    delta.at<double>(i, 0) = step;
    x_plus = x_pred + delta;
    delta.at<double>(i, 0) = -step;
    x_minus = x_pred + delta;

    h(x_plus, z_tmp1);
    h(x_minus, z_tmp2);
    H_column = z_tmp1 - z_tmp2;
    H_column = H_column * (1.0 / (2 * step));
  }
}

void KalmanSensorEkf::update_x(const cv::Mat& x_pred, cv::Mat& x)
{
  // x = x_pred + K * (z - h(x_pred))
  h(x_pred, z_pred);
  z_residual = z_pred * -1 + z;
  x_gain = K * z_residual;
  x = x_pred * 1 + x_gain;
}

KalmanSensorEkf::KalmanSensorEkf(const KalmanSensorEkf& k) : KalmanSensor(k)
{
  delta = k.delta.clone();
  x_plus = k.x_plus.clone();
  x_minus = k.x_minus.clone();
  z_tmp1 = k.z_tmp1.clone();
  z_tmp2 = k.z_tmp2.clone();
}

KalmanSensorEkf::KalmanSensorEkf(int _n, int _m) : KalmanSensor(_n, _m)
{
  delta = cv::Mat::zeros(n, 1, CV_64FC1);
  x_plus = cv::Mat::zeros(n, 1, CV_64FC1);
  x_minus = cv::Mat::zeros(n, 1, CV_64FC1);
  z_tmp1 = cv::Mat::zeros(m, 1, CV_64FC1);
  z_tmp2 = cv::Mat::zeros(m, 1, CV_64FC1);
}

KalmanSensorEkf::~KalmanSensorEkf()
{
  delta.release();
  x_plus.release();
  x_minus.release();
  z_tmp1.release();
  z_tmp2.release();
}

void KalmanEkf::update_F(unsigned long tick)
{
  // By default we update the F by calculating Jacobian numerically
  // TODO
  double dt = (tick - prev_tick) / 1000.0;
  const double step = 0.000001;
  F.setTo(cv::Scalar::all(0));
  for (int i = 0; i < n; i++)
  {
    cv::Mat F_column;
    F_column = F.col(i);

    delta.setTo(cv::Scalar::all(0));
    delta.at<double>(i, 0) = step;
    x_plus = x + delta;
    delta.at<double>(i, 0) = -step;
    x_minus = x + delta;

    f(x_plus, x_tmp1, dt);
    f(x_minus, x_tmp2, dt);
    F_column = x_tmp1 - x_tmp2;
    F_column = F_column * (1.0 / (2 * step));
  }
}

void KalmanEkf::predict_x(unsigned long tick)
{
  double dt = (tick - prev_tick) / 1000.0;
  f(x, x_pred, dt);
}

KalmanEkf::KalmanEkf(int _n) : Kalman(_n)
{
  delta = cv::Mat::zeros(n, 1, CV_64FC1);
  x_plus = cv::Mat::zeros(n, 1, CV_64FC1);
  x_minus = cv::Mat::zeros(n, 1, CV_64FC1);
  x_tmp1 = cv::Mat::zeros(n, 1, CV_64FC1);
  x_tmp2 = cv::Mat::zeros(n, 1, CV_64FC1);
}

KalmanEkf::~KalmanEkf()
{
  delta.release();
  x_plus.release();
  x_minus.release();
  x_tmp1.release();
  x_tmp2.release();
}

void KalmanVisualize::img_matrix(const cv::Mat& mat, int top, int left)
{
  int roi_t = -top;
  int roi_b = -(img.rows - mat.rows - top);
  int roi_l = -left;
  int roi_r = -(img.cols - mat.cols - left);
  img.adjustROI(roi_t, roi_b, roi_l, roi_r);
  for (int j = 0; j < mat.rows; j++)
  {
    for (int i = 0; i < mat.cols; i++)
    {
      double d = mat.at<double>(j, i);
      if (d < 0)
        d = -d;
      double c1 = 0, c2 = 0, c3 = 0;
      if (d < 0.1)
      {
        c1 = 0 + ((d - 0.0) / (0.1 - 0.0) * (127 - 0));
      }
      else if (d < 1.0)
      {
        c1 = 127 + ((d - 0.1) / (1.0 - 0.1) * (255 - 127));
      }
      else if (d < 10.0)
      {
        c1 = 255;
        c2 = 0 + ((d - 1.0) / (10.0 - 1.0) * (255 - 0));
      }
      else if (d < 100.0)
      {
        c1 = 255;
        c2 = 255;
        c3 = 0 + ((d - 10.0) / (100.0 - 10.0) * (255 - 0));
      }
      else
      {
        c1 = 255;
        c2 = 255;
        c3 = 255;
      }
      if (d < 0)
      {  // BRG
        img.at<cv::Vec3b>(j, i) = cv::Vec3b(c3, c2, c1);
      }
      else
      {  // BGR
        img.at<cv::Vec3b>(j, i) = cv::Vec3b(c2, c1, c3);
      }
    }
  }
  img.adjustROI(-roi_t, -roi_b, -roi_l, -roi_r);
}

void KalmanVisualize::Init()
{
  n = kalman->get_n();
  m = sensor->get_m();
  int img_width = std::max(3 + n + 3 + n + 5 + m + 5,
                           1 + n + 1 + n + 1 + n + 1 + m + 1 + n + 1);
  int img_height = 1 + n + 1 + std::max(n, m + 1 + m) + 1;
  img =
      cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(64, 64, 64));
  img_legend = cv::imread("Legend.png");
  if (!img.empty())
  {
    for (img_scale = 1; img_scale < 50; img_scale++)
    {
      if (img_scale * img_width > img_legend.cols)
      {
        break;
      }
    }
    img_show = cv::Mat(cv::Size(img_width * img_scale,
                                img_legend.rows + img_height * img_scale),
                       CV_8UC3, cv::Scalar(64, 64, 64));
    int roi_bot = img_show.rows - img_legend.rows;
    int roi_right = img_show.cols - img_legend.cols;
    img_show.adjustROI(0, -roi_bot, 0, -roi_right);
    img_legend.copyTo(img_show);
    img_show.adjustROI(0, roi_bot, 0, roi_right);
    cv::namedWindow("KalmanVisualize");
  }
  else
  {
    img_scale = 1;
    img_show = cv::Mat(img_width * img_scale, img_height * img_scale, CV_8UC3,
                       cv::Scalar(64, 64, 64));
    cv::namedWindow("KalmanVisualize", 0);
  }
}

void KalmanVisualize::out_matrix(const cv::Mat& m, char* name)
{
  if (m.cols == 1)
  {
    std::cout << name << " = [";
    for (int j = 0; j < m.rows; j++)
    {
      std::cout << " " << m.at<double>(j, 0);
    }
    std::cout << "]^T" << std::endl;
  }
  else if (m.rows == 1)
  {
    std::cout << name << " = [";
    for (int i = 0; i < m.cols; i++)
    {
      std::cout << " " << m.at<double>(0, i);
    }
    std::cout << "]^T" << std::endl;
  }
  else
  {
    std::cout << name << " = [" << std::endl;
    for (int j = 0; j < m.rows; j++)
    {
      for (int i = 0; i < m.cols; i++)
      {
        std::cout << " " << m.at<double>(j, i);
      }
      std::cout << std::endl;
    }
    std::cout << "]" << std::endl;
  }
}

KalmanVisualize::KalmanVisualize(Kalman* _kalman, KalmanSensor* _sensor)
{
  kalman = _kalman;
  sensor = _sensor;
  kalman_ext = _kalman;
  sensor_ext = _sensor;
  Init();
}

KalmanVisualize::KalmanVisualize(KalmanCore* _kalman, KalmanSensorCore* _sensor)
{
  kalman = _kalman;
  sensor = _sensor;
  kalman_ext = NULL;
  sensor_ext = NULL;
  Init();
}

KalmanVisualize::~KalmanVisualize()
{
  img.release();
}

void KalmanVisualize::update_pre()
{
  img_matrix(kalman->x, 1, 1);  // 1
  if (kalman_ext && sensor_ext)
  {
    int y = std::max(2 + n, 3 + m + m);
    img_matrix(kalman_ext->P, 1, y);  // n
  }
}

void KalmanVisualize::update_post()
{
  img_matrix(kalman->F, 3, 1);               // n
  img_matrix(kalman->x_pred, 4 + n, 1);      // 1
  img_matrix(sensor->H, 6 + n, 1);           // n
  img_matrix(sensor->z_pred, 7 + n + n, 1);  // 1
  img_matrix(sensor->z, 7 + n + n, 2 + m);
  img_matrix(sensor->z_residual, 9 + n + n, 1);   // 1
  img_matrix(sensor->K, 11 + n + n, 1);           // m
  img_matrix(sensor->x_gain, 12 + n + n + m, 1);  // 1
  img_matrix(kalman->x, 14 + n + n + m, 1);       // 1
  if (kalman_ext && sensor_ext)
  {
    int y = std::max(2 + n, 3 + m + m);
    img_matrix(kalman_ext->Q, 2 + n, y);             // n
    img_matrix(kalman_ext->P_pred, 3 + n + n, y);    // n
    img_matrix(sensor_ext->R, 4 + n + n + n, y);     // m
    img_matrix(kalman_ext->P, img.cols - 1 - n, y);  // n
  }
  if (!img_legend.empty())
  {
    int roi_t = -img_legend.rows;
    int roi_b = -img_show.rows + img.rows * img_scale + img_legend.rows;
    int roi_l = 0;
    int roi_r = -img_show.cols + img.cols * img_scale;
    img_show.adjustROI(roi_t, roi_b, roi_l, roi_r);
    cv::resize(img, img_show, img_show.size(), 0, 0, cv::INTER_NEAREST);
    img_show.adjustROI(-roi_t, -roi_b, -roi_l, -roi_r);
  }
  else
  {
    cv::resize(img, img_show, img_show.size(), 0, 0, cv::INTER_NEAREST);
  }
}

void KalmanVisualize::show()
{
  // out_matrix(sensor->K, "K");
  cv::imshow("KalmanVisualize", img_show);
}

}  // namespace alvar
