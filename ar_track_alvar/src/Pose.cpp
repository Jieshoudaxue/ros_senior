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

#include "ar_track_alvar/Pose.h"

using namespace std;

namespace alvar
{
using namespace std;

void Pose::Output() const
{
  cout << quaternion[0] << "," << quaternion[1] << "," << quaternion[2] << ","
       << quaternion[3] << "|";
  cout << translation[0] << "," << translation[1] << "," << translation[2]
       << endl;
}

Pose::Pose() : Rotation()
{
  translation_mat = cv::Mat(4, 1, CV_64F, translation);
  translation_mat.setTo(cv::Scalar::all(0));
  translation_mat.at<double>(3, 0) = 1;
}

Pose::Pose(const cv::Mat& tra, const cv::Mat& rot, RotationType t)
  : Rotation(rot, t)
{
  translation_mat = cv::Mat(4, 1, CV_64F, translation);
  translation_mat.setTo(cv::Scalar::all(0));
  translation_mat.at<double>(3, 0) = 1;
  // Fill in translation part
  translation_mat.at<double>(0, 0) = tra.at<double>(0, 0);
  translation_mat.at<double>(1, 0) = tra.at<double>(1, 0);
  translation_mat.at<double>(2, 0) = tra.at<double>(2, 0);
}

Pose::Pose(const cv::Mat& mat) : Rotation(mat, MAT)
{
  translation_mat = cv::Mat(4, 1, CV_64F, translation);
  translation_mat.setTo(cv::Scalar::all(0));
  translation_mat.at<double>(3, 0) = 1;
  // Fill in translation part
  if (mat.cols == 4)
  {
    translation_mat.at<double>(0, 0) = mat.at<double>(0, 3);
    translation_mat.at<double>(1, 0) = mat.at<double>(1, 3);
    translation_mat.at<double>(2, 0) = mat.at<double>(2, 3);
  }
}

Pose::Pose(const Pose& p) : Rotation(p)
{
  translation_mat = cv::Mat(4, 1, CV_64F, translation);
  p.translation_mat.copyTo(translation_mat);
}

void Pose::Reset()
{
  quaternion_mat.setTo(cv::Scalar::all(0));
  quaternion_mat.at<double>(0, 0) = 1;
  translation_mat.setTo(cv::Scalar::all(0));
}

void Pose::SetMatrix(const cv::Mat& mat)
{
  double tmp[9];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      tmp[i * 3 + j] = mat.at<double>(i, j);

  Mat9ToQuat(tmp, quaternion);
  if (mat.cols == 4)
  {
    translation_mat.at<double>(0, 0) = mat.at<double>(0, 3);
    translation_mat.at<double>(1, 0) = mat.at<double>(1, 3);
    translation_mat.at<double>(2, 0) = mat.at<double>(2, 3);
    translation_mat.at<double>(3, 0) = 1;
  }
}

void Pose::GetMatrix(cv::Mat& mat) const
{
  if (mat.cols == 3)
  {
    QuatToMat9(quaternion, mat.ptr<double>(0));
  }
  else if (mat.cols == 4)
  {
    cv::setIdentity(mat);
    QuatToMat16(quaternion, mat.ptr<double>(0));
    mat.at<double>(0, 3) = translation_mat.at<double>(0, 0);
    mat.at<double>(1, 3) = translation_mat.at<double>(1, 0);
    mat.at<double>(2, 3) = translation_mat.at<double>(2, 0);
  }
}

void Pose::GetMatrixGL(double gl[16], bool mirror)
{
  if (mirror)
    Mirror(false, true, true);
  cv::Mat gl_mat = cv::Mat(4, 4, CV_64F, gl);
  GetMatrix(gl_mat);
  gl_mat = gl_mat.t();
  if (mirror)
    Mirror(false, true, true);
}

void Pose::SetMatrixGL(double gl[16], bool mirror)
{
  double gll[16];
  memcpy(gll, gl, sizeof(double) * 16);
  cv::Mat gl_mat = cv::Mat(4, 4, CV_64F, gll);
  gl_mat = gl_mat.t();
  SetMatrix(gl_mat);
  if (mirror)
    Mirror(false, true, true);
}

void Pose::Transpose()
{
  double tmp[16];
  cv::Mat tmp_mat = cv::Mat(4, 4, CV_64F, tmp);
  GetMatrix(tmp_mat);
  tmp_mat = tmp_mat.t();
  SetMatrix(tmp_mat);
}

void Pose::Invert()
{
  double tmp[16];
  cv::Mat tmp_mat = cv::Mat(4, 4, CV_64F, tmp);
  GetMatrix(tmp_mat);
  tmp_mat = tmp_mat.inv();
  SetMatrix(tmp_mat);
}

void Pose::Mirror(bool x, bool y, bool z)
{
  double tmp[16];
  cv::Mat tmp_mat = cv::Mat(4, 4, CV_64F, tmp);
  GetMatrix(tmp_mat);
  MirrorMat(tmp_mat, x, y, z);
  SetMatrix(tmp_mat);
}

void Pose::SetTranslation(const cv::Mat& tra)
{
  translation_mat.at<double>(0, 0) = tra.at<double>(0, 0);
  translation_mat.at<double>(1, 0) = tra.at<double>(1, 0);
  translation_mat.at<double>(2, 0) = tra.at<double>(2, 0);
  translation_mat.at<double>(3, 0) = 1;
}
void Pose::SetTranslation(const double* tra)
{
  translation[0] = tra[0];
  translation[1] = tra[1];
  translation[2] = tra[2];
  translation[3] = 1;
}
void Pose::SetTranslation(const double x, const double y, const double z)
{
  translation[0] = x;
  translation[1] = y;
  translation[2] = z;
  translation[3] = 1;
}
void Pose::GetTranslation(cv::Mat& tra) const
{
  tra.at<double>(0, 0) = translation_mat.at<double>(0, 0);
  tra.at<double>(1, 0) = translation_mat.at<double>(1, 0);
  tra.at<double>(2, 0) = translation_mat.at<double>(2, 0);
  if (tra.rows == 4)
    tra.at<double>(3, 0) = 1;
}

Pose& Pose::operator=(const Pose& p)
{
  memcpy(quaternion, p.quaternion, 4 * sizeof(double));
  memcpy(translation, p.translation, 4 * sizeof(double));
  return *this;
}

}  // namespace alvar
