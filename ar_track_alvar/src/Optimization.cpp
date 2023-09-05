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
#include "ar_track_alvar/Optimization.h"
#include "time.h"
#include <opencv2/highgui.hpp>

#include <iostream>
using namespace std;

namespace alvar
{
Optimization::Optimization(int n_params, int n_meas)
{
  estimate_param = 0;
  J = cv::Mat::zeros(n_meas, n_params, CV_64F);
  JtJ = cv::Mat::zeros(n_params, n_params, CV_64F);
  tmp = cv::Mat::zeros(n_params, n_meas, CV_64F);
  W = cv::Mat::zeros(n_meas, n_meas, CV_64F);
  diag = cv::Mat::zeros(n_params, n_params, CV_64F);
  err = cv::Mat::zeros(n_meas, 1, CV_64F);
  delta = cv::Mat::zeros(n_params, 1, CV_64F);
  x_minus = cv::Mat::zeros(n_params, 1, CV_64F);
  x_plus = cv::Mat::zeros(n_params, 1, CV_64F);
  x_tmp1 = cv::Mat::zeros(n_meas, 1, CV_64F);
  x_tmp2 = cv::Mat::zeros(n_meas, 1, CV_64F);
  tmp_par = cv::Mat::zeros(n_params, 1, CV_64F);
}

Optimization::~Optimization()
{
  J.release();
  JtJ.release();
  diag.release();
  tmp.release();
  W.release();
  err.release();
  delta.release();
  x_plus.release();
  x_minus.release();
  x_tmp1.release();
  x_tmp2.release();
  tmp_par.release();
  estimate_param = 0;
}

double Optimization::CalcTukeyWeight(double residual, double c)
{
  // const double c = 3; // squared distance in the model tracker
  double ret = 0;

  if (fabs(residual) <= c)
  {
    double tmp = 1.0 - ((residual / c) * (residual / c));
    ret = ((c * c) / 6.0) * (1.0 - tmp * tmp * tmp);
  }
  else
    ret = (c * c) / 6.0;

  if (residual)
    ret = fabs(sqrt(ret) / residual);
  else
    ret = 1.0;  // ???

  return ret;
}

double Optimization::CalcTukeyWeightSimple(double residual, double c)
{
  // const double c = 3;
  double ret = 0;

  double x2 = residual * residual;
  if (x2 < c * c)
    return residual;
  else
    return c;
}

void Optimization::CalcJacobian(cv::Mat& x, cv::Mat& J,
                                EstimateCallback Estimate)
{
  const double step = 0.001;

  J.setTo(cv::Scalar::all(0));
  for (int i = 0; i < J.cols; i++)
  {
    cv::Mat J_column = J.col(i);

    delta.setTo(cv::Scalar::all(0));
    delta.at<double>(i, 0) = step;
    x_plus = x + delta;
    delta.at<double>(i, 0) = -step;
    x_minus = x + delta;

    Estimate(x_plus, x_tmp1, estimate_param);
    Estimate(x_minus, x_tmp2, estimate_param);
    J_column = x_tmp1 - x_tmp2;
    J_column = J_column * 1.0 / (2 * step);
  }
}

double Optimization::Optimize(
    cv::Mat& parameters,    // Initial values are set
    cv::Mat& measurements,  // Some observations
    double stop, int max_iter, EstimateCallback Estimate, void* param,
    OptimizeMethod method,
    const cv::Mat& parameters_mask,  // Mask indicating non-constant parameters)
    const cv::Mat& J_mat, const cv::Mat& weights)
{
  int n_params = parameters.rows;
  int n_meas = measurements.rows;
  double error_new = 0;
  double error_old = 0;
  double n1, n2;
  int cntr = 0;
  estimate_param = param;
  lambda = 0.001;

  while (true)
  {
    if (J_mat.empty())
      CalcJacobian(parameters, J, Estimate);
    else
      J = J_mat;

    // Zero the columns for constant parameters
    // TODO: Make this into a J-sized mask matrix before the iteration loop
    if (!parameters_mask.empty())
      for (int i = 0; i < parameters_mask.rows; i++)
      {
        if (parameters_mask.at<uchar>(i, 0) == 0)
        {
          cv::Rect rect;
          rect.height = J.rows;
          rect.width = 1;
          rect.y = 0;
          rect.x = i;
          cv::Mat foo = J(rect);
          foo.setTo(cv::Scalar::all(0));
        }
      }

    Estimate(parameters, x_tmp1, estimate_param);
    err = measurements - x_tmp1;  // err = residual
    error_old = cv::norm(err, cv::NORM_L2);

    switch (method)
    {
      case (GAUSSNEWTON):

        cv::mulTransposed(J, JtJ, true);
        cv::invert(JtJ, JtJ, cv::DECOMP_SVD);
        cv::gemm(JtJ, J, 1.0, 0, 0, tmp, cv::GEMM_2_T);  // inv(JtJ)Jt

        delta = tmp * err;
        parameters = delta + parameters;

        // Lopetusehto
        n1 = cv::norm(delta);
        n2 = cv::norm(parameters);

        if (((n1 / n2) < stop) || (cntr >= max_iter))
          goto end;

        break;

      case (LEVENBERGMARQUARDT):
        cv::setIdentity(diag, cv::Scalar(lambda));

        if (!weights.empty())
          for (int k = 0; k < W.rows; ++k)
            W.at<double>(k, k) = weights.at<double>(k);

        // JtWJ
        if (!weights.empty())
        {
          cv::gemm(J, W, 1, 0, 0, tmp, cv::GEMM_1_T);
          cv::gemm(tmp, J, 1, 0, 0, JtJ, 0);
        }
        else
          cv::mulTransposed(J, JtJ, true);

        // JtJ + lambda*I
        // or JtWJ + lambda*I if weights are used...
        JtJ = JtJ + diag;
        cv::invert(JtJ, JtJ, cv::DECOMP_SVD);
        cv::gemm(JtJ, J, 1.0, 0, 0, tmp, cv::GEMM_2_T);

        if (!weights.empty())
          cv::gemm(tmp, W, 1, 0, 0, tmp, 0);

        delta = tmp * err;
        tmp_par = delta * parameters;

        Estimate(tmp_par, x_tmp1, estimate_param);
        err = measurements - x_tmp1;

        error_new = cv::norm(err, cv::NORM_L2);

        if (error_new < error_old)
        {
          tmp_par.copyTo(parameters);
          lambda = lambda / 10.0;
        }
        else
        {
          lambda = lambda * 10.0;
        }
        if (lambda > 10)
          lambda = 10;
        if (lambda < 0.00001)
          lambda = 0.00001;

        n1 = cv::norm(delta);
        n2 = cv::norm(parameters);

        if ((n1 / n2) < stop || (cntr >= max_iter))
        {
          goto end;
        }

        break;

      case (TUKEY_LM):

        cv::setIdentity(diag, cv::Scalar(lambda));

        // Tukey weights
        for (int k = 0; k < W.rows; ++k)
        {
          if (!weights.empty())                 // If using weight vector
            if (weights.at<double>(k) != -1.0)  // If true weight given
              W.at<double>(k, k) = weights.at<double>(k);  // Use given weight
            else
              W.at<double>(k, k) = CalcTukeyWeight(
                  err.at<double>(k), 3);  // otherwise use Tukey weight
          else
            W.at<double>(k, k) = CalcTukeyWeight(
                err.at<double>(k), 3);  // Otherwise use Tukey weight
        }

        cv::gemm(J, W, 1, 0, 0, tmp, cv::GEMM_1_T);
        cv::gemm(tmp, J, 1, 0, 0, JtJ, 0);
        JtJ = JtJ * diag;
        cv::invert(JtJ, JtJ, cv::DECOMP_SVD);
        cv::gemm(JtJ, J, 1.0, 0, 0, tmp, cv::GEMM_2_T);
        cv::gemm(tmp, W, 1, 0, 0, tmp, 0);
        delta = tmp * err;
        tmp_par = delta * parameters;

        Estimate(tmp_par, x_tmp1, estimate_param);
        err = measurements - x_tmp1;

        error_new = cv::norm(err, cv::NORM_L2);

        if (error_new < error_old)
        {
          tmp_par.copyTo(parameters);
          lambda = lambda / 10.0;
        }
        else
        {
          lambda = lambda * 10.0;
        }
        if (lambda > 10)
          lambda = 10;
        if (lambda < 0.00001)
          lambda = 0.00001;

        n1 = cv::norm(delta);
        n2 = cv::norm(parameters);

        if (((n1 / n2) < stop) || (cntr >= max_iter))
        {
          goto end;
        }

        break;
    }
    ++cntr;
  }

end:

  return error_old;
}

}  // namespace alvar
