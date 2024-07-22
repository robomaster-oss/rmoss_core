// Copyright 2024 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <random>

#include "rmoss_util/extended_kalman_filter.hpp"

// 状态量维度
constexpr unsigned int XN = 2;

// 定义状态转移方程
struct StateTransitionModel
{
  explicit StateTransitionModel(double dt)
  : dt_(dt) {}
  template<typename T>
  Eigen::Matrix<T, XN, 1> operator()(const Eigen::Matrix<T, XN, 1> & x) const
  {
    Eigen::Matrix<T, XN, 1> x_new;
    x_new[0] = x[0] + x[1] * dt_;
    x_new[1] = x[1];
    return x_new;
  }

  double dt_;
};

// 定义观测方程
struct MeasurementModel
{
  template<typename T>
  Eigen::Matrix<T, 1, 1> operator()(const Eigen::Matrix<T, XN, 1> & x) const
  {
    Eigen::Matrix<T, 1, 1> z;
    z[0] = x[0];
    return z;
  }
};

TEST(ExtendedKalmanFilterTest, MultipleIterations) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, 0.1);

  Eigen::Matrix<double, XN, 1> x;
  x << 0, 0;
  Eigen::Matrix<double, XN, XN> P;
  P << 1, 0, 0, 1;
  Eigen::Matrix<double, XN, XN> Q;
  Q << 0.01, 0, 0, 0.01;

  rmoss_util::ExtendedKalmanFilter<XN> kf(x, P);

  for (int i = 0; i < 20; ++i) {
    // 预测步骤
    kf.predict(StateTransitionModel(1.0), Q);

    // 生成假设的测量值
    Eigen::Matrix<double, 1, 1> z;
    z << (i + 1 + dist(gen));

    // 更新步骤
    Eigen::Matrix<double, 1, 1> R;
    R << 0.01;

    x = kf.update(MeasurementModel(), R, z);
  }

  // 最终状态值检查
  EXPECT_NEAR(x[0], 20.0, 1.0);
  EXPECT_NEAR(x[1], 1.0, 0.1);
}
