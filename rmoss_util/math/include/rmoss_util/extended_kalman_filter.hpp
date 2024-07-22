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

#ifndef RMOSS_UTIL__EXTENDED_KALMAN_FILTER_HPP_
#define RMOSS_UTIL__EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <ceres/ceres.h>

// 一些Eigen的工具函数，参考https://github.com/TomLKoller/ADEKF
namespace Eigen
{
// 获取一个Eigen矩阵的行数
template<typename Derived>
static constexpr int dof =
  internal::traits<typename Derived::PlainObject>::RowsAtCompileTime;

// 用于绕过Eigen表达式模板，获取表达式结果。如果不使用eval()，auto将无法正常获取表达式结果类型。
template<typename T> auto eval(const T & result) {return result;}

template<typename BinaryOp, typename LhsType, typename RhsType>
auto eval(const CwiseBinaryOp<BinaryOp, LhsType, RhsType> & result)
{
  // The eval() function returns the result of the operation
  // If this is not used auto will not work correctly
  return result.eval();
}

template<typename XprType, int BlockRows, int BlockCols, bool InnerPanel>
auto eval(const Block<XprType, BlockRows, BlockCols, InnerPanel> & result)
{
  return result.eval();
}

}  // namespace Eigen

namespace rmoss_util
{

/**
 * @class ExtendedKalmanFilter
 *
 * @brief 可自动微分的扩展卡尔曼滤波器
 *
 * @param XN 状态量维度
 */
template<int XN>
class ExtendedKalmanFilter
{
  using VectorX = Eigen::Matrix<double, XN, 1>;
  using MatrixXX = Eigen::Matrix<double, XN, XN>;

public:
  ExtendedKalmanFilter(const VectorX & x, const MatrixXX & P)
  : x_(x), P_(P) {}

  /**
   * @brief 重置状态
   */
  void reset(const VectorX & x, const MatrixXX & P)
  {
    x_ = x;
    P_ = P;
  }

  /**
   * 预测步
   * @brief 使用自动微分的雅可比矩阵，预测状态估计
   * @tparam 状态转移方程f(x,u)，Functor
   * @tparam 控制量类型
   * @param transition_model 状态转移方程
   * @param Q 过程噪声协方差
   * @param u 控制量
   */
  template<typename StateTransitionModel, typename ... Controls>
  VectorX predict(
    StateTransitionModel transition_model, const MatrixXX & Q,
    const Controls &... u)
  {
    // 绑定控制参数到状态转移方程
    auto f = std::bind(transition_model, std::placeholders::_1, u ...);

    // x_jet(i).v[i]保存状态量x_(i)的导数
    Eigen::Matrix<ceres::Jet<double, XN>, XN, 1> x_jet;
    x_jet.setZero();
    for (int i = 0; i < XN; i++) {
      x_jet(i).a = x_(i);
      // 对自身求导为1
      x_jet(i).v[i] = 1.0;
    }

    // 状态转移
    auto x_p_jet = f(x_jet);

    // 获取雅可比矩阵同时更新状态量
    for (int i = 0; i < XN; ++i) {
      x_(i) = x_p_jet[i].a;
      F_.row(i) = x_p_jet[i].v.transpose();
    }

    P_ = F_ * P_ * F_.transpose() + Q;

    return x_;
  }

  /**
   * 更新步
   * @tparam Measurement 测量量类型
   * @tparam MeasurementModel 观测方程 h(x,v)，Functor
   * @param measurementModel 观测方程
   * @param R 测量噪声协方差
   * @param z 测量值
   */
  template<typename Measurement, typename MeasurementModel,
    int ZN = Eigen::dof<Measurement>>
  VectorX update(
    MeasurementModel h,
    const Eigen::Matrix<double, Eigen::dof<Measurement>,
    Eigen::dof<Measurement>> & R,
    const Measurement & z)
  {
    // 观测方程的雅可比矩阵
    Eigen::Matrix<double, ZN, XN> H;
    Eigen::Matrix<double, ZN, 1> z_hat;
    H.setZero();
    z_hat.setZero();

    // x_jet(i).v[i]保存状态量x_(i)的导数
    Eigen::Matrix<ceres::Jet<double, XN>, XN, 1> x_jet;
    for (int i = 0; i < XN; i++) {
      x_jet(i).a = x_(i);
      // 对自身求导为1
      x_jet(i).v[i] = 1.0;
    }

    // 观测方程
    auto z_jet = Eigen::eval(h(x_jet));

    for (int i = 0; i < ZN; i++) {
      z_hat(i) = z_jet[i].a;
      H.row(i) = z_jet[i].v.transpose();
    }
    auto K = Eigen::eval(P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse());

    x_ = x_ + K * (z - z_hat);
    P_ = (MatrixXX::Identity() - K * H) * P_;

    return x_;
  }

  VectorX get_state() const {return x_;}

  MatrixXX get_covariance() const {return P_;}

private:
  // 当前状态分布的均值
  VectorX x_;

  // 当前状态分布的协方差
  MatrixXX P_;

  // 状态转移方程的雅可比矩阵
  MatrixXX F_;
};
}  // namespace rmoss_util
#endif  // RMOSS_UTIL__EXTENDED_KALMAN_FILTER_HPP_
