#include <algorithm>
#include <cmath>
#include <iostream>

#include "vehicle_model.h"

VehicleModel::VehicleModel(const vehicle_model_args& args) {
  wheelbase_ = args.wheelbase;
  steer_min_ = args.steer_min;
  steer_max_ = args.steer_max;
  accel_min_ = args.accel_min;
  accel_max_ = args.accel_max;
  max_speed_ = args.max_speed;
  ts_ = args.time_step;
  n_ = args.horizon;
  z_ = std::vector<double>(n_, 0.0);
  o_ = std::vector<double>(n_, 0.0);
}

// Find the next state of the vehicle given the current state and control input.
State VehicleModel::ForwardSimulate(const State& state,
                                    const Control& control) {
  const double acc = control.acc;
  const double yaw_rate =
      std::min(std::max(control.yaw_rate, accel_min_), accel_max_);

  State next_state;
  // 状态方程
  next_state.x =
      state.x + std::cos(state.theta) * (state.v * ts_ + 0.5 * acc * ts_ * ts_);
  next_state.y =
      state.y + std::sin(state.theta) * (state.v * ts_ + 0.5 * acc * ts_ * ts_);
  next_state.v = std::min(std::max(state.v + acc * ts_, 0.0), max_speed_);
  next_state.theta = state.theta + yaw_rate * ts_;

  return next_state;
}

std::vector<Eigen::MatrixXd> VehicleModel::GetAMatrixs(
    std::vector<double> velocitys, std::vector<double> theta,
    std::vector<double> acceletations) {
  std::vector<Eigen::MatrixXd> matrixs;

  if (velocitys.size() != theta.size() ||
      velocitys.size() != acceletations.size()) {
    return matrixs;
  }

  matrixs.resize(velocitys.size());
  for (int k = 0; k < velocitys.size(); ++k) {
    Eigen::MatrixXd m(4, 4);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (i == j) {
          m(i, j) = o_[k];
        } else {
          m(i, j) = z_[k];
        }
      }
    }

    m(0, 2) = std::cos(theta[k]) * ts_;
    m(0, 3) = -(velocitys[k] * ts_ + 0.5 * acceletations[k] * ts_ * ts_) *
              std::sin(theta[k]);
    m(1, 2) = std::sin(theta[k]) * ts_;
    m(1, 3) = (velocitys[k] * ts_ + 0.5 * acceletations[k] * ts_ * ts_) *
              std::cos(theta[k]);
    matrixs[k] = m;
  }

  return matrixs;
}

std::vector<Eigen::MatrixXd> VehicleModel::GetBMatrixs(
    std::vector<double> theta) {
  std::vector<Eigen::MatrixXd> matrixs;
  if (theta.size() != n_) {
    return matrixs;
  }

  matrixs.resize(n_);
  for (int k = 0; k < n_; ++k) {
    Eigen::MatrixXd m(2, 4);
    m << 0.5 * ts_ * ts_ * std::cos(theta[k]), z_[k],
        0.5 * ts_ * ts_ * std::sin(theta[k]), z_[k], ts_ * o_[k], z_[k], z_[k],
        ts_ * o_[k];
    matrixs[k] = m;
  }

  return matrixs;
}