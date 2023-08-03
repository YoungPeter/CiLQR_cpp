#include <algorithm>
#include <cmath>
#include <iostream>

#include "basic_function.h"
#include "local_planner.h"

Constraints::Constraints(const Constrains_args& args,
                         const std::array<double, 2>& car_args)
    : obstacle_(Obstacle(args.Obstacle_args, 1, car_args)) {
  control_cost_ = Eigen::MatrixXd::Zero(2, 2);
  control_cost_(0, 0) = args.w_acc;
  control_cost_(1, 1) = args.w_yawrate;

  state_cost_ = Eigen::MatrixXd::Zero(4, 4);
  state_cost_(0, 0) = args.w_pos;
  state_cost_(1, 1) = args.w_pos;
  state_cost_(2, 2) = args.w_vel;
}

std::tuple<double, Eigen::MatrixXd, Eigen::MatrixXd>
Constraints::BarrierFunction(double q1, double q2, double c,
                             const Eigen::MatrixXd& c_dot) {
  const double exp_q2_c = std::exp(q2 * c);
  const double b = q1 * exp_q2_c;
  const auto b_dot = q1 * q2 * exp_q2_c * c_dot;
  const auto b_ddot =
      q1 * std::pow(q2, 2) * exp_q2_c * c_dot * c_dot.transpose();
  return std::make_tuple(b, b_dot, b_ddot);
}

std::pair<MatrixXds, MatrixXds> Constraints::GetStateCostDerivatives(
    const std::vector<State>& states, const Eigen::VectorXd& coeffs,
    const std::vector<double>& x_local_plan,
    const std::vector<State>& npc_traj) {
  MatrixXds l_x;
  MatrixXds l_xx;
  for (int i = 0; i < args_.horizon; ++i) {
    // Offset in path derivative
    const auto [x_r, y_r] =
        FindClosestPoint(states[i], poly_coeffs, x_local_plan);
    Eigen::MatrixXd temp(4, 1);
    temp << states[i].x - x_r, states[i].y - y_r,
        states[i].v - args_.desired_speed, 0.0;
    Eigen::MatrixXd traj_cost = 2 * state_cost_ * temp;

    // Compute first order derivative.
    Eigen::MatrixXd l_x_i = traj_cost;

    // Compute second order derivative
    Eigen::MatrixXd l_xx_i = state_cost_ * 2.0;

    // Obstacle derivative
    for (int j = 0; j < number_of_npc_; j++) {
      const auto [b_dot_obs, b_ddot_obs] =
          obstacle_.GetObstacleCostDerivatives(npc_traj, i, states[i]);
      l_x_i += b_dot_obs;
      l_xx_i += b_ddot_obs;
    }

    l_x.push_back(l_x_i);
    l_xx.push_back(l_xx_i);
  }

  return std::make_pair(l_x, l_xx);
}

std::pair<MatrixXds, MatrixXds> Constraints::GetControlCostDerivatives(
    const std::vector<State>& states, const std::vector<Control>& control) {
  Eigen::MatrixXd P1(2, 1), P2(2, 1);
  P1 << 1.0, 0.0;
  P2 << 0.0, 1.0;

  MatrixXds l_u;
  MatrixXds l_uu;
  for (int i = 0; i < args_.horizon; ++i) {
    Eigen::MatrixXd control_i(1, 2);
    control_i << control[i].acc, control[i].yaw_rate;

    const auto temp_m1 = control_i * P1;

    const double c1 = temp_m1(0, 0) - args_.acc_limits[1];
    const auto [b_1, b_dot_1, b_ddot_1] =
        BarrierFunction(args_.q1_acc, args_.q2_acc, c1, P1);

    const double c2 = args_.acc_limits[0] - temp_m1(0, 0);
    const auto [b_2, b_dot_2, b_ddot_2] =
        BarrierFunction(args_.q1_acc, args_.q2_acc, c2, -P1);

    const double velocity = states[i].v;
    const auto temp_m2 = control_i * P2;

    const double c3 =
        temp_m2(0, 0) -
        velocity * std::tan(args_.steer_angle_limits[1]) / args_.wheelbase;
    const auto [b_3, b_dot_3, b_ddot_3] =
        BarrierFunction(args_.q1_yawrate, args_.q2_yawrate, c3, P2);

    const double c4 =
        velocity * std::tan(args_.steer_angle_limits[0]) / args_.wheelbase -
        temp_m2(0, 0);
    const auto [b_4, b_dot_4, b_ddot_4] =
        BarrierFunction(args_.q1_yawrate, args_.q2_yawrate, c4, P2);

    const Eigen::MatrixXd l_u_i =
        b_dot_1 + b_dot_2 + b_dot_3 + b_dot_4 + 2 * control_i * control_cost_;
    const Eigen::MatrixXd l_uu_i =
        b_ddot_1 + b_ddot_2 + b_ddot_3 + b_ddot_4 + 2 * control_cost_;

    l_u.push_back(l_u_i);
    l_uu.push_back(l_uu_i);
  }

  return std::make_pair(l_u, l_uu);
}

std::tuple<MatrixXds, MatrixXds, MatrixXds, MatrixXds>
Constraints::GetCostDerivatives(const std::vector<State>& states,
                                const std::vector<Control>& control,
                                const Eigen::VectorXd& ploy_coeffs,
                                const std::vector<double>& x_local_plan,
                                const std::vector<State>& npc_traj) {
  const auto [l_u, l_uu] = GetControlCostDerivatives(states, control);
  const auto [l_x, l_xx] =
      GetStateCostDerivatives(states, ploy_coeffs, x_local_plan, npc_traj);

  MatrixXds l_ux(args_.num_ctrls,
                 Eigen::MatrixXd::Zero(args_.num_states, args_.horizon));

  return std::make_tuple(l_x, l_xx, l_u, l_uu, l_ux);
}

double Constraints::GetTotalCost(const std::vector<State>& states,
                                 const std::vector<Control>& control,
                                 const Eigen::VectorXd& ploy_coeffs,
                                 const std::vector<double>& x_local_plan,
                                 const std::vector<State>& npc_traj) {
  double total_cost = 0;
  for (int i = 0; i < args_.horizon; ++i) {
    const auto [x_r, y_r] =
        FindClosestPoint(states[i], ploy_coeffs, x_local_plan);
    State ref_state{x_r, y_r, args_.desired_speed, 0.0};

    Eigen::MatrixXd state_diff(4, 1);
    state_diff << states[i].x - ref_state[i].x, states[i].y - ref_state[i].y,
        states[i].v - ref_state[i].v, states[i].theta - ref_state[i].theta;

    const auto c_state_m = state_diff.transpose() * state_cost_ * state_diff;
    const double c_state = c_state_m(0, 0);

    Eigen::MatrixXd control_i(1, 2);
    control_i << control[i].acc, control[i].yaw_rate;
    const auto c_control_m = control_i * control_cost * control_i.transpose();
    const double c_control = c_control_m(0, 0);

    total_cost = c_state + c_control;
  }

  return total_cost;
}