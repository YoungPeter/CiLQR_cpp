#include <tuple>

#include "basic_type.h"

class Obstacle {
 public:
  Obstacle(const Obstacle_args& args, int track_id,
           const std::array<double, 2>& car_args)
      : args_(args), track_id_(track_id) {
    car_length_ = car_args[0];
    car_width_ = car_args[1];
  }

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> GetObstacleCostDerivatives(
      const std::vector<State>& npc_traj, int i, const State& ego_state);

 private:
  Obstacle_args args_;
  double car_length_;
  double car_width_;
  int track_id_;

  std::tuple<double, Eigen::MatrixXd, Eigen::MatrixXd> BarrierFunction(
      double q1, double q2, double c, const Eigen::MatrixXd& c_dot);
};
