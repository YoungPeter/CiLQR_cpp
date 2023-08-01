

#include "basic_type.h"

// A vehicle model with 4 dof.
// State - [x, y, vel, theta]
// Control - [acc, yaw_rate]
class VehicleModel {
 public:
  VehicleModel(const vehicle_model_args& args);

  State ForwardSimulate(const State& state, const Control& control);

  std::vector<Eigen::MatrixXd> GetAMatrixs(std::vector<double> velocitys,
                                           std::vector<double> theta,
                                           std::vector<double> acceletations);

  std::vector<Eigen::MatrixXd> GetBMatrixs(std::vector<double> theta);

 private:
  double wheelbase_;
  double steer_min_;
  double steer_max_;
  double accel_min_;
  double accel_max_;
  double max_speed_;
  double ts_;
  double n_;
  std::vector<double> z_;
  std::vector<double> o_;
};
