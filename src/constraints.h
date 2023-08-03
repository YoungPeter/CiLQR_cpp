#include <tuple>

#include "basic_type.h"
#include "obstacle.h"

using MatrixXds = std::vector<Eigen::MatrixXd>;

class Constraints {
 public:
  Constraints(const Constrains_args& args,
              const std::array<double, 2>& car_args);

  std::tuple<MatrixXds, MatrixXds, MatrixXds, MatrixXds> GetCostDerivatives(
      const std::vector<State>& states, const std::vector<Control>& control,
      const Eigen::VectorXd& ploy_coeffs,
      const std::vector<double>& x_local_plan,
      const std::vector<State>& npc_traj);

  double GetTotalCost(const std::vector<State>& states,
                      const std::vector<Control>& control,
                      const Eigen::VectorXd& ploy_coeffs,
                      const std::vector<double>& x_local_plan,
                      const std::vector<State>& npc_traj);

 private:
  Constrains_args args_;
  Eigen::MatrixXd control_cost_;
  Eigen::MatrixXd state_cost_;
  int number_of_npc_;
  Obstacle obstacle_;

  std::tuple<double, Eigen::MatrixXd, Eigen::MatrixXd> BarrierFunction(
      double q1, double q2, double c, const Eigen::MatrixXd& c_dot);

  std::pair<double, double> FindClosestPoint(
      const State& states, const Eigen::VectorXd& coeffs,
      const std::vector<double>& x_local_plan);

  std::pair<MatrixXds, MatrixXds> GetStateCostDerivatives(
      const std::vector<State>& states, const Eigen::VectorXd& coeffs,
      const std::vector<double>& x_local_plan,
      const std::vector<State>& npc_traj);

  std::pair<MatrixXds, MatrixXds> GetControlCostDerivatives(
      const std::vector<State>& states, const std::vector<Control>& control);
};
