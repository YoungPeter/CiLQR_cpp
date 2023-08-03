

#include "basic_type.h"

// A vehicle model with 4 dof.
// State - [x, y, vel, theta]
// Control - [acc, yaw_rate]
class LocalPlanner {
 public:
  LocalPlanner(int number_of_local_wpts, int poly_order)
      : number_of_local_wpts_(number_of_local_wpts), poly_order_(poly_order) {}

  void SetGlobalPlanner(const std::vector<Pos2d>& global_plan) {
    global_plan_ = global_plan;
  }

  void SetEgoState(const Eigen::MatrixXd& ego_state) { ego_state_ = ego_state; }

  std::pair<std::vector<Pos2d>, Eigen::MatrixXd> GetLocalPlan();

 private:
  int number_of_local_wpts_ = 0;
  int poly_order_ = 0;
  std::vector<Pos2d> global_plan_;
  Eigen::MatrixXd ego_state_;

  int GetClosestNodeIndex(const Pos2d& node);
  std::vector<Pos2d> GetLocalWaypoints();
};
