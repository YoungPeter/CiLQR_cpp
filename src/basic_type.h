#include <Eigen/Dense>
#include <vector>

struct State {
  double x;
  double y;
  double v;
  double theta;
};

struct Control {
  double acc;
  double yaw_rate;
};

struct vehicle_model_args {
  double wheelbase;
  double steer_min;
  double steer_max;
  double accel_min;
  double accel_max;
  double max_speed;
  double time_step;
  int horizon;
};

struct Pos2d {
  double x;
  double y;
};

struct Obstacle_args {
  double t_safe;
  double s_safe_a;
  double s_safe_b;
  double ego_rad;
  double ego_lf;
  double ego_lr;
  double q1_front;
  double q2_front;
  double q1_rear;
  double q2_rear;
};

struct Constrains_args {
  double w_acc;
  double w_yawrate;
  double w_pos;
  double w_vel;
  double num_states;
  int horizon;
  double desired_speed;
  double num_ctrls;
  std::vector<double> acc_limits;
  double q1_acc;
  double q2_acc;
  std::vector<double> steer_angle_limits;
  double wheelbase;
  double q1_yawrate;
  double q2_yawrate;
  double number_of_local_wpts;
  Obstacle_args obstacle_args;
};
