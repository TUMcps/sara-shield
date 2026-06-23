#include "safety_shield/safety_shield.h"

namespace safety_shield {

SafetyShield::SafetyShield()
    : max_s_stop_(0),
      v_max_allowed_({0, 0, 0}),
      a_max_allowed_({0, 0, 0}),
      j_max_allowed_({0, 0, 0}),
      a_max_ltt_({0, 0, 0}),
      j_max_ltt_({0, 0, 0}) {
  robot_reach_ = new RobotReach();
  human_reach_ = new HumanReach();
  verify_ = new VerifyISO();
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(int nb_joints, double sample_time, double max_s_stop,
                           const std::vector<double>& v_max_allowed, const std::vector<double>& a_max_allowed,
                           const std::vector<double>& j_max_allowed, const std::vector<double>& a_max_path,
                           const std::vector<double>& j_max_path, const LongTermTraj& long_term_trajectory,
                           RobotReach* robot_reach, HumanReach* human_reach, VerifyISO* verify,
                           const std::vector<reach_lib::AABB>& environment_elements, ShieldType shield_type,
                           ContactType eef_contact_type)
    : nb_joints_(nb_joints),
      max_s_stop_(max_s_stop),
      v_max_allowed_(v_max_allowed),
      a_max_allowed_(a_max_allowed),
      j_max_allowed_(j_max_allowed),
      a_max_ltt_(a_max_path),
      j_max_ltt_(j_max_path),
      sample_time_(sample_time),
      path_s_(0),
      path_s_discrete_(0),
      long_term_trajectory_(long_term_trajectory),
      robot_reach_(robot_reach),
      human_reach_(human_reach),
      verify_(verify),
      environment_elements_(environment_elements),
      shield_type_(shield_type),
      eef_contact_type_(eef_contact_type) {
  sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }

  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
  } else {
    is_safe_ = false;
  }
  Motion goal_motion = computesPotentialTrajectory(is_safe_, prev_dq);
  updateSafePath(is_safe_);
  next_motion_ = getCurrentMotion();
  reachability_set_duration_ = reachability_set_interval_size_ * sample_time_;
  buildMaxContactEnergies();
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(double sample_time, std::string trajectory_config_file, std::string robot_config_file,
                           std::string mocap_config_file, double init_x, double init_y, double init_z, double init_roll,
                           double init_pitch, double init_yaw, const std::vector<double>& init_qpos,
                           const std::vector<reach_lib::AABB>& environment_elements, ShieldType shield_type,
                           ContactType eef_contact_type)
    : sample_time_(sample_time),
      path_s_(0),
      path_s_discrete_(0),
      environment_elements_(environment_elements),
      shield_type_(shield_type),
      eef_contact_type_(eef_contact_type) {
  try{
    ///////////// Build robot reach
    spdlog::info("Loading robot configuration from: {}", robot_config_file);
    robot_reach_ = buildRobotReach(robot_config_file, init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
    if (robot_reach_ == nullptr) {
        throw std::runtime_error("Failed to build robot reach from config file: " + robot_config_file);
    }
    nb_joints_ = robot_reach_->getNbJoints();
    ////////////// Setting trajectory variables
    spdlog::info("Loading trajectory configuration from: {}", trajectory_config_file);
    int velocity_method_int;
    readTrajectoryConfig(trajectory_config_file, max_s_stop_, q_min_allowed_, q_max_allowed_, v_max_allowed_,
                        a_max_allowed_, j_max_allowed_, a_max_ltt_, j_max_ltt_, v_safe_, alpha_i_max_,
                        velocity_method_int, reachability_set_interval_size_);
    reachability_set_duration_ = reachability_set_interval_size_ * sample_time_;
    sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
    RobotReach::VelocityMethod velocity_method = static_cast<RobotReach::VelocityMethod>(velocity_method_int);
    robot_reach_->setVelocityMethod(velocity_method);
    //////////// Build human reach
    spdlog::info("Loading human/mocap configuration from: {}", mocap_config_file);
    human_reach_ = buildHumanReach(mocap_config_file);
    if (human_reach_ == nullptr) {
        throw std::runtime_error("Failed to build human reach from config file: " + mocap_config_file);
      }
    buildMaxContactEnergies();
    ///////////// Build verifier
    verify_ = new safety_shield::VerifyISO();

    // Initialize the long term trajectory, verified and intended trajectory
    if (shield_type_ == ShieldType::OFF) {
      is_safe_ = true;
    } else {
      is_safe_ = false;
    }
    // initialize goal with the current state
    Motion init_motion = Motion(0.0, init_qpos);
    std::vector<Motion> initial_traj;
    initial_traj.push_back(init_motion);
    verified_trajectory_ = initial_traj;
    verified_trajectory_index_ = 0;
    if (path_consistent_) {
      // Initialize the long term trajectory
      std::vector<Motion> long_term_traj;
      long_term_traj.push_back(init_motion);
      long_term_trajectory_ = buildTrajectory(long_term_traj, shield_type_ == ShieldType::PFL);
      // initialize monitored path 
      Motion goal_motion = computesPotentialTrajectory(is_safe_, init_motion.getVelocity());
      updateSafePath(is_safe_);
      next_motion_ = getCurrentMotion();
      spdlog::info("Safety shield with path consistent braking created.");
    } else {
      intended_trajectory_ = initial_traj;
      intended_trajectory_index_ = 0;
      // initialize goal and next motion with init motion
      next_motion_ = init_motion;
      new_goal_motion_ = init_motion;
      spdlog::info("Safety shield with non path consistent braking created.");
    }
  } catch (const std::exception& e) {
    spdlog::error("SafetyShield constructor failed: {}", e.what());
    spdlog::error("Robot config file: {}", robot_config_file);
    spdlog::error("Trajectory config file: {}", trajectory_config_file);
    spdlog::error("Mocap config file: {}", mocap_config_file);
    throw std::runtime_error("SafetyShield initialization failed: " + std::string(e.what()));
  } catch (...) {
    spdlog::error("SafetyShield constructor failed with unknown exception");
    spdlog::error("Robot config file: {}", robot_config_file);
    spdlog::error("Trajectory config file: {}", trajectory_config_file);
    spdlog::error("Mocap config file: {}", mocap_config_file);
    throw std::runtime_error("SafetyShield initialization failed with unknown error");
  }
}

void SafetyShield::reset(double init_x, double init_y, double init_z, double init_roll, double init_pitch,
                         double init_yaw, const std::vector<double>& init_qpos, double current_time,
                         const std::vector<reach_lib::AABB>& environment_elements, ShieldType shield_type,
                         ContactType eef_contact_type) {
  shield_type_ = shield_type;
  robot_reach_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
  human_reach_->reset();
  eef_contact_type_ = eef_contact_type;
  buildMaxContactEnergies();
  environment_elements_ = environment_elements;
  std::vector<double> prev_dq;
  for (int i = 0; i < nb_joints_; i++) {
    prev_dq.push_back(0.0);
  }
  if (shield_type_ == ShieldType::OFF) {
    is_safe_ = true;
  } else {
    is_safe_ = false;
  }
  new_ltt_ = false;
  new_goal_ = false;
  new_ltt_processed_ = false;
  intended_path_correct_ = false;
  path_s_ = 0;
  path_s_discrete_ = 0;
  cycle_begin_time_ = current_time;
  intended_path_ = Path();
  failsafe_path_ = Path();
  verified_path_ = Path();

  // initialize goal with the current state
  Motion init_motion = Motion(0.0, init_qpos);
  std::vector<Motion> initial_traj;
  initial_traj.push_back(init_motion);
  verified_trajectory_ = initial_traj;
  verified_trajectory_index_ = 0;
  if (path_consistent_) {
    // Initialize the long term trajectory
    std::vector<Motion> long_term_traj;
    long_term_traj.push_back(init_motion);
    long_term_trajectory_ = buildTrajectory(long_term_traj, shield_type_ == ShieldType::PFL);
    // initialize monitored path 
    Motion goal_motion = computesPotentialTrajectory(is_safe_, init_motion.getVelocity());
    updateSafePath(is_safe_);
    next_motion_ = getCurrentMotion();
    spdlog::info("Safety shield with path consistent braking resetted.");
  } else {
    intended_trajectory_ = initial_traj;
    intended_trajectory_index_ = 0;
    // initialize goal and next motion with init motion
    next_motion_ = init_motion;
    new_goal_motion_ = init_motion;
    spdlog::info("Safety shield with non path consistent braking resetted.");
  }
}

bool SafetyShield::planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max,
                                    Path& path) {
  if (a_max < 0 || fabs(acc) > a_max) {
    // spdlog::info("Planning unsuccessful due to a_max = {} < 0 || fabs({}) > {}", a_max, fabs(acc), a_max);
    return false;
  }
  try {
    path.setCurrent(false);
    path.setPosition(pos);
    path.setVelocity(vel);
    path.setAcceleration(acc);
    double epsilon = 1e-6;

    // If current velocity is close to goal and acceleration is approx zero -> Do nothing
    if (fabs(vel - ve) < epsilon && fabs(acc) < epsilon) {
      std::vector<double> new_times = {sample_time_, sample_time_, sample_time_};
      std::vector<double> new_jerks = {0.0, 0.0, 0.0};
      path.setPhases(new_times, new_jerks);
      return true;
    }
    // Time to reach zero acceleration with maximum jerk
    double t_to_a_0 = roundToTimestep(abs(acc) / j_max);
    // Velocity at zero acceleration
    double v_at_a_0 = vel + acc * t_to_a_0 / 2;
    if (v_at_a_0 < 0) {
      spdlog::warn("Velocity at zero acceleration is negative. The path calculations will be incorrect.");
    }

    // If the velocity at zero acceleration is the goal velocity, use t_to_a_0 and max jerk and return
    if (fabs(v_at_a_0 - ve) < epsilon) {
      std::vector<double> new_times = {t_to_a_0, t_to_a_0, t_to_a_0};
      std::vector<double> new_jerks = {-acc / t_to_a_0, 0, 0};
      path.setPhases(new_times, new_jerks);
      return true;
    }

    double a_peak, t01, t12, t23, v01, v12, v23, correct_int;
    // If we are accelerating
    if (acc > 0) {
      // Reducing the acceleration to zero with max jerk would lead to a velocity larger than target
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max * a_max + acc * acc / 2) / j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      } else {
        if (vel + (a_max * a_max - acc * acc / 2) / j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    // If we are decelerating (a <= 0)
    else {
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max * a_max - acc * acc / 2) / j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / (a_peak));
          v12 = a_peak * t12;
        } else {
          a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
          t01 = roundToTimestep((acc - a_peak) / j_max);
          t23 = roundToTimestep(-a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      } else {
        if (vel + (a_max * a_max + acc * acc / 2) / j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
          v12 = a_peak * t12;
        } else {
          a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
          t01 = roundToTimestep((a_peak - acc) / j_max);
          t23 = roundToTimestep(a_peak / j_max);
          v01 = (acc + a_peak) * t01 / 2;
          v23 = a_peak * t23 / 2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    correct_int = ve - vel - v01 - v12 - v23;
    a_peak += correct_int / ((t01 + t23) / 2 + t12 + epsilon);
    double j01;
    if (t01 >= epsilon) {
      j01 = (a_peak - acc) / (t01);
    } else {
      j01 = 0;
    }
    double j12 = 0;
    double j23;
    if (t23 >= epsilon) {
      j23 = -a_peak / (t23);
    } else {
      j23 = 0;
    }
    if (t01 < 0 || t12 < 0 || t23 < 0) {
      // spdlog::info("planSafetyShield calculated time negative. t01 = {}, t12 = {}, t23 = {}", t01, t12, t23);
      return false;
    }
    std::vector<double> new_times = {t01, t01 + t12, t01 + t12 + t23};
    std::vector<double> new_jerks = {j01, j12, j23};
    path.setPhases(new_times, new_jerks);
    return true;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::planSafetyShield: {}", exc.what());
    return false;
  }
}

void SafetyShield::calculateMaxAccJerk(const std::vector<double>& prev_speed, const std::vector<double>& a_max_part,
                                       const std::vector<double>& j_max_part, double& a_max_manoeuvre,
                                       double& j_max_manoeuvre) {
  double new_c, new_d;
  // Calculate dds_max
  double denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_c = (a_max_allowed_[0] - std::abs(a_max_part[0])) / denom;
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_c = (a_max_allowed_[i] - std::abs(a_max_part[i])) / denom;
    min_c = (new_c < min_c) ? new_c : min_c;
  }
  a_max_manoeuvre = (min_c < 0) ? 0 : min_c;
  // Calculate ddds_max
  denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_d = (j_max_allowed_[0] - 3 * std::abs(a_max_part[0]) * a_max_manoeuvre - std::abs(j_max_part[0])) / denom;
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_d = (j_max_allowed_[i] - 3 * std::abs(a_max_part[i]) * a_max_manoeuvre - std::abs(j_max_part[i])) / denom;
    min_d = (new_d < min_d) ? new_d : min_d;
  }
  j_max_manoeuvre = (min_d < 0) ? 0 : min_d;
}

Motion SafetyShield::computesPotentialTrajectory(bool v, const std::vector<double>& prev_speed) {
  try {
    // s_int indicates the index of the entire traveled way
    while (path_s_ >= (path_s_discrete_ + 1) * sample_time_) {
      long_term_trajectory_.increasePosition();
      if (new_ltt_) {
        new_long_term_trajectory_.increasePosition();
      }
      path_s_discrete_++;
    }

    // find maximum acceleration and jerk authorised
    double a_max_manoeuvre, j_max_manoeuvre;
    // One could use new_long_term_trajectory_.getMaxAccelerationWindow(path_s_discrete_) instead of a_max_ltt but there
    // are many bugs to be solved before.
    if (!new_ltt_) {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
    } else {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
    }

    intended_path_correct_ =
          planSafetyShield(verified_path_.getPosition(), verified_path_.getVelocity(), verified_path_.getAcceleration(),
                           1, a_max_manoeuvre, j_max_manoeuvre, intended_path_);
    // spdlog::info("Intended path planning correct = {}.", intended_path_correct_);
    std::vector<double> monitored_path_times = {};
    std::vector<double> monitored_path_jerks = {};
    // Only plan new failsafe trajectory if the intended path planning was successful.
    if (intended_path_correct_) {
      // Add one time step of the intended path to the new monitored path.
      monitored_path_times.push_back(sample_time_);
      monitored_path_jerks.push_back(intended_path_.getJerk());
      // spdlog::info("Added intended step with jerk = {}.", intended_path_.getJerk());
      // advance one step on intended path
      intended_path_.increment(sample_time_);
      bool failsafe_planning_success;
      failsafe_planning_success =
          planSafetyShield(intended_path_.getPosition(), intended_path_.getVelocity(), intended_path_.getAcceleration(),
                           0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_);
      // Check the validity of the planned path
      // spdlog::info("Failsafe path planning correct = {}.", failsafe_planning_success);
      if (!failsafe_planning_success || intended_path_.getPosition() < failsafe_path_.getPosition()) {
        intended_path_correct_ = false;
      }
    }
    // If all planning was correct, use new failsafe path with single intended step
    if (intended_path_correct_) {
      // Combine the first time step of the intended path with the failsafe path!
      std::vector<double> failsafe_times = failsafe_path_.getTimes();
      std::vector<double> failsafe_jerks = failsafe_path_.getJerks();
      for (std::size_t i = 0; i < failsafe_times.size(); i++) {
        monitored_path_times.push_back(failsafe_times[i] + sample_time_);
        monitored_path_jerks.push_back(failsafe_jerks[i]);
      }
      monitored_path_ = Path(true, verified_path_.getPosition(), verified_path_.getVelocity(), verified_path_.getAcceleration(), 
                             monitored_path_times, monitored_path_jerks);
      // spdlog::info("New monitored path");
    } else {
      // If planning failed, use previous verified path
      monitored_path_ = verified_path_;
      // spdlog::info("Remain on verified path");
    }

    //// Calculate start and goal pos of intended motion
    double s_d, ds_d, dds_d;
    monitored_path_.getFinalMotion(s_d, ds_d, dds_d);
    double ddds_d = 0.0;
    // Calculate goal
    Motion goal_motion;
    if (new_ltt_) {
      goal_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    } else {
      goal_motion = long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d);
    }
    std::vector<double> monitored_times = monitored_path_.getTimes();
    goal_motion.setTime(monitored_times[monitored_times.size()-1]);
    return goal_motion;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
    throw exc;
    return Motion();
  }
}

/*
bool SafetyShield::planPFLFailsafe(double a_max_manoeuvre, double j_max_manoeuvre) {
  // Calculate maximal Carthesian velocity in the short-term plan
  double s_d, ds_d, dds_d;
  // Calculate goal
  bool is_under_iso_velocity = false;
  // v_max is maximum of LTT or STP and v_limit is how much path velocity needs to be scaled to be under v_iso
  // First, plan a failsafe path that ends in a complete stop. This is the longest failsafe path possible.
  bool planning_success =
      planSafetyShield(intended_path_.getPosition(), intended_path_.getVelocity(), intended_path_.getAcceleration(),
                       0.0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  if (!planning_success) {
    return false;
  }
  failsafe_path_2_.getFinalMotion(s_d, ds_d, dds_d);
  double v_max;
  if (!new_ltt_) {
    v_max = long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  } else {
    v_max = new_long_term_trajectory_.getMaxofMaximumCartesianVelocityWithS(s_d);
  }
  // Check if the entire long-term trajectory is under iso-velocity
  is_under_iso_velocity = v_max <= v_safe_;
  double v_limit;
  // TODO: if the current velocity is under iso-velocity, we dont need to compute the failsafe-path of the PFL-criterion
  if (!is_under_iso_velocity) {
    v_limit = v_safe_ / v_max;
    if (v_limit > 1 || v_limit < 0) {
      spdlog::error("v_limit not between 0 and 1");
    }
  } else {
    v_limit = intended_path_.getVelocity();
  }
  // This is the PFL failsafe path that ends in a velocity that is under v_iso.
  double eps_ds = 0.001;
  // Plan to be slightly lower than v_limit to eliminate numerical errors
  planning_success =
      planSafetyShield(intended_path_.getPosition(), intended_path_.getVelocity(), intended_path_.getAcceleration(),
                       v_limit - eps_ds, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
  // If the entire long-term trajectory is under iso-velocity, we dont need to check the velocity of the failsafe-path
  if (!is_under_iso_velocity) {
    // Check if the entire failsafe path is under iso-velocity
    // The failsafe path could start at d_s < 1, so the velocity of the failsafe path could be smaller than the LTT
    // velocity.
    double max_d_s = failsafe_path_2_.getMaxVelocity();
    is_under_v_limit_ = max_d_s < v_limit;
  } else {
    is_under_v_limit_ = true;
  }
  return planning_success;
}
*/

bool SafetyShield::checkMotionForJointLimits(Motion& motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (motion.getAngle()[i] < q_min_allowed_[i] || motion.getAngle()[i] > q_max_allowed_[i]) {
      return false;
    }
  }
  return true;
}

bool SafetyShield::checkTrajectoryForJointLimits(std::vector<Motion>& trajectory) {
  for (auto& motion : trajectory) {
    if (!checkMotionForJointLimits(motion)) {
      return false;
    }
  }
  return true;
}

std::vector<Motion> SafetyShield::goalPlanningRuckig(const Motion& start_motion, const Motion& goal_motion,
                                                     const double start_time) {
  std::vector<Motion> planned_trajectory = computeGoalTrajectory(
      start_motion, goal_motion, v_max_allowed_, a_max_allowed_, j_max_allowed_, sample_time_, start_time);
  return planned_trajectory;
}

std::vector<Motion> SafetyShield::computeFailsafeTrajectoryNonPathConsistent(const Motion& start_motion,
                                                                             const double start_time) {
  std::vector<Motion> failsafe_trajectory =
      computeStoppingTrajectory(start_motion, v_max_allowed_, a_max_allowed_, j_max_allowed_, sample_time_, start_time);
  return failsafe_trajectory;
}

std::vector<Motion> SafetyShield::computeMonitoredTrajectory(const Motion& current_motion,
                                                             const Motion& motion_after_intended_step) {
  std::vector<Motion> failsafe_trajectory =
      computeFailsafeTrajectoryNonPathConsistent(motion_after_intended_step, sample_time_);
  std::vector<Motion> monitored_trajectory;
  monitored_trajectory.reserve(1 + failsafe_trajectory.size());
  monitored_trajectory.push_back(current_motion);
  monitored_trajectory.insert(monitored_trajectory.end(), std::make_move_iterator(failsafe_trajectory.begin()),
                              std::make_move_iterator(failsafe_trajectory.end()));
  return monitored_trajectory;
}
// move to traj utils?
std::vector<Motion> SafetyShield::computeIntervalEdgeMotions(const std::vector<Motion>& monitored_trajectory,
                                                             const std::vector<double>& time_points) {
  std::vector<Motion> interval_edges_motions;
  interval_edges_motions.reserve(time_points.size());
  interval_edges_motions.emplace_back(monitored_trajectory[0]);
  int k = 1;
  double t = sample_time_;
  double eps = 1e-6;  // small epsilon to compare time points
  // Iterate through the monitored trajectory and collect motions at the specified time points
  for (int i = 1; i < monitored_trajectory.size(); ++i) {
    if (k >= time_points.size()) {
      break;  // no more time points to process
    }
    if (std::fabs(t - time_points[k]) < eps) {  // reached next requested stamp
      interval_edges_motions.emplace_back(monitored_trajectory[i]);
      ++k;
    }
    t += sample_time_;
  }
  return interval_edges_motions;
}

Motion SafetyShield::step(double cycle_begin_time) {
  cycle_begin_time_ = cycle_begin_time;
  try {
    // Get current motion
    Motion current_motion = getCurrentMotion();
    if (path_consistent_) {
      evaluateNewLTTProcessed(current_motion);
      // If there is a new long term trajectory (LTT), always override is_safe with false.
      if (new_ltt_ && !new_ltt_processed_) {
        is_safe_ = false;
      }
    }
    // Check if there is a new goal motion
    if (new_goal_) {
      newGoalPlanning(current_motion);
    }
    // compute monitored trajectory
    if (path_consistent_) {
      monitored_trajectory_ = stepPathConistent(current_motion);
    } else {
      try {
        monitored_trajectory_ = stepNonPathConistent(current_motion);
      } catch (const std::exception& e) {
        spdlog::warn("SafetyShield::step: stepNonPathConistent failed: {}", e.what());
        is_safe_ = false;
        spdlog::info("SafetyShield::step: stay on verified trajectory");
        updateSafePath(is_safe_);
        return getCurrentMotion();
      }
    }
    // create time points for sparse vector of motions
    const int N = monitored_trajectory_.size() - 1;
    std::vector<double> time_points_sparse =
        calcTimePointsForEquidistantIntervals(0, sample_time_ * N, reachability_set_duration_);
    // Compute interval edge motions
    std::vector<Motion> interval_edges_motions =
        computeIntervalEdgeMotions(monitored_trajectory_, time_points_sparse);
    // compute dynamics of interval edges motions
    LongTermTraj sparse_trajectory =
        buildTrajectory(interval_edges_motions, shield_type_ == ShieldType::PFL);
    // verify safety of the sparse trajectory
    is_safe_ = verifySafety(sparse_trajectory);
    // update the safe path that we want to continue on
    updateSafePath(is_safe_);
    // Select the next motion
    next_motion_ = getCurrentMotion();
    next_motion_.setTime(cycle_begin_time);
    new_ltt_processed_ = true;
    return next_motion_;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
    return getCurrentMotion();
  }
}

std::vector<Motion> SafetyShield::stepNonPathConistent(const Motion& current_motion) {
  // If not safe or new goal is set, plan new intended trajectory
  if (!is_safe_) {
    std::vector<Motion> intended_trajectory = goalPlanningRuckig(current_motion, new_goal_motion_);
    if (intended_trajectory.size() > 1) {
      intended_trajectory_ = intended_trajectory;
      intended_trajectory_index_ = 0;
    }
  }
  // Get next intended motion
  incrementTrajectoryIndex(intended_trajectory_index_, intended_trajectory_);
  Motion motion_after_intended_step = intended_trajectory_[intended_trajectory_index_];
  // Compute monitored trajectory (intended + failsafe)
  std::vector<Motion> monitored_trajectory = computeMonitoredTrajectory(current_motion, motion_after_intended_step);
  return monitored_trajectory;
}

std::vector<Motion> SafetyShield::stepPathConistent(const Motion& current_motion) {
  // Compute monitored trajectory
  Motion goal_motion = computesPotentialTrajectory(is_safe_, current_motion.getVelocity());
  // we have to start from time 0 as 1. in interpolation time will be set to 0 and 2. the time of
  // goal motion equals to the time of full braking but not relative to current time - it is
  // relative to 0
  std::vector<double> time_points_dense =
      calcTimePointsForEquidistantIntervals(0.0, goal_motion.getTime(), sample_time_);
  std::vector<Motion> monitored_trajectory = getMotionsFromCurrentLTTandPath(time_points_dense);
  return monitored_trajectory;
}

bool SafetyShield::verifySafety(const LongTermTraj& sparse_trajectory) {
  if (shield_type_ == ShieldType::OFF) {
    return true;
  }
  interval_edge_motions_ = sparse_trajectory.getTrajectory();
  time_points_ = sparse_trajectory.getTimePoints();
  // check trajectory for joint limits
  if (!checkTrajectoryForJointLimits(interval_edge_motions_)) {
    return false;
  }
  bool is_safe = false;
  std::vector<double> alpha_i = sparse_trajectory.getAlphaI();

  robot_capsules_time_intervals_ = robot_reach_->reachTimeIntervals(interval_edge_motions_, alpha_i);
  human_capsules_time_intervals_ = human_reach_->humanReachabilityAnalysisTimeIntervals(cycle_begin_time_, time_points_);
  int collision_index = -1;
  if (shield_type_ == ShieldType::PFL) {
    // Verify Enegry limits of potential contact scenarios
    is_safe = verifyContactEnergySafetyByContactType(sparse_trajectory, time_points_, collision_index);
  } else {
    // Verify if the robot and human reachable sets are collision free
    is_safe = verify_->verifyHumanReachTimeIntervals(robot_capsules_time_intervals_, human_capsules_time_intervals_,
                                                     collision_index);
  }
  // for visualization in hrgym, reachability sets of last timestep are used
  if (is_safe) {
    robot_capsules_ = robot_capsules_time_intervals_[robot_capsules_time_intervals_.size() - 1];
    human_capsules_ = human_capsules_time_intervals_[human_capsules_time_intervals_.size() - 1];
  } else {
    robot_capsules_ = robot_capsules_time_intervals_[collision_index];
    human_capsules_ = human_capsules_time_intervals_[collision_index];
  }
  return is_safe;
}

void SafetyShield::evaluateNewLTTProcessed(Motion& current_motion) {
  // If the new LTT was processed at least once and is labeled safe (stopped = safe for replanning), replace old LTT with new one.
  if (new_ltt_ && new_ltt_processed_ && (is_safe_ || current_motion.isStopped())) {
    long_term_trajectory_ = new_long_term_trajectory_;
    new_ltt_ = false;
    new_goal_ = false;
  }
}

void SafetyShield::newGoalPlanning(Motion& current_motion) {
  if (!new_goal_) {
    throw std::logic_error("SafetyShield::newGoalPlanning called without new goal.");
  }
  // Check if current motion has acceleration and jerk values that lie in the plannable ranges
  if (!checkCurrentMotionForReplanning(current_motion)) {
    new_ltt_ = false;
    return;
  }
  if (path_consistent_) {
    // Check if the starting position of the last replanning was very close to the current position
    bool last_close = new_ltt_ && abs(path_s_ - last_replan_s_) < sample_time_;
    // Only replan if the current joint position is different from the last.
    bool new_ltt_calculated = false;
    if (!last_close) {
      new_ltt_calculated = calculateLongTermTrajectory(current_motion, new_goal_motion_,
                                                       new_long_term_trajectory_);
    }
    // A replanning happend, so set the last replan position to the current position
    if (!last_close && new_ltt_calculated) {
      last_replan_s_ = path_s_;
    }
    // If we calculated a new LTT, we need to process it in the next step
    // If the last replan was close to the current position, we treat this like a new plan
    if (last_close || new_ltt_calculated) {
      new_ltt_ = true;
      new_ltt_processed_ = false;
    } else {
      new_ltt_ = false;
    }
  } else {
    std::vector<Motion> intended_trajectory = goalPlanningRuckig(current_motion, new_goal_motion_);
    if (intended_trajectory.size() > 1) {
      intended_trajectory_ = intended_trajectory;  // Only assign if valid
      intended_trajectory_index_ = 0;              // Reset index
      new_goal_ = false;
    } else {
      spdlog::warn("SafetyShield::newGoalPlanning: No valid intended trajectory calculated.");
      return;  // No valid trajectory, do not proceed
    }
  }
}
bool SafetyShield::verifyContactEnergySafetyByContactType(const LongTermTraj& sparse_trajectory,
                                                          std::vector<double> time_points,
                                                          int& collision_index) {
  std::vector<Motion> interval_edges_motions = sparse_trajectory.getTrajectory();
  std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator> vel_cap_start, vel_cap_end;
  buildRobotVelocityPointers(sparse_trajectory, vel_cap_start, vel_cap_end);
  std::vector<double> velocity_errors =
      robot_reach_->calculateMaxVelErrors(sample_time_, v_max_allowed_, a_max_allowed_, j_max_allowed_);
  std::vector<std::vector<double>> human_radii = human_reach_->getAllHumanRadii();
  std::vector<std::unordered_map<int, std::set<int>>> unclampable_body_part_maps = human_reach_->getUnclampableMap();
  std::unordered_map<int, std::set<int>> unclampable_enclosures_map = robot_reach_->getUnclampableEnclosures();
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertias_beginning_of_intervals =
      sparse_trajectory.getInertiaMatricesAtBeginningOfIntervals();
  return verify_->verifyHumanReachEnergyContactCases(
      robot_capsules_time_intervals_, human_capsules_time_intervals_, inertias_beginning_of_intervals,
      std::vector<Motion>(interval_edges_motions.begin(), interval_edges_motions.end() - 1),
      max_contact_energies_unconstrained_, max_contact_energies_constrained_, environment_elements_, human_radii,
      unclampable_body_part_maps, unclampable_enclosures_map, vel_cap_start, vel_cap_end, velocity_errors,
      collision_index);
}

void SafetyShield::buildRobotVelocityPointers(
    const LongTermTraj& sparse_trajectory,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_start,
    std::vector<std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator>& vel_cap_end) {
  for (int i = 0; i < sparse_trajectory.getLength() - 1; i++) {
    vel_cap_start.push_back(sparse_trajectory.getVelocityCapsuleIterator(i));
    vel_cap_end.push_back(sparse_trajectory.getVelocityCapsuleIterator(i + 1));
  }
}

Motion SafetyShield::getCurrentMotion() {
  Motion current_pos;
  current_pos = verified_trajectory_[verified_trajectory_index_];
  // we reset the time at each cycle
  // we can set time for both to 0 as done in interpolate however for ltt we keep the s parameter
  // how to handel s - currently just time - guess we can safely do
  current_pos.setTime(0.0);
  return current_pos;
}

bool SafetyShield::checkCurrentMotionForReplanning(Motion& current_motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (std::abs(current_motion.getAcceleration()[i]) > a_max_ltt_[i]) {
      return false;
    }
  }
  return true;
}

void SafetyShield::newLongTermTrajectory(const std::vector<double>& goal_position,
                                         const std::vector<double>& goal_velocity) {
  try {
    std::vector<double> motion_q;
    motion_q.reserve(nb_joints_);
    std::vector<double> motion_dq;
    motion_dq.reserve(nb_joints_);
    for (int i = 0; i < nb_joints_; i++) {
      motion_q.push_back(std::clamp(goal_position[i], q_min_allowed_[i] + planning_qpos_tolerance_,
                                    q_max_allowed_[i] - planning_qpos_tolerance_));
      motion_dq.push_back(std::clamp(goal_velocity[i], -v_max_allowed_[i], v_max_allowed_[i]));
    }
    new_goal_motion_ = Motion(cycle_begin_time_, motion_q, motion_dq);
    new_goal_ = true;
    new_ltt_ = false;
    new_ltt_processed_ = false;
    // High negative number to make sure we replan the LTT.
    // last_replan_s_ = -1E9;
  } catch (const std::exception& exc) {
    spdlog::error("Exception in SafetyShield::newLongTermTrajectory: {}", exc.what());
  }
}

void SafetyShield::setLongTermTrajectory(LongTermTraj& traj) {
  Motion current_motion = getCurrentMotion();
  // Check if robot is at stop
  if (!current_motion.isStopped()) {
    throw RobotMovementException();
  }
  Motion start = traj.getNextMotionAtIndex(0);
  // Check if traj starts at the same position
  if (!current_motion.hasSamePos(&start)) {
    throw TrajectoryException("Given LTT does not start at current robot position.");
  }
  // Check if traj starts at v=0
  if (!current_motion.hasSameVel(&start)) {
    std::stringstream ss;
    ss << "Given LTT does not start with velocity 0.0. Start velocity is [";
    for (size_t i = 0; i < start.getVelocity().size(); ++i) {
      if (i != 0) ss << ",";
      ss << start.getVelocity()[i];
    }
    ss << "].";
    std::string s = ss.str();
    throw TrajectoryException(s);
  }
  // Check if traj ends in stop
  if (!traj.getNextMotionAtIndex(traj.getLength() - 1).isStopped()) {
    throw TrajectoryException("Given LTT does not end in a complete stop of the robot (v = a = j = 0.0)");
  }
  // Replace LTT
  new_long_term_trajectory_ = traj;
  new_ltt_ = true;
  new_ltt_processed_ = true;
}

bool SafetyShield::calculateLongTermTrajectory(const Motion& start_motion, const Motion& goal_motion,
                                               LongTermTraj& ltt) {
  double new_time = path_s_;
  std::vector<Motion> new_traj = computeGoalTrajectory(start_motion, goal_motion, v_max_allowed_, 
                                                        a_max_ltt_, j_max_ltt_, sample_time_, path_s_);
  if (new_traj.empty()) {
    spdlog::error("calculateLongTermTrajectory: computeGoalTrajectory returned empty trajectory");
    return false;
  }
  ltt = LongTermTraj(new_traj, sample_time_, path_s_discrete_, v_max_allowed_, a_max_allowed_, j_max_allowed_,
                     alpha_i_max_);
  return true;
}

void SafetyShield::buildMaxContactEnergies() {
  max_contact_energies_unconstrained_ = {};
  max_contact_energies_constrained_ = {};
  // First, add the link contact energies
  for (int i = 0; i < nb_joints_ - 1; i++) {
    max_contact_energies_unconstrained_.push_back(human_reach_->getMaxContactEnergyFromType(false, ContactType::LINK));
    max_contact_energies_constrained_.push_back(human_reach_->getMaxContactEnergyFromType(true, ContactType::LINK));
  }
  // Then, add the EEF contact energies
  max_contact_energies_unconstrained_.push_back(human_reach_->getMaxContactEnergyFromType(false, eef_contact_type_));
  max_contact_energies_constrained_.push_back(human_reach_->getMaxContactEnergyFromType(true, eef_contact_type_));
}

std::vector<Motion> SafetyShield::getMotionsFromCurrentLTTandPath(const std::vector<double>& time_points) {
  LongTermTraj& ltt = long_term_trajectory_;
  if (new_ltt_) {
    ltt = new_long_term_trajectory_;
  }
  return getMotionsOfAllTimeStepsFromPath(ltt, monitored_path_, time_points);
}

LongTermTraj SafetyShield::buildTrajectory(const std::vector<Motion>& interval_edges_motion,
                                           bool compute_dynamics) {
  LongTermTraj trajectory;
  if (compute_dynamics) {
    trajectory = LongTermTraj(interval_edges_motion, sample_time_, *robot_reach_, path_s_discrete_,
                              v_max_allowed_, a_max_allowed_, j_max_allowed_, sliding_window_k_);
  } else {
    trajectory = LongTermTraj(interval_edges_motion, sample_time_, path_s_discrete_, v_max_allowed_,
                              a_max_allowed_, j_max_allowed_, sliding_window_k_, alpha_i_max_);
  }
  return trajectory;
}

std::vector<std::vector<double>> SafetyShield::computeCapsuleSpeedsTimeIntervals() {
  std::vector<std::vector<double>> capsule_speeds;
  capsule_speeds.reserve(interval_edge_motions_.size() - 1);
  for (int i = 0; i < static_cast<int>(interval_edge_motions_.size()) - 1; ++i) {
    robot_reach_->calculateAllTransformationMatricesAndCapsules(interval_edge_motions_[i].getAngleRef());
    std::vector<RobotReach::CapsuleVelocity> cap_vels =
        robot_reach_->calculateAllCapsuleVelocities(interval_edge_motions_[i].getVelocityRef());
    std::vector<double> speeds;
    speeds.reserve(cap_vels.size());
    for (const auto& cap_vel : cap_vels) {
      speeds.push_back(cap_vel.v1.v.norm());
    }
    capsule_speeds.push_back(std::move(speeds));
  }
  return capsule_speeds;
}
}  // namespace safety_shield
   // namespace safety_shield
