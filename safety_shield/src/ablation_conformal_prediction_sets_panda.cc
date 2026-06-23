#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <time.h>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

int main() {
  double sample_time = 0.001;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_panda.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_panda.yaml");
  std::string mocap_config_file = std::string("../config/mujoco_mocap.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256};
  reach_lib::AABB table = reach_lib::AABB({-1.50, -0.88, init_z - 0.2}, {1.05, 1.0, init_z - 0.176});
  std::vector<reach_lib::AABB> environment_elements = {table};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::SSM;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements,
                                  shield_type);

  // Dummy human measurement: Far away so that shield is always safe.
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] =  reach_lib::Point(10.0, 10.0,  0.3653);
  dummy_human_meas[1] =  reach_lib::Point(10.0, 10.0,  0.3661);
  dummy_human_meas[2] =  reach_lib::Point(10.0, 10.0,  0.5656);
  dummy_human_meas[3] =  reach_lib::Point(10.0, 10.0,  0.35673341);
  dummy_human_meas[4] =  reach_lib::Point(10.0, 10.0,  0.35305421);
  dummy_human_meas[5] =  reach_lib::Point(10.0, 10.0,  0.70079994);
  dummy_human_meas[6] =  reach_lib::Point(10.0, 10.0,  0.30875003);
  dummy_human_meas[7] =  reach_lib::Point(10.0, 10.0,  0.30646656);
  dummy_human_meas[8] =  reach_lib::Point(10.0, 10.0,  0.75369877);
  dummy_human_meas[9] =  reach_lib::Point(10.0, 10.0,  0.42744498);
  dummy_human_meas[10] = reach_lib::Point(10.0, 10.0,  0.42934275);
  dummy_human_meas[11] = reach_lib::Point(10.0 ,10.0,  0.96759635);
  dummy_human_meas[12] = reach_lib::Point(10.0, 10.0,  0.87549717);
  dummy_human_meas[13] = reach_lib::Point(10.0, 10.0,  0.87259628);
  dummy_human_meas[14] = reach_lib::Point(10.0, 10.0,  1.03259597);
  dummy_human_meas[15] = reach_lib::Point(10.0, 10.0,  0.90599443);
  dummy_human_meas[16] = reach_lib::Point(10.0, 10.0,  0.90509315);
  dummy_human_meas[17] = reach_lib::Point(10.0, 10.0,  0.89318002);
  dummy_human_meas[18] = reach_lib::Point(10.0, 10.0,  0.89177984);
  dummy_human_meas[19] = reach_lib::Point(10.0, 10.0,  0.90218228);
  dummy_human_meas[20] = reach_lib::Point(10.0, 10.0,  0.89958196);
  dummy_human_meas[21] = reach_lib::Point(10.0, 10.0,  0.89398712);
  dummy_human_meas[22] = reach_lib::Point(10.0, 10.0,  0.8933868);

  std::vector<std::vector<double>> goals = {
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256}
  };

  // Compute and set the first trajectory segment.
  int n_joints = 7;
  int n_robot_caps = 7;
  std::vector<double> zeros(n_joints, 0.0);
  int current_goal_idx = 1;

  safety_shield::Motion start_m(0.0, goals[0], zeros);
  safety_shield::Motion goal_m(0.0, goals[current_goal_idx], zeros);
  safety_shield::LongTermTraj current_ltt;
  shield.calculateLongTermTrajectory(start_m, goal_m, current_ltt);
  shield.setLongTermTrajectory(current_ltt);

  int ltt_starting_index = current_ltt.getStartingIndex();
  int ltt_end_index = ltt_starting_index + current_ltt.getLength() - 1;

  spdlog::info("Debug started.");
  std::string output_file_name = "ablation_conformal_prediction_sets_panda";
  double t = 0.0;
  double path_s = 0.0;
  double last_s = 0.0;

  std::filesystem::create_directories(std::filesystem::path("../data/" + output_file_name + ".csv").parent_path());
  std::ofstream myfile;
  myfile.open("../data/" + output_file_name + ".csv");

  // One row per (step, interval).
  // Columns: time, path_s, interval_idx, tp_start, tp_end,
  //          then for each robot capsule: x1, y1, z1, x2, y2, z2, r, speed
  myfile << "time, path_s, interval_idx, tp_start, tp_end";
  for (int i = 0; i < n_robot_caps; i++) {
    myfile << ", cap_r_" << i << "_x1, cap_r_" << i << "_y1, cap_r_" << i << "_z1";
    myfile << ", cap_r_" << i << "_x2, cap_r_" << i << "_y2, cap_r_" << i << "_z2";
    myfile << ", cap_r_" << i << "_r";
    myfile << ", cap_r_" << i << "_speed";
  }
  myfile << "\n";

  while (true) {
    t += sample_time;
    shield.humanMeasurement(dummy_human_meas, t);
    safety_shield::Motion next_motion = shield.step(t);
    path_s = path_s + next_motion.getS() - last_s;
    last_s = next_motion.getS();

    // Retrieve per-interval data for this step.
    std::vector<double> time_points = shield.getTimePoints();
    std::vector<std::vector<std::vector<double>>> robot_capsules = shield.getAllRobotReachCapsulesOverTime();
    std::vector<std::vector<double>> capsule_speeds = shield.computeCapsuleSpeedsTimeIntervals();

    int n_intervals = static_cast<int>(robot_capsules.size());
    for (int k = 0; k < n_intervals; k++) {
      myfile << t << ", " << path_s << ", " << k
             << ", " << time_points[k] << ", " << time_points[k + 1];
      for (int i = 0; i < n_robot_caps; i++) {
        const auto& cap = robot_capsules[k][i];
        myfile << ", " << cap[0] << ", " << cap[1] << ", " << cap[2]
               << ", " << cap[3] << ", " << cap[4] << ", " << cap[5]
               << ", " << cap[6];
        myfile << ", " << capsule_speeds[k][i];
      }
      myfile << "\n";
    }

    // Check if the current trajectory segment is complete.
    double motion_s = next_motion.getS();
    if (static_cast<int>(motion_s / sample_time) == (ltt_end_index + 1)) {
      current_goal_idx++;
      if (current_goal_idx >= (int)goals.size()) {
        break;
      }
      spdlog::info("Computing new LTT.");
      // Compute next trajectory. path_s_discrete_ is now at the correct starting index.
      safety_shield::Motion new_start(0.0, goals[current_goal_idx - 1], zeros);
      safety_shield::Motion new_goal(0.0, goals[current_goal_idx], zeros);
      shield.calculateLongTermTrajectory(new_start, new_goal, current_ltt);
      shield.setLongTermTrajectory(current_ltt);
      ltt_starting_index = current_ltt.getStartingIndex();
      ltt_end_index = ltt_starting_index + current_ltt.getLength() - 1;
    }
  }
  myfile.close();

  myfile.open("../data/" + output_file_name + "_path_s.csv");
  myfile << path_s << "\n";
  myfile.close();

  spdlog::info("Debug finished.");
  return 0;
}
