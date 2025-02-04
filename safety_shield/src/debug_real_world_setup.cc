#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>

#include "reach_lib.hpp"
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

void get_meas_from_file(std::string file_path, int input_freq, std::vector<double>& time, std::vector<std::vector<reach_lib::Point>>& mocap_data) {
  std::ifstream file(file_path);
  std::string line;

  if (!file.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    return;
  }

  // Read the file line by line
  int c = 0;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> point;
    std::vector<reach_lib::Point> mocap_data_row;
    c++;
    if (c % input_freq != 0) {
      continue;
    }

    // Read the time value
    std::getline(ss, cell, ',');
    double t = std::stod(cell);
    time.push_back(t);

    // Read the x, y, z values and skip the e values
    int column_index = 1;
    while (std::getline(ss, cell, ',')) {
      if (column_index % 4 != 0) { // Skip the e columns (every 4th column)
        point.push_back(std::stod(cell));
      } else {
        // Build point
        reach_lib::Point p(point[0], point[1], point[2]);
        mocap_data_row.push_back(p);
        point.clear();
      }
      column_index++;
    }
    mocap_data.push_back(mocap_data_row);
  }
  file.close();
  return;
}

std::vector<reach_lib::Point> get_human_measurement_far() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[1] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[2] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[3] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[4] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[5] =  reach_lib::Point(0.16, -0.69, -0.05);
  dummy_human_meas[6] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[7] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[8] =  reach_lib::Point(10.0, 10.0, 10.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> get_human_measurement_close() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[1] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[2] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[3] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[4] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[5] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[6] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[7] =  reach_lib::Point(0.0, 0.0, 0.0);
  dummy_human_meas[8] =  reach_lib::Point(0.0, 0.0, 0.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> get_human_measurement_clamp() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[1] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[2] =  reach_lib::Point(0.033, -0.614, -0.1);
  dummy_human_meas[3] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[4] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[5] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[6] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[7] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[8] =  reach_lib::Point(0.0, -10.0, 0.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> get_human_measurement_clamp_2() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[1] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[2] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[3] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[4] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[5] =  reach_lib::Point(0.3717, -0.5504, -0.2033);
  dummy_human_meas[6] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[7] =  reach_lib::Point(0.0, -10.0, 0.0);
  dummy_human_meas[8] =  reach_lib::Point(0.0, -10.0, 0.0);
  return dummy_human_meas;
}

std::vector<reach_lib::Point> get_human_measurement_pendulum() {
  std::vector<reach_lib::Point> dummy_human_meas(9);
  dummy_human_meas[0] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[1] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[2] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[3] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[4] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[5] =  reach_lib::Point(0.16, -0.69, -0.05);
  dummy_human_meas[6] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[7] =  reach_lib::Point(10.0, 10.0, 10.0);
  dummy_human_meas[8] =  reach_lib::Point(10.0, 10.0, 10.0);
  return dummy_human_meas;
}

int main() {
  double sample_time = 0.006;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/human_reach_TUM_lab.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = -0.054;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = -M_PI_2;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({-1.50, -0.88, init_z - 0.2}, {1.05, 1.0, init_z - 0.176});
  std::vector<reach_lib::AABB> environment_elements = {table};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::PFL;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type);

  std::vector<std::vector<double>> desired_goals_pendulum = {
    {-1.4, 1.5, -0.4, M_PI_2, -M_PI_2, 0.0},
    {0.3, 1.5, -0.4, M_PI_2, -M_PI_2, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  std::vector<std::vector<double>> desired_goals_clamping_table = {
    {0, 1.1, -1.1, 0, -0.75, 0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  std::vector<std::vector<double>> desired_goals_clamping_table_2 = {
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };
  std::vector<std::vector<double>> demo_trajectory = {
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {-0.65, 1.15, -1.15, 0.0, -0.9, 0.0},
    {-0.65,  6.05618294e-01, -7.85817200e-01,  4.31075533e-04, -1.74986551e+00, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  };

  // Define scenario
  std::vector<std::vector<double>> desired_goals = demo_trajectory;
  // auto human_measurement = get_human_measurement_clamp_2();
  std::vector<std::vector<reach_lib::Point>> human_measurements;
  std::vector<double> time;
  // Recording is in 1000 Hz but sample time is 0.006 s
  int input_freq = 6;
  get_meas_from_file("../data/out_ssm_jakob_mocap.csv", 6, time, human_measurements);

  // n = shield_frequency / RL_frequency = 5 / 250 = 1 / 50
  spdlog::info("Debug started.");
  int goal_number = 0;
  std::vector<double> goal = desired_goals[goal_number];
  shield.newLongTermTrajectory(goal, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  bool episode_finished = false;
  bool use_close_measurement = false;
  double t = 0.0;
  double start_time = 60.0;
  double end_time = 360.0;
  int measurement_counter = 0;
  clock_t clkStart;
  clock_t clkFinish;
  std::ofstream myfile;
  myfile.open("../data/ssm_jakob_simulated.csv");
  myfile << "time, is_safe, collision_type, calc_time, q_1, q_2, q_3, q_4, q_5, q_6, dq_1, dq_2, dq_3, dq_4, dq_5, dq_6, ddq_1, ddq_2, ddq_3, ddq_4, ddq_5, ddq_6" << std::endl;
  
  while(time[measurement_counter] < end_time) {
    if (time[measurement_counter] < start_time) {
      measurement_counter++;
      continue;
    }
    auto human_measurement = human_measurements[measurement_counter];
    measurement_counter++;
    shield.humanMeasurement(human_measurement, t);
    t += sample_time;
    clkStart = clock();
    safety_shield::Motion next_motion = shield.step(t);
    clkFinish = clock();
    double calc_time = (double)(clkFinish - clkStart) / CLOCKS_PER_SEC;
    bool is_safe = shield.getSafety();
    safety_shield::CollisionType collision_type = shield.getCollisionType();

    myfile << time[measurement_counter] << ", " << is_safe << ", " << (int) collision_type << ", " << calc_time << ", ";
    for (int i = 0; i < 6; i++) {
      myfile << next_motion.getAngle()[i] << ", ";
    }
    for (int i = 0; i < 6; i++) {
      myfile << next_motion.getVelocity()[i] << ", ";
    }
    for (int i = 0; i < 6; i++) {
      if (i == 5) {
        myfile << next_motion.getAcceleration()[i] << std::endl;
        break;
      }
      myfile << next_motion.getAcceleration()[i] << ", ";
    }
    safety_shield::Motion* goal_motion = new safety_shield::Motion(0.0, desired_goals[goal_number]);
    if (next_motion.hasSamePos(goal_motion, 0.03)) {
      goal_number++;
      if (goal_number == desired_goals.size()) {
        episode_finished = true;
        shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
        goal_number = 0;
      }
      std::vector<double> goal = desired_goals[goal_number];
      shield.newLongTermTrajectory(goal, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    delete goal_motion;
  }
  myfile.close();
  spdlog::info("Debug finished.");
  return 0;
}
