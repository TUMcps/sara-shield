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

int main(int argc, char* argv[]) {

  std::string shield_type_str = "PFL";
  safety_shield::ShieldType shield_type;
  if (argc >= 2) {
    shield_type_str = argv[1];
  }
  if (shield_type_str == "SSM") {
    shield_type = safety_shield::ShieldType::SSM;
  } else if (shield_type_str == "PFL") {
    shield_type = safety_shield::ShieldType::PFL;
  } else if (shield_type_str == "REDUCEDSPEED") {
    shield_type = safety_shield::ShieldType::REDUCEDSPEED;
  } else if (shield_type_str == "SWITCH") {
    shield_type = safety_shield::ShieldType::SWITCH;
  } else if (shield_type_str == "REFLECTEDMASS") {
    shield_type = safety_shield::ShieldType::REFLECTEDMASS;
  } else if (shield_type_str == "ZONESSM") {
    shield_type = safety_shield::ShieldType::ZONESSM;
  } else if (shield_type_str == "OFF") {
    shield_type = safety_shield::ShieldType::OFF;
  } else {
    throw std::invalid_argument("Invalid shield type.");
  }

  std::string contact_type_str = "BLUNT";
  if (argc >= 3) {
    contact_type_str = argv[2];
  }
  safety_shield::ContactType contact_type;
  if (contact_type_str == "BLUNT") {
    contact_type = safety_shield::ContactType::BLUNT;
  } else if (contact_type_str == "WEDGE") {
    contact_type = safety_shield::ContactType::WEDGE;
  } else if (contact_type_str == "EDGE") {
    contact_type = safety_shield::ContactType::EDGE;
  } else if (contact_type_str == "SHEET") {
    contact_type = safety_shield::ContactType::SHEET;
  } else {
    throw std::invalid_argument("Invalid contact type.");
  }

  // Alternative: human_reach_ablation_no_contact_class
  std::string mocap_file_option = "human_reach_TUM_lab";
  if (argc >= 4) {
    mocap_file_option = std::string(argv[3]);
  }
  std::string mocap_config_file = "../config/" + mocap_file_option + ".yaml";

  std::string file_str = "vicon_p_3_robco_mocap";
  if (argc >= 5) {
    file_str = std::string(argv[4]);
  }
  std::string file_name = "ablation/" + file_str;
  std::string output_file_name = "ablation_results_panda/" + file_str + "_" + shield_type_str + "_" + contact_type_str + "_" + mocap_file_option;

  double sample_time = 0.006;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_panda.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_panda.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = -M_PI_2;
  std::vector<double> init_qpos = {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256};
  reach_lib::AABB table = reach_lib::AABB({-1.50, -0.88, init_z - 0.2}, {1.05, 1.0, init_z - 0.176});
  std::vector<reach_lib::AABB> environment_elements = {table};

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type, contact_type);

  std::vector<std::vector<double>> demo_trajectory = {
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {1.0, 0.533267277686203, -0.020114979003495896, -2.3596095499198384, 0.030138281349929918, 2.9126388093456885, 0.6678120509939178},
    {1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
    {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256},
  };

  // Define scenario
  std::vector<std::vector<double>> goals = demo_trajectory;

  // precompute long term trajectories
  std::vector<safety_shield::LongTermTraj>* precomputed_llts = new std::vector<safety_shield::LongTermTraj>(goals.size() - 1);
  shield.calculateLongTermTrajectory(goals[0], { 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0 }, goals[1], (*precomputed_llts)[0], 0.0);
  for (int i = 1; i < (goals.size() - 1); i++) {
    safety_shield::LongTermTraj previous_ltt = (*precomputed_llts)[i - 1];
    safety_shield::Motion previous_ltt_motion = previous_ltt.getMotion(previous_ltt.getLength() - 1);
    shield.calculateLongTermTrajectory(previous_ltt_motion.getAngle(), { 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0 }, goals[i + 1], (*precomputed_llts)[i], previous_ltt_motion.getTime() + sample_time);
  }
  int current_ltt_idx = 0;
  shield.setLongTermTrajectory((*precomputed_llts)[current_ltt_idx]);

  // auto human_measurement = get_human_measurement_clamp_2();
  std::vector<std::vector<reach_lib::Point>> human_measurements;
  std::vector<double> time;
  // Set this to 6 if the recording is in 1000 Hz but sample time is 0.006 s
  int input_freq = 1;
  // std::string file_name = "out_sim_ssm_near_mocap";
  get_meas_from_file("../data/" + file_name + ".csv", input_freq, time, human_measurements);

  // n = shield_frequency / RL_frequency = 5 / 250 = 1 / 50
  spdlog::info("Debug started.");
  double t = 0.0;
  double start_time = 10.0;
  double end_time = 190.0;
  int measurement_counter = 0;
  double path_s = 0.0;
  double last_s = 0.0;
  clock_t clkStart;
  clock_t clkFinish;
  std::ofstream myfile;
  std::filesystem::create_directories(std::filesystem::path("../data/" + output_file_name + ".csv").parent_path());
  myfile.open("../data/" + output_file_name + ".csv");
  myfile << "time, path_s, is_safe, collision_type, calc_time, ";
  myfile << "q_1, q_2, q_3, q_4, q_5, q_6, q_7, ";
  myfile << "dq_1, dq_2, dq_3, dq_4, dq_5, dq_6, dq_7, ";
  myfile << "ddq_1, ddq_2, ddq_3, ddq_4, ddq_5, ddq_6, ddq_7, ";
  myfile << "human_pos_1_x, human_pos_1_y, human_pos_1_z, ";
  int n_human_caps = 8;
  for (int i = 0; i < n_human_caps; i++) {
    myfile << "cap_h_" << i << "_x_1, cap_h_" << i << "_y_1, " << "cap_h_" << i << "_z_1, "; 
    myfile << "cap_h_" << i << "_x_2, cap_h_" << i << "_y_2, " << "cap_h_" << i << "_z_2, ";
    myfile << "cap_h_" << i << "_r, ";
  }
  int n_robot_caps = 7;
  for (int i = 0; i < n_robot_caps; i++) {
    myfile << "cap_r_" << i << "_x_1, cap_r_" << i << "_y_1, " << "cap_r_" << i << "_z_1, "; 
    myfile << "cap_r_" << i << "_x_2, cap_r_" << i << "_y_2, " << "cap_r_" << i << "_z_2, ";
    if (i == n_robot_caps -1) {
      myfile << "cap_r_" << i << "_r";
      continue;
    }
    myfile << "cap_r_" << i << "_r, ";
  }
  myfile << std::endl;

  while(time[measurement_counter] < end_time && measurement_counter < human_measurements.size()-1) {
    if (time[measurement_counter] < start_time) {
      measurement_counter++;
      continue;
    }
    auto human_measurement = human_measurements[measurement_counter];
    t += sample_time;
    shield.humanMeasurement(human_measurement, t);
    clkStart = clock();
    safety_shield::Motion next_motion = shield.step(t);
    path_s = path_s + next_motion.getS() - last_s;
    last_s = next_motion.getS();
    clkFinish = clock();
    double calc_time = (double)(clkFinish - clkStart) / CLOCKS_PER_SEC;
    bool is_safe = shield.getSafety();
    safety_shield::CollisionType collision_type = shield.getCollisionType();

    myfile << time[measurement_counter] << ", " << path_s << ", " << is_safe << ", " << (int) collision_type << ", " << calc_time << ", ";
    for (int i = 0; i < n_robot_caps; i++) {
      myfile << next_motion.getAngle()[i] << ", ";
    }
    for (int i = 0; i < n_robot_caps; i++) {
      myfile << next_motion.getVelocity()[i] << ", ";
    }
    for (int i = 0; i < n_robot_caps; i++) {
      myfile << next_motion.getAcceleration()[i] << ", ";
    }
    myfile << human_measurement[0].x << ", " << human_measurement[0].y << ", " << human_measurement[0].z << ",";
    if (shield_type != safety_shield::ShieldType::OFF) {
      std::vector<std::vector<double>> human_capsules = shield.getHumanReachCapsules();
      for (int i = 0; i < n_human_caps; i++) {
        myfile << human_capsules[i][0] << ", " << human_capsules[i][1] << ", " << human_capsules[i][2] << ", " << human_capsules[i][3] << ", " << human_capsules[i][4] << ", " << human_capsules[i][5] << ", " << human_capsules[i][6] << ", ";
      }
      std::vector<std::vector<double>> robot_capsules = shield.getRobotReachCapsules();
      for (int i = 0; i < n_robot_caps; i++) {
        myfile << robot_capsules[i][0] << ", " << robot_capsules[i][1] << ", " << robot_capsules[i][2] << ", " << robot_capsules[i][3] << ", " << robot_capsules[i][4] << ", " << robot_capsules[i][5] << ", " << robot_capsules[i][6];
        if (i == n_robot_caps -1) {
          break;
        }
        myfile << ", ";
      }
    } else {
      for (int i = 0; i < n_human_caps; i++) {
        myfile << "0, 0, 0, 0, 0, 0, 0";
      }
      for (int i = 0; i < n_robot_caps; i++) {
        myfile << "0, 0, 0, 0, 0, 0, 0, 0";
        if (i == n_robot_caps -1) {
          break;
        }
        myfile << ", ";
      }
    }
    
    myfile << std::endl;
    // Next trajectory switch
    double motion_s = next_motion.getS();
    safety_shield::LongTermTraj current_traj = (*precomputed_llts)[current_ltt_idx];
    safety_shield::Motion last_motion_current_traj = current_traj.getMotion(current_traj.getLength() - 1);

    int starting_index = (*precomputed_llts)[current_ltt_idx].getStartingIndex();
    int current_ltt_length = (*precomputed_llts)[current_ltt_idx].getLength();
    int end_index = starting_index + current_ltt_length - 1;
    if (static_cast<int>(motion_s / sample_time) == (end_index + 1)) {
      current_ltt_idx = (current_ltt_idx + 1) % (goals.size() - 1);
      if (current_ltt_idx == 0) {
        std::vector<double> init_qpos = {-1.0, -0.48962515831014314, -0.0057917847133297975, -2.305122170785585, 0.008369797545514732, 1.8581314941538343, 0.7974554929694256};
        shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, 0.0, environment_elements, shield_type);
        t = 0.0;
        last_s = 0.0;
      }

      if (!next_motion.isStopped(5e-3, 0.5, 401) || !next_motion.hasSamePos(&last_motion_current_traj, 1e-2)) {
        std::stringstream ss;
        ss << "next_motion.isStopped(5e-3, 0.5, 401) = ";
        ss << next_motion.isStopped(5e-3, 0.5, 401);
        ss << "\n";
        ss << "next_motion.hasSamePos(&last_motion_current_traj, 1e-7)) = ";
        ss << next_motion.hasSamePos(&last_motion_current_traj, 1e-7);
        std::string s = ss.str();
        throw safety_shield::TrajectoryException(s);
      }
      shield.setLongTermTrajectory((*precomputed_llts)[current_ltt_idx]);
    }
    measurement_counter++;
  }
  myfile.close();
  myfile.open("../data/" + output_file_name + "_path_s.csv");
  myfile << path_s << std::endl;
  myfile.close();
  spdlog::info("Debug finished.");
  return 0;
}
