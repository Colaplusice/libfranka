// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include "examples_common.h"
/**
 * @example generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>  <is_gripper: 0 or 1>" << std::endl;
    return -1;
  }
  try {
    freopen("output.txt", "w", stdout);
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    static franka::Model model = robot.loadModel();
    setDefaultBehavior(robot);
    static franka::RobotState initial_state;
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

  
    bool is_gripper = std::stoi(argv[2]);

    if (is_gripper) {
      franka::Gripper gripper(argv[1]);
      gripper.homing();
      double grasping_width = 0.0005;
      franka::GripperState gripper_state = gripper.readOnce();
      std::cout << "this is width:"
                << "  " << gripper_state.max_width << "  " << grasping_width;
      if (gripper_state.max_width < grasping_width) {
        std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
        return -1;
      }
      // Grasp the object.
      if (!gripper.grasp(grasping_width, 0.1, 60)) {
        std::cout << "Failed to grasp object." << std::endl;
        return -1;
      }
      // Wait 3s and check afterwards, if the object is still grasped.
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

      gripper_state = gripper.readOnce();
      if (!gripper_state.is_grasped) {
        std::cout << "Object lost." << std::endl;
        return -1;
      }
    }

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    // running configuration
    static double time_max = 10;
    static double omega_max = 1.0;
    static double init_error_x;
    static double init_error_y;
    static double init_error_z;
    double beta = 0.08;
    // Eigen::Matrix<double, 7, 1> noise;
    // noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    // 0.1 0.1 0.1
    Eigen::Matrix<double, 6, 1> three_noise;
    // three_noise << 0.5, 0.5,0,0,0,0,0;
    three_noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    static Eigen::Matrix<double, 6, 1> feedback;
    feedback << 0, 0, 0, 0, 0, 0;
    double time = 0.0;
    constexpr double r = 0.05;
    constexpr double alpha = M_PI / 6;

    // gripper function
    std::array<double, 16> initial_pose;
    auto gripper_function = [&time, &initial_pose](
                                const franka::RobotState& robot_state,
                                franka::Duration period) -> franka::CartesianPose {
      time += period.toSec() * 0.3;
      // std::cout<<"this is time "<<time<<std::endl;
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
        print_position(initial_pose);
      }
      double rz = 0.0069 * time;
      std::array<double, 16> new_pose = initial_pose;
      new_pose[14] -= rz;
      if (time >= 10) {
        for (int i = 0; i < 16; i++) {
          std::cout << robot_state.O_T_EE_c[i] << "  ";
          if (i % 4 == 0)
            std::cout << std::endl;
        }

        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    };
    // gripper_function finished
    // draw_function

    time = 0;
    auto draw_function = [=, &time](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::JointVelocities {
      if (time == 0) {
        initial_state = robot_state;
        init_error_x = robot_state.O_T_EE[12] - initial_state.O_T_EE[12];
        init_error_y = robot_state.O_T_EE[13] - initial_state.O_T_EE[13];
        init_error_z = robot_state.O_T_EE[14] - initial_state.O_T_EE[14];
      }

      time += period.toSec();
      // origin joint velocity
      // double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      // double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
      // get state variables
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());
      Eigen::Matrix<double, 6, 7> half_jacobian;
      half_jacobian << jacobian;
      Eigen::MatrixXd pse_inverse_jacobian = pseudoinverse(half_jacobian);


      double phi_sin = 2 * M_PI * sin(0.5 * M_PI * time / time_max);
      double phi_gds = phi_sin * sin(0.5 * M_PI * time / time_max);
      double phiDot = phi_sin * M_PI * cos(0.5 * M_PI * time / time_max) / time_max;

      double r_gds = 0.08;
      double rx =
          r_gds * std::cos(2 * phi_gds) * std::cos(phi_gds) + initial_state.O_T_EE[12] - r_gds;
      double ry = r_gds * std::cos(2 * phi_gds) * std::sin(phi_gds) + initial_state.O_T_EE[13];
      double rz = initial_state.O_T_EE[14];

      double drx = -r_gds * phiDot *
                   (2 * std::sin(2 * phi_gds) * std::cos(phi_gds) +
                    std::cos(2 * phi_gds) * std::sin(phi_gds));
      double dry = -r_gds * phiDot *
                   (2 * std::sin(2 * phi_gds) * std::sin(phi_gds) -
                    std::cos(2 * phi_gds) * std::cos(phi_gds));
      double drz = 0;
      Eigen::Matrix<double, 3, 1> rr;
      Eigen::Matrix<double, 3, 1> rd;
      rd << rx, ry, rz;
      rr << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
      Eigen::Matrix<double, 6, 1> qt;
      qt << drx, dry, drz, 0, 0, 0;
      feedback += half_jacobian * dq - qt;
      Eigen::Matrix<double, 7, 1> joint_volocity = pse_inverse_jacobian * qt;
      Eigen::Matrix<double, 7, 1> res = (pse_inverse_jacobian * feedback);
      joint_volocity -= beta * res + pse_inverse_jacobian * (three_noise * time / time_max);
    
      std::array<double, 7> joint_data;
      Eigen::Matrix<double, 7, 1>::Map(joint_data.data(), joint_volocity.rows(),
                                       joint_volocity.cols()) = joint_volocity;
      franka::JointVelocities velocities = {joint_data};
      if (time >= time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        // return franka::MotionFinished(origin_velocities);
        return franka::MotionFinished(velocities);
      }
      // return origin_velocities;
      return velocities;
    };
    // run robot
    robot.control(gripper_function);
    time = 0;
    robot.control(draw_function);
        time = 0;

    MotionGenerator motion_generators(0.5, q_goal);

    robot.control(motion_generators);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
