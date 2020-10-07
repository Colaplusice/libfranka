// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include "examples_common.h"
/**
 * @example generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
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
    double beta=0.01;
    Eigen::Matrix<double, 7, 1> noise;
    noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    // 0.1 0.1 0.1
    Eigen::Matrix<double, 6, 1> three_noise;
    three_noise << 0.1, 0.1, 0.1,0,0,0;
    static Eigen::Matrix<double, 6, 1> feedback;
    feedback << 0, 0, 0,0,0,0;
    double time = 0.0;
    constexpr double r = 0.05;
    constexpr double alpha = M_PI / 6;

    //gripper function
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
      double rz = 0.0096 * time;
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

      double phi_sin = 2 * M_PI * std::sin(0.5 * M_PI * time / time_max);
      double phi_sinDot = M_PI * M_PI * std::cos(0.5 * M_PI * time / time_max) / time_max;
      double phi = phi_sin * std::sin(0.5 * M_PI * time / time_max);
      double phiDot = phi_sin * M_PI * std::cos(0.5 * M_PI * time / time_max) / time_max;
      double phiDotDot = M_PI * phi_sinDot * std::cos(0.5 * M_PI * time / time_max) / time_max -
                         M_PI * M_PI * phi_sin * std::sin(0.5 * M_PI * time / time_max) /
                             (2 * time_max * time_max);
      double rx = r * std::cos(2 * phi + alpha) + 0 - r * std::cos(alpha) + init_error_x +
                  initial_state.O_T_EE[12];
      double ry = r * std::sin(phi + alpha) + 0 - r * std::sin(alpha) + initial_state.O_T_EE[13];
      double rz = init_error_z + initial_state.O_T_EE[14];
      // desired end effector position
      // double rx = r * time_max * (time - time_max * sin(2 * M_PI * time / time_max) / (2 * M_PI))
      // /
      //                 (2 * M_PI) +
      //             init_error_x + initial_state.O_T_EE[12];
      // double ry = init_error_y + initial_state.O_T_EE[13];
      // double rz = init_error_z + initial_state.O_T_EE[14];

      Eigen::Matrix<double, 3, 1> rr;
      Eigen::Matrix<double, 3, 1> rd;
      rd << rx, ry, rz;
      rr << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];

      //  double drx = -2 * r * std::sin(2 * phi + alpha) * phiDot;
      //  double dry = r * std::cos(phi + alpha) * phiDot;
      //  double drz = 0;
      double drx = -2 * r * std::sin(2 * phi + alpha) * phiDot;
      double dry = r * std::cos(phi + alpha) * phiDot;
      double drz = 0;
      double desired_v = r * time_max * (1 - cos(2 * M_PI * time / time_max)) / (2 * M_PI);
      // double sin_drx = desired_v;
      // double sin_drx = 0.1;
      // double sin_dry = 0;
      // double sin_drz = 0;
      Eigen::Matrix<double, 6, 1> qt;
      // qt << sin_drx, sin_dry, sin_drz;
      qt << drx, dry, drz,0,0,0;
      //  j*rd
      //  feedback +=(half_jacobian*dq-qt);
      // Eigen::Matrix<double, 7, 1> joint_volocity = pse_inverse_jacobian * (qt - 1 * (rr - rd));
      // Eigen::Matrix<double, 7, 1> joint_volocity = pse_inverse_jacobian * (qt);
      //  Eigen::Matrix<double,3,1>a=;
      feedback += half_jacobian * dq - qt;
      Eigen::Matrix<double, 7, 1> joint_volocity = pse_inverse_jacobian * qt;
      Eigen::Matrix<double, 7, 1> res = (pse_inverse_jacobian * feedback);
      // Eigen::Matrix<double,3,1>noise;
      // noise<<0.1,0.1,0.1;
      // Eigen::Matrix<double,7,1>noises;
      // noises=pse_inverse_jacobian*noise;
      // Eigen::Matrix<double,7,1>ran_m=randomm_matrix(0,200);
      joint_volocity -= (beta * res)+ pse_inverse_jacobian*three_noise;
      // franka::JointVelocities origin_velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
      std::array<double, 7> joint_data;
      Eigen::Matrix<double, 7, 1>::Map(joint_data.data(), joint_volocity.rows(),
                                       joint_volocity.cols()) = joint_volocity;
      franka::JointVelocities velocities = {joint_data};
      // cout begin
      // std::cout << "initial state" << std::endl;
      // for (int i = 0; i < 16; i++) {
      //   std::cout << initial_state.O_T_EE[i] << "  ";
      //   if ((i + 1) % 4 == 0)
      //     std::cout << std::endl;
      // }
      // std::cout << "~~~~~~~~" << std::endl;
      // std::cout << "rr:" << std::endl << rr << std::endl;
      // std::cout << "dq:" << std::endl << dq << std::endl;
      // std::cout << "qt:" << std::endl << qt << std::endl;
      // // std::cout << "rd:" << std::endl << rd << std::endl;
      // std::cout << "rr-rd:" << std::endl << rr - rd << std::endl;
      // std::cout << "this is jacobian" << std::endl << jacobian << std::endl;
      // std::cout << "feedback" << std::endl << feedback << std::endl;
      // //std::cout << "random m" << std::endl << ran_m << std::endl;
      // std::cout << "current added feedback" << std::endl << half_jacobian * dq - qt << std::endl;
      // std::cout << "current res" << std::endl << res << std::endl;
      // std::cout << "this is joint velocity" << std::endl;
      // for (size_t i = 0; i < 7; i++) {
      //   std::cout << joint_data[i] << " ";
      // }
      // std::cout << std::endl;
      // std::cout << "~~~~~~~~" << std::endl;
      // cout end
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
    robot.control(draw_function);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
