// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include "examples_common.h"

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


int main(int argc, char** argv) {
  // usage: ./generate_cartesian_pose_motion <hostname> <object-width>
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>"
              << " <object-width>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);
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

    // configurations
    freopen("angle6.txt", "w", stdout);

    std::array<double, 16> initial_pose;
    double time = 0.0;

    //gripper function
    auto gripper_function = [&time, &initial_pose](
                                const franka::RobotState& robot_state,
                                franka::Duration period) -> franka::CartesianPose {
      time += period.toSec() * 0.3;
      // std::cout<<"this is time "<<time<<std::endl;
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
        //print_position(initial_pose);
      }
      double rz = 0.0096 * time;
      std::array<double, 16> new_pose = initial_pose;
      new_pose[14] -= rz;
      if (time >= 10) {
        // for (int i = 0; i < 16; i++) {
        //   std::cout << robot_state.O_T_EE_c[i] << "  ";
        //   if (i % 4 == 0)
        //     std::cout << std::endl;
        // }

        // std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    };
    // gripper_function finished


    robot.control(gripper_function);
    //std::cout << "stage 1 finished" << std::endl;
    time = 0;

    // stage two
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec() * 0.3;
      // std::cout<<"this is time "<<time<<std::endl;
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      constexpr double r = 0.08;
      constexpr double alpha = M_PI / 6;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double phi_sin = 2 * M_PI * std::sin(0.5 * M_PI * time / 10);
      double phi_sinDot = M_PI * M_PI * std::cos(0.5 * M_PI * time / 10) / 10;
      double phi = phi_sin * std::sin(0.5 * M_PI * time / 10);
      double phiDot = phi_sin * M_PI * std::cos(0.5 * M_PI * time / 10) / 10;
      double phiDotDot = M_PI * phi_sinDot * std::cos(0.5 * M_PI * time / 10) / 10 -
                         M_PI * M_PI * phi_sin * std::sin(0.5 * M_PI * time / 10) / (2 * 10 * 10);
      double rx = r * std::cos(2 * phi + alpha) + 0 - r * std::cos(alpha);
      double ry = r * std::sin(phi + alpha) + 0 - r * std::sin(alpha);
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += rx;
      new_pose[13] += ry;
      //print_position(robot_state.O_T_EE_c);
      for (size_t i = 0; i < 7; i++)
      {
        std::cout<<robot_state.q[i]<<" ";
      }
      std::cout<<std::endl;
      

      if (time >= 10) {
        //std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
