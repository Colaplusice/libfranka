// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    setDefaultBehavior(robot);
    std::cout<<"not recover"<<std::endl;
        // First move the robot to a suitable joint configuration
    //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    
   std::array<double, 7> q_goal = {{-0.000127832000000000	,-0.785472000000000,	-0.00135502000000000,	-2.35425000000000,	-0.000335558000000000,1.57164000000000,	0.785465000000000}};

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
    std::array<double, 7> initial_position;
    double time = 0.0;
    static std::vector<std::array<double, 7>> all_joint_values = read_joint_position_from_file("/home/ubuntu/Documents/angle6.txt");
    std::cout<<"vector length"<<all_joint_values.size()<<std::endl;
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec()*0.3;
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      std::array<double, 7> current_value = all_joint_values.front();
      all_joint_values.erase(all_joint_values.begin());
      // std::cout<<"robot"<<std::endl;

      // for (size_t i = 0; i < 7; i++)
      // {
      // std::cout<<robot_state.q[i]<<" ";
      // }
      //std::cout<<std::endl;
      std::cout<<"read"<<std::endl;
      for (size_t i = 0; i < 7; i++)
      {
      std::cout<<current_value[i]<<" ";
      }
      std::cout<<std::endl;
      franka::JointPositions output = {{current_value[0],current_value[1],current_value[2],current_value[3],current_value[4],current_value[5],current_value[6]}};
      if (time >= 20.0||all_joint_values.size()==0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
