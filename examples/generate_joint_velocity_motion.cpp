#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_joint_velocity_motion <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);

    double time_max = 4.0;
    double omega_max = 0.2;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&) -> franka::JointVelocities {
      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      time += 0.001;
      if (time > 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::Stop;
      }
      return {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}