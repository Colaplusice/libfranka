// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include "examples_common.h"

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */



void print_position(std::array<double, 42> initial_pose) {
  std::cout << "this is jacobian position" << std::endl;
  for (int i = 0; i < 42; i++) {
    std::cout << initial_pose[i] << "  ";
    if ((i + 1) % 7 == 0)
      std::cout << std::endl;
  }
}

Eigen::MatrixXf pseudoinverse(Eigen::MatrixXf m)
{
    //Eigen::Matrix<float,2,3> m;
  //  m<<0.68,0.597,-0.211, 0.823,0.566,-0.605;
   // m<<1,2,3,4,5,6;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd =m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
   // Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV); 
    const Eigen::MatrixXf singularValues = svd.singularValues();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());
	singularValuesInv.setZero();
	double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
	 	if (singularValues(i) > pinvtoler)
	 		singularValuesInv(i, i) = 1.0f / singularValues(i);
	 	else
	 		singularValuesInv(i, i) = 0.f;
	 }
    Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
    std::cout << pinvmat << std::endl;
	return pinvmat;
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables

    
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      // print the jacobian array
      // print_position(jacobian_array);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      // jacobian matrix
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

      Eigen::Matrix<double, 3, 7> half_jacobian;
      // only get three lines
      for (int i = 0; i < half_jacobian.rows(); i++) {
        for (int j = 0; j < half_jacobian.cols(); j++) {
          half_jacobian(i, j) = jacobian(i, j);
        }
      }

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      // compute error to desired equilibrium pose
      Eigen::MatrixXf pse_inverse_jacobian=pseudoinverse(half_jacobian);
      
      double phi_sin = 2 * M_PI * std::sin(0.5 * M_PI * time / 10);
      double phi_sinDot = M_PI * M_PI * std::cos(0.5 * M_PI * time / 10) / 10;
      double phi = phi_sin * std::sin(0.5 * M_PI * time / 10);
      double phiDot = phi_sin * M_PI * std::cos(0.5 * M_PI * time / 10) / 10;
      double phiDotDot = M_PI * phi_sinDot * std::cos(0.5 * M_PI * time / 10) / 10 -
                         M_PI * M_PI * phi_sin * std::sin(0.5 * M_PI * time / 10) / (2 * 10 * 10);
      double rx = r * std::cos(2 * phi + alpha) + 0 - r * std::cos(alpha);
      double ry = r * std::sin(phi + alpha) + 0 - r * std::sin(alpha);

    double drx = r * std::cos(2 * phi + alpha) + 0 - r * std::cos(alpha);
      double dry = r * std::sin(phi + alpha) + 0 - r * std::sin(alpha);


  //     float  drx(jj,1)=-2*r*sin(2*phi+alpha)*phiDot;
  //  dry(jj,1)=r*cos(phi+alpha)*phiDot;
  //  drz(jj,1)=0;
      pse_inverse_jacobian*(half_jacobian*dq-dq)

      // position error

      std::cout << "position: " << position << std::endl;
      std::cout << "positiond: " << position_d << std::endl;
      std::cout << "positiond: " << position_d << std::endl;

      for (int i = 0; i < dq.size(); i++) {
        std::cout << "dq " << dq[i] << "  " << std::endl;
      }

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      // matrix transpose multiple
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };
    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
