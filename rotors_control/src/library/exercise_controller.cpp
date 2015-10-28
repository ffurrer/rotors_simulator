/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/exercise_controller.h"

namespace rotors_control {

ExerciseController::ExerciseController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

ExerciseController::~ExerciseController() {}

void ExerciseController::InitializeParameters() {
  //////////////////////////////////////////////////////////////////////////////
  // Exercise 4
  // TODO(instsructions): Calculate the allocation matrix here and assign it to
  //       controller_parameters_.allocation_matrix_.
  //       First make sure your matrix has the proper size, you can do so by:
  //          controller_parameters_.allocation_matrix_->resize(rows, columns);
  //       Then, you can use the shift operator << to assign values to your
  //       varibale.
  //       For instance if you want a Eigen vector a_vector to be filled with
  //       the values 1,2,3 you can do so by setting:
  //          a_vector << 1, 2, 3;

  // TODO(write code)
  // controller_parameters_.allocation_matrix_->resize(rows, columns);
  // controller_parameters_.allocation_matrix_ << ... ;

  // We initilize the inertia matrix I here.
  controller_parameters_.inertia_.setZero();
  controller_parameters_.inertia_(0, 0) = vehicle_parameters_.mass_;
  controller_parameters_.inertia_.block<3, 3>(1, 1) = vehicle_parameters_.inertia_;

  //
  // // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // // A^{ \dagger} = A^T*(A*A^T)^{-1}
  // TODO(code)
  // controller_parameters_.pinverse_allocation_matrix_.resize(...);
  // controller_parameters_.pinverse_allocation_matrix_ = ...;

  initialized_params_ = true;
}

void ExerciseController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }
  //////////////////////////////////////////////////////////////////////////////
  // Exercise 4
  Eigen::Vector3d roll_pitch_yaw;
  // Get the roll-pitch-yaw angles from a quaternion and assign them to roll_pitch_yaw.
  quat2rpy(odometry_.orientation, &roll_pitch_yaw);

  // TODO (code): Follow the comments.
  double position_error_z, velocity_error_z, rotation_error_roll, rotation_error_pitch, rotation_error_yaw, omega_error_roll, omega_error_pitch, omega_error_yaw;

  // Calculate the position and velocity errors in z-direction
  // position_error_z = ...
  // velocity_error_z = ...
  // Convert body-frame angular rates to Euler angular rates. (T_E_B * \omega)
  Eigen::Matrix3d T_E_B;
  // T_E_B << ...
  // Eigen::Vector3d roll_pitch_yaw_derivatives = ...

  // Calcualte the rotation errors in roll, pitch and yaw.
  // rotation_error_roll = ...
  // rotation_error_pitch = ...
  // rotation_error_yaw = ...
  // Calculate the errors on omega.
  // omega_error_roll = ...
  // omega_error_pitch = ...
  // omega_error_yaw = ...

  // Assign the values to the control inputs u_1, u_2, u_3, and u_4.
  // double u_1 = ...
  // double u_2 = ...
  // double u_3 = ...
  // double u_4 = ...

  // Eigen::Vector4d u(u_1, u_2, u_3, u_4);
  // Now calculate the actual rotor velocities.
  // *rotor_velocities = ...

  //////////////////////////////////////////////////////////////////////////////
  // Exercise 5
  // Get the desired rotation matrix.
  Eigen::Vector3d position_error;

  // Transform velocity to world frame.
  // const Eigen::Matrix3d R_W_I = ...
  // Eigen::Vector3d velocity_W =  ...

  // Calculate the velocity error in the world frame.
  Eigen::Vector3d velocity_error;
  // velocity_error = ...
  Eigen::Vector3d e_z(Eigen::Vector3d::UnitZ());

  Eigen::Vector3d k_position(controller_parameters_.k_position_x_, controller_parameters_.k_position_y_, controller_parameters_.k_position_z_);
  Eigen::Vector3d k_velocity(controller_parameters_.k_velocity_x_, controller_parameters_.k_velocity_y_, controller_parameters_.k_velocity_z_);

  Eigen::Vector3d desired_acceleration_vector;
  // desired_acceleration_vector = ...

  // Calcualte u_1 and assign it.
  // u_1 = ...

  Eigen::Vector3d b1_des, b2_des, b3_des;
  double yaw = command_trajectory_.getYaw();
  // b1_des << ...
  // b3_des = ...
  // b2_des = ...

  Eigen::Matrix3d R_des;
  // Set the columns of R_des, you can use R_des.col(i) = ...
  // R_des.col(0) = ...
  // R_des.col(1) = ...
  // R_des.col(2) = ...

  // Angle error according to lee et al.
  // Eigen::Matrix3d angle_error_matrix = ...
  Eigen::Vector3d angle_error;
  // Get a vector from your skew matrix.
  // vectorFromSkewMatrix(angle_error_matrix, &angle_error);
  // Calculate u_2, u_3, and u_4 (Hint: Calculate \omega \cross J\omega first).
  Eigen::Vector3d omega_cross_J_omega;
  // omega_cross_J_omega = ...
  // u_2 = ...
  // u_3 = ...
  // u_4 = ...

  // Eigen::Vector4d u_tracking(u_1, u_2, u_3, u_4);
  // *rotor_velocities = ...
}

void ExerciseController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void ExerciseController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

}
