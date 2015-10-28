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
  //controller_parameters_.allocation_matrix_->resize(rows, columns);
  //controller_parameters_.allocation_matrix_ << ... ;
  controller_parameters_.allocation_matrix_.resize(4, 6);
  const double s = sin(M_PI/6);
  const double c = cos(M_PI/6);
  double b = 8.5486e-6;
  double d = 0.136e-6;
  double l = 0.215;
  controller_parameters_.allocation_matrix_ << b, b, b, b, b, b,
                                               s*l*b, 1*l*b, s*l*b, -s*l*b, -1*l*b, -s*l*b,
                                               -c*l*b, 0, c*l*b, c*l*b, 0, -c*l*b,
                                               -1*d, 1*d, -1*d, 1*d, -1*d, 1*d;

  // We initilize the inertia matrix I here.
  controller_parameters_.inertia_.setZero();
  controller_parameters_.inertia_(0, 0) = vehicle_parameters_.mass_;
  controller_parameters_.inertia_.block<3, 3>(1, 1) = vehicle_parameters_.inertia_;

  //
  // // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // // A^{ \dagger} = A^T*(A*A^T)^{-1}
  // TODO(code)
  //controller_parameters_.pinverse_allocation_matrix_.resize(...);
  // controller_parameters_.pinverse_allocation_matrix_ = ...;

  controller_parameters_.pinverse_allocation_matrix_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  controller_parameters_.pinverse_allocation_matrix_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();

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

  Eigen::Vector3d roll_pitch_yaw;
  quat2rpy(odometry_.orientation, &roll_pitch_yaw);

  // TODO (code): Replace the assignment below with your calculated rotor
  //              velocities.
  double position_error_z, velocity_error_z, rotation_error_roll, rotation_error_pitch, rotation_error_yaw, omega_error_roll, omega_error_pitch, omega_error_yaw;

  position_error_z = command_trajectory_.position_W.z() - odometry_.position.z();
  velocity_error_z = - odometry_.velocity.z();
  // Convert body-frame angular rates to Euler angular rates. (T_E_B * \omega)
  Eigen::Matrix3d T_E_B;
  T_E_B << 1, sin(roll_pitch_yaw.x())*tan(roll_pitch_yaw.y()), cos(roll_pitch_yaw.x())*tan(roll_pitch_yaw.y()),
           0, cos(roll_pitch_yaw.x()),                         -sin(roll_pitch_yaw.x()),
           0, sin(roll_pitch_yaw.x())/cos(roll_pitch_yaw.y()), cos(roll_pitch_yaw.x())/cos(roll_pitch_yaw.y());
  Eigen::Vector3d roll_pitch_yaw_derivatives = T_E_B * odometry_.angular_velocity;

  rotation_error_roll = roll_pitch_yaw.x();
  rotation_error_pitch = roll_pitch_yaw.y();
  rotation_error_yaw = roll_pitch_yaw.z();
  omega_error_roll = roll_pitch_yaw_derivatives.x();
  omega_error_pitch = roll_pitch_yaw_derivatives.y();
  omega_error_yaw = roll_pitch_yaw_derivatives.z();
  double u_1 = controller_parameters_.k_position_z_ * position_error_z + controller_parameters_.k_velocity_z_ * velocity_error_z + vehicle_parameters_.gravity_;
  double u_2 = -controller_parameters_.k_p_roll_ * rotation_error_roll - controller_parameters_.k_d_roll_ * omega_error_roll;
  double u_3 = -controller_parameters_.k_p_pitch_ * rotation_error_pitch - controller_parameters_.k_d_pitch_ * omega_error_pitch;
  double u_4 = -controller_parameters_.k_p_yaw_ * rotation_error_yaw - controller_parameters_.k_d_yaw_ * omega_error_yaw;
  Eigen::Vector4d u(u_1, u_2, u_3, u_4);

  *rotor_velocities = controller_parameters_.pinverse_allocation_matrix_ * controller_parameters_.inertia_ * u;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();

///////////////////////////////////////////////////////////////////////////////////////////
  // results to exercise 5
  // Get the desired rotation matrix.
  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;
  Eigen::Vector3d e_z(Eigen::Vector3d::UnitZ());
  Eigen::Vector3d desired_acceleration_vector;

  Eigen::Vector3d k_position(controller_parameters_.k_position_x_, controller_parameters_.k_position_y_, controller_parameters_.k_position_z_);
  Eigen::Vector3d k_velocity(controller_parameters_.k_velocity_x_, controller_parameters_.k_velocity_y_, controller_parameters_.k_velocity_z_);
  desired_acceleration_vector = -position_error.cwiseProduct(k_position)
      - velocity_error.cwiseProduct(k_velocity) + vehicle_parameters_.gravity_ * e_z;
  u_1 = desired_acceleration_vector.dot(R_W_I.inverse() * e_z);

  Eigen::Vector3d b1_des;
  double yaw = command_trajectory_.getYaw();
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = desired_acceleration_vector / desired_acceleration_vector.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R_W_I - R_W_I.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);
  Eigen::Vector3d omega_cross_J_omega;
  omega_cross_J_omega = odometry_.angular_velocity.cross(vehicle_parameters_.inertia_ * odometry_.angular_velocity);
  u_2 = -controller_parameters_.k_p_roll_ * angle_error.x() - controller_parameters_.k_d_roll_ * odometry_.angular_velocity.x() + omega_cross_J_omega.x();
  u_3 = -controller_parameters_.k_p_pitch_ * angle_error.y() - controller_parameters_.k_d_pitch_ * odometry_.angular_velocity.y()  + omega_cross_J_omega.y();
  u_4 = -controller_parameters_.k_p_yaw_ * angle_error.z() - controller_parameters_.k_d_yaw_ * odometry_.angular_velocity.z()  + omega_cross_J_omega.z();

  Eigen::Vector4d u_tracking(u_1, u_2, u_3, u_4);

  *rotor_velocities = controller_parameters_.pinverse_allocation_matrix_ * controller_parameters_.inertia_ * u_tracking;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
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
