/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace gnss_simulator {

struct GnssSimParams {
  double latlon_noise_std;           // Latitude and longitude noise STD (m)
  double hgt_noise_std;              // Height noise STD (m)
  double north_east_vel_noise_std;   // North and east velocity noise STD (m/s)
  double down_vel_noise_std;         // Down velocity noise STD (m/s)
  double roll_pitch_noise_std;       // Roll/pitch noise STD (rad)
  double yaw_noise_std;              // Yaw noise STD (rad)
  double roll_pitch_rate_noise_std;  // Roll/pitch rate noise STD (rad/s)
  double yaw_rate_noise_std;         // Yaw rate noise STD (rad/s)
  Eigen::Vector3d p_bs_b;  // Lever arm between GNSS antenna and body frame (m)
  Eigen::Matrix3d C_s_b;   // Rotation matrix GNSS antenna to body frame (-)
};

struct GnssModelEnableSettings {
  bool enable_pvt;    // Enable GNSS PVT measurements
  bool enable_att;    // Enable GNSS attitude measurements
  bool enable_pitch;  // Enable GNSS pitch measurement (false = roll measured)
};

struct GnssPvtMeasurement {
  Eigen::Vector3d p_es_e;  // Position of sensor in Earth-centered Earth-fixed
                           // frame (rad,rad,m)
  Eigen::Vector3d v_ns_n;  // Velocity of sensor in navigation frame (m/s)
  Eigen::Matrix<double, 3, 3>
      position_llh_covariance;  // Position covariance matrix (m^2)
  Eigen::Matrix<double, 3, 3>
      velocity_ned_covariance;  // Velocity covariance matrix (m^2/s^2)
  uint8_t mode;                 // GNSS mode
  bool is_valid;                // Measurement valid flag
};

struct GnssAttMeasurement {
  Eigen::Vector3d attitude_euler;    // Attitude in Euler angles (rad)
  Eigen::Vector3d angular_velocity;  // Angular velocity (rad/s)
  Eigen::Matrix<double, 3, 3>
      attitude_euler_covariance;  // Attitude covariance matrix (rad^2)
  Eigen::Matrix<double, 3, 3>
      angular_velocity_covariance;  // Angular velocity covariance matrix
                                    // (rad^2/s^2)
  uint8_t mode;                     // GNSS mode
  bool is_valid;                    // Measurement valid flag
};

}  // namespace gnss_simulator