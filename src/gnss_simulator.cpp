/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <iomanip>
#include <iostream>

#include "gnss_simulator.hpp"

namespace gnss_simulator {

GnssSimulator::GnssSimulator()
    : sample_time_pvt_(0.05),
      sample_time_att_(0.05),
      p_eo_e_(Eigen::Vector3d::Zero()),
      randomNumberGenerator_(42),
      seed_(42),
      normalDistribution_(0, 1),
      use_fixed_random_numbers_(false) {
  // GNSS simulation parameters
  gnssSimParams_.latlon_noise_std = 0.006;  // 0.6 cm
  gnssSimParams_.hgt_noise_std = 0.01;      // 1.0 cm

  gnssSimParams_.north_east_vel_noise_std = 0.03;  // 3.0 cm/s
  gnssSimParams_.down_vel_noise_std = 0.03;        // 3.0 cm/s

  gnssSimParams_.roll_pitch_noise_std = 0.05 * M_PI / 180.0;
  gnssSimParams_.yaw_noise_std = 0.03 * M_PI / 180.0;

  gnssSimParams_.yaw_rate_noise_std = 0.05 * M_PI / 180.0;
  gnssSimParams_.roll_pitch_rate_noise_std = 0.05 * M_PI / 180.0;

  gnssSimParams_.p_bs_b = Eigen::Vector3d::Zero();
  gnssSimParams_.C_s_b = Eigen::Matrix3d::Identity();

  // GNSS model enable settings
  gnssModelEnableSettings_.enable_pvt = true;
  gnssModelEnableSettings_.enable_att = true;
  gnssModelEnableSettings_.enable_pitch = true;
}

GnssSimulator::GnssSimulator(GnssSimParams gnssSimParams,
                             GnssModelEnableSettings gnssModelEnableSettings,
                             Eigen::Vector3d p_eo_e, double sample_time_pvt,
                             double sample_time_att, unsigned int seed)
    : gnssSimParams_(gnssSimParams),
      gnssModelEnableSettings_(gnssModelEnableSettings),
      sample_time_pvt_(sample_time_pvt),
      sample_time_att_(sample_time_att),
      p_eo_e_(p_eo_e),
      randomNumberGenerator_(seed),
      seed_(seed),
      normalDistribution_(0, 1),
      use_fixed_random_numbers_(false) {
  // Constructor body is intentionally left empty
}

GnssSimulator::~GnssSimulator() {
  // Destructor body is intentionally left empty
}

GnssPvtMeasurement GnssSimulator::generatePvtMeasurement(
    const Eigen::Vector3d& p_eb_e, const Eigen::Vector3d& v_eb_b,
    const Eigen::Quaterniond& q_b_n, const Eigen::Vector3d& w_eb_b) {
  // Initialize measurement
  GnssPvtMeasurement gnssPvtMeasurement;

  // Rotation matrix from body frame to navigation frame
  Eigen::Matrix3d C_b_n = q_b_n.toRotationMatrix();

  // Pre-calculate trigonometric functions
  const double sin_lat = std::sin(p_eb_e(0));
  const double cos_lat = std::cos(p_eb_e(0));
  const double sin_lon = std::sin(p_eb_e(1));
  const double cos_lon = std::cos(p_eb_e(1));

  // // Rotation matrix from ECEF navigation to ECEF frame
  Eigen::Matrix3d C_n_e;
  C_n_e.row(0) << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon;
  C_n_e.row(1) << -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon;
  C_n_e.row(2) << cos_lat, 0, -sin_lat;

  // Generate position measurement with lever arm effect
  gnssPvtMeasurement.p_es_e = p_eb_e + C_n_e * C_b_n * gnssSimParams_.p_bs_b;

  // Generate velocity measurement with lever arm effect
  gnssPvtMeasurement.v_ns_n =
      C_b_n * (v_eb_b + w_eb_b.cross(gnssSimParams_.p_bs_b));

  Eigen::Matrix3d posNedCovariance = Eigen::Matrix3d::Identity();

  posNedCovariance(0, 0) =
      gnssSimParams_.latlon_noise_std * gnssSimParams_.latlon_noise_std;
  posNedCovariance(1, 1) =
      gnssSimParams_.latlon_noise_std * gnssSimParams_.latlon_noise_std;
  posNedCovariance(2, 2) =
      gnssSimParams_.hgt_noise_std * gnssSimParams_.hgt_noise_std;

  Eigen::Matrix3d posGeodeticCovariance =
      navigation_utilities::convertNedToGeodeticEcefCovariance(
          posNedCovariance, gnssPvtMeasurement.p_es_e);

  // Declare normal distributed position and velocity noise
  Eigen::Vector3d nu_p;
  Eigen::Vector3d nu_v;

  if (use_fixed_random_numbers_) {
    // Generate fixed position and velocity noise
    nu_p = Eigen::Vector3d(1.0, 2.0, 3.0);
    nu_v = Eigen::Vector3d(4.0, 5.0, 6.0);
  } else {
    // Generate normal distributed position noise
    nu_p(0) = std::sqrt(posGeodeticCovariance(0, 0)) *
              drawRandNormalDistNum();  // (rad)
    nu_p(1) = std::sqrt(posGeodeticCovariance(1, 1)) *
              drawRandNormalDistNum();  // (rad)
    nu_p(2) = std::sqrt(posGeodeticCovariance(2, 2)) *
              drawRandNormalDistNum();  // (m)

    // Generate normal distributed velocity noise
    nu_v(0) = gnssSimParams_.north_east_vel_noise_std * drawRandNormalDistNum();
    nu_v(1) = gnssSimParams_.north_east_vel_noise_std * drawRandNormalDistNum();
    nu_v(2) = gnssSimParams_.down_vel_noise_std * drawRandNormalDistNum();
  }
  // Add noise to antenna position and velocity measurement
  gnssPvtMeasurement.p_es_e += nu_p;
  gnssPvtMeasurement.v_ns_n += nu_v;

  // Assign position covariance
  gnssPvtMeasurement.position_llh_covariance = Eigen::Matrix3d::Identity();

  gnssPvtMeasurement.position_llh_covariance(0, 0) = posNedCovariance(0, 0);
  gnssPvtMeasurement.position_llh_covariance(1, 1) = posNedCovariance(1, 1);
  gnssPvtMeasurement.position_llh_covariance(2, 2) = posNedCovariance(2, 2);

  // Assign velocity covariance
  gnssPvtMeasurement.velocity_ned_covariance = Eigen::Matrix3d::Identity();

  gnssPvtMeasurement.velocity_ned_covariance(0, 0) =
      std::pow(gnssSimParams_.north_east_vel_noise_std, 2);
  gnssPvtMeasurement.velocity_ned_covariance(1, 1) =
      std::pow(gnssSimParams_.north_east_vel_noise_std, 2);
  gnssPvtMeasurement.velocity_ned_covariance(2, 2) =
      std::pow(gnssSimParams_.down_vel_noise_std, 2);

  // Assign mode and validity flag
  gnssPvtMeasurement.mode = 4;  // RTK with fixed ambiguities
  gnssPvtMeasurement.is_valid = true;

  return gnssPvtMeasurement;
}

GnssAttMeasurement GnssSimulator::generateAttMeasurement(
    const Eigen::Vector3d& p_eb_e, const Eigen::Quaterniond& q_b_n,
    const Eigen::Vector3d& w_eb_b, const bool convert_output_to_degrees) {
  // Initialize measurement
  GnssAttMeasurement gnssAttMeasurement;

  // Rotation matrix from body frame to navigation frame
  Eigen::Matrix3d C_b_n = q_b_n.toRotationMatrix();

  // Generate attitude measurement
  gnssAttMeasurement.attitude_euler =
      navigation_utilities::convertQuaternionToEulerAngles(q_b_n);

  // Calculate local Earth rotation vector
  Eigen::Vector3d localPos = p_eb_e;
  Eigen::Vector3d w_ie_b =
      C_b_n.transpose() *
      navigation_utilities::getLocalEarthRotationVector(localPos);

  // Get rotation matrix from body frame to sensor frame
  Eigen::Matrix3d C_b_s = gnssSimParams_.C_s_b.transpose();

  // Calculate angular velocity of body frame with respect to ECI frame
  Eigen::Vector3d w_ib_b = w_eb_b + w_ie_b;

  // Rotate into from body to sensor representation frame
  Eigen::Vector3d w_ib_s = C_b_s * w_ib_b;

  // Angular velocity between sensor frame and sensor frame is zero
  Eigen::Vector3d w_bs_s = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Calculate angular velocity of sensor frame with respect to ECI frame
  Eigen::Vector3d w_is_s = w_ib_s + w_bs_s;

  // Assign measurement
  gnssAttMeasurement.angular_velocity = w_is_s;

  // Declare normal distributed attitude Euler and angular velocity noise
  Eigen::Vector3d nu_q;
  Eigen::Vector3d nu_w;

  if (use_fixed_random_numbers_) {
    // Generate fixed attitude Euler and angular velocity noise
    nu_q = Eigen::Vector3d(0.1, 0.2, 0.3);
    nu_w = Eigen::Vector3d(0.4, 0.5, 0.6);
  } else {
    // Generate normal distributed attitude Euler noise
    nu_q(0) = gnssSimParams_.roll_pitch_noise_std * drawRandNormalDistNum();
    nu_q(1) = gnssSimParams_.roll_pitch_noise_std * drawRandNormalDistNum();
    nu_q(2) = gnssSimParams_.yaw_noise_std * drawRandNormalDistNum();

    // Generate normal distributed angular velocity noise
    nu_w(0) =
        gnssSimParams_.roll_pitch_rate_noise_std * drawRandNormalDistNum();
    nu_w(1) =
        gnssSimParams_.roll_pitch_rate_noise_std * drawRandNormalDistNum();
    nu_w(2) = gnssSimParams_.yaw_rate_noise_std * drawRandNormalDistNum();
  }
  // Add noise to attitude Euler and angular velocity measurement
  gnssAttMeasurement.attitude_euler += nu_q;
  gnssAttMeasurement.angular_velocity += nu_w;

  // Assign attitude Euler covariance
  gnssAttMeasurement.attitude_euler_covariance = Eigen::Matrix3d::Identity();

  gnssAttMeasurement.attitude_euler_covariance(0, 0) =
      std::pow(gnssSimParams_.roll_pitch_noise_std, 2);
  gnssAttMeasurement.attitude_euler_covariance(1, 1) =
      std::pow(gnssSimParams_.roll_pitch_noise_std, 2);
  gnssAttMeasurement.attitude_euler_covariance(2, 2) =
      std::pow(gnssSimParams_.yaw_noise_std, 2);

  // Assign angular velocity covariance
  gnssAttMeasurement.angular_velocity_covariance = Eigen::Matrix3d::Identity();

  gnssAttMeasurement.angular_velocity_covariance(0, 0) =
      std::pow(gnssSimParams_.roll_pitch_rate_noise_std, 2);
  gnssAttMeasurement.angular_velocity_covariance(1, 1) =
      std::pow(gnssSimParams_.roll_pitch_rate_noise_std, 2);
  gnssAttMeasurement.angular_velocity_covariance(2, 2) =
      std::pow(gnssSimParams_.yaw_rate_noise_std, 2);

  // Only roll or pitch angle can be measured with GNSS compass
  if (gnssModelEnableSettings_.enable_pitch) {
    // Set roll angle to zero
    gnssAttMeasurement.attitude_euler(0) = 0.0;

    // Heading, pitch (roll = 0), aux antenna positions obtained with
    // float ambiguities
    gnssAttMeasurement.mode = 2;
  } else {
    // Set pitch angle to zero
    gnssAttMeasurement.attitude_euler(1) = 0.0;

    // Heading, pitch (roll = 0), aux antenna positions obtained with
    // float ambiguities
    gnssAttMeasurement.mode = 2;
  }

  // Convert measurements to degrees if requested
  if (convert_output_to_degrees) {
    gnssAttMeasurement.attitude_euler =
        gnssAttMeasurement.attitude_euler * 180.0 / M_PI;
    gnssAttMeasurement.angular_velocity =
        gnssAttMeasurement.angular_velocity * 180.0 / M_PI;
    gnssAttMeasurement.attitude_euler_covariance =
        gnssAttMeasurement.attitude_euler_covariance * 180.0 / M_PI * 180.0 /
        M_PI;
    gnssAttMeasurement.angular_velocity_covariance =
        gnssAttMeasurement.angular_velocity_covariance * 180.0 / M_PI * 180.0 /
        M_PI;
  }
  // Assign validity flag
  gnssAttMeasurement.is_valid = true;

  return gnssAttMeasurement;
}

GnssSimParams GnssSimulator::getGnssSimParams() const {
  return gnssSimParams_;
}

GnssModelEnableSettings GnssSimulator::getGnssModelEnableSettings() const {
  return gnssModelEnableSettings_;
}

double GnssSimulator::getSampleTimePvt() const {
  return sample_time_pvt_;
}

double GnssSimulator::getSampleTimeAtt() const {
  return sample_time_att_;
}

unsigned int GnssSimulator::getSeed() const {
  return seed_;
}

bool GnssSimulator::getUseFixedRandomNumbersFlag() const {
  return use_fixed_random_numbers_;
}

void GnssSimulator::setGnssSimParams(const GnssSimParams& gnssSimParams) {
  gnssSimParams_ = gnssSimParams;
}

void GnssSimulator::setGnssModelEnableSettings(
    const GnssModelEnableSettings& gnssModelEnableSettings) {
  gnssModelEnableSettings_ = gnssModelEnableSettings;
}

void GnssSimulator::setSampleTimePvt(const double& sample_time_pvt) {
  sample_time_pvt_ = sample_time_pvt;
}

void GnssSimulator::setSampleTimeAtt(const double& sample_time_att) {
  sample_time_att_ = sample_time_att;
}

void GnssSimulator::setSeed(const unsigned int seed) {
  randomNumberGenerator_.seed(seed);
  seed_ = seed;
}

void GnssSimulator::setUseFixedRandomNumbersFlag(
    const bool useFixedRandomNumbers) {
  use_fixed_random_numbers_ = useFixedRandomNumbers;
}

std::stringstream GnssSimulator::printGnssSimulatorParameters() {
  std::stringstream ss;

  ss << "**********************************************************************"
        "****************************************************************"
     << "\n";
  ss << std::left << std::setw(50) << "Starting GNSS Simulator"
     << "\n";
  ss << "**********************************************************************"
        "****************************************************************"
     << "\n";

  // GNSS simulation parameters
  ss << std::left << "Simulation parameters:\n";
  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50)
     << "latlon_noise_std:" << gnssSimParams_.latlon_noise_std << " m\n";

  ss << std::left << std::setw(50)
     << "hgt_noise_std:" << gnssSimParams_.hgt_noise_std << " m\n";

  ss << std::left << std::setw(50)
     << "north_east_vel_noise_std:" << gnssSimParams_.north_east_vel_noise_std
     << " m/s\n";

  ss << std::left << std::setw(50)
     << "down_vel_noise_std:" << gnssSimParams_.down_vel_noise_std << " m/s\n";

  ss << std::left << std::setw(50) << "roll_pitch_noise_std:"
     << gnssSimParams_.roll_pitch_noise_std * 180 / M_PI << " deg\n";

  ss << std::left << std::setw(50)
     << "yaw_noise_std:" << gnssSimParams_.yaw_noise_std * 180 / M_PI
     << " deg\n";

  ss << std::left << std::setw(50) << "roll_pitch_rate_noise_std:"
     << gnssSimParams_.roll_pitch_rate_noise_std * 180 / M_PI << " deg/s\n";

  ss << std::left << std::setw(50)
     << "yaw_rate_noise_std:" << gnssSimParams_.yaw_rate_noise_std * 180 / M_PI
     << " deg/s\n";

  ss << std::left << std::setw(50)
     << "p_bs_b: " << gnssSimParams_.p_bs_b.transpose() << " m\n";

  ss << std::left << std::setw(50) << "C_bs_b:"
     << gnssSimParams_.C_s_b.eulerAngles(2, 1, 0).transpose() * 180 / M_PI
     << " deg\n";

  ss << "**********************************************************************"
        "****************************************************************"
     << "\n";

  // GNSS model enable settings
  ss << std::left << "Model enable settings:\n";

  ss << std::left << std::setw(50)
     << "enable_pvt:" << gnssModelEnableSettings_.enable_pvt << "\n";

  ss << std::left << std::setw(50)
     << "enable_att:" << gnssModelEnableSettings_.enable_att << "\n";

  ss << std::left << std::setw(50)
     << "enable_pitch:" << gnssModelEnableSettings_.enable_pitch << "\n";

  ss << "\n";

  // Sampling times and frequencies
  ss << std::left << std::setw(50) << "Sampling time PVT:" << std::fixed
     << std::setprecision(6) << sample_time_pvt_ << " s\n";

  ss << std::left << std::setw(50) << "Sampling time ATT:" << std::fixed
     << std::setprecision(6) << sample_time_att_ << " s\n";

  ss << std::left << std::setw(50) << "Sampling frequency PVT:" << std::fixed
     << std::setprecision(6) << 1.0 / sample_time_pvt_ << " Hz\n";

  ss << std::left << std::setw(50) << "Sampling frequency ATT:" << std::fixed
     << std::setprecision(6) << 1.0 / sample_time_att_ << " Hz\n";

  ss << "**********************************************************************"
        "****************************************************************"
     << "\n";

  return ss;
}

/**
 * @brief Draws a random number from a normal distribution.
 *
 * This function draws a random number from a normal distribution with the given
 * mean and standard deviation.
 *
 * @return Single random number from the normal distribution.
 */
double GnssSimulator::drawRandNormalDistNum() {
  // Generate normal distributed random number
  return normalDistribution_(randomNumberGenerator_);
}

}  // namespace gnss_simulator
