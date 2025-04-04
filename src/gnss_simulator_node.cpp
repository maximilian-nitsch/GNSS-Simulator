/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "gnss_simulator_package/gnss_simulator.hpp"

#include "nanoauv_sensor_driver_interfaces/msg/gnss_attitude.hpp"
#include "nanoauv_sensor_driver_interfaces/msg/gnss_pvt.hpp"

namespace gnss_simulator {

class GnssSimulatorNode : public rclcpp::Node {
 public:
  // Constructor
  explicit GnssSimulatorNode(std::shared_ptr<GnssSimulator> pGnssSimulator);

  // Destructor
  ~GnssSimulatorNode(){};

 private:
  // Declaration and retrieval functions for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveModelParameterSettings();
  void declareAndRetrieveModelEnableSettings();

  // GNSS PVT and attitude callback functions
  void gnssPvtSimulatorCallback();
  void gnssAttitudeSimulatorCallback();

  // Odometry callback functions
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometryTimeOutCallback();

  // tf2 static broadcaster callback function
  void publishStaticTf2Transforms();

  // GNSS simulator class object
  std::shared_ptr<GnssSimulator> pGnssSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Timers
  rclcpp::TimerBase::SharedPtr pPvtTimer_;
  rclcpp::TimerBase::SharedPtr pAttitudeTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

  // Custom GNSS PVT (position, velocity, time) publisher
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::GnssPvt>::SharedPtr
      pGnssPvtPublisher_;

  // Custom GNSS attitude (GNSS compass) publisher
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::GnssAttitude>::
      SharedPtr pGnssAttitudePublisher_;

  // NavSatFix publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      pNavSatFixPublisher_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Geodetic position of local NED origin (reference point)
  Eigen::Vector3d p_eb_e_ref_;

  // Sample time of GNSS PVT and attitude (compass)
  double sample_time_pvt_;
  double sample_time_att_;
};

GnssSimulatorNode::GnssSimulatorNode(
    std::shared_ptr<GnssSimulator> pGnssSimulator)
    : Node("gnss_simulator_node"),
      pGnssSimulator_(pGnssSimulator),
      lastOdomTimestamp_(rclcpp::Time(0.0)),
      first_odometry_received_(false),
      odometry_timeout_(false),
      p_eb_e_ref_(Eigen::Vector3d(-70.667997328 * M_PI / 180,
                                  -8.266998932 * M_PI / 180,
                                  40.0)),  // Neymayer Station III
      sample_time_pvt_(0.05),
      sample_time_att_(0.05) {

  RCLCPP_INFO(this->get_logger(), "Configuring GNSS simulator node...");

  // Declare and retrieve settings
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveModelParameterSettings();
  declareAndRetrieveModelEnableSettings();

  // Declare launch file parameter to get odometry topic name
  this->declare_parameter<std::string>("topic_name", "/auv/odometry");

  // Retrieve the topic name parameter
  std::string odometry_topic_name;
  this->get_parameter("topic_name", odometry_topic_name);

  // Print GNSS simulator parameters
  std::stringstream ss = pGnssSimulator_->printGnssSimulatorParameters();
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  // Initialize the GNSS PVT publisher
  pGnssPvtPublisher_ =
      this->create_publisher<nanoauv_sensor_driver_interfaces::msg::GnssPvt>(
          "pvt", 10);

  // Initialize the GNSS attitude publisher
  pGnssAttitudePublisher_ = this->create_publisher<
      nanoauv_sensor_driver_interfaces::msg::GnssAttitude>("attitude", 10);

  // Initialize the NavSatFix publisher
  pNavSatFixPublisher_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 10);

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_name, 10,
      std::bind(&GnssSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Create timer for timeout
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&GnssSimulatorNode::odometryTimeOutCallback, this));

  // Create timer for GNSS PVT (position, velocity, time) measurements
  pPvtTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sample_time_pvt_ * 1000)),
      std::bind(&GnssSimulatorNode::gnssPvtSimulatorCallback, this));

  // Create timer for GNSS attitude (compass) measurements
  pAttitudeTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sample_time_att_ * 1000)),
      std::bind(&GnssSimulatorNode::gnssAttitudeSimulatorCallback, this));

  RCLCPP_INFO(
      this->get_logger(),
      "GNSS simulator node initialized. GNSS simulator node waiting for "
      "first odometry message...");
}

/**
 * @brief Declare and retrieve general settings from the YAML file.
 *
 * @details This function declares and retrieves the generalsettings from the
 * YAML file. The parameters are directly set in the simulator.
 */
void GnssSimulatorNode::declareAndRetrieveGeneralSettings() {
  // Local parameters
  int seed;
  bool use_constant_seed;

  // Preceding hierarchies of YAML file
  std::string prev_hrs = "gnss_simulator.general_settings.";

  // Declare parameters
  this->declare_parameter<double>(prev_hrs + "sample_time_pvt");
  this->declare_parameter<double>(prev_hrs + "sample_time_attitude");
  this->declare_parameter<int>(prev_hrs + "seed");
  this->declare_parameter<bool>(prev_hrs + "use_constant_seed");

  // Retrieve parameters
  this->get_parameter(prev_hrs + "sample_time_pvt", sample_time_pvt_);
  this->get_parameter(prev_hrs + "sample_time_attitude", sample_time_att_);
  this->get_parameter(prev_hrs + "seed", seed);
  this->get_parameter(prev_hrs + "use_constant_seed", use_constant_seed);

  // Set GNSS PVT and attitude sample times
  this->pGnssSimulator_->setSampleTimePvt(sample_time_pvt_);
  this->pGnssSimulator_->setSampleTimeAtt(sample_time_att_);

  // Set seed depending on the use_constant_seed flag
  if (use_constant_seed == false) {
    // Draw a random seed from the random device
    pGnssSimulator_->setSeed(seed);
    RCLCPP_INFO(this->get_logger(), "Using random seed: %d", seed);
  } else {
    // Set the random number generator seed
    pGnssSimulator_->setSeed(seed);
    RCLCPP_INFO(this->get_logger(), "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve model parameter settings from the YAML file.
 *
 * @details This function declares and retrieves the model parameter settings
 * from the YAML file. The parameters are directly set in the simulator.
 */
void GnssSimulatorNode::declareAndRetrieveModelParameterSettings() {
  // Local parameters
  GnssSimParams gnssSimParams;
  std::vector<double> p_bs_b;
  std::vector<double> euler_s_b;

  // Preceding hierarchies of YAML file
  std::string prev_hrs = "gnss_simulator.model_parameter_settings.";

  // Declare parameters
  this->declare_parameter<std::vector<double>>(prev_hrs +
                                               "ned_geodetic_reference");
  this->declare_parameter<double>(prev_hrs + "latlon_noise_std");
  this->declare_parameter<double>(prev_hrs + "hgt_noise_std");
  this->declare_parameter<double>(prev_hrs + "north_east_vel_noise_std");
  this->declare_parameter<double>(prev_hrs + "down_vel_noise_std");
  this->declare_parameter<double>(prev_hrs + "roll_pitch_noise_std");
  this->declare_parameter<double>(prev_hrs + "yaw_noise_std");
  this->declare_parameter<double>(prev_hrs + "roll_pitch_rate_noise_std");
  this->declare_parameter<double>(prev_hrs + "yaw_rate_noise_std");
  this->declare_parameter<std::vector<double>>(
      prev_hrs + "lever_arm_body_to_sensor_frame");
  this->declare_parameter<std::vector<double>>(prev_hrs +
                                               "rotation_body_to_sensor_frame");

  // Retrieve parameters
  this->get_parameter(prev_hrs + "latlon_noise_std",
                      gnssSimParams.latlon_noise_std);
  this->get_parameter(prev_hrs + "hgt_noise_std", gnssSimParams.hgt_noise_std);
  this->get_parameter(prev_hrs + "north_east_vel_noise_std",
                      gnssSimParams.north_east_vel_noise_std);
  this->get_parameter(prev_hrs + "down_vel_noise_std",
                      gnssSimParams.down_vel_noise_std);
  this->get_parameter(prev_hrs + "roll_pitch_noise_std",
                      gnssSimParams.roll_pitch_noise_std);
  this->get_parameter(prev_hrs + "yaw_noise_std", gnssSimParams.yaw_noise_std);
  this->get_parameter(prev_hrs + "roll_pitch_rate_noise_std",
                      gnssSimParams.roll_pitch_rate_noise_std);
  this->get_parameter(prev_hrs + "yaw_rate_noise_std",
                      gnssSimParams.yaw_rate_noise_std);
  this->get_parameter(prev_hrs + "lever_arm_body_to_sensor_frame", p_bs_b);
  this->get_parameter(prev_hrs + "rotation_body_to_sensor_frame", euler_s_b);

  // Convert noise standard deviations from degrees to radians
  gnssSimParams.roll_pitch_noise_std *= M_PI / 180.0;
  gnssSimParams.yaw_noise_std *= M_PI / 180.0;
  gnssSimParams.roll_pitch_rate_noise_std *= M_PI / 180.0;
  gnssSimParams.yaw_rate_noise_std *= M_PI / 180.0;

  // Assign double vector to GNSS simulation parameters
  gnssSimParams.p_bs_b[0] = p_bs_b[0];
  gnssSimParams.p_bs_b[1] = p_bs_b[1];
  gnssSimParams.p_bs_b[2] = p_bs_b[2];

  // Convert Euler angles to quaternion and assign to GNSS simulation
  // parameters
  gnssSimParams.C_s_b =
      Eigen::AngleAxisd(euler_s_b[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(euler_s_b[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler_s_b[2], Eigen::Vector3d::UnitX());

  // Set GNSS simulation parameters
  this->pGnssSimulator_->setGnssSimParams(gnssSimParams);
}

/**
 * @brief Declare and retrieve model enable settings from the YAML file.
 *
 * @details This function declares and retrieves the model enable settings
 * from the YAML file. The parameters are directly set in the simulator.
 */
void GnssSimulatorNode::declareAndRetrieveModelEnableSettings() {
  // Local parameters
  GnssModelEnableSettings gnssModelEnableSettings;

  // Preceding hierarchies of YAML file
  std::string prev_hrs = "gnss_simulator.model_enable_settings.";

  // Declare parameters
  this->declare_parameter<bool>(prev_hrs + "enable_pvt");
  this->declare_parameter<bool>(prev_hrs + "enable_att");
  this->declare_parameter<bool>(prev_hrs + "enable_pitch");

  // Retrieve parameters
  this->get_parameter(prev_hrs + "enable_pvt",
                      gnssModelEnableSettings.enable_pvt);
  this->get_parameter(prev_hrs + "enable_att",
                      gnssModelEnableSettings.enable_att);
  this->get_parameter(prev_hrs + "enable_pitch",
                      gnssModelEnableSettings.enable_pitch);

  // Set GNSS model enable settings
  this->pGnssSimulator_->setGnssModelEnableSettings(gnssModelEnableSettings);
}

/**
 * @brief GNSS PVT (position, velocity, time) simulator loop callback
 * function.
 *
 * @details This function is called periodically by a timer with the specified
 * sample rate and publishes the customGNSS PVT (position, velocity, time)
 * message.
 */
void GnssSimulatorNode::gnssPvtSimulatorCallback() {
  // Get current time
  rclcpp::Time currentTimestamp = now();

  // Declare custom GNSS PVT and attitude message
  nanoauv_sensor_driver_interfaces::msg::GnssPvt gnssPvtCustomMsg;

  // Declare NavSatFix message
  sensor_msgs::msg::NavSatFix navSatFixMsg;

  // Declare diagnostic message and array message
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS PVT";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic message to diagnostic array message
    gnssPvtCustomMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom GNSS PVT message with only diagnostic message
    pGnssPvtPublisher_->publish(gnssPvtCustomMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;
  } else {
    // **************************************************************************
    // Ground truth odometry message odometry extraction
    // **************************************************************************

    // Read out odometry message
    Eigen::Vector3d p_nb_n_true;
    Eigen::Quaterniond q_b_n_true;
    Eigen::Vector3d v_nb_b_true;  // = v_eb_b
    Eigen::Vector3d w_nb_b_true;  // = w_eb_b

    // Extract position from odometry message
    p_nb_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.position.x;
    p_nb_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.position.y;
    p_nb_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.position.z;

    // Convert NED position to geodetic ECEF position given a reference
    // position
    Eigen::Vector3d p_eb_e_true =
        navigation_utilities::convertNedToGeodeticEcef(p_nb_n_true,
                                                       p_eb_e_ref_);

    // Extract attitude from odometry message
    q_b_n_true.w() = groundTruthOdomMsg_.get()->pose.pose.orientation.w;
    q_b_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.orientation.x;
    q_b_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.orientation.y;
    q_b_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.orientation.z;

    // Extract linear velocity from odometry message
    v_nb_b_true.x() = groundTruthOdomMsg_.get()->twist.twist.linear.x;
    v_nb_b_true.y() = groundTruthOdomMsg_.get()->twist.twist.linear.y;
    v_nb_b_true.z() = groundTruthOdomMsg_.get()->twist.twist.linear.z;

    // Extract angular velocity from odometry message
    w_nb_b_true.x() = groundTruthOdomMsg_.get()->twist.twist.angular.x;
    w_nb_b_true.y() = groundTruthOdomMsg_.get()->twist.twist.angular.y;
    w_nb_b_true.z() = groundTruthOdomMsg_.get()->twist.twist.angular.z;

    // Generate GNSS PVT measurement
    GnssPvtMeasurement gnssPvtMeasurement =
        pGnssSimulator_->generatePvtMeasurement(p_eb_e_true, v_nb_b_true,
                                                q_b_n_true, w_nb_b_true);

    // **************************************************************************
    // GnssPvt Custom Message
    // **************************************************************************
    gnssPvtCustomMsg.header.stamp = currentTimestamp;
    gnssPvtCustomMsg.header.frame_id = "gnss_main_antenna_link";

    gnssPvtCustomMsg.position_llh.x = gnssPvtMeasurement.p_es_e(0);
    gnssPvtCustomMsg.position_llh.y = gnssPvtMeasurement.p_es_e(1);
    gnssPvtCustomMsg.position_llh.z = gnssPvtMeasurement.p_es_e(2);

    gnssPvtCustomMsg.velocity_ned.x = gnssPvtMeasurement.v_ns_n(0);
    gnssPvtCustomMsg.velocity_ned.y = gnssPvtMeasurement.v_ns_n(1);
    gnssPvtCustomMsg.velocity_ned.z = gnssPvtMeasurement.v_ns_n(2);

    // Assign position and velocity covariance
    for (int i = 0; i < 9; i++) {
      gnssPvtCustomMsg.position_llh_covariance[i].data =
          gnssPvtMeasurement.position_llh_covariance(i);
      gnssPvtCustomMsg.velocity_ned_covariance[i].data =
          gnssPvtMeasurement.velocity_ned_covariance(i);
    }

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS PVT";
    diagnosticMsg.message = "GNSS PVT measurement nominal";

    // Assign mode and validity flags
    gnssPvtCustomMsg.mode.data = gnssPvtMeasurement.mode;
    gnssPvtCustomMsg.is_valid.data = gnssPvtMeasurement.is_valid;

    // **************************************************************************
    // NavSatFix Message
    // **************************************************************************
    navSatFixMsg.header.stamp = currentTimestamp;
    navSatFixMsg.header.frame_id = "gnss_main_antenna_link";

    navSatFixMsg.latitude = gnssPvtMeasurement.p_es_e(0) * 180.0 / M_PI;
    navSatFixMsg.longitude = gnssPvtMeasurement.p_es_e(1) * 180.0 / M_PI;
    navSatFixMsg.altitude = gnssPvtMeasurement.p_es_e(2);

    // Assign position covariance
    for (int i = 0; i < 9; i++) {
      navSatFixMsg.position_covariance[i] =
          gnssPvtMeasurement.position_llh_covariance(i);
    }

    // Assign covariance type
    navSatFixMsg.position_covariance_type =
        2;  // COVARIANCE_TYPE_DIAGONAL_KNOWN

    // Assign status flags
    navSatFixMsg.status.status = 2;       // with ground-based augmentation
    navSatFixMsg.status.service = 1 | 8;  // GPS and GALILEO
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  // Check if odometry message frequency is too slow
  if (timeSinceLastOdom.seconds() > sample_time_att_ &&
      odometry_timeout_ == false) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS PVT";
    diagnosticMsg.message =
        "Ground truth odometry message frequency is too slow!"
        " GNSS simulator ground truth frequency higher than odometry!"
        " Increase odometry message frequency!";

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 5000,
        "Ground truth odometry message frequency is too slow!");
  }

  // Check if odometry has timed out
  if (odometry_timeout_ == true) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS PVT";
    diagnosticMsg.message =
        "No ground truth odometry message received since than 5 seconds!"
        " GNSS simulator stalling!";

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 5000,
        "No ground truth odometry message since more than 5 seconds!");
  }

  // Add diagnostic message to diagnostic array message
  diagnosticArrayMsg.status.push_back(diagnosticMsg);
  diagnosticArrayMsg.header.stamp = currentTimestamp;

  // Add diagnostic array message to custom GNSS PVT message
  gnssPvtCustomMsg.diagnostic_array = diagnosticArrayMsg;

  // Publish GNSS PVT message
  pGnssPvtPublisher_->publish(gnssPvtCustomMsg);

  // Publish NavSatFix message
  pNavSatFixPublisher_->publish(navSatFixMsg);
}

/**
 * @brief GNSS attitude (compass) simulator loop callback function.
 *
 * @details This function is called periodically by a timer with the specified
 * sample rate and publishes the custom GNSS attitude (compass) message.
 */
void GnssSimulatorNode::gnssAttitudeSimulatorCallback() {
  // Get current time
  rclcpp::Time currentTimestamp = now();

  // Declare custom GNSS attitude message
  nanoauv_sensor_driver_interfaces::msg::GnssAttitude gnssAttitudeCustomMsg;

  // Declare diagnostic message and array message
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS Compass";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic message to diagnostic array message
    gnssAttitudeCustomMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom GNSS attitude message with only diagnostic message
    pGnssAttitudePublisher_->publish(gnssAttitudeCustomMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;
  } else {
    // **************************************************************************
    // Ground truth odometry message odometry extraction
    // **************************************************************************

    // Read out odometry message
    Eigen::Vector3d p_nb_n_true;
    Eigen::Quaterniond q_b_n_true;
    Eigen::Vector3d w_nb_b_true;  // = w_eb_b

    // Extract position from odometry message
    p_nb_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.position.x;
    p_nb_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.position.y;
    p_nb_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.position.z;

    // Convert NED position to geodetic ECEF position given a reference
    // position
    Eigen::Vector3d p_eb_e_true =
        navigation_utilities::convertNedToGeodeticEcef(p_nb_n_true,
                                                       p_eb_e_ref_);

    // Extract attitude from odometry message
    q_b_n_true.w() = groundTruthOdomMsg_.get()->pose.pose.orientation.w;
    q_b_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.orientation.x;
    q_b_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.orientation.y;
    q_b_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.orientation.z;

    // Extract angular velocity from odometry message
    w_nb_b_true.x() = groundTruthOdomMsg_.get()->twist.twist.angular.x;
    w_nb_b_true.y() = groundTruthOdomMsg_.get()->twist.twist.angular.y;
    w_nb_b_true.z() = groundTruthOdomMsg_.get()->twist.twist.angular.z;

    // Generate GNSS attitude measurement
    GnssAttMeasurement gnssAttMeasurement =
        pGnssSimulator_->generateAttMeasurement(p_eb_e_true, q_b_n_true,
                                                w_nb_b_true);

    // **************************************************************************
    // GnssAttitude Custom Message
    // **************************************************************************
    gnssAttitudeCustomMsg.header.stamp = currentTimestamp;
    gnssAttitudeCustomMsg.header.frame_id = "gnss_main_antenna_link";

    gnssAttitudeCustomMsg.attitude_rpy.x = gnssAttMeasurement.attitude_euler(0);
    gnssAttitudeCustomMsg.attitude_rpy.y = gnssAttMeasurement.attitude_euler(1);
    gnssAttitudeCustomMsg.attitude_rpy.z = gnssAttMeasurement.attitude_euler(2);

    gnssAttitudeCustomMsg.angular_velocity.x =
        gnssAttMeasurement.angular_velocity(0);
    gnssAttitudeCustomMsg.angular_velocity.y =
        gnssAttMeasurement.angular_velocity(1);
    gnssAttitudeCustomMsg.angular_velocity.z =
        gnssAttMeasurement.angular_velocity(2);

    // Assign attitude and angular velocity covariance
    for (int i = 0; i < 9; i++) {
      gnssAttitudeCustomMsg.attitude_rpy_covariance[i].data =
          gnssAttMeasurement.attitude_euler_covariance(i);
      gnssAttitudeCustomMsg.angular_velocity_covariance[i].data =
          gnssAttMeasurement.angular_velocity_covariance(i);
    }

    // Assign mode and validity flags
    gnssAttitudeCustomMsg.mode.data = gnssAttMeasurement.mode;
    gnssAttitudeCustomMsg.is_valid.data = gnssAttMeasurement.is_valid;

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "GNSS Simulator";
    diagnosticMsg.hardware_id = "GNSS Compass";
    diagnosticMsg.message = "GNSS attitude measurement nominal";

    // Calculate time since last odometry message
    rclcpp::Duration timeSinceLastOdom =
        rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

    // Check if odometry message frequency is too slow
    if (timeSinceLastOdom.seconds() > sample_time_pvt_ &&
        odometry_timeout_ == false) {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diagnosticMsg.name = "GNSS Simulator";
      diagnosticMsg.hardware_id = "GNSS Compass";
      diagnosticMsg.message =
          "Ground truth odometry message frequency is too slow!"
          " GNSS simulator ground truth frequency higher than odometry!"
          " Increase odometry message frequency!";

      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *get_clock(), 5000,
          "Ground truth odometry message frequency is too slow!");
    }

    // Check if odometry has timed out
    if (odometry_timeout_ == true) {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      diagnosticMsg.name = "GNSS Simulator";
      diagnosticMsg.hardware_id = "GNSS Compass";
      diagnosticMsg.message =
          "No ground truth odometry message received since than 5 seconds!"
          " GNSS simulator stalling!";

      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *get_clock(), 5000,
          "No ground truth odometry message since more than 5 seconds!");
    }

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic array message to custom GNSS attitude message
    gnssAttitudeCustomMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish GNSS attitude message
    pGnssAttitudePublisher_->publish(gnssAttitudeCustomMsg);
  }
}

/**
 * @brief Odometry callback function.
 *
 * @details This function is called when a new odometry message is received.
 * The ground truth odometry message is assigned to the ground truth odometry
 * message member variable of the node class.
 *
 * @param[in] msg Pointer to the odometry message
 */
void GnssSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (first_odometry_received_ == false) {
    first_odometry_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "First ground truth odometry message received! GNSS simulator "
                "now running nominal!");

    RCLCPP_INFO(this->get_logger(), "Subscribed to odometry topic: %s",
                pOdometrySubscriber_->get_topic_name());
  }

  // Reset odometry timeout flag
  if (odometry_timeout_ == true) {
    odometry_timeout_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Ground truth odometry message received after timeout! GNSS "
                "simulator now running nominal!");

    RCLCPP_INFO(this->get_logger(), "Subscribed to odometry topic: %s",
                pOdometrySubscriber_->get_topic_name());
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  // Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 *
 * @details This function is called when no ground truth odometry message is
 * received after a defined number of seconds. The odometry timeout flag is
 * set to true and a warning message is printed to the console.
 */
void GnssSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                       "No ground truth odometry message since more than 5 "
                       "seconds! GNSS simulator now starting to stale!");
}

/**
 * @brief Publish static tf2 transformations.
 *
 * @details This function publishes static tf2 transformations between the
 * base_link (body frame) and the gnss_link (sensor frame).
 */
void GnssSimulatorNode::publishStaticTf2Transforms() {
  // Get GNSS simulator parameters
  GnssSimParams gnssSimParams = this->pGnssSimulator_->getGnssSimParams();

  // Convert rotation matrix to quaternion (sensor to body frame)
  Eigen::Quaterniond q_s_b = Eigen::Quaterniond(gnssSimParams.C_s_b);

  // Fill tf2 transform message between base_link (body) and gnss_link
  // (sensor)
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = now();

  tfMsg.header.frame_id = "base_link";
  tfMsg.child_frame_id = "gnss_main_antenna_link";

  tfMsg.transform.translation.x = gnssSimParams.p_bs_b(0);
  tfMsg.transform.translation.y = gnssSimParams.p_bs_b(1);
  tfMsg.transform.translation.z = gnssSimParams.p_bs_b(2);

  tfMsg.transform.rotation.w = q_s_b.w();
  tfMsg.transform.rotation.x = q_s_b.y();
  tfMsg.transform.rotation.y = q_s_b.x();
  tfMsg.transform.rotation.z = q_s_b.z();

  pStaticTf2Broadcaster_->sendTransform(tfMsg);
}

}  // namespace gnss_simulator

/**
 * @brief Main function of the GNSS simulator node.
 *
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 *
 * @return int return value
 */
int main(int argc, char** argv) {
  // Create GNSS simulator object
  std::shared_ptr<gnss_simulator::GnssSimulator> pGnssSimulator =
      std::make_shared<gnss_simulator::GnssSimulator>();

  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<gnss_simulator::GnssSimulatorNode>(pGnssSimulator));
  rclcpp::shutdown();

  return 0;
}
