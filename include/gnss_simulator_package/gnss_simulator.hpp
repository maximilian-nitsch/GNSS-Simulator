/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <memory>
#include <random>

#include "navigation_utilities_package/navigation_utilities.hpp"

#include "gnss_simulator_structures.hpp"

namespace gnss_simulator {

class GnssSimulator {
 public:
  /* ***************************************************************************
   * Public member functions
   ****************************************************************************/
  GnssSimulator();

  GnssSimulator(GnssSimParams gnssSimParams,
                GnssModelEnableSettings gnssModelEnableSettings,
                Eigen::Vector3d p_eo_e, double sample_time_pvt,
                double sample_time_att, unsigned int seed);

  ~GnssSimulator();

  // GNSS PVT measurement generation function
  GnssPvtMeasurement generatePvtMeasurement(const Eigen::Vector3d& p_eb_e,
                                            const Eigen::Vector3d& v_eb_b,
                                            const Eigen::Quaterniond& q_b_n,
                                            const Eigen::Vector3d& w_eb_b);

  // GNSS attitude measurement generation function
  GnssAttMeasurement generateAttMeasurement(
      const Eigen::Vector3d& p_eb_e, const Eigen::Quaterniond& q_b_n,
      const Eigen::Vector3d& w_eb_b,
      const bool convert_output_to_degrees = false);

  // Getter functions
  GnssSimParams getGnssSimParams() const;
  GnssModelEnableSettings getGnssModelEnableSettings() const;
  double getSampleTimePvt() const;
  double getSampleTimeAtt() const;
  unsigned int getSeed() const;
  bool getUseFixedRandomNumbersFlag() const;

  // Setter functions
  void setGnssSimParams(const GnssSimParams& gnssSimParams);
  void setGnssModelEnableSettings(
      const GnssModelEnableSettings& gnssModelEnableSettings);
  void setSampleTimePvt(const double& sample_time_pvt);
  void setSampleTimeAtt(const double& sample_time_att);
  void setSeed(const unsigned int seed);
  void setUseFixedRandomNumbersFlag(const bool useFixedRandomNumbers);

  // Print function
  std::stringstream printGnssSimulatorParameters();

 private:
  /* ***************************************************************************
   * Private member functions
   ****************************************************************************/

  // Random number generation functions
  double drawRandNormalDistNum();

  /* ***************************************************************************
   * Private member variables
   ****************************************************************************/

  // GNSS simulator parameters
  GnssSimParams gnssSimParams_;

  // GNSS model enable settings
  GnssModelEnableSettings gnssModelEnableSettings_;

  // Sample times
  double sample_time_pvt_;
  double sample_time_att_;

  // Local NED origin (o) in geodetic (ECEF) coordinates
  Eigen::Vector3d p_eo_e_;

  // Random number generator for normal distribution and uniform distribution
  std::mt19937 randomNumberGenerator_;
  unsigned int seed_;
  std::normal_distribution<> normalDistribution_;

  // Flag to use fixed random numbers
  bool use_fixed_random_numbers_;
};

}  // namespace gnss_simulator