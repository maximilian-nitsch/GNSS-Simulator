# C++/ROS 2 Simple GNSS-Simulator
![Build](https://github.com/maximilian-nitsch/GNSS-Simulator/actions/workflows/ci.yaml/badge.svg)<!-- -->
![License](https://img.shields.io/github/license/maximilian-nitsch/GNSS-Simulator.svg)<!-- -->
[![Last Commit](https://img.shields.io/github/last-commit/maximilian-nitsch/GNSS-Simulator)](https://github.com/maximilian-nitsch/GNSS-Simulator/commits/main)<!-- -->
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://index.ros.org/doc/ros2/Installation/Humble/)<!-- -->
[![Release](https://img.shields.io/github/v/release/maximilian-nitsch/GNSS-Simulator)](https://github.com/maximilian-nitsch/GNSS-Simulator/releases)<!-- -->
[![Open Issues](https://img.shields.io/github/issues/maximilian-nitsch/GNSS-Simulator)](https://github.com/maximilian-nitsch/GNSS-Simulator/issues)<!-- -->
[![Contributors](https://img.shields.io/github/contributors/maximilian-nitsch/GNSS-Simulator)](https://github.com/maximilian-nitsch/GNSS-Simulator/graphs/contributors)

<!--- protected region package header begins -->
**Author:**
- Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>

**Affiliation:** Institute of Automatic Control - RWTH Aachen University

**Maintainer:**
  - Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>
<!--- protected region package header ends -->

## Description
This project provides a simple GNSS simulator written in C++.

The simulator implements the following features:
- PVT measurement generation with lever-arm effect and white noise
- Attitude measurement (GNSS compass) with lever-arm effect generation with white noise
- All parameters for a GNSS receiver can be configured in a YAML file

An example config for a Septentrio Mosaic-H GNSS receiver is provided.

## Table of Contents

- [Dependencies](#dependencies)
- [ROS 2 Nodes](#ros-2-nodes)
  - [Publishers](#publisher-node)
  - [Subscribers](#subscriber-node)
- [Installation](#installation)
- [Usage](#usage)
- [Coding Guidelines](#coding-guidelines)
- [References](#references)
- [Reports](#reports)
- [Contributing](#contributing)
- [License](#license)

# Dependencies

This project depends on the following literature and libraries:

- **Eigen3**: Eigen is a C++ template library for linear algebra: [Eigen website](https://eigen.tuxfamily.org/).
- **ROS 2 Humble**: ROS 2 is a set of software libraries and tools for building robot applications: [ROS 2 Installation page](https://docs.ros.org/en/humble/Installation.html).
- **Navigation Utilities Package**: ROS 2 package with common navigation utilities for TRIPLE-GNC: [Navigation Utilities](https://gitlab.informatik.uni-bremen.de/triple/gnc/utilities/navigation-utilities)


## ROS 2 Node Description

The GNSS simulator node implements three publishers and subscribes to one topic.
ROS 2 services or actions are not provided.

### Publishers

This node publishes the following topics:

| Topic Name       | Message Type        | Description                        | Link     |
|------------------|---------------------|------------------------------------|----------|
| `gnss_pvt`   | `nanoauv_sensor_driver_interfaces/GnssPvt.msg`   | Custom GNSS PVT (position, velocity, time) data | [GnssPvt.msg](https://gitlab.informatik.uni-bremen.de/triple/gnc/interfaces/-/blob/b95efc88d33a9e439025056c988c6459589b86e5/nanoauv_sensor_driver_interfaces/msg/GnssPvt.msg) |
| `gnss_attitude`  | `nanoauv_sensor_driver_interfaces/GnssAttitude.msg` | Custom GNSS attitude (compass) data | [GnssAttitude.msg](https://gitlab.informatik.uni-bremen.de/triple/gnc/interfaces/-/blob/b95efc88d33a9e439025056c988c6459589b86e5/nanoauv_sensor_driver_interfaces/msg/GnssAttitude.msg) |
| `nav_sat_fix`  | `sensor_msgs/NavSatFix.msg` | Navigation Satellite fix for any Global Navigation Satellite System | [NavSatFix.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) |

### Subscribers

This node subscribes to the following topics:

| Topic Name        | Message Type        | Description                        | Link     |
|-------------------|---------------------|------------------------------------|----------|
| `odometry`| `nav_msgs/Odometry.msg`| Estimate of a position and velocity in free space | [Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) |


# Installation

To install the `gnss_simulator_package`, you need to follow these steps:

1. **Install Eigen3**: Eigen3 is a dependency for your package. You can install it using your package manager. For example, on Ubuntu, you can install it using the following command:

    ```bash
    sudo apt-get install libeigen3-dev
    ```

2. **Install ROS 2 Humble**: Ensure you have ROS 2 (Humble) installed. You can follow the official installation instructions provided by ROS 2. Visit [ROS 2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions tailored to your platform.

3. **Clone the Package**: Clone the package repository to your ROS 2 workspace. If you don't have a ROS 2 workspace yet, you can create one using the following commands:

    ```bash
    mkdir -p /path/to/ros2_workspace/src
    cd /path/to/ros2_workspace/src
    ```

    Now, clone the package repository:

    ```bash
    git clone <repository_url>
    ```

    Replace `<repository_url>` with the URL of your package repository.

4. **Build the Package**: Once the package is cloned, you must build it using colcon, the default build system for ROS 2. Navigate to your ROS 2 workspace and run the following command:

    ```bash
    cd /path/to/ros2_workspace
    colcon build
    ```

    This command will build all the packages in your workspace, including the newly added package.

5. **Source the Workspace**: After building the package, you need to source your ROS 2 workspace to make the package available in your ROS 2 environment. Run the following command:

    ```bash
    source /path/to/ros2_workspace/install/setup.bash
    ```

    Replace `/path/to/ros2_workspace` with the actual path to your ROS 2 workspace.

That's it! Your `gnss_simulator_package` should now be installed along with its dependencies and ready to use in your ROS 2 environment.

## Usage

1. **Configure your YAML file** for your GNSS receiver or use the default file.

2. **Start the GNSS simulator** with the launch file:
    ```bash
    ros2 launch gnss_simulator_package gnss_simulator.launch.py
    ```
  The GNSS simulator prints your settings and waits for a ground truth odometry message.

3. **Provide an odometry publisher** from you vehicle simulation.

4. **Check ROS 2 topics** The GNSS measurements should now be published.

**Important Usage Information**:
- The odometry message must be published with at least the GNSS PVT and attitude sample times.
- The messages `/gnss_pvt/diagnostic_array` and `/gnss_attitude/diagnostic_array` will show `WARN` if the odometry rate is lower.
- If no odometry message is published, the messages `/gnss_pvt/diagnostic_array` and `/gnss_attitude/diagnostic_array` will show `STALE`.
- If everything is correct `/gnss_pvt/diagnostic_array` and `/gnss_attitude/diagnostic_array` will show `OK`. 

## Coding Guidelines

This project follows these coding guidelines:
- https://google.github.io/styleguide/cppguide.html
- http://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html 

## Contributing

You can see the [CONTRIBUTING](CONTRIBUTING) file for details if you'd like to contribute to the project.

## License

This project is licensed under the BSD-3-Clause License. Please look at the [LICENSE](LICENSE) file for details.
