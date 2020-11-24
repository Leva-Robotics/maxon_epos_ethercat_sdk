# Maxon EtherCAT SDK

## Overview
This is a C++ library providing a high-level interface for controlling [Maxon](https://www.maxonmc.com/) motor controllers of the [Gold line](https://www.maxonmc.com/products/harsh-environment/servo-drive-gold-family/) over EtherCAT (using the [CANopen over EtherCAT CoE](https://www.ethercat.org/en/technology.html#1.9.1) protocol).

The lower level EtherCAT communication is handled by the [soem_interface](https://github.com/leggedrobotics/soem_interface) library.

The maxon_epos_ethercat_sdk is developed on Ubuntu 18.04 LTS with [ROS Melodic](https://wiki.ros.org/melodic).

The source code is released under the GPLv3 license.
A copy of the license is available in the *COPYING* file.

**Authors:** Jonas Junger, Johannes Pankert

**Maintainer:** Johannes Pankert, pankertj@ethz.ch

**Contributors:** Fabio Dubois, Lennart Nachtigall, Markus Staeuble, Martin Wermelinger

## Installation

### Dependencies

#### Catkin Packages

| Repo                | url                                                   | License      | Content                             |
|:-------------------:|:-----------------------------------------------------:|:------------:|:-----------------------------------:|
| soem_interface      | https://github.com/leggedrobotics/soem_interface.git  | GPLv3        | Low-level EtherCAT functionalities  |
| ethercat_sdk_master | https://github.com/leggedrobotics/ethercat_sdk_master | BSD 3-Clause | High-level EtherCAT functionalities |
| message_logger      | https://github.com/leggedrobotics/message_logger.git  | BSD 3-Clause | simple log streams                  |

#### System Dependencies (Ubuntu 18.04 LTS)

- [ROS Melodic](https://wiki.ros.org/melodic)
- catkin
- yaml-cpp

### Building from Source

To build the library from source, clone the latest version from this repository and from the dependencies into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/leggedrobotics/soem_interface.git
    git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
    git clone https://github.com/leggedrobotics/message_logger.git
    git clone https://github.com/leggedrobotics/maxon_epos_ethercat_sdk.git
	cd ../
	catkin build maxon_ethercat 

To build the examples, execute the following command inside of your catkin workspace:
	
	catkin build maxon_examples
	

## Firmware version
This library is known to work with the following firmware versions:
- 01.01.15.00
- 01.01.16.00
