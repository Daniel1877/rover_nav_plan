# Rover Msgs

## Overview

This package contains custom message definitions used for the rover platforms of the Planetary Robotics Laboratory at ESA.

**Keywords:** msgs, rover, package

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The Rover Msgs package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/rover_msgs.git
	cd ../
	colcon build

## Usage

Include the message package in your `CMakeLists.txt` and `package.xml` (C++) or simply your `package.xml` and start using the messages.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
