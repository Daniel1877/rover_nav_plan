# Launch Helpers

## Overview

This package contains helper functions to be used in the python ros2 launch files.

**Keywords:** launch, ros2, python

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The Launch Helpers package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Xacro](http://wiki.ros.org/xacro) (XML Markup language simplyfing URDF robot model definition.).

		sudo apt-get install ros-foxy-xacro

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

    cd ros2_ws/src
    git clone https://github.com/esa-prl/launch_helpers.git
    cd ../
    colcon build
    source install/setup.zsh


## Usage

Include the module in your launch file 'example.launch.py'.

    import launch_helpers

Or import the required functions directly:
    
    from launch_helpers import add_namespace_to_yaml


See 'launch_helpers/__init__.py' for available functions.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org