<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# JSBSim Bridge
[![Build Tests](https://github.com/Auterion/px4-jsbsim-bridge/workflows/Build%20Tests/badge.svg?branch=master)](https://github.com/Auterion/px4-jsbsim-bridge/actions?query=workflow%3A%22Build+Tests%22)

JSBSim bridge is a integration of the PX4 mavlink HIL interface to communicate with [jsbsim](https://github.com/JSBSim-Team/jsbsim)


## Installation (JSBSim)
JSBSim rolling releases is available in the [release page](https://github.com/JSBSim-Team/jsbsim/releases) of JSBSim

To have the visualization available, install flightgear.

```
apt install flightgear
```

## Running the bridge
When run from the Firmware, an example can be run with the following
```
make px4_sitl jsbsim
```

To run without the flightgear visualization,
```
HEADLESS=1 make px4_sitl jsbsim
```

## Running the bridge with ROS
Clone the `px4-jsbsim-bridge` package into your catkin workspace:
```
cd <path_to_catkin_ws>/src
git clone https://github.com/Auterion/px4-jsbsim-bridge.git
```
Build the  `jsbsim_bridge` catkin package:
```
catkin build jsbsim_bridge
```
:::note
You must have already set MAVROS in your workspace (if not, follow the instructions in the [MAVROS installation guide](../ros/mavros_installation.md)).
:::
To start JSBSim through ROS using the launch file as shown:
```
roslaunch jsbsim_bridge px4_jsbsim_bridge.launch
```
