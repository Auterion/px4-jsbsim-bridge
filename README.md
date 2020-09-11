<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# JSBSim Bridge
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
