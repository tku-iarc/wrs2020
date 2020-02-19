# TKU TIMDA for WRS 2020

## Requirements

### Setup:
```bash
$ mkdir -p wrs_ws/src && cd wrs_ws
$ git clone https://github.com/tku-iarc/wrs2020.git src/
```

### ROS Packages:
```bash
$ sudo apt-get install ros-<distro>-pcl-ros
$ sudo apt-get install ros-<distro>-camera-info-manager
```
mir_robot requirements:
```bash
$ sudo apt-get install ros-<distro>-amcl
$ sudo apt-get install ros-<distro>-robot-localization
$ sudo apt-get install ros-<distro>-map-server
$ sudo apt-get install ros-<distro>-sbpl-lattice-planner
$ sudo apt-get install ros-<distro>-nav-core-adapter
$ sudo apt-get install ros-<distro>-dwb-plugins
```

### Install modbus library `libmodbus`:
**See [linear_motion/README.md](linear_motion/README.md)**

### Python packages:
```bash
$ cd src/
$ cd pip install -r requirements.txt
```
