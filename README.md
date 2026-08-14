# BANYU_ROBOTX Repo

## Installation

Install these packages:
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
* **`ros_gz` for Gazebo Harmonic (MANDATORY)**:
  ```bash
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-gzharmonic
  ```
  > [!CAUTION]
  > You **MUST** install `ros-humble-ros-gzharmonic`. 
  > Using the default `ros-humble-ros-gz` will cause transport protocol incompatibility errors and break camera streaming and robot control topics.
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)
* BehaviorTree.CPP multiarch symlink (if required on Ubuntu 22.04):
  ```bash
  sudo ln -sf /opt/ros/humble/lib/x86_64-linux-gnu/libbehaviortree_cpp.so /opt/ros/humble/lib/libbehaviortree_cpp.so
  ```

Build ArduSub for SITL:
```sh
cd ~/ardupilot
./waf configure --board sitl
./waf sub
```

Populate the workspace:
```sh
mkdir -p <WORKSPACE>/src
cd <WORKSPACE>/src
git clone git@github.com:BanyubramantaITS/BANYU_ROBOTX.git
vcs import < BANYU_ROBOTX/import.repos
```

Get dependencies:
```sh
rosdep update
rosdep install -y --from-paths . --ignore-src
```

MAVROS depends on GeographicLib, and [GeographicLib needs some datasets](https://ardupilot.org/dev/docs/ros-install.html):
```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

Build the workspace:
```sh
cd <WORKSPACE>
colcon build --symlink-install
```
