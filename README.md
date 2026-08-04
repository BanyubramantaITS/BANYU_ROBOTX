# BANYU_ROBOTX Repo

## Installation

Install these packages:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
* [ros_gz for Humble + Harmonic (ros-humble-gzharmonic)](https://gazebosim.org/docs/latest/ros_installation/)
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)

Build ArduSub for SITL:
```sh
cd ~/ardupilot
./waf configure --board sitl
./waf sub
```

Populate the workspace:
```sh
mkdir -p ~/colcon_ws/src
cd colcon_ws/src
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
cd ~/colcon_ws
colcon build --symlink-install
```
