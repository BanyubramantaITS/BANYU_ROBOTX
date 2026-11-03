# BANYU_ROBOTX Repo

## Installation

Install these packages:
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) (or Humble)
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
* **Gazebo APT repository**:
  ```bash
  sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  ```
* **`ros_gz` and dependencies**:
  ```bash
  rosdep install --from-paths src -i -r -y
  ```
  *(In Jazzy, `ros-jazzy-ros-gz` natively targets Gazebo Harmonic)*
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)

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
git clone -b ros2 git@github.com:ArduPilot/ardupilot_gazebo.git
git clone -b banyu git@github.com:BanyubramantaITS/bluerov2_gz.git
git clone -b main https://github.com/Barunastra-ITS/rx-sos-communication-itsn.git RobotX-SoS-communication-ITSN
git clone -b main git@github.com:BanyubramantaITS/banyu-robotx-models.git   # private; Task 2 world props
cd <WORKSPACE>
ln -sf src/BANYU_ROBOTX/.agents .agents
```

`rx_msgs` comes from `RobotX-SoS-communication-ITSN` and tracks `main`; Task 2 needs at least
commit `a2a1ccc` (`rx_msgs/PipelineBuoys`). Update an existing checkout with `git pull` in each
repository under `src/` and rebuild.

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

Build the workspace (`ros_gz` is built from source here, so target the installed Gazebo Harmonic explicitly):
```sh
cd <WORKSPACE>
export GZ_VERSION=harmonic
colcon build --symlink-install
```

## Running Simulation

Launch the unified simulation stack (Gazebo Harmonic + SITL + Autonomy). MAVROS is started
separately, in another terminal: `./src/BANYU_ROBOTX/tools/scripts/launch_mavros.sh --sim`
(robot: without `--sim`, after `run_mavproxy.sh`).

- **Proof of Readiness (POR)**: use `tools/scripts/launch_por.sh` (profile selection, sim and
  real AUV, safety checks). Guide: [`docs/por/README.md`](docs/por/README.md), tuning:
  [`docs/por/tuning.md`](docs/por/tuning.md).
  ```sh
  env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --sim --profile safe
  ```

- **Task 2 (Pipeline Inspection & Ring Repair)**:
  ```sh
  ros2 launch banyu_bringup main_launch.launch.py mode:=task2
  ```

Headless simulation runs (without Gazebo GUI):
```sh
ros2 launch banyu_bringup main_launch.launch.py headless:=True mode:=por
```

Autonomy mission stack only (against running simulation or hardware):
```sh
ros2 launch banyu_bringup mission.launch.py mode:=por
# or for Task 2 sim fixture:
ros2 launch banyu_bringup mission.launch.py mode:=task2 use_sim:=True
```

Physical AUV hardware bringup (cameras, teleop safety, mission; MAVROS from `launch_mavros.sh`):
```sh
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por
```

## Hardware Setup

Devices, udev names, kill switch protocol, RTK GPS chain, bring-up order and debug scripts:
[`docs/hardware_setup.md`](docs/hardware_setup.md). Install the udev rules with
[`docs/udev_rules.md`](docs/udev_rules.md).


