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
vcs import < BANYU_ROBOTX/import.repos
cd <WORKSPACE>
ln -sf src/BANYU_ROBOTX/AGENTS.md AGENTS.md
ln -sf src/BANYU_ROBOTX/.agents .agents
```

`rx_msgs` comes from `import.repos` and tracks `main`; Task 2 needs at least commit `a2a1ccc`
(`rx_msgs/PipelineBuoys`). Update an existing checkout with `vcs pull .` from `src/` and rebuild.

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

Launch the unified simulation stack (Gazebo Harmonic + SITL + MAVROS + Autonomy):

- **Proof of Readiness (POR)**:
  ```sh
  ros2 launch banyu_bringup main_launch.launch.py mode:=por
  ```

- **Task 2 (Pipeline Inspection & Ring Repair)**:
  ```sh
  ros2 launch banyu_bringup main_launch.launch.py mode:=task2
  ```

Headless mission runs (without Gazebo GUI):
```sh
ros2 launch banyu_bringup mission.launch.py mode:=por
# or for Task 2 sim fixture:
ros2 launch banyu_bringup mission.launch.py mode:=task2 use_sim:=True
```

## Hardware Setup: USB udev Rules

To ensure persistent device nodes on the companion computer across reconnects and reboots:

1. Create a udev rules file at `/etc/udev/rules.d/99-banyu.rules`:
   ```udev
   # Pixhawk 1 (Vendor: 1209, Product: 5741)
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5741", SYMLINK+="pixhawk", MODE="0666", GROUP="dialout"

   # u-blox GNSS RTK Receiver (Vendor: 1546, Product: 01a9)
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps_rtk", MODE="0666", GROUP="dialout"
   ```

2. Reload and apply the rules:
   ```sh
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

3. Verify device symlinks:
   ```sh
   ls -l /dev/pixhawk /dev/gps_rtk
   ```


