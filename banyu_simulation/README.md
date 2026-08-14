# Banyu Simulation (`banyu_simulation`)

This package provides the Gazebo simulation environment, 3D world models, and launch configurations for the **BANYU RobotX / BlueROV2** underwater system.

---

## 📦 Package Contents

- **`worlds/proof_of_readiness.world`**: Main simulation world featuring buoyancy physics, underwater environment settings, and lighting.
- **`models/Gate/`**: 3D underwater gate model (`model.xacro` & `model.sdf`) with PBR textures.
- **`models/por_pool/`**: Proof of Readiness pool model (`model.xacro`).
- **`models/yellow_buoy/`**: Yellow navigation buoy model (`model.xacro`).

---

## 📐 Model Specifications

### Underwater Gate Model (`models/Gate/`)
- **Width**: **2.0m** between vertical post centers ($Y = \pm 1.0\text{m}$, total crossbar length = $2.08\text{m}$).
- **Depth Placement**: Top crossbar top surface is placed **1.0m below the water surface** ($Z = -1.0\text{m}$, where water surface $Z = 0.0\text{m}$).
- **Textures**: PBR materials configured for Gazebo Sim (`gz sim`) rendering (`gate.png`).

### Pool & Water Level
- **Water Surface**: $Z = 0.0\text{m}$.
- **Pool Floor**: Spawns at $Z = -2.5\text{m}$.

---

## 🚀 Prerequisites & Setup
 
> [!IMPORTANT]
> **Gazebo Harmonic Bridge Requirement**:
> Ensure `ros-humble-ros-gzharmonic` is installed. The default ROS 2 Humble bridge package (`ros-humble-ros-gz`) targets Fortress and is incompatible with Gazebo Harmonic.
> ```bash
> sudo apt-get install -y ros-humble-ros-gzharmonic
> ```

Build the packages in your ROS 2 workspace:

```bash
cd <WORKSPACE>
colcon build --packages-select banyu_simulation banyu_bringup --symlink-install
source install/setup.bash
```

---

## 🎮 Launch Usage

Launch the Proof of Readiness simulation environment using `banyu_bringup`:

### 1. Default Simulation Mode (Gazebo + SITL + QGC)
Launches Gazebo Sim with all models (pool, gate, buoy) and ArduSub SITL via `sim_vehicle.py`. **QGroundControl (QGC)** automatically connects via UDP port 14550.

```bash
ros2 launch banyu_bringup proof_of_readiness.launch.py
```

### 2. Bare ArduSub SITL Mode
Runs Gazebo Sim with the bare `ardusub` executable:

```bash
ros2 launch banyu_bringup proof_of_readiness.launch.py use_ardusub_sitl:=False
```

### 3. Real ROV Hardware Mode
Disables Gazebo Sim and SITL processes to run autonomy nodes connected to a physical ROV tether:

```bash
ros2 launch banyu_bringup proof_of_readiness.launch.py use_sim:=False fcu_url:=udp://192.168.2.1:14550@
```

---

## ⚙️ Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_sim` | `True` | Set to `True` for Gazebo + SITL simulation; set to `False` for real hardware deployment. |
| `use_sim_time` | `True` | Enables ROS 2 simulation time (`/clock`). |
| `use_ardusub_sitl` | `True` | Set to `True` for `sim_vehicle.py` (with UDP 14550 for QGC); `False` for bare `ardusub`. |
| `fcu_url` | `udp://127.0.0.1:14550@` | MAVLink FCU endpoint URL (`udp://192.168.2.1:14550@` for real ROV tether). |

---

## 🛰️ QGroundControl (QGC) Connection

* When running the default launcher (`use_ardusub_sitl:=True`), `sim_vehicle.py` broadcasts MAVLink packets to **UDP port 14550**. Open QGroundControl and it will auto-connect.
* If using bare `ardusub` (`use_ardusub_sitl:=False`), connect QGroundControl manually to **TCP port 5760** (Host: `127.0.0.1`, Port: `5760`).
