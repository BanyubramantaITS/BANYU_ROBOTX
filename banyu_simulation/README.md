# Banyu Simulation (`banyu_simulation`)

This package provides the Gazebo simulation environment, 3D world models, and launch configurations for the **BANYU RobotX / BlueROV2** underwater system.

---

## 📦 Package Contents

- **`worlds/main_world.world`**: Task 2 world (`mode:=task2`); loads `banyu::DvlMavlink` and `banyu::UnderwaterTurbidity`.
- **`worlds/proof_of_readiness.world`**: POR world (`mode:=por`) featuring buoyancy physics, underwater environment settings, lighting, and the DVL system plugin.
- **`worlds/pipeline_lights.sdf`**: RobotX 2026 Task 2 underwater pipeline world with 4 optical light nodes, magnetometer sensor plugin, and inspection camera.
- **`models/Gate/`**: 3D underwater gate model (`model.xacro` & `model.sdf`) with PBR textures.
- **`models/por_pool/`**: Proof of Readiness pool model (`model.xacro`).
- **`models/yellow_buoy/`**: Yellow navigation buoy model (`model.xacro`).
- **`models/pipeline_robotx/`**: Schedule 40 PVC competition pipeline structure with 3 base stands (`mesh.stl`).
- **`models/robotx_light_node/`**: Competition optical light node with 4-layer optical stack (point light, WS2812 lenses, glow halo, lid) and TLV493D 3-axis magnetometer.
- **`models/underwater_magnetic_activated_light/`** (symlink: `underwater-magnetic-activated-light`): Composite model assembling `pipeline_robotx` with the 4 `robotx_light_node`s on the pipe crown.
- **`models/underwater_pipeline/`**: Modular straight pipeline assembly.
- **`scripts/`**: RobotX Task 2 Python state machine controller, API, CLI monitor, and test suites.

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

## 🧩 World Systems (`src/`)

`main_world.world` gets its Task 2 props from `banyu_robotx_models` (a `package.xml` dependency of this package; it is not listed in `import.repos`). Systems built here:

- **`banyu::DvlMavlink`** (loaded by `main_world.world`, simulation only): DVL velocity → ArduSub SITL as `VISION_POSITION_DELTA`; see `banyu_bringup/README.md`.
- **`banyu::HydrophoneArray`** (loaded by the `bluerov2_task2` model): pinger TDOA arrivals.
- **`banyu::PipelineLights`** (built here; no world in this repo loads it directly): turns one listed light node RED and the rest GREEN. The seed comes from `<seed>`, or from `BANYU_LIGHT_SEED` (the launch arg `light_seed`) if that is set.
- **`banyu::UnderwaterTurbidity`** (loaded by `main_world.world`): an Ogre2 post-process on the listed `<camera>` sensors. It applies range-based per-channel attenuation plus backscatter veil. Only the server-side sensor image gets it; the GUI view stays clear. The coefficients are placeholders, not measured venue water.

---

## 🛁 DVL System Plugin

> [!NOTE]
> The DVL exists only in simulation. The team has no physical DVL, so it is
> never used on the real AUV.

`worlds/proof_of_readiness.world` declares `gz-sim-dvl-system`, which services
the Water Linked A50 mounted on `bluerov2_heavy` (the sensor itself lives in the
`bluerov2_gz` repo) and publishes `gz.msgs.DVLVelocityTracking` on `/dvl/velocity`.

> [!IMPORTANT]
> The DVL plugin **must** be declared at world level. Putting it inside a model
> is silently ignored — no error, no topic. `gz-sim-sensors-system` is the
> opposite case: it stays in the model and must **not** also appear here, or the
> render thread crashes with `Scene already exists with name: scene`.

Check the sensor is alive and has bottom lock:

```bash
gz topic -e -t /dvl/velocity -n 1
```

`target { type: DVL_TARGET_BOTTOM }` is bottom lock; `DVL_TARGET_WATER_MASS`
means the beams are not reaching the pool floor. In the PoR pool the slant range
sits around 0.57–1.80 m through the whole mission, well inside the sensor's
0.05 m minimum.

The DVL is only *used* below `dive_depth` — EKF3 runs on GPS at the surface and
switches source sets on the way down. Note that SITL's synthetic GPS is derived
from Gazebo ground truth and keeps streaming underwater, so it is the active
source set, not GPS availability, that decides what the filter is really using.
Both halves are documented in
[`banyu_bringup/README.md`](../banyu_bringup/README.md#-navigation-sources-gps-on-the-surface-dvl-underwater).

---

## 🚀 Prerequisites & Setup
 
> [!IMPORTANT]
> **Gazebo Harmonic Bridge Requirement**:
> In ROS 2 Jazzy, `ros-jazzy-ros-gz` natively bridges Gazebo Harmonic. (If using ROS 2 Humble, install `ros-humble-ros-gzharmonic`).
> Ensure official Gazebo Harmonic packages and rosdeps are installed:
> ```bash
> rosdep install --from-paths src -i -r -y
> ```

Build the packages in your ROS 2 workspace:

```bash
cd <WORKSPACE>
colcon build --packages-select banyu_simulation banyu_bringup --symlink-install
source install/setup.bash
```

---

## 🎮 Launch Usage

Launch the simulation environment using `banyu_bringup`:

### 1. Default Simulation Mode (Gazebo + SITL + QGC)
Launches Gazebo Sim with all models and ArduSub SITL via `sim_vehicle.py`. **QGroundControl (QGC)** automatically connects via UDP port 14550.

- **Proof of Readiness (POR)**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=por
  ```

- **Task 2 (Pipeline & Rings)**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=task2
  ```

### 2. Bare ArduSub SITL Mode
Runs Gazebo Sim with the bare `ardusub` executable:

```bash
ros2 launch banyu_bringup main_launch.launch.py mode:=por use_ardusub_sitl:=False
```

### 3. Real ROV Hardware Mode
Disables Gazebo Sim and SITL processes to run autonomy nodes connected to a physical ROV:

```bash
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por fcu_url:=udp://127.0.0.1:14551@
```

---

## ⚙️ Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `mode` | `por` | Competition mission: `por` (Proof of Readiness) or `task2` (Pipeline & Rings). |
| `use_sim` | `True` | Set to `True` for Gazebo + SITL simulation; set to `False` for real hardware deployment. |
| `use_sim_time` | `True` | Enables ROS 2 simulation time (`/clock`). |
| `use_ardusub_sitl` | `True` | Set to `True` for `sim_vehicle.py` (with UDP 14550 for QGC); `False` for bare `ardusub`. |
| `fcu_url` | `udp://127.0.0.1:14551@` | MAVLink FCU endpoint URL (`udp://127.0.0.1:14551@` for SITL; `udp://192.168.2.1:14550@` for real ROV tether). |

---

## 🛰️ QGroundControl (QGC) Connection

* When running the default launcher (`use_ardusub_sitl:=True`), `sim_vehicle.py` broadcasts MAVLink packets to **UDP port 14550**. Open QGroundControl and it will auto-connect.
* If using bare `ardusub` (`use_ardusub_sitl:=False`), connect QGroundControl manually to **TCP port 5760** (Host: `127.0.0.1`, Port: `5760`).
