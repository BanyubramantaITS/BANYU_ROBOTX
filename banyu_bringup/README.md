# Banyu Bringup (`banyu_bringup`)

This package contains launch files and bringup configurations for running the **BANYU RobotX / BlueROV2** system in both Gazebo simulation and real hardware environments.

---

## 🚀 Launch Files

### `launch/proof_of_readiness.launch.py`

Main bringup script for the Proof of Readiness task environment.

#### Usage Commands

- **Default Simulation Mode (Gazebo + SITL)**:
  ```bash
  ros2 launch banyu_bringup proof_of_readiness.launch.py
  ```

- **Bare ArduSub SITL Mode**:
  ```bash
  ros2 launch banyu_bringup proof_of_readiness.launch.py use_ardusub_sitl:=False
  ```

- **Real Hardware Mode**:
  ```bash
  ros2 launch banyu_bringup proof_of_readiness.launch.py use_sim:=False fcu_url:=udp://192.168.2.1:14550@
  ```

---

## ⚙️ Launch Options

| Argument | Options | Default | Description |
| :--- | :--- | :--- | :--- |
| `use_sim` | `True` / `False` | `True` | Runs Gazebo simulation and SITL when `True`; skips sim for real ROV hardware when `False`. |
| `use_sim_time` | `True` / `False` | `True` | Sets `use_sim_time` parameter for ROS 2 nodes. |
| `use_ardusub_sitl` | `True` / `False` | `True` | Uses `sim_vehicle.py` SITL wrapper (UDP 14550 for QGC) when `True`; uses bare `ardusub` when `False`. |
| `fcu_url` | string | `udp://127.0.0.1:14550@` | Target MAVLink FCU address (`udp://192.168.2.1:14550@` for physical BlueROV2). |
