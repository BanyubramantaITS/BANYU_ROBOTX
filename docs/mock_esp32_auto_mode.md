# Mock ESP32 Auto/Mission Mode (Land Testing Guide)

This guide explains how to use the **Mock ESP32 Auto/Mission Mode** implemented in `banyu_teleop` for dry land testing and demonstrations on the physical BANYU ROV.

---

## 1. Overview & Land Safety Motivation

Running autonomous missions (`banyu_mission`) in water requires healthy depth sensors, camera visuals, and navigation locks. When demonstrating or verifying thruster operation on **land** (e.g. in a workshop, lab, or booth):
1. Normal autonomous missions cannot run because there is no water for depth holding or course gates for visual servoing.
2. Thrusters spinning in air at full power can quickly overheat motors and ESCs, create hazardous flying debris, or damage bearings.
3. Therefore, the mock auto mode commands **all 8 thrusters to spin SLOWLY** (~10% thrust, configurable).
4. Manual stick commands from the transmitter are **strictly rejected / locked out** while in this mode to ensure deterministic demonstration behavior.
5. Returning the transmitter switch to **MANUAL** or **DISARM** immediately halts the mock mode and restores manual stick control or disarms the propulsion system.

---

## 2. Transmitter Switch Controls (RadioMaster TX16S)

The mock auto mode triggers directly from the 3-position **`SB`** switch (kill/auto channel):

| Control | Function | Position | PWM Band | Action in Mock Auto Mode |
| :--- | :--- | :--- | :--- | :--- |
| **`SB`** (3-pos) | **Auto / Kill Switch** | **Up (`↑`)** | `< 1200` (e.g. `988`) | **Starts Mock Auto Mode**: Automatically arms (if configured), switches flight mode to `MANUAL`, streams slow forward + slow heave commands to turn on all 8 thrusters, and locks out manual sticks. |
| **`SB`** (3-pos) | **Auto / Kill Switch** | **Mid (`-`)** | `1200`–`1750` (`1500`) | **Cancels Mock Auto Mode**: Immediately releases RC overrides and restores full manual stick control on the transmitter. |
| **`SB`** (3-pos) | **Auto / Kill Switch** | **Down (`↓`)** | `> 1750` (e.g. `2011`) | **Emergency Kill / Disarm**: Immediately cancels mock mode, releases overrides, sends disarm to Pixhawk, and sends kill command `'2'` to ESP32. |
| **`SF`** (2-pos) | **Arm / Disarm Switch** | **Up (`↑`)** | `< 1500` (e.g. `988`) | **Disarm**: Cancels mock auto mode immediately and disarms thrusters. |
| **`SF`** (2-pos) | **Arm / Disarm Switch** | **Down (`↓`)** | `>= 1500` (e.g. `2011`) | **Arm**: Manual vehicle arming. |

---

## 3. How All 8 Thrusters Turn On Slowly

The BlueROV2 Heavy frame uses `SUB_FRAME_VECTORED_6DOF` with 8 thrusters:
- **Motors 1–4 (Horizontal Vectored)**: Controlled by Forward (surge), Lateral (sway), and Yaw.
- **Motors 5–8 (Vertical Vectored)**: Controlled by Throttle (heave), Pitch, and Roll.

When mock auto mode activates, `banyu_teleop` streams continuous override pulses at 20 Hz (`/mavros/rc/override`):
- `Forward` (Channel 5): set to `mock_auto_forward_pwm` (default **`1540`** PWM, neutral is 1500).
- `Throttle` (Channel 3): set to `mock_auto_throttle_pwm` (default **`1540`** PWM, neutral is 1500).
- `Pitch`, `Roll`, `Yaw`, `Lateral`: set to **`1500`** PWM (neutral hold).
- Switch channels (`arm_channel`, `mode_channel`, `kill_channel`): kept strictly at **`0` (`CHAN_RELEASE`)** so switch operations are never overridden.

**Result:**
- Motors 1, 2, 3, 4 spin slowly forward.
- Motors 5, 6, 7, 8 spin slowly upward.
- **All 8 thrusters run simultaneously at safe, gentle speeds for land demonstration.**

---

## 4. Manual Stick Lockout & Rejection Mechanism

In ArduSub's `RC_Channel.cpp`, whenever a channel has an active override (`has_override() == true`):
```cpp
if (has_override()) {
    radio_in = override_value;
} else {
    radio_in = hal.rcin->read(ch_in);
}
```
Because `banyu_teleop` streams overrides for all motion axes (Pitch, Roll, Throttle, Yaw, Forward, Lateral), ArduSub completely ignores physical radio receiver pulses from the transmitter sticks. Deflecting sticks while in auto mode produces zero effect on the vehicle.

---

## 5. Parameter Reference

The single source of truth for `banyu_teleop` parameters is `banyu_teleop/config/teleop_rcin.yaml`:

```yaml
banyu_teleop:
  ros__parameters:
    # Enable/disable mock auto mode
    enable_mock_auto: true

    # Automatically call /mavros/cmd/arming when entering auto mode if not already armed
    auto_arm_on_auto_mode: true

    # Flight mode used during mock auto (MANUAL provides pure open-loop motor spin without sensor windup)
    mock_auto_flight_mode: "MANUAL"

    # Streaming rate (Hz) for RC overrides
    mock_auto_publish_rate_hz: 20.0

    # Slow PWM values (1500 is neutral stopped, 1100-1900 range)
    mock_auto_forward_pwm: 1540     # Spins horizontal thrusters 1-4
    mock_auto_throttle_pwm: 1540    # Spins vertical thrusters 5-8
    mock_auto_lateral_pwm: 1500     # Neutral
    mock_auto_yaw_pwm: 1500         # Neutral
    mock_auto_pitch_pwm: 1500       # Neutral
    mock_auto_roll_pwm: 1500        # Neutral

    # Velocity setpoints (used if flight mode is set to GUIDED)
    mock_auto_linear_x: 0.1         # 0.1 m/s forward
    mock_auto_linear_z: 0.1         # 0.1 m/s heave
```

---

## 6. Hardware ESP32 Simulator (`mock_esp32.py`)

If testing on a laptop or computer without the physical ESP32 board attached, run the standalone simulator:

```bash
# Start mock ESP32 pseudo-terminal (creates /tmp/ttyESP32)
python3 src/BANYU_ROBOTX/tools/scripts/mock_esp32.py

# In another terminal, point banyu_teleop at the virtual port
ros2 run banyu_teleop banyu_teleop --ros-args -p serial_port:=/tmp/ttyESP32
```

The simulator features:
- Ingestion of `'0'`, `'1'`, `'2'` from `banyu_teleop`
- Heartbeat watchdog for `esp32_bridge_node` (500 ms timeout in mode 2)
- Interactive CLI: type `0`, `1`, or `2` into the terminal to simulate hardware switch toggles.
