# Rencana Improvement Arsitektur Workspace untuk Adaptasi SAUVC

Status: **Design Note / Roadmap** (dokumen perencanaan; belum mengubah implementasi kode).

Dokumen ini merangkum evaluasi arsitektur `src/BANYU_ROBOTX` saat ini serta rekomendasi perbaikan untuk mengadaptasi workspace dari kompetisi **RobotX** (berbasis USV/UAV/GCS `rx_msgs`, GPS surface transit, dan pipeline survey) ke kompetisi **SAUVC** (full underwater, kolam, tetherless, multi-task berbasis visi, akustik, dan aktuator).

---

## 1. Ringkasan Kondisi Saat Ini & Masalah Utama

Berdasarkan inspeksi kode sumber saat ini:

1. **Monolithic `mission_manager` dan `controller`**
   - [`banyu_mission/src/banyu_mission.cpp`](../banyu_mission/src/banyu_mission.cpp) (1.340 baris) mencampur fungsi BehaviorTree runner, validasi parameter POR & Task 2, koordinasi run GCS (`RunCoordinator`), sinkronisasi buoy/target, registrasi inline node (`WaitUntil` untuk `KeepClearOfZones`, `SetRingRecording`), dan fallback FCU (`disarmVehicle`, `setFlightMode`).
   - [`banyu_vehicle_execution/src/vehicle_execution_node.cpp`](../banyu_vehicle_execution/src/vehicle_execution_node.cpp) (1.422 baris) menggabungkan telemetri, watchdog safety, goal arbiter, course fence, Task 4 zone guard, camera heave, encoder RC override / setpoint, dan registrasi 19 action server dalam satu node.
   - [`banyu_vehicle_execution/src/action_servers/por_action_servers.cpp`](../banyu_vehicle_execution/src/action_servers/por_action_servers.cpp) (1.123 baris) menampung seluruh action server POR dan `TimedMoveServer` dalam satu file.
2. **Duplikasi Parameter (Belum *One Source of Truth*)**
   - Parameter yang sama dideklarasikan dan disalin ulang di beberapa blok node pada YAML yang sama. Contoh: `autonomy_mode` ditulis 3–4 kali di [`por_params_sauvc.yaml`](../banyu_bringup/config/por_params_sauvc.yaml), [`por_params_timed_safe.yaml`](../banyu_bringup/config/por_params_timed_safe.yaml), dan [`task2_params.yaml`](../banyu_bringup/config/task2_params.yaml).
   - Daftar nama topic dan action (`/banyu/actions/...`, `/mavros/...`) diduplikasi di blok `banyu_mission`, `vehicle_execution_node`, dan `banyu_vehicle_status`.
   - Geometri pemasangan sensor juga terduplikasi antara URDF/xacro dan YAML parameter (seperti dicatat di [`tf2_future_improvements.md`](tf2_future_improvements.md)).
   - [`mission.launch.py`](../banyu_bringup/launch/mission.launch.py) menumpuk banyak file YAML overlay secara berurutan (`task2_tier_*`, `task2_sensors_*`, `task2_site_*`, `task2_hydrophone_on`, `task2_dive_midpoint`, `task2_test_mission`, `task2_waypoint_mission`, `task2_no_auto_start`), sehingga sumber nilai efektif sulit ditelusuri cepat.
3. **Ledakan Jumlah Parameter**
   - [`por_mission_config.hpp`](../banyu_mission/include/banyu_mission/por_mission_config.hpp) (212 baris) dan [`por_mission_config.cpp`](../banyu_mission/src/por_mission_config.cpp) (618 baris) mendeklarasikan puluhan hingga >100 parameter per-segmen (`circle_turn_1..4_*`, `circle_forward_1..4_*`, `timed_pass_gate_*`, `sauvc_*`, `assist_*`) hanya untuk dipindahkan ke BehaviorTree Blackboard dan diteruskan ke `TimedMove`.

---

## 2. Prinsip Perbaikan Inti (Core Refactoring)

### 2.1 Merapikan `mission_manager` (`banyu_mission`)

Pisahkan tanggung jawab `BanyuMission` menjadi modul-modul kecil dengan batas yang jelas:

- **Mission Runner (BT Core):** Hanya memuat XML tree, mengeksekusi tick loop, dan memantau kondisi abort/interupsi.
- **Mission Lifecycle Provider (Strategi Terpisah):**
  - *RobotX Mode:* `RunCoordinator` + `rx_msgs/Command` + reset consumers.
  - *SAUVC / Standalone Mode:* Lifecycle lokal ringan (Ready → Armed/Submerged → Running → Surfaced/Disarmed) tanpa ketergantungan `task2_tier`, `survey_progress`, `buoy_target`, atau `Task4Zones`.
- **Vehicle Command Helper:** Memisahkan panggilan service MAVROS (`SetMode`, `CommandBool`) dari logika BT.
- **Node Registrars Terpisah:** Pindahkan inline node (`KeepClearOfZones`, `SetRingRecording`, dll.) keluar dari `banyu_mission.cpp` ke header/source masing-masing di `banyu_mission/src/action/`.

### 2.2 Merapikan `controller` (`banyu_vehicle_execution`)

Pecah `VehicleExecutionNode` dan `por_action_servers.cpp` menjadi tiga lapisan:

1. **Vehicle Output & Safety Layer (`VehicleBridge` / `SafetyGate`):**
   - Watchdog telemetri, safety fault check, `GoalArbiter`, dan output encoder (`PositionTarget` `0x5C7` maupun `OverrideRCIn`).
   - Isolasikan logic khusus permukaan RobotX (`applyFence`, `zoneGuardIntent`, `fenceGuardIntent`) ke modul opsional yang hanya aktif di mode RobotX Task 2/4.
2. **Shared Control Primitives (`banyu_vehicle_execution/control/`):**
   - Helper terpusat untuk **Depth PID**, **Heading PID** (dengan shortest-angle wrap & deadband), **Visual Servoing PID**, serta **PWM slew-rate limiter / anti-windup** agar tidak ada duplikasi rumus P-controller di tiap action server.
3. **Modular Action Servers (`action_servers/`):**
   - Satu file/kelas untuk satu skill generik (hindari menggabungkan 10 server dalam `por_action_servers.cpp`).
   - Utamakan skill yang composable dan reusable untuk semua task SAUVC (lihat Bagian 3.5).

### 2.3 One Source of Truth untuk Parameter

Terapkan aturan berikut di seluruh workspace:

- **Shared Global Parameters (`/**`):** Parameter lintas-node seperti `autonomy_mode`, `fallback_mode`, dan `use_sim_time` didefinisikan sekali di section global YAML (atau disuntikkan sekali dari launch) lalu dibaca oleh `banyu_mission`, `vehicle_execution_node`, dan `banyu_vehicle_status`.
- **Hilangkan Duplikasi Topic/Action di YAML:** Nama topic dan action internal (`/banyu/actions/...`, `/banyu/vehicle_status`) sebaiknya didefinisikan sebagai konstanta kontrak di C++/Python header bersama (atau `banyu_msgs`). Jika memang perlu diubah per robot/sim, gunakan fitur standar ROS 2 `remappings` di file launch—bukan mendefinisikan puluhan parameter `topics.*` dan `actions.*` di setiap YAML.
- **Geometri dari URDF/TF:** Posisi dan orientasi kamera depan, kamera bawah, probe/dropper, dan hydrophone disimpan hanya di URDF/xacro dan dipublish lewat `robot_state_publisher` (lihat Bagian 3.6).
- **Profil Launch Ringkas:** Untuk SAUVC, gunakan maksimal **1 base config** (`sauvc_base.yaml`) + **1 environment override** (`sim.yaml` atau `hardware.yaml`), bukan 8 overlay bertingkat.

### 2.4 Parameter Secukupnya (Minimal & Tepat Guna)

- **Batasi YAML pada Parameter Fisik & Tuning:** Simpan di YAML hanya nilai yang benar-benar merupakan karakteristik fisik AUV atau tuning kontrol:
  - Mapping channel RC & gain PWM (`rc_override.*`)
  - Batas aman kecepatan (`max_linear_speed`, `max_yaw_rate`)
  - Gain PID depth/heading/visual (`depth_pid.*`, `yaw_pid.*`, `visual_pid.*`)
  - Threshold deteksi / konfigurasi model detector
- **Parameter Per-Langkah Misi Pindah ke BT XML:** Hapus pola `circle_turn_1..4_*` dan `sauvc_*` dari C++ `PorMissionConfig` dan YAML. Nilai spesifik suatu langkah misi (mis. target kedalaman `1.0 m`, sudut putar `180 deg`, timeout `10 s`) ditulis langsung sebagai **BT input ports** di file XML misi (atau di file daftar waypoint/task jika dinamis).
- **Target Efek:** Penambahan segmen atau variasi task baru di SAUVC cukup mengedit XML Behavior Tree tanpa menyentuh C++ `Config::load()`, `Config::validate()`, `Config::apply()`, dan puluhan baris YAML.

---

## 3. Improvement Khusus untuk Kompetisi SAUVC

Kompetisi SAUVC memiliki karakteristik yang berbeda jauh dengan RobotX Task 2. Berikut area adaptasi teknis yang perlu disiapkan:

### 3.1 Pemisahan Bersih dari Domain RobotX

Di SAUVC tidak ada GCS RoboCommand (`rx_msgs`), tidak ada komunikasi USV/UAV, tidak ada GPS di kolam/bawah air, dan tidak ada Task 4 keep-out zone.

- Buat mode/entrypoint khusus (mis. `mode:=sauvc` atau branch/paket terpisah untuk stack SAUVC) yang **tidak me-load** `banyu_communication` (`uuv_comm_node`, `pipeline_buoys_node`), `rx_msgs`, `Course` fence, `Task4Zones`, maupun `gps_available` check pada preflight/readiness.
- Pada `banyu_vehicle_status` dan `banyu_mission`, pastikan readiness underwater hanya memeriksa kesehatan FCU, IMU/depth freshness, kill switch, dan kamera/detector yang relevan.

### 3.2 Upgrade Persepsi: Integrasi Hailo-8 YOLO ke ROS 2 + Tracker

Saat ini dukungan Hailo-8 baru tersedia sebagai script setup dan uji standalone ([`tools/scripts/setup/yolo_setup.sh`](../tools/scripts/setup/yolo_setup.sh) dan [`tools/scripts/check/yolo_test.py`](../tools/scripts/check/yolo_test.py)), sementara node ROS 2 [`por_detection.cpp`](../banyu_detection/src/por_detection.cpp) masih memakai HSV blob dan hanya mempublish satu objek terbaik per frame lewat [`Detection.msg`](../banyu_msgs/msg/Detection.msg) (`object_id`: `0=gate`, `1=yellow buoy`, `2=GREEN`, `3=RED`).

Untuk task SAUVC (Qualification Gate, Flare/Obstacles, Drum/Bucket warna/simbol, Mat/Golf Ball), diperlukan:

1. **Node Detektor YOLO ROS 2 (`yolo_detector_node`):**
   - Memanfaatkan `~/venv-yolo` dan `.hef` di Hailo-8 pada hardware, dengan backend fallback (ONNX/OpenCV DNN/Ultralytics) untuk simulasi Gazebo di PC.
   - Mendukung **dual camera** secara simultan atau bergantian (`front_camera` dan `bottom_camera`) karena driver keduanya sudah tersedia di [`sub_bringup.launch.py`](../banyu_bringup/launch/sub_bringup.launch.py) dan [`hardware_params.yaml`](../banyu_bringup/config/hardware_params.yaml).
2. **Kontrak Pesan Multi-Deteksi:**
   - Tambahkan pesan baru (mis. `banyu_msgs/Detection2DArray` atau `vision_msgs/Detection2DArray`) yang membawa daftar seluruh objek dalam satu frame (`class_id` / `label`, `confidence`, bounding box normalized & pixel, `offset_x`, `offset_y`, `apparent_size`).
   - Tetap pertahankan bridge/adapter jika masih ada node legacy yang membaca `banyu_msgs/Detection`.
3. **Target Filtering & Temporal Tracker:**
   - Tambahkan filter sederhana (EMA / Kalman 2D di image space + N-frame confirmation) agar pantulan cahaya permukaan kolam atau flicker deteksi tidak membuat yaw/sway controller berosilasi.
   - Sediakan opsi **color/ROI verification** (kombinasi bbox YOLO + verifikasi HSV/geometri) untuk membedakan warna drum/flare/mat saat pencahayaan kolam berubah.

### 3.3 Subsistem Payload & Aktuator

Hasil penelusuran workspace menunjukkan belum ada action server maupun node khusus aktuator misi (saat ini hanya ada kontrol lampu AUX9 di [`teleop_rcin.yaml`](../banyu_teleop/config/teleop_rcin.yaml) via `MAV_CMD_DO_SET_SERVO`).

Untuk SAUVC (ball dropper, torpedo launcher, atau gripper/claw):

- Buat **Actuator Interface** (`ActuatorCommand` service/action) yang mendukung:
  - Mapping nama payload → channel servo Pixhawk (`MAV_CMD_DO_SET_SERVO`) atau pin ESP32/GPIO, didefinisikan di satu config payload.
  - Parameter PWM `idle`, `actuate`, dan `duration_ms` serta proteksi double-trigger.
  - Reset otomatis ke posisi aman (`idle`) saat `disarm` / `abort` / start run baru.
- Buat BT node generik `<TriggerPayload name="drop_ball" payload="dropper" />`.

### 3.4 Pipeline Hydrophone / Acoustic Pinger untuk Hardware

Workspace sudah memiliki [`pinger_localization_node.cpp`](../banyu_detection/src/pinger_localization_node.cpp) dan solver TDOA least-squares di [`acoustics.cpp`](../banyu_detection/src/acoustics.cpp) (`/banyu/pinger/arrivals` → `/banyu/pinger/observation`). Namun saat ini sumber datanya hanya plugin simulasi Gazebo, dan di mode RobotX opsinya dimatikan (`use_hydrophone:=False`).

Untuk adaptasi SAUVC:

1. **Driver & DSP Hydrophone Hardware:** Buat node akuisisi sinyal hydrophone (ADC/USB audio/MCU serial) yang melakukan bandpass filter di frekuensi target SAUVC (mis. 25–45 kHz sesuai regulasi tahun berjalan), deteksi onset / cross-correlation (GCC-PHAT), lalu mempublish arrival time ke `/banyu/pinger/arrivals` atau bearing ke `/banyu/pinger/observation` dengan field `confidence` / `snr`.
2. **Proteksi Multipath Kolam:** Dinding kolam memantulkan sinyal akustik. Node DSP dan action homing perlu menyertakan *ping interval gating*, *SNR threshold*, dan *median/circular smoothing* pada bearing.
3. **SAUVC Pinger Homing Skill:** Buat action server khusus `TrackPinger` / `ApproachPinger` yang terhubung ke BT SAUVC (saat ini pembaca pinger hanya ada di `SearchPipelineServer` milik Task 2).

### 3.5 Kontrol Underwater & Skill Primitif yang Lebih Matang

Saat ini [`por_sauvc_mission.xml`](../banyu_mission/bt/por_sauvc_mission.xml) hanya mengandalkan rangkaian `TimedMove` (open-loop waktu, depth hold Bar30, dan IMU yaw cap yang berhenti mendadak lalu mengalami coasting/overshoot).

Rekomendasi perbaikan kontrol untuk SAUVC:

- **Skill Primitif Terpisah & Closed-Loop:**
  - `DiveToDepth`: Kontrol kedalaman khusus dengan settling window yang jelas.
  - `TurnToHeading` / `TurnByAngle`: Closed-loop IMU heading dengan P/PD + settling tolerance (bukan sekadar memutus thrust saat melewati sudut target sehingga bebas overshoot).
  - `MoveTimed` / `MoveWithHeadingHold`: Gerak lurus/sway dengan heading lock aktif dan batas waktu.
  - `SearchTarget`: Pola pencarian terstruktur (yaw sweep, lateral sweep, atau lawnmower mini) yang otomatis berhenti ketika `target_class` terdeteksi stabil selama N frame.
  - `VisualAlignAndApproach`: Visual servoing berbasis `offset_x`, `offset_y`, dan `apparent_size` / area bbox dari kamera depan maupun kamera bawah (untuk hover di atas drum/mat sebelum drop/pick).
- **Perbaikan Loop `ALT_HOLD` + RC Override:**
  - Tambahkan **PID penuh** (bukan P-only) + deadband kompensasi thruster + slew-rate limiter pada konversi velocity intent → PWM RC override agar gerakan halus dan baterai/ESC lebih efisien.
  - Pisahkan *feedforward buoyancy trim* bila AUV sedikit positif/negatif buoyant.
- **Opsional Dead-Reckoning / Velocity Sensor:**
  - Sesuai catatan repo, tanpa DVL estimasi posisi EKF (`nogps_odom_beacon.py`) hanya akurat untuk depth + heading. Jika di SAUVC tim menambahkan DVL atau optical flow bawah air, siapkan interface `nav_msgs/Odometry` bersih agar `MoveDistance` / `SearchPattern` dapat berjalan berbasis jarak nyata, bukan sekadar estimasi waktu.

### 3.6 TF2 di Hardware & Kalibrasi Kamera

Sesuai rencana pada [`tf2_future_improvements.md`](tf2_future_improvements.md):

- Jalankan `robot_state_publisher` (atau `static_transform_publisher`) di [`sub_bringup.launch.py`](../banyu_bringup/launch/sub_bringup.launch.py) untuk mempublish tree `base_link -> front_camera_optical_frame`, `base_link -> bottom_camera_optical_frame`, `base_link -> payload_dropper_link`, dan `base_link -> hydrophone_1..N`.
- Gunakan TF untuk menghitung offset antara **kamera bawah** dan **posisi fisik gripper/dropper** saat melakukan visual alignment di atas drum/mat, sehingga perubahan posisi bracket kamera di frame AUV cukup diubah di satu tempat (URDF).

### 3.7 Operasi Kolam Tetherless & Simulasi SAUVC

- **Start Sequence Tetherless di Kolam:**
  - Di SAUVC, AUV harus dilepas tanpa kabel dan tanpa remote RC saat start. Pastikan jalur `bypass_rc` + trigger start (mis. magnetic reed switch / tombol fisik via ESP32 atau delay timer pasca-arming) memiliki state machine yang jelas di `banyu_mission` dan indikator LED yang mudah dibaca penyelam (siap, countdown, running, fault/kill).
- **World Simulasi Kolam SAUVC (`banyu_simulation`):**
  - Saat ini world simulasi yang tersedia baru `proof_of_readiness.world` dan `main_world.world`. Tambahkan `sauvc_pool.world` yang memodelkan dimensi kolam, gate, orange flare, bucket/drum, mat/ball, dan model pinger akustik agar seluruh BT SAUVC, YOLO detector, dan pinger homing dapat diuji end-to-end sebelum pengujian kolam nyata.

---

## 4. Rekomendasi Tahapan Eksekusi (Incremental Rollout)

Agar refactor tidak melumpuhkan kemampuan uji coba AUV di kolam:

1. **Fase 1 — Rapikan Parameter & Config SAUVC**
   - Sederhanakan parameter YAML (`one source of truth` untuk `autonomy_mode`, limits, PID) dan pindahkan parameter per-segmen dari `PorMissionConfig` ke BT XML ports.
2. **Fase 2 — Pecah `vehicle_execution_node` & `banyu_mission`**
   - Ekstrak shared PID/control helpers dan pecah action servers menjadi file modular per skill; pisahkan lifecycle SAUVC dari logika GCS/RobotX (`RunCoordinator`, fence, zones).
3. **Fase 3 — Integrasi Persepsi YOLO + Aktuator SAUVC**
   - Hadirkan `yolo_detector_node` (Hailo-8 + multi-detection msg + tracker) dan `ActuatorServer` untuk dropper/gripper, lalu uji di Gazebo (`sauvc_pool.world`).
4. **Fase 4 — Hydrophone Hardware, TF2, & Pool Readiness**
   - Integrasikan TF statis di `sub_bringup.launch.py`, driver/DSP hydrophone hardware, serta prosedur start tetherless untuk uji coba kolam penuh.
