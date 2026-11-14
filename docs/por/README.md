# POR: Cara Menjalankan

Dokumen POR (Proof of Readiness) ada tiga:

| Dokumen | Isi |
|---|---|
| **README.md** (ini) | Memilih profil, menjalankan di sim dan di robot, prosedur keselamatan |
| [`tuning.md`](tuning.md) | Cara tuning: parameter yang penting, kalibrasi di kolam, warna kamera, dead thruster |
| [`sim_results.md`](sim_results.md) | Hasil simulasi, model gerak sim, skenario kerusakan |

Semua perintah dijalankan dari root workspace `ros2_ws_robotx/`. Shell-nya zsh, jadi perintah
`ros2` dijalankan lewat `bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ...'`,
atau buka `bash` dulu lalu source kedua file itu.

## 1. Gambaran singkat

- Lintasan: start → 3 m → gate (lebar 2 m, palang 1 m di bawah air) → 10 m → marker. Robot
  lewat gate, mengitari marker, lalu kembali lewat gate.
- POR terbang di mode **ALT_HOLD + RC override** (bukan GUIDED). Kedalaman ditahan Bar30,
  heading dari kompas/IMU. Stack mengirim "stick" ke Pixhawk lewat `/mavros/rc/override`.
  Robot tidak punya GPS dan DVL di bawah air, jadi tidak ada estimasi posisi: rute dikendalikan
  **waktu + kekuatan stick per segmen**.
- **Satu run = satu file profil** (`banyu_bringup/config/por_params*.yaml`). Profil berisi
  setting semua node POR; yang berbeda antar-profil terutama rute (durasi dan kecepatan tiap
  segmen) dan pemakaian kamera.
- Semua dijalankan lewat [`tools/scripts/por/launch_por.sh`](../../tools/scripts/por/launch_por.sh).

## 2. Memilih profil

### Profil

Hanya ada tiga profil. Pakai RC atau tidak dipilih saat launch (`--no-rc`, §4.4), bukan di
file profil, jadi satu file dipakai untuk dua-duanya.

| `--profile` | File | Rute |
|---|---|---|
| `safe` (default sim) | `por_params_timed_safe.yaml` | kotak dengan empat putaran 90° di tempat (berhenti oleh IMU), tanpa geser; maju full stick (1,33), kedalaman 1,4 ± 0,1 m. Dituning di trial 30 Okt |
| `fastest` | `por_params_timed_fastest.yaml` | diamond (maju + geser diagonal) di sekitar marker, mundur lewat gate; kedalaman 1,0 ± 0,3 m |
| `replay` | `por_params_replay.yaml` | putar ulang rute yang dikemudikan manual (§4.8) |

- **Thruster 2 + 3 mati:** buat profil dari hasil ukur di kolam dengan
  `tools/scripts/por/dead_thruster_por.py` ([`tuning.md`](tuning.md) §6), lalu pakai
  `--profile <nama file>`.
- **Sensor tidak bisa dipercaya (murni timer):** set `use_sensors: false` di salinan profil
  (heading hold mati, dive dan putaran open-loop), lalu pakai `--profile <path>`.
- Profil lain (`fast`, `slow`, `hybrid`, `vision`, folder `blind/` dan `sim_dead_thruster/`)
  sudah dihapus; masih ada di riwayat git kalau dibutuhkan.

`config/sim_faults/` berisi file parameter ArduPilot untuk mensimulasikan kerusakan di SITL
(bukan profil, **jangan** dimuat ke robot).

### File lain di `config/` (bukan profil)

| File | Fungsi |
|---|---|
| `task2_params.yaml` | Profil Task 2 (bukan POR) |
| `hardware_params.yaml` | Device kamera dan port ESP32 di robot |
| `mavros_config.yaml` | Setting MAVROS |
| `banyu_defaults.parm` | Parameter ArduSub bersama sim dan robot: dimuat otomatis ke SITL, di robot dimuat sekali lewat QGC (§4.1) |
| `sim_only.parm` | Tuning khusus SITL (gain attitude sim). **Jangan** dimuat ke Pixhawk robot |
| `degraded_thrusters_2_3.parm` | Parameter Pixhawk untuk mematikan thruster 2+3 (**belum diverifikasi**, lihat §8) |
| `sim_no_gps.parm` | Sim: menghapus GPS SITL (dipakai otomatis oleh `use_gps:=False`) |
| `*_camera_info.yaml` | Kalibrasi kamera |

## 3. Menjalankan di simulasi

```bash
env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --sim --profile safe
env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --sim --headless --profile fastest
```

MAVROS tidak dijalankan oleh launch file mana pun. Di terminal lain:

```bash
./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh --sim
```

Restart MAVROS setiap kali sim di-restart (waktu sim mulai dari 0 lagi).

Tunggu sampai MAVROS terhubung (`ros2 topic echo /mavros/state --once` → `connected: true`),
lalu:

```bash
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"   # kalau perlu berhenti
```

Robot spawn di permukaan (-7, 0, -0.1). Gate di x ≈ -4, marker di x ≈ 6. Misi selesai dengan
log `BehaviorTree execution finished: status=SUCCESS`.

Argumen tambahan diteruskan ke `main_launch.launch.py`:

| Argumen | Fungsi |
|---|---|
| `use_gps:=False` | GPS SITL dihapus sejak boot dan `nogps_odom_beacon` dijalankan, seperti robot asli. `launch_por.sh --sim` mengirimnya otomatis; tambahkan `use_gps:=True` kalau ingin sim dengan GPS |
| `extra_sitl_params:=$PWD/src/BANYU_ROBOTX/banyu_bringup/config/sim_faults/dead_thrusters_2_3.parm` | Simulasi thruster 2+3 mati |

Catatan sim:
- Jalankan dengan `env -u TMUX`; di dalam tmux `sim_vehicle.py` rusak.
- Proses lama (`ardusub`, `mavproxy`) bisa tertinggal dan membuat sim baru gagal. Launcher
  membersihkannya, kecuali dengan `--no-clean`.
- Kira-kira 1 dari 3 launch, Gazebo GUI tidak pernah mengirim world ke server. Launch ulang.
- Laptop dengan GPU PRIME: jalankan dengan `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`.
- Kalau laptop satu jaringan dengan robot, pakai `ROS_DOMAIN_ID` lain dan
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Kalau tidak, traffic robot memicu
  "Vehicle safety or run authorization fault".
- Gazebo berjalan sekitar 0,7× real time; semua waktu di dokumen ini waktu sim.

## 4. Menjalankan di robot

### 4.1 Persiapan sekali

1. **Parameter Pixhawk** (QGroundControl): muat `banyu_bringup/config/banyu_defaults.parm`
   (bukan `sim_only.parm`), lalu reboot. Setelah itu simpan dump parameter dari QGC
   (Parameters → Tools → Save to file) dan cek:
   ```bash
   python3 src/BANYU_ROBOTX/tools/scripts/check/check_pixhawk_params.py <dump>.params
   ```
   Skrip ini membandingkan dump dengan `banyu_defaults.parm`, frame, fungsi motor SERVO1–8, dan
   mendeteksi Bar30. Harus berakhir dengan `OK`. Yang penting untuk robot:
   - `RCMAP_FORWARD 2`, `RCMAP_LATERAL 1`, `RCMAP_THROTTLE 3`, `RCMAP_YAW 4`,
     `RCMAP_PITCH 0`, `RCMAP_ROLL 0` (§4.2).
   - `VISO_TYPE 1`, `EK3_SRC2_POSXY 0`, `EK3_SRC2_VELXY 6`, `EK3_SRC2_POSZ 1`,
     `EK3_SRC2_VELZ 0`, `EK3_SRC2_YAW 1`: dipakai `nogps_odom_beacon` supaya EKF
     mempublikasikan kedalaman dan heading tanpa GPS. Tanpa `EK3_SRC2_YAW 1`, EKF kehilangan
     sumber heading begitu `nogps_odom_beacon` memilih source set 2.
   - `FS_EKF_ACTION 0`, `FS_GCS_ENABLE 0`.
   - `FS_THR_ENABLE 1`: kalau link TX16S putus, ArduSub menetralkan stick setelah
     `RC_FS_TIMEOUT` (1 s). Dengan `0`, nilai stick terakhir tetap dipakai dan thruster terus
     berputar di MANUAL. Jangan `2` (SURFACE): di bawah air link selalu putus.
   - `MAV_GCS_SYSID 255` (sysid MAVROS), lihat §4.2.
   - **Rentang daya thruster** `MOT_PWM_MIN 1100`, `MOT_PWM_MAX 1900` (penuh): sejak board
     power ESC diganti (2026-11-13); dengan batas 50% (`1300 1700`) dua thruster vertikal hampir
     tidak bisa menyelam. Rentang ini berlaku untuk semua mode (MANUAL dari TX16S, ALT_HOLD + RC
     override POR, GUIDED Task 2) karena dipotong di mixer ArduSub. Untuk membatasi lagi:
     50% = `1300 1700`, 60% = `1260 1740`, 70% = `1220 1780`. `MOT_PWM_TYPE` harus 0
     (DShot/8/9 membuat ArduSub mengabaikan rentang ini). QGC Motor Test memakai PWM mentah.
     `launch_por.sh` tidak memeriksa batas ini. Untuk mengecek nilai di Pixhawk, jalankan
     `tools/scripts/check/check_thrust_limit.py` manual setelah `run_mavproxy.sh` (lewat port 14552).
   - Cek juga `FS_EKF_THRESH` dan `ARMING_CHECK` (tidak diatur file ini).
2. **Device** di mini-PC:
   ```bash
   ls -l /dev/pixhawk /dev/esp32_kill /dev/front_camera /dev/bottom_camera /dev/gps_rtk
   ```
   Semua config memakai nama udev ini (`tools/udev/99-banyu.rules`, lihat
   [`../udev_rules.md`](../udev_rules.md)), jadi urutan `ttyACM*`/`video*` setelah cabut-pasang
   tidak berpengaruh. Kamera depan = "H264 USB Camera", kamera bawah = "2M WebCam" (Horus B).
   Semua perangkat, kill switch, GPS RTK dan script debug: [`../hardware_setup.md`](../hardware_setup.md)
   (cek cepat: `tools/scripts/check/check_devices.sh`).

### 4.2 Peta kanal RC dan parameter Pixhawk

Hasil cek robot (Oktober 2026: halaman Radio QGC + dump parameter Pixhawk):

| Input | Kanal | Pixhawk |
|---|---|---|
| Stick maju/mundur (kanan vertikal) | CH2 | `RCMAP_FORWARD 2` |
| Stick geser (kanan horizontal) | CH1 | `RCMAP_LATERAL 1` |
| Stick naik/turun (kiri vertikal) | CH3 | `RCMAP_THROTTLE 3` |
| Stick yaw (kiri horizontal) | CH4 | `RCMAP_YAW 4` |
| SB (kill/arm, teleop) | CH6 | – |
| SC (mode terbang, teleop) | CH8 | `RC8_OPTION 213` (mount pitch; tidak berpengaruh, kamera depan fixed) |
| – | – | `RCMAP_PITCH 0`, `RCMAP_ROLL 0` (tidak dipakai) |

Semua profil (`rc_override.channel_*` = 2/1/3/4), stick di `teleop_rcin.yaml`, default
`dead_thruster_por.py --rc-channels`, takeover Task 2 (CH2), dan RCMAP SITL di
`banyu_defaults.parm` sudah memakai peta ini, jadi sim dan robot sama. Tidak ada switch di kanal
stick. Override misi tetap terbaca di `/mavros/rc/in` (ArduPilot melaporkan nilai override),
jadi kanal SB/SC tidak boleh pernah dipakai sebagai kanal stick.

Pengaman:
- `launch_por.sh --real` (mode RC) **menolak jalan** kalau kanal override sama dengan kanal
  SB/SC (`--allow-channel-overlap` untuk memaksa; jangan).
- `banyu_teleop` mencetak ERROR saat start kalau SB/SC ada di kanal stick.

**Temuan di dump parameter Pixhawk (Oktober 2026) dan keputusannya:**

| Parameter di Pixhawk | Akibat | Perbaikan |
|---|---|---|
| `MAV_GCS_SYSID 1` (`_HI 0`) | ArduSub hanya menerima RC override dari sysid 1. MAVROS memakai sysid 255, jadi **semua perintah gerak misi POR diabaikan**: robot arm tapi tidak bergerak | **Diputuskan: set `MAV_GCS_SYSID 255`** (juga ada di `banyu_defaults.parm`). GCS lain yang mengirim joystick/override harus memakai sysid 255 juga |
| `VISO_TYPE 0`, `EK3_SRC2_VELXY 0`, `EK3_SRC2_YAW 0` | `banyu_defaults.parm` belum dimuat. Tanpa GPS, `nogps_odom_beacon` memilih source set 2, yang di Pixhawk ini tidak punya sumber yaw | **Belum**: muat `banyu_defaults.parm` (§4.1) sebelum run otonom |
| `RC8_OPTION 213` (mount pitch), `RC7_OPTION 214` (mount yaw), `MNT1_RC_RATE 30` | SC (CH8) juga menggerakkan tilt mount kamera dengan laju 30°/s selama tidak di tengah: SC atas memiringkan kamera sampai -90°, SC bawah sampai +20°. Arah kamera depan saat misi jadi tergantung riwayat posisi SC | **Aman**: kamera depan fixed, tidak ada servo tilt. Kalau suatu saat dipasang mount, set `RC8_OPTION 0` |
| `BARO2_DEVID 0` | Saat dump diambil, Bar30 belum dipasang (hanya baro internal). ALT_HOLD butuh Bar30 | Pasang Bar30 untuk run di air; cek `BARO2_DEVID` bukan 0 sebelum menyelam |
| `FS_EKF_ACTION 1` | Berbeda dengan repo (`0`) | Ikuti `banyu_defaults.parm` |
| `FS_THR_ENABLE 0` (dump 2026-11-06) | Link RC putus saat MANUAL: stick terakhir tetap dipakai, thruster terus berputar (hanya failsafe pilot input `FS_PILOT_INPUT 1` yang menetralkan setelah 3 s, dan itu di-reset oleh RC override/`MANUAL_CONTROL` dari GCS mana pun) | **Belum**: set `FS_THR_ENABLE 1` (ada di `banyu_defaults.parm`) |

### 4.3 Run dengan TX16S

1. Terminal 1, bridge Pixhawk:
   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/run/run_mavproxy.sh            # opsional: <device> <IP laptop QGC>
   ```
2. Terminal 2, MAVROS (biarkan jalan; tidak perlu restart saat ganti profil):
   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh            # opsional: --fcu-url /dev/ttyACM0:921600
   ```
3. Terminal 3, stack POR (**`--profile` wajib** di robot). Launch file tidak pernah menjalankan
   MAVROS; launcher menolak jalan kalau `mavros_node` belum ada. Ganti profil cukup Ctrl+C di
   terminal ini lalu jalankan lagi.
   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --real --profile safe --dry-run   # cek dulu
   ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --real --profile safe
   ```
   Tambah `--no-cam` untuk profil timer supaya CPU lebih ringan.
3. SB **bawah** (kill) selama robot dipegang di darat.
4. Cek kesiapan (§4.5), lalu masukkan robot ke air menghadap gate. Arah start harus sejajar
   garis gate–marker, error maksimal ±4°.
5. SB **atas** (autonomous: LED AUTO ESP32, Pixhawk arm).
6. Start:
   ```bash
   ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
   ```

Switch TX16S (sumber: [`banyu_teleop/config/teleop_rcin.yaml`](../../banyu_teleop/config/teleop_rcin.yaml)):

| Switch | Posisi | Arti |
|---|---|---|
| SB | atas | Autonomous (`2`): LED AUTO ESP32, arm Pixhawk, misi boleh jalan, SC dan stick/gimbal dikunci |
| SB | tengah | Manual (`1`): misi berhenti, override dilepas, stick langsung berfungsi |
| SB | bawah | Kill (`0`): LED EMERGENCY ESP32, Pixhawk disarm |
| SC | atas / tengah / bawah | STABILIZE / MANUAL / ALT_HOLD, hanya saat SB tengah atau bawah |
| SF | – | tidak dipakai |

**Kill switch ESP32 (firmware `indikator_banyu.ino`).** Board ini tidak punya relay: ia hanya
menyalakan LED (MANUAL / AUTO / EMERGENCY) dan membaca input kill fisik (ADC). Thruster diputus oleh
kill fisik itu sendiri, bukan oleh software. Protokol serial (`/dev/esp32_kill`, 115200): PC
mengirim baris `its0` (manual), `its1` (auto) atau `its2` (emergency) diakhiri `\n`; board melapor
`itsN` tiap 500 ms dan mengabaikan `its0`/`its1` selama kill fisik aktif. Penomoran board berbeda
dari `operational_mode`: teleop memetakan `0 kill → its2`, `1 manual → its0`, `2 auto → its1`, dan
mengirim ulang tiap 1 s (state hilang kalau board reboot).

Kalau board melapor `its2` padahal SB tengah/atas, kill fisik sedang aktif: teleop menerbitkan
kill (`operational_mode 0`, misi berhenti), disarm Pixhawk, dan **menahan** kill itu sampai SB
digeser ke bawah, supaya robot tidak arm sendiri saat kill dilepas. Laporan board terekam di
`/banyu/hardware/kill_switch_board`. Kalau input kill belum tersambung (board selalu melapor
`its2`), robot tidak bisa arm; untuk uji darat saja, set
`use_hardware_kill: false` di `teleop_rcin.yaml`.

### 4.4 Run tanpa TX16S (`--no-rc`)

Berlaku untuk semua profil. `--no-rc` mematikan teleop, menjalankan `esp32_trigger_node` (kill
switch ESP32), dan meneruskan `bypass_rc:=True` ke `banyu_vehicle_status`, jadi kesegaran RC tidak
dicek. `bypass_rc` tidak ditulis di file profil.

Mode kendaraan mengikuti perintah trigger terakhir (`'2'` autonomous, `'1'` manual: misi
berhenti/tidak bisa start, `'0'` kill); sebelum ada perintah dianggap autonomous. Node menerjemahkan
`'0'/'1'/'2'` ke baris `its2/its0/its1`. Kill fisik yang dilaporkan board menahan kill sampai ada
perintah `'0'`. Selain topic
di bawah, bisa lewat service (juga mengganti warna LED ESP32):
`ros2 service call /banyu/trigger/command banyu_msgs/srv/TriggerCommand "{command: '2'}"` atau
`ros2 service call /banyu/trigger/set std_srvs/srv/SetBool "{data: true}"` (`true` = `'2'`,
`false` = `'0'`).

```bash
./src/BANYU_ROBOTX/tools/scripts/run/run_mavproxy.sh                     # terminal 1
./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh                    # terminal 2, biarkan jalan
./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --real --no-rc --profile safe   # terminal 3
ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '2'}"   # autonomous (LED AUTO)
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

Berhenti:
- `ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"`: misi berhenti, Pixhawk disarm.
- `ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '0'}"`: LED EMERGENCY,
  status kendaraan jadi kill (`operational_mode 0`), misi berhenti, Pixhawk disarm.

Dengan `bypass_rc`, kendaraan dianggap autonomous selama tidak ada perintah kill yang masih baru.
Tidak ada takeover dari stick. Siapkan selalu kill switch fisik.

### 4.5 Cek kesiapan

```bash
ros2 topic echo /banyu/vehicle_status --once
```

| Field | Harus |
|---|---|
| `fcu_connected`, `fcu_status_fresh` | `true` |
| `rc_fresh` | `true` |
| `operational_mode` | `2` (SB atas, atau `'2'` di mode no-RC) |
| `hardware_kill_active` | `false` |
| `operator_takeover` | `false` |

Selama belum ada sumber kill switch yang melapor (teleop, ESP32 bridge, atau ESP32 trigger),
`operational_mode` di robot bernilai `0`, dan misi tidak bisa start.

Di log launch harus ada `[nogps_odom_beacon] selecting EKF3 source set 2`, dan
`ros2 topic hz /mavros/local_position/pose` sekitar 10 Hz.

Untuk profil kamera, cek juga deteksi: `ros2 topic echo /banyu/detection --once`
(`object_id` 0 = gate, 1 = marker).

### 4.6 Berhenti darurat

| Cara | Efek |
|---|---|
| SB tengah | Misi berhenti, stick langsung pegang kendali (pilih mode dengan SC) |
| SB bawah | LED EMERGENCY, Pixhawk disarm |
| Kill switch fisik | Thruster diputus hardware; teleop disarm dan menahan kill sampai SB bawah |
| `/banyu/mission/abort` | Misi berhenti, fallback `DISARM`: thruster berhenti, robot naik karena daya apung |
| Ctrl+C stack | Override berhenti; ArduSub membuang override setelah ~3 s (`RC_OVERRIDE_TIME`) dan stick netral (failsafe pilot input) |

Semua profil POR memakai fallback `DISARM`, juga saat misi selesai.

### 4.7 Uji darat (mock auto)

Untuk memastikan semua thruster berputar tanpa air. **Jangan pernah dipakai untuk misi.**

```bash
./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --real --profile safe --mock-auto
```

- SB atas: arm, mode MANUAL, override 1540 µs di forward dan throttle (8 thruster berputar
  pelan), stick dikunci.
- SB tengah: mock berhenti, stick kembali. SB bawah: kill + disarm.
- Default `enable_mock_auto` sekarang `false` di `teleop_rcin.yaml`, jadi teleop yang dijalankan
  sendiri (`teleop.launch.py`) juga tidak memutar thruster.
- Tanpa ESP32: `python3 src/BANYU_ROBOTX/tools/scripts/debug/mock_esp32.py` membuat `/tmp/ttyESP32`;
  jalankan teleop dengan `-p serial_port:=/tmp/ttyESP32`.

### 4.8 Teach & replay (rekam manual, putar ulang otomatis)

Kemudikan rute sekali dengan tangan, lalu robot mengulangnya sendiri. Robot tidak tahu posisinya
(tanpa DVL/GPS), jadi yang diulang adalah **stick** (maju, geser, yaw) menurut waktu, sementara
**kedalaman** (Bar30) dan **heading** (IMU) mengikuti rekaman secara closed loop. Arus, baterai
dan posisi start yang berbeda tetap membuat hasilnya bergeser.

1. **Rekam (teach).** Jalankan stack seperti biasa (profil apa saja, rekaman bag aktif), arm,
   pilih **ALT_HOLD**, kemudikan rute dengan joystick QGC (atau TX16S) dari permukaan sampai
   selesai, lalu disarm. **Jangan** start misi. Stick diambil dari `/mavros/rc/in` (TX16S, atau
   joystick QGC kalau ada receiver RC di Pixhawk). **Tanpa receiver** Pixhawk tidak melaporkan
   kanal RC, jadi joystick QGC diambil dari log MAVProxy (`MANUAL_CONTROL`): `run_mavproxy.sh`
   menulis satu log per sesi ke `~/banyu_bags/mavproxy/` (`$BANYU_MAVPROXY_LOG_DIR`), dan
   extractor memilih sumbernya sendiri. Gain joystick (`#Gain is NN%`) ikut dihitung; stick penuh
   = 400 × gain µs, jadi naikkan gain (dan `JS_GAIN_MAX`) kalau rute butuh dorongan penuh. Selama tidak ada aksi misi yang jalan, `vehicle_execution` melepas
   stick (`rc_override.idle_release_s`), jadi tidak melawan pilot. Mulai dari posisi dan arah
   yang sama dengan run nanti. Pegang stick dengan stabil (jangan diketuk-ketuk) dan pisahkan
   belokan dari gerak maju/geser.
2. **Potong jadi track** (di komputer robot, setelah stack dihentikan):

   ```bash
   python3 src/BANYU_ROBOTX/tools/scripts/por/por_teach_extract.py ~/banyu_bags/<run> --list
   python3 src/BANYU_ROBOTX/tools/scripts/por/por_teach_extract.py ~/banyu_bags/<run> --name rute1
   ```

   Default: jendela arm terakhir, dari input stick pertama sampai terakhir + 1 s. `--window N`,
   `--start`/`--end` (detik dari awal bag) untuk memilih lain, `--tlog <file>` untuk memaksa log
   MAVProxy tertentu. Baris `sticks from` menunjukkan sumber yang dipakai. Hasil: `~/banyu_replays/rute1.csv`
   (`$BANYU_REPLAY_DIR`). Ringkasan yang dicetak (durasi, kedalaman maks, perubahan heading)
   harus cocok dengan yang dikemudikan.
3. **Putar ulang:**

   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --real --no-rc \
     --profile replay --replay-track rute1
   ```

   Tanpa `--replay-track` dipakai track terbaru. Banner menampilkan file, durasi dan
   kedalaman maks track; launcher menolak jalan kalau track tidak ada.
4. **Kalau satu kaki berhenti terlalu cepat** (mis. baterai turun, dorongan sama tapi jarak
   lebih pendek), panjangkan kaki itu di track tanpa rekam ulang:

   ```bash
   python3 src/BANYU_ROBOTX/tools/scripts/por/por_track_stretch.py rute1 --list
   python3 src/BANYU_ROBOTX/tools/scripts/por/por_track_stretch.py rute1 --hold 18:1.5 --name rute1_long
   ```

   `--hold T:S` menahan stick (dan heading/kedalaman rekaman) pada waktu track `T` selama `S`
   detik lebih lama, lalu menggeser semua sesudahnya. Pilih `T` di bagian stabil kaki itu (lihat
   `--list`); `--hold` boleh diulang untuk beberapa kaki. Track asli tidak diubah. Hindari `T`
   yang sedang yaw (ada peringatan): menahannya menambah putaran.

- **Matikan joystick di QGC sebelum start.** QGC mengirim stick ~24 Hz selama joystick aktif,
  dan ArduSub memakai override terakhir, jadi stick netral dari QGC melawan override misi (ini
  juga berlaku untuk semua profil timed selama tether masih tersambung).
- Kalau robot tertinggal > 0,4 m dari kedalaman rekaman (misalnya turun lebih lambat), jam
  track berhenti (stick netral) sampai selisihnya < 0,2 m, maksimal 30 s per run
  (`replay.depth_pause_m/depth_resume_m/max_pause_s`). Jadi kaki horizontal tidak dimulai
  terlalu dangkal di atas palang gate.
- Koreksi heading: `replay.heading_kp` (1,0), maksimal `replay.max_heading_correction_rad_s`
  (0,3 rad/s) di atas stick yaw rekaman. Profil replay menaikkan `control.max_yaw_rate` ke 1,5
  supaya putaran stick penuh (~1,3 rad/s) tidak terpotong.
- Kecepatan turun/naik dibatasi `max_depth_speed_mps` (0,4). Setelah track selesai, robot naik
  dengan kaki `timed_surface_*` lalu disarm.
- Rekam ulang track setelah mengubah `JS_GAIN`, `RCMAP_*` atau gain `rc_override` di profil:
  track menyimpan defleksi PWM, dan replay mengubahnya kembali lewat gain `rc_override` yang
  sedang dipakai.

**Di simulasi** (untuk mengukur seberapa mirip replay dengan teach): rekam kedua run dengan
`--record`. Bag sim memakai waktu sim dan berisi posisi sebenarnya dari Gazebo
(`/sim/ground_truth/odom`).

```bash
./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --sim --profile safe --record     # teach (QGC joystick)
python3 src/BANYU_ROBOTX/tools/scripts/por/por_teach_extract.py ~/banyu_bags/<teach run> --name sim1
./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --sim --profile replay --replay-track sim1 --record
python3 src/BANYU_ROBOTX/tools/scripts/por/por_path_compare.py ~/banyu_bags/<replay run>
```

`por_path_compare.py` menemukan track dan bag teach sendiri, menyejajarkan kedua run pada waktu
track (jeda kedalaman tidak dihitung sebagai drift), mencetak error posisi, kedalaman dan
heading, dan menyimpan `<replay run>/path_compare.png`. Kolam sim dalamnya ~2,5 m: robot yang
menyentuh dasar hampir tidak bisa bergerak maju. Hasil sim adalah kasus terbaik; di kolam
sungguhan arus, baterai dan posisi start menambah drift.

## 5. Referensi `launch_por.sh`

| Flag | Default | Fungsi |
|---|---|---|
| `--sim` / `--real` | `--real` | Target: Gazebo + SITL, atau robot (`sub_bringup.launch.py`) |
| `--profile <alias atau path>` | sim: `safe`; robot: **wajib** | Profil (`safe`, `fastest`, `replay`, atau path relatif ke `config/`) |
| `--headless`, `--gui` | GUI | Sim: tanpa/dengan GUI Gazebo |
| `--no-clean` | – | Sim: jangan bunuh proses SITL lama |
| `--rc` / `--no-rc` | `--rc` | Robot: teleop TX16S, atau `esp32_trigger_node` + `bypass_rc:=True` (§4.4) |
| `--no-cam` | – | Robot: tanpa driver kamera |
| `--mock-auto` | – | Robot: uji darat (§4.7) |
| `--replay-track <nama atau path>` | track terbaru | Profil replay: track di `~/banyu_replays` (§4.8) |
| `--record` / `--no-record` | robot: rekam; sim: tidak | Rekam run ke `~/banyu_bags` (sim: waktu sim + ground truth, §4.8) |
| `--record-images` / `--no-record-images` | robot: ya; sim: tidak | Gambar kamera terkompresi di bag |

MAVROS tidak dijalankan `launch_por.sh`. Pakai `launch_mavros.sh` (`--real` default, `--sim`
untuk waktu sim, `--fcu-url <url>`, `--dry-run`) di terminal sendiri. `launch_por.sh --real`
menolak jalan tanpa MAVROS, dan memperingatkan kalau ada lebih dari satu `mavros_node`.
| `--allow-channel-overlap` | – | Paksa jalan walau kanal override = kanal switch (berbahaya) |
| `--dry-run` | – | Cetak perintah dan cek device tanpa menjalankan |

Argumen lain diteruskan ke `ros2 launch`, misalnya `extra_sitl_params:=...` (sim) atau `use_gps:=True`.

Pengecekan yang dilakukan launcher di robot: `--profile` wajib, dan kanal override tidak boleh
sama dengan kanal SB/SC.

## 6. Troubleshooting

| Gejala | Penyebab / perbaikan |
|---|---|
| `Mode change failed: Guided requires position` | Profil lama dengan GUIDED. Semua profil POR harus `autonomy_mode: "ALT_HOLD"` |
| Misi tidak mau start, `rc_fresh: false` | TX16S mati atau `/mavros/rc/in` kosong. Tanpa TX, pakai `--no-rc` |
| Misi tidak mau start, `operational_mode: 0` | SB belum di atas / teleop belum menerima RC / belum kirim `'2'` (no-RC) |
| Misi langsung berhenti setelah start dan mode bolak-balik | Bentrok kanal SB dengan override (§4.2) |
| Arm ditolak | Lihat pesan pre-arm di QGC. Jangan matikan `ARMING_CHECK` sembarangan |
| Robot arm di ALT_HOLD tapi tidak bergerak sama sekali | `MAV_GCS_SYSID` Pixhawk tidak mencakup 255 (sysid MAVROS), jadi RC override ditolak (§4.2) |
| `/mavros/local_position/pose` tidak ada | `nogps_odom_beacon` tidak jalan atau `VISO_TYPE` belum 1. ALT_HOLD tetap bisa terbang (heading dari IMU) |
| Sim: "Vehicle safety or run authorization fault" | Traffic robot masuk ke sim; ganti `ROS_DOMAIN_ID` |
| Sim: `ArduPilot controller has reset`, port 5501 | Proses `ardusub` lama masih hidup; jalankan tanpa `--no-clean` |
| `/mavros/state` tidak ada / `launch_por.sh`: "MAVROS is not running" | Jalankan `launch_mavros.sh` (sim: `--sim`) di terminal lain |

## 7. Arsitektur singkat

```
TX16S → receiver → Pixhawk (ArduSub) ←USB→ run_mavproxy.sh ─┬→ QGC laptop (14550)
                                                            ├→ MAVROS (14551)
                                                            └→ nogps_odom_beacon (14552)
banyu_teleop: baca SB/SC dari /mavros/rc/in + laporan ESP32 → LED ESP32 + /banyu/hardware/kill_switch
banyu_vehicle_status: FCU + RC + kill switch → /banyu/vehicle_status
banyu_mission (BehaviorTree) → action → vehicle_execution_node → /mavros/rc/override
por_detection: kamera depan → /banyu/detection (dipakai profil vision/hybrid)
```

Tree yang dipakai ditentukan oleh flag di `banyu_mission.mission`: default
`bt/por_timed_mission.xml`; `use_vision_assist: true` → `por_vision_assisted_mission.xml`;
`use_vision_return: true` → `por_hybrid_mission.xml`; `use_vision: true` →
`por_mission.xml` (full vision, belum dituning untuk ALT_HOLD).

## 8. Masalah terbuka (butuh keputusan atau data dari robot)

1. **Parameter Pixhawk** (§4.2): set `MAV_GCS_SYSID 255` dan muat `banyu_defaults.parm`; pasang
   Bar30 sebelum run di air.
2. **`degraded_thrusters_2_3.parm`** memakai `SERVOx_FUNCTION 0`, yang di SITL diabaikan
   ArduSub (motor tetap jalan). `135` (k_trim) terbukti mematikan motor di sim. Perlu dicek di
   bench sebelum dipakai.
3. **Profil dead-thruster robot** perlu kecepatan diagonal hasil ukur
   (`dead_thruster_por.py --measured-speed`, lihat [`tuning.md`](tuning.md) §6).
4. **Bar30 mati** (`dead_thruster_por.py --no-depth-sensor`): trim kedalaman harus diukur di robot; di sim
   hanya berhasil dengan dithering per segmen ([`sim_results.md`](sim_results.md)).
5. **Task 2 di robot** (bukan POR): GUIDED kehilangan posisi saat menyelam tanpa DVL; analisis dan
   opsi di [`../task2_underwater_position.md`](../task2_underwater_position.md).
6. **Batas thruster 50%** (§4.1): profil timed dan gain `rc_override` dituning sebelum batas
   ini (thrust penuh). Dengan batas, kecepatan per stick turun, jadi kaki timed menempuh jarak
   lebih pendek; hasil di [`sim_results.md`](sim_results.md) perlu diulang dan durasi kaki
   dituning ulang.
7. **Toleransi ALT_HOLD** (error kecepatan dan heading) belum diukur ulang; hasil sim saat ini
   satu run nominal per profil.
