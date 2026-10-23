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
- Semua dijalankan lewat [`tools/scripts/launch_por.sh`](../../tools/scripts/launch_por.sh).

## 2. Memilih profil

### Cara cepat

```
Robot sehat, kamera & warna sudah dituning  →  vision
Robot sehat, kamera ragu                    →  safe   (lebih cepat: fast, lalu fastest)
Uji pertama di kolam baru / cek bocor       →  slow
Thruster 2 + 3 mati                         →  buat profil dengan dead_thruster_por.py (tuning.md §6)
Sensor & kamera tidak bisa dipercaya        →  blind/<profil>.yaml
```

### Katalog

Waktu = waktu sim sampai lewat gate kembali (ALT_HOLD, tanpa GPS). Di kolam angkanya akan
berbeda sampai kecepatan dikalibrasi ([`tuning.md`](tuning.md)).

| `--profile` | File | Rute | Kamera | Waktu sim |
|---|---|---|---|---|
| `default` | `por_params.yaml` | strafe box aman (sama dengan `safe`) | – | ~37 s |
| `safe` | `por_params_timed_safe.yaml` | strafe box aman | – | ~37 s |
| `fast` | `por_params_timed_fast.yaml` | strafe box cepat | – | ~32 s |
| `fastest` | `por_params_timed_fastest.yaml` | diamond, mundur lewat gate | – | ~25 s |
| `turn` | `por_params_timed_fastest_turn.yaml` | diamond, putar 180°, maju lewat gate | – | ~28 s |
| `vision` | `por_params_vision_assisted.yaml` | diamond + koreksi kamera | gate & marker, fallback timer | ~36 s |
| `hybrid` | `por_params_hybrid.yaml` | strafe box, pulang pakai kamera | gate, fallback timer | ~44 s |
| `slow` | `por_params_timed_slow.yaml` | strafe box pelan | – | ~71 s |

- **Strafe box**: yaw tetap. Robot geser kanan, maju melewati marker, geser kiri, mundur, geser
  kanan, lalu mundur lewat gate. Tidak ada belokan, jadi tidak tergantung kompas. Paling toleran.
- **Diamond**: empat sisi diagonal (maju+geser sekaligus) di sekitar marker. Paling cepat, tapi
  paling sensitif terhadap error kecepatan.
- **Kamera** di `vision` dan `hybrid` selalu dibatasi waktu. Kalau kamera gagal, robot lanjut
  dengan timer (sudah dites di sim dengan kamera mati).

### Folder tambahan

| Folder | Isi | Boleh di robot? |
|---|---|---|
| `config/blind/` | Profil yang sama dengan tabel di atas, tapi **murni timer**: `use_sensors: false`, heading hold mati, dive dan putaran open-loop. Untuk kondisi baro/IMU/kamera tidak bisa dipercaya. | Ya (`--profile blind/por_params_timed_safe.yaml`) |
| `config/sim_dead_thruster/` | Profil dead-thruster yang dikalibrasi di **sim** (kecepatan sim, trim sim). | **Tidak**, ditolak `launch_por.sh --real` |
| `config/sim_faults/` | File parameter ArduPilot untuk mensimulasikan kerusakan di SITL. | **Tidak** |

Tidak ada profil dead-thruster untuk robot di repo. Kalau thruster 2 + 3 mati, buat profilnya
dari hasil ukur di kolam dengan `tools/scripts/dead_thruster_por.py` ([`tuning.md`](tuning.md)
§6). Hasilnya ditulis ke `config/` dan dipakai dengan `--profile <nama file>`. Versi sim ada di
`config/sim_dead_thruster/` sebagai contoh.

### File lain di `config/` (bukan profil)

| File | Fungsi |
|---|---|
| `task2_params.yaml` | Profil Task 2 (bukan POR) |
| `hardware_params.yaml` | Device kamera dan port ESP32 di robot |
| `mavros_config.yaml` | Setting MAVROS |
| `banyu_defaults.parm` | Parameter ArduSub: dimuat otomatis ke SITL, di robot dimuat sekali lewat QGC (§4.1) |
| `degraded_thrusters_2_3.parm` | Parameter Pixhawk untuk mematikan thruster 2+3 (**belum diverifikasi**, lihat §8) |
| `sim_no_gps.parm` | Sim: menghapus GPS SITL (dipakai otomatis oleh `use_gps:=False`) |
| `*_camera_info.yaml` | Kalibrasi kamera |

## 3. Menjalankan di simulasi

```bash
env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --sim --profile safe
env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --sim --headless --profile fastest
```

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
| `use_gps:=False` | GPS SITL dihapus sejak boot, seperti robot asli (default POR) |
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

1. **Parameter Pixhawk** (QGroundControl): muat `banyu_bringup/config/banyu_defaults.parm`,
   lalu reboot. Yang penting untuk robot:
   - `RCMAP_FORWARD 2`, `RCMAP_LATERAL 1`, `RCMAP_THROTTLE 3`, `RCMAP_YAW 4`,
     `RCMAP_PITCH 0`, `RCMAP_ROLL 0` (§4.2).
   - `VISO_TYPE 1`, `EK3_SRC2_POSXY 0`, `EK3_SRC2_VELXY 6`, `EK3_SRC2_POSZ 1`,
     `EK3_SRC2_VELZ 0`, `EK3_SRC2_YAW 1`: dipakai `nogps_odom_beacon` supaya EKF
     mempublikasikan kedalaman dan heading tanpa GPS. Tanpa `EK3_SRC2_YAW 1`, EKF kehilangan
     sumber heading begitu `ekf_source_switch` pindah ke source set 2.
   - `FS_EKF_ACTION 0`, `FS_GCS_ENABLE 0`.
   - `MAV_GCS_SYSID 255` (sysid MAVROS), lihat §4.2.
   - Cek juga `FS_EKF_THRESH` dan `ARMING_CHECK` (tidak diatur file ini).
2. **Device** di mini-PC:
   ```bash
   ls -l /dev/pixhawk /dev/ttyACM* /dev/ttyUSB*
   v4l2-ctl --list-devices
   ```
   ESP32 diasumsikan `/dev/ttyUSB0` (`teleop_rcin.yaml`, `hardware_params.yaml`), kamera depan
   `/dev/video0`. Nama ini bisa tertukar setelah cabut-pasang; sebaiknya pakai udev rule
   ([`../udev_rules.md`](../udev_rules.md)).

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
| `VISO_TYPE 0`, `EK3_SRC2_VELXY 0`, `EK3_SRC2_YAW 0` | `banyu_defaults.parm` belum dimuat. Tanpa GPS, `ekf_source_switch` memakai source set 2, yang di Pixhawk ini tidak punya sumber yaw | **Belum**: muat `banyu_defaults.parm` (§4.1) sebelum run otonom |
| `RC8_OPTION 213` (mount pitch), `RC7_OPTION 214` (mount yaw), `MNT1_RC_RATE 30` | SC (CH8) juga menggerakkan tilt mount kamera dengan laju 30°/s selama tidak di tengah: SC atas memiringkan kamera sampai -90°, SC bawah sampai +20°. Arah kamera depan saat misi jadi tergantung riwayat posisi SC | **Aman**: kamera depan fixed, tidak ada servo tilt. Kalau suatu saat dipasang mount, set `RC8_OPTION 0` |
| `BARO2_DEVID 0` | Saat dump diambil, Bar30 belum dipasang (hanya baro internal). ALT_HOLD butuh Bar30 | Pasang Bar30 untuk run di air; cek `BARO2_DEVID` bukan 0 sebelum menyelam |
| `FS_EKF_ACTION 1` | Berbeda dengan repo (`0`) | Ikuti `banyu_defaults.parm` |

### 4.3 Run dengan TX16S

1. Terminal 1, bridge Pixhawk:
   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh            # opsional: <device> <IP laptop QGC>
   ```
2. Terminal 2, stack POR (**`--profile` wajib** di robot):
   ```bash
   ./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --real --profile safe --dry-run   # cek dulu
   ./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --real --profile safe
   ```
   Tambah `--no-cam` untuk profil timer supaya CPU lebih ringan.
3. SB **bawah** (kill) selama robot dipegang di darat.
4. Cek kesiapan (§4.5), lalu masukkan robot ke air menghadap gate. Arah start harus sejajar
   garis gate–marker, error maksimal ±4°.
5. SB **atas** (autonomous: relay ESP32 nyala, Pixhawk arm).
6. Start:
   ```bash
   ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
   ```

Switch TX16S (sumber: [`banyu_teleop/config/teleop_rcin.yaml`](../../banyu_teleop/config/teleop_rcin.yaml)):

| Switch | Posisi | Arti |
|---|---|---|
| SB | atas | Autonomous (`2`): relay ESP32 nyala, arm Pixhawk, misi boleh jalan, SC dikunci |
| SB | tengah | Manual (`1`): misi berhenti, override dilepas, stick langsung berfungsi |
| SB | bawah | Kill (`0`): relay ESP32 putus, Pixhawk disarm |
| SC | atas / tengah / bawah | STABILIZE / MANUAL / ALT_HOLD, hanya saat SB tengah atau bawah |
| SF | – | tidak dipakai |

### 4.4 Run tanpa TX16S (`--no-rc`)

Hanya untuk profil dengan `bypass_rc: true` (launcher menolak profil lain). Saat ini tidak ada
profil seperti itu di repo; buat dengan `dead_thruster_por.py --no-rc` atau tambahkan
`bypass_rc: true` di bagian `banyu_vehicle_status` sebuah salinan profil. Teleop tidak
dijalankan; relay ESP32 dikendalikan `esp32_trigger_node`.

```bash
./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh
./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --real --no-rc --profile por_params_dead_thruster_2_3_no_rc_no_pos.yaml
ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '2'}"   # relay nyala (autonomous)
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

Berhenti:
- `ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"`: misi berhenti, Pixhawk disarm.
- `ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '0'}"`: relay putus,
  status kendaraan jadi kill (`operational_mode 0`), misi berhenti, Pixhawk disarm.

Dengan `bypass_rc`, kendaraan dianggap autonomous selama tidak ada perintah kill yang masih baru.
Tidak ada takeover dari stick. Siapkan selalu kill relay fisik.

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

Di log launch harus ada `EKF source set 2 accepted` (dari `ekf_source_switch`), dan
`ros2 topic hz /mavros/local_position/pose` sekitar 10 Hz.

Untuk profil kamera, cek juga deteksi: `ros2 topic echo /banyu/detection --once`
(`object_id` 0 = gate, 1 = marker).

### 4.6 Berhenti darurat

| Cara | Efek |
|---|---|
| SB tengah | Misi berhenti, stick langsung pegang kendali (pilih mode dengan SC) |
| SB bawah | Relay ESP32 putus, Pixhawk disarm |
| `/banyu/mission/abort` | Misi berhenti, fallback `DISARM`: thruster berhenti, robot naik karena daya apung |
| Ctrl+C stack | Override berhenti; ArduSub membuang override setelah ~3 s (`RC_OVERRIDE_TIME`) dan stick netral (failsafe pilot input) |

Semua profil POR memakai fallback `DISARM`, juga saat misi selesai.

### 4.7 Uji darat (mock auto)

Untuk memastikan semua thruster berputar tanpa air. **Jangan pernah dipakai untuk misi.**

```bash
./src/BANYU_ROBOTX/tools/scripts/launch_por.sh --real --profile safe --mock-auto
```

- SB atas: arm, mode MANUAL, override 1540 µs di forward dan throttle (8 thruster berputar
  pelan), stick dikunci.
- SB tengah: mock berhenti, stick kembali. SB bawah: kill + disarm.
- Default `enable_mock_auto` sekarang `false` di `teleop_rcin.yaml`, jadi teleop yang dijalankan
  sendiri (`teleop.launch.py`) juga tidak memutar thruster.
- Tanpa ESP32: `python3 src/BANYU_ROBOTX/tools/scripts/mock_esp32.py` membuat `/tmp/ttyESP32`;
  jalankan teleop dengan `-p serial_port:=/tmp/ttyESP32`.

## 5. Referensi `launch_por.sh`

| Flag | Default | Fungsi |
|---|---|---|
| `--sim` / `--real` | `--real` | Target: Gazebo + SITL, atau robot (`sub_bringup.launch.py`) |
| `--profile <alias atau path>` | sim: `default`; robot: **wajib** | Profil (alias di §2, atau path relatif ke `config/`) |
| `--headless`, `--gui` | GUI | Sim: tanpa/dengan GUI Gazebo |
| `--no-clean` | – | Sim: jangan bunuh proses SITL lama |
| `--rc` / `--no-rc` | `--rc` | Robot: teleop TX16S, atau `esp32_trigger_node` + profil `bypass_rc` |
| `--no-cam` | – | Robot: tanpa driver kamera |
| `--mock-auto` | – | Robot: uji darat (§4.7) |
| `--allow-channel-overlap` | – | Paksa jalan walau kanal override = kanal switch (berbahaya) |
| `--dry-run` | – | Cetak perintah dan cek device tanpa menjalankan |

Argumen lain diteruskan ke `ros2 launch`, misalnya `use_gps:=False`.

Pengecekan yang dilakukan launcher di robot: `--profile` wajib, profil `sim_*` ditolak,
`--no-rc` butuh `bypass_rc: true`, dan kanal override tidak boleh sama dengan kanal SB/SC.

## 6. Troubleshooting

| Gejala | Penyebab / perbaikan |
|---|---|
| `Mode change failed: Guided requires position` | Profil lama dengan GUIDED. Semua profil POR harus `autonomy_mode: "ALT_HOLD"` |
| Misi tidak mau start, `rc_fresh: false` | TX16S mati atau `/mavros/rc/in` kosong. Tanpa TX, pakai `--no-rc` dengan profil `bypass_rc` |
| Misi tidak mau start, `operational_mode: 0` | SB belum di atas / teleop belum menerima RC / belum kirim `'2'` (no-RC) |
| Misi langsung berhenti setelah start dan mode bolak-balik | Bentrok kanal SB dengan override (§4.2) |
| Arm ditolak | Lihat pesan pre-arm di QGC. Jangan matikan `ARMING_CHECK` sembarangan |
| Robot arm di ALT_HOLD tapi tidak bergerak sama sekali | `MAV_GCS_SYSID` Pixhawk tidak mencakup 255 (sysid MAVROS), jadi RC override ditolak (§4.2) |
| `/mavros/local_position/pose` tidak ada | `nogps_odom_beacon` tidak jalan atau `VISO_TYPE` belum 1. ALT_HOLD tetap bisa terbang (heading dari IMU) |
| Sim: "Vehicle safety or run authorization fault" | Traffic robot masuk ke sim; ganti `ROS_DOMAIN_ID` |
| Sim: `ArduPilot controller has reset`, port 5501 | Proses `ardusub` lama masih hidup; jalankan tanpa `--no-clean` |

## 7. Arsitektur singkat

```
TX16S → receiver → Pixhawk (ArduSub) ←USB→ run_mavproxy.sh ─┬→ QGC laptop (14550)
                                                            ├→ MAVROS (14551)
                                                            └→ nogps_odom_beacon (14552)
banyu_teleop: baca SB/SC dari /mavros/rc/in → relay ESP32 + /banyu/hardware/kill_switch
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
5. **Toleransi ALT_HOLD** (error kecepatan dan heading) belum diukur ulang; hasil sim saat ini
   satu run nominal per profil.
