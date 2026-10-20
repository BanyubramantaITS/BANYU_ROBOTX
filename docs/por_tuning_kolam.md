# Panduan Tuning POR Timer di Kolam (Real World)

Panduan ini untuk strategi POR tanpa kamera (timer + kecepatan per segmen).
Hasil simulasi dan model gerak ada di [`por_timed_tuning.md`](por_timed_tuning.md).

| Profil | File | Waktu sim (lewat gate kembali) | Toleransi sim |
|---|---|---|---|
| Strafe box (**paling aman**) | `por_params_timed_fast.yaml` | 55 s | kecepatan −8%..+5%, heading ±6° |
| Diamond mundur (**tercepat**) | `por_params_timed_fastest.yaml` | 33,3 s | kecepatan ±6%, heading +6° (gagal −6°) |
| Diamond putar balik | `por_params_timed_fastest_turn.yaml` | 39,1 s | putaran harus akurat ±3° |
| **Diamond + kamera** | `por_params_vision_assisted.yaml` | lihat bagian 8 | kamera mengoreksi heading, jarak, dan arah ke gate |

Tuning dikerjakan berurutan: kalibrasi kecepatan dulu, baru segmen satu per
satu. Mulai dari strafe box, setelah itu diamond.

## 0. Persiapan

- **Alat:** stopwatch, meteran 20 m, penanda di lantai kolam (tali atau garis
  ke arah marker), satu orang yang merekam dari pinggir kolam.
- **Laptop:** pakai `export ROS_DOMAIN_ID=<sama dengan robot>` supaya bisa
  melihat topic robot.
- **Abort selalu siap:**
  `ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"`,
  atau switch SB di TX16S.
- **Baterai:** tuning di level baterai yang sama dengan saat lomba, dan catat
  voltasenya.
- **Sensor:** set `use_sensors: true` supaya depth hold baro aktif.

Launch di robot:

```bash
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por \
  por_params_file:=$PWD/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_fast.yaml
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

## 1. Kalibrasi kecepatan (paling penting)

Buat YAML tes yang hanya berisi **1 segmen**, semua segmen lain durasinya 0.
Contohnya maju saja:

```yaml
dive_duration_s: 5.0                 # turun dulu ke kedalaman
dive_speed_x_mps: 0.0
timed_pass_gate_duration_s: 10.0
timed_pass_gate_speed_x_mps: 1.1     # kecepatan yang mau dikalibrasi
# segmen lain: *_duration_s: 0.0
```

Ukur jarak tempuhnya, lalu hitung kecepatan aslinya:

**`v_real = jarak_terukur / (durasi − dead_time)`**

| Gerakan | Dipakai di | Dead time awal |
|---|---|---|
| Maju 1,1 m/s | diamond outbound & return (versi putar balik) | 0,55 s |
| Mundur 1,1 m/s | diamond return (versi mundur) | 0,85 s |
| Diagonal (vx 1,0 dan vy ±1,2) | 4 sisi diamond | ukur jarak X dan Y terpisah |
| Maju 0,9, strafe 0,7, mundur 0,9 | strafe box | 0,55 / 0,7 / 0,85 s |

Ulangi tiap gerakan **3 kali** dan pakai rata-ratanya. Targetnya, hasil tiap
pengulangan konsisten sampai **±5%**. Lebih dari itu, loop bisa menabrak
dinding belakang: marker hanya 3,5 m dari dinding.

## 2. Hitung durasi tiap segmen

**`durasi = jarak_rencana / v_real + dead_time`**

Lintasan: start → gate 3 m → marker 10 m.

**Diamond** (`por_params_timed_fastest.yaml`):

| Segmen | Jarak rencana |
|---|---|
| `dive` + `approach_pole` | 9,7 m (sampai 3,3 m sebelum marker) |
| `circle_turn_1` (+x, −y) | 2,6 m maju, 2,7 m ke kanan |
| `circle_forward_1` (+x, +y) | 2,6 m maju, 2,7 m ke kiri (lewat samping marker) |
| `circle_turn_2` (−x, +y) | 2,6 m mundur, 2,7 m ke kiri |
| `circle_forward_2` (−x, −y) | sampai kembali ke garis tengah (durasinya sedikit lebih lama) |
| `return_to_gate` | sekitar 9,7 m, sampai 2 m lewat gate |

**Strafe box** (`por_params_timed_fast.yaml`):

| Segmen | Gerakan |
|---|---|
| `circle_turn_1` | strafe kanan 2 m |
| `circle_forward_1` | maju 3,5 m |
| `circle_turn_2` | strafe kiri 4 m |
| `circle_forward_2` | mundur 3,5 m |
| `circle_turn_3` | strafe kanan 2 m |
| `return_to_gate` | mundur sampai 2 m lewat gate |

## 3. Tuning segmen satu per satu (urutan wajib)

Setiap segmen dites dengan segmen berikutnya diset durasi 0. Lanjut ke segmen
berikutnya hanya kalau segmen ini sudah pas 2 kali berturut-turut.

1. **Dive.** Robot harus sudah di bawah palang gate (kedalaman lebih dari
   1,2 m) sebelum sampai gate. Kalau belum, tambah `dive_duration_s` atau
   kurangi `dive_speed_x_mps`.
2. **Approach.** Robot harus berhenti 3,3 m sebelum marker (diamond) atau 2 m
   (strafe box).
3. **Loop.** Robot harus kembali ke garis tengah, di depan marker.
4. **Return.** Robot harus lewat gate di tengah, lalu muncul ke permukaan.

## 4. Khusus versi putar balik (`por_params_timed_fastest_turn.yaml`)

Putarannya harus akurat **±3°**. Di sim, putaran yang kelebihan 8° membuat
robot meleset 2 m dari gate.

1. Tes `return_turn` saja di tempat: robot diam, putar sekali, lalu ukur
   heading akhirnya pakai kompas atau QGC.
2. Atur target putaran sesuai hasilnya:

   | Hasil | Perbaikan |
   |---|---|
   | Putaran kurang dari 180° / robot lewat gate ke **kiri** | tambah `return_turn_target_angle_rad` (+0,03 rad ≈ +2°) |
   | Putaran lebih dari 180° / robot lewat gate ke **kanan** | kurangi `return_turn_target_angle_rad` |

3. `return_turn_duration_s` hanya batas waktu maksimum. Biarkan sedikit lebih
   lama dari waktu putaran sebenarnya.
4. Pure timer (`use_sensors: false`) tidak memakai cap IMU, jadi putaran hanya
   diatur durasi. Set `return_turn_duration_s` sampai putarannya pas 180°.
   Ini jauh lebih susah, jadi tetap pakai IMU.

## 5. Arah start (sumber error kedua)

- Arahkan robot ke garis gate–marker dengan error **maksimal ±4°**.
- Caranya, sejajarkan badan robot dengan tali atau garis di lantai kolam yang
  menunjuk ke marker.
- Error 4° di jarak 13 m sama dengan sekitar 0,9 m ke samping.

## 6. Troubleshooting

| Gejala | Perbaikan |
|---|---|
| Menabrak marker atau loop terlalu dekat | kurangi `approach_pole_duration_s` |
| Loop tidak mengitari marker (terlalu pendek) | tambah `approach_pole_duration_s` |
| Menabrak dinding belakang | kurangi `approach_pole_duration_s`, atau kecepatan aslinya lebih tinggi dari kalibrasi (kalibrasi ulang) |
| Loop tidak menutup (kembali ke kiri atau kanan garis tengah) | sesuaikan durasi `circle_forward_2` (diamond) atau `circle_turn_3` (strafe box) |
| Lewat gate melenceng ke samping | cek arah start dulu, lalu keseimbangan strafe kiri dan kanan |
| Belum lewat gate saat surface | tambah `return_to_gate_duration_s` |
| Menabrak dinding start | kurangi `return_to_gate_duration_s` |
| Menyentuh palang gate | tambah `gate_depth_m` (maksimal 2,0) |
| Hasil berubah-ubah antar run | baterai drop, jadi kalibrasi ulang di voltase lomba |

## 7. Urutan profil yang disarankan

1. **Strafe box** (`por_params_timed_fast.yaml`), sampai 3 kali sukses
   berturut-turut.
2. **Diamond mundur** (`por_params_timed_fastest.yaml`). Kecepatannya beda,
   jadi kalibrasi lagi di 1,1 m/s.
3. **Diamond putar balik** (`por_params_timed_fastest_turn.yaml`), hanya kalau
   putarannya konsisten ±3°.

## 8. Strategi Diamond + Kamera (`por_params_vision_assisted.yaml`)

Gate **kuning**, marker **oranye**. Kerangkanya tetap timer diamond D3,
ditambah koreksi kamera. Setiap langkah kamera punya batas waktu, dan kalau
kamera gagal robot lanjut dengan timer.

| # | Langkah | Kamera | Fallback |
|---|---|---|---|
| 0 | `AlignGate` dari posisi start (geser + atur kedalaman ke tengah gate) | gate | lewati setelah `assist_gate_align_max_s` |
| 1–3 | dive + approach (timer) | – | – |
| 3b | `AlignMarker` **hanya geser ke samping** sampai marker di tengah (memperbaiki error heading start) | marker | lewati setelah `assist_marker_align_max_s` |
| 4 | diamond (timer) | – | – |
| 5 | `return_turn` 180° (IMU) | – | – |
| 6 | `SearchGate` (maks 1,5 s), lalu `TurnAround` mengunci heading ke gate, lalu `PassGate` **maju** sambil steering ke gate | gate | gate tidak terlihat: `return_to_gate` timer maju dengan heading IMU |
| 7 | surface | – | – |

Jarak tetap diatur timer, karena estimasi jarak pole dari kamera tidak bisa
dipercaya. Di sim, jarak asli 3 m terbaca 11–14 m karena sisi pole yang
gelap tidak lolos HSV. Kalibrasi kecepatan ±6% tetap wajib.

Hasil sim (gate kuning, marker oranye):

| Tes | Hasil | Lewat gate kembali |
|---|---|---|
| Nominal | ✅ | 42,3 s |
| Heading −6° (timer-only gagal) | ✅ | 45,0 s |
| Heading +6° | ✅ | 45,7 s |
| Speed +6% / −6% | ✅ / ✅ | 41,7 / 42,1 s |
| Tanpa kamera (detector dibutakan, semua fallback timer) | ✅ | 45,0 s |

```bash
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por \
  por_params_file:=$PWD/src/BANYU_ROBOTX/banyu_bringup/config/por_params_vision_assisted.yaml
```

Set `assist_*_max_s: 0.0` untuk mematikan satu koreksi kamera.

Aksi kamera bergerak memakai heading dari `/mavros/local_position/pose`.
Kalau pose tidak ada (tanpa DVL), heading diambil dari `/mavros/imu/data`.

## 9. Tuning warna gate & marker pakai slider

Detector memakai HSV terpisah per objek (`color_mode: "per_object"`):

| Objek | Parameter | Default |
|---|---|---|
| Gate (kuning) | `gate_h_min/max`, `gate_s_min/max`, `gate_v_min/max` | H 20–35, S 80–255, V 80–255 |
| Marker (oranye) | `marker_h_min/max`, `marker_s_min/max`, `marker_v_min/max` | H 5–18, S 120–255, V 100–255 |
| Filter | `min_area`, `morph_kernel`, `buoy_aspect_max` | 300, 3, 65 |

Hue OpenCV bernilai 0–179. Kalau `h_min > h_max`, range-nya melingkar lewat 0
(dipakai untuk merah, misalnya `h_min 170`, `h_max 10`).

### Cara A: tool slider + preview mask (disarankan)

Robot menjalankan stack seperti biasa. Di laptop (dengan `ROS_DOMAIN_ID` sama
dengan robot):

```bash
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py          # lewat WiFi (compressed)
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py --raw    # sim / kabel LAN
```

- Ada 3 jendela slider: **Gate HSV (yellow)**, **Marker HSV (orange)**, dan **Filter**.
- Slider langsung mengubah parameter detector di robot, tanpa restart.
- Jendela **POR HSV tuner** menampilkan gambar annotated, mask gate, dan mask
  marker dari detector di robot.
- Tombol **`s`** menyimpan nilai saat ini (lewat `/por_detection/save_params`)
  ke `~/por_detection_tuned.yaml` di robot, dan juga mencetak snippet YAML-nya
  di terminal.
- Tombol **`r`** membaca ulang nilai dari robot. Tombol **`q`** keluar.

Target tuning:
- Mask gate: putih **hanya** di tiang dan palang gate.
- Mask marker: putih **hanya** di pole marker.
- Lantai, garis lane, dan pantulan cahaya harus hitam.

Urutan geser slider:
1. Persempit `h_min`/`h_max` sampai warna lain hilang.
2. Naikkan `s_min` untuk membuang air atau lantai yang pucat.
3. Atur `v_min` untuk bayangan.
4. Naikkan `min_area` untuk membuang bintik kecil.

### Cara B: `rqt_reconfigure`

```bash
ros2 run rqt_reconfigure rqt_reconfigure     # pilih /por_detection
ros2 run rqt_image_view rqt_image_view        # /banyu/detection/debug/annotated/compressed
```

Semua parameter HSV muncul sebagai slider.

### Cara C: offline dari rekaman

Rekam dulu video kamera di kolam (atau `ros2 bag record /front_camera/image_raw`
lalu export frame-nya), kemudian:

```bash
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py --source rekaman_kolam.mp4
```

Spasi untuk pause dan `s` untuk menyimpan ke `~/por_detection_tuned.yaml`.

### Cara D: jendela OpenCV di robot (butuh monitor atau X forwarding)

`ros2 launch banyu_bringup sub_bringup.launch.py mode:=por debug:=True`

### Kalibrasi jarak marker dan gate

`AlignMarker` menjaga jarak 3,0–3,6 m dari marker, berdasarkan estimasi jarak
dari ukuran objek di gambar. Estimasi ini harus cocok dengan jarak asli.

1. Taruh robot 3,0 m (diukur meteran) di depan marker.
2. Jalankan `ros2 topic echo /banyu/detection/marker --once` dan lihat
   `distance`.
3. Koreksi `buoy_reference_width_m` (bagian `por_detection` di YAML profil):
   **`baru = lama × 3,0 / distance_terbaca`**.
4. Lakukan hal yang sama untuk gate dengan `gate_reference_width_m` dan
   `gate_reference_height_m`.

Di sim, marker terbaca 1,6 m saat jarak aslinya sekitar 2,7 m, sehingga
`buoy_reference_width_m` diset 0,5.

### Simpan permanen

Salin nilai dari `~/por_detection_tuned.yaml` ke
`banyu_detection/config/por_detection_params.yaml`, lalu rebuild:
`colcon build --packages-select banyu_detection`.

Cek deteksi:
- `ros2 topic echo /banyu/detection/gate --once`
- `ros2 topic echo /banyu/detection/marker --once`

## Catatan

- `por_params_hybrid.yaml` (`use_vision_return: true`): strafe box timer, lalu
  pulang pakai kamera.
- `use_vision: true` menjalankan full vision (`bt/por_mission.xml`).
- Kalau sim dijalankan di laptop yang satu jaringan dengan robot, pakai
  `ROS_DOMAIN_ID` yang berbeda. Kalau tidak, traffic robot memicu
  "Vehicle safety or run authorization fault" di sim.

## 10. Thruster horizontal mati (2 depan-kiri + 3 belakang-kanan)

Profil: `banyu_bringup/config/por_params_dead_thruster_2_3.yaml` (timer + IMU,
tanpa kamera).

Frame robot adalah `FRAME_CONFIG = 2` (Vectored-6DOF):
- Motor 1–4 horizontal: 1 depan-kanan, 2 depan-kiri, 3 belakang-kanan, 4 belakang-kiri.
- Motor 5–8 vertikal.

Kalau motor 2 + 3 mati, pasangan 1 + 4 hanya bisa mendorong di **satu diagonal
badan** (perintah `vx = vy`, ke arah depan-kiri) dan masih bisa yaw. Karena itu
setiap gerakan dikirim di sepanjang diagonal tersebut:

| # | Segmen | Gerakan |
|---|---|---|
| 1 | `dive_to_gate` | turun sambil yaw −45° (berbasis waktu), supaya diagonal menghadap lintasan |
| 3 | `approach_pole` | maju di sepanjang diagonal sampai radius orbit sebelum marker |
| 4 | `circle_turn_1` | yaw −90° (batas IMU) |
| 5 | `circle_turn_2` | **orbit**: maju di sepanjang diagonal sambil yaw. Berhenti oleh IMU pada 360° (`target_angle 6.23`), bukan oleh waktu |
| 6 | `circle_turn_3` | yaw +90° kembali (batas IMU) |
| 8 | `return_to_gate` | mundur di sepanjang diagonal lewat gate |

Hasil sim (thruster 2 + 3 dimatikan): lewat gate keluar di y = 0,08 m, orbit
359°, lewat gate kembali di y = 0,10 m. Orbit yang dibatasi IMU dipakai karena
orbit berbasis waktu meleset ±14° antar run, yang berarti sekitar 2 m di gate.

### Setting cepat dengan generator

Jangan edit angka satu per satu. Ukur di kolam, lalu buat ulang profilnya:

```bash
python3 src/BANYU_ROBOTX/tools/scripts/dead_thruster_por.py \
  --measured-speed 0.42 --start-to-gate 3.0 --gate-to-marker 10.0 --orbit-radius 2.3
```

| Opsi | Arti | Default |
|---|---|---|
| `--measured-speed` | kecepatan asli di sepanjang diagonal saat perintah `--speed-cmd` (ukur 10 s pakai meteran) | = `--speed-cmd` |
| `--speed-cmd` | kecepatan perintah di sepanjang diagonal | 0,5 m/s |
| `--start-to-gate`, `--gate-to-marker` | jarak lintasan | 3,2 / 10,0 m |
| `--orbit-radius` | radius orbit di sekitar marker | 2,3 m |
| `--past-gate` | jarak mundur setelah lewat gate | 2,0 m |
| `--live-diagonal` | arah dorong pasangan yang masih hidup: `forward-left` (2 + 3 mati) atau `forward-right` (1 + 4 mati) | `forward-left` |
| `--dive-turn-deg`, `--dive-yaw-rate` | putaran saat dive | 45°, 0,49 rad/s |
| `--turn-cap`, `--orbit-cap` | sudut berhenti IMU (sedikit kurang dari target untuk menyerap overshoot) | 1,45 / 6,23 rad |

### Urutan tes di kolam

1. **Arah diagonal.** Jalankan approach saja: robot harus bergerak ke
   **depan-kiri** badan. Kalau ke depan-kanan, pakai `--live-diagonal forward-right`.
2. **Dive −45°.** Cek heading setelah dive di QGC. Kalau kurang atau lebih,
   ubah `--dive-turn-deg` (misalnya 40 atau 50).
3. **Kecepatan.** Ukur jarak 10 s, isi `--measured-speed`, lalu buat ulang
   profilnya.
4. **Orbit.** Bandingkan heading di akhir orbit dengan heading di awal orbit
   (lihat di QGC).
   - Putarannya **kurang** dari 360°: naikkan `--orbit-cap` sedikit
     (+0,05 rad ≈ 3°).
   - Putarannya **lebih**: turunkan `--orbit-cap`.
   Error heading di sini menjadi error samping di gate, sekitar 0,17 m per
   derajat.
5. Lakukan full run, lalu ulangi langkah 3–4 sampai lewat gate di tengah.

## 11. Jalankan mission dari terminal tanpa RC (`bypass_rc`)

Mission hanya mau jalan kalau `vehicle_status` melaporkan `rc_fresh = true` dan
`operational_mode = 2`, dan keduanya normalnya datang dari transmitter (channel
RC dan switch SB). Dengan `bypass_rc: true` (parameter node `banyu_vehicle_status`):

- `rc_fresh` selalu benar dan tidak ada `operator_takeover` dari stick.
- `operational_mode` = `bypass_operational_mode` (2 = autonomous).
- Pesan **baru** di `/banyu/hardware/kill_switch` tetap dihormati, jadi
  `ros2 topic pub` bisa mematikan mission. Pesan yang basi **tidak** membunuh,
  tidak seperti mode biasa.
- FCU tetap harus terhubung (`/mavros/state connected`), dan mission tetap
  mengarm sendiri lewat `ArmAndSetGuided`.

Profil: `por_params_dead_thruster_2_3_no_rc.yaml`
(`dead_thruster_por.py --no-rc` membuat ulang profil ini dari angka hasil ukur).
Parameter yang sama bisa ditambahkan ke profil mana pun:

```yaml
banyu_vehicle_status:
  ros__parameters:
    bypass_rc: true
    bypass_operational_mode: 2
```

Launch (teleop dimatikan supaya mode uji darat dan switch SB tidak ikut campur):

```bash
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por enable_teleop:=False \
  por_params_file:=$HOME/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_rc.yaml
ros2 topic echo /banyu/vehicle_status --once    # fcu_connected, rc_fresh, operational_mode: 2
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

Pengaman tanpa RC (RC tidak lagi bisa mengambil alih):
- Berhenti: `ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"`.
- Kill lewat topic: `ros2 topic pub -r 5 /banyu/hardware/kill_switch std_msgs/msg/UInt8 "{data: 0}"`.
- Ctrl+C pada launch menghentikan setpoint, dan tetap sediakan kill relay fisik.
- Relay ESP32 tidak lagi diperintah oleh teleop. Kalau perlu mengaktifkannya
  tanpa teleop: `stty -F /dev/ttyUSB0 115200 raw -echo; printf '0' > /dev/ttyUSB0`
  (autonomous), dan `printf '2' > /dev/ttyUSB0` untuk kill. Itu byte yang sama
  dengan yang dikirim teleop.
- ArduSub tetap memeriksa pre-arm. Kalau arming ditolak karena tidak ada input
  RC, cek `ARMING_CHECK` di QGC; jangan matikan pemeriksaan lain.

## 12. Tanpa posisi (tanpa DVL/GPS): ALT_HOLD + RC override

GUIDED (dan AUTO) di ArduSub butuh estimasi posisi. Kalau EKF tidak punya
posisi, mission berhenti di `ArmAndSetGuided` dengan pesan
`Mode change failed: Guided requires position`. Mode tanpa posisi adalah ALT_HOLD:
kedalaman ditahan baro dan heading oleh kompas. Di mode ini stack mengirim
"stick" lewat `/mavros/rc/override`.

Profil: `por_params_dead_thruster_2_3_no_rc_no_pos.yaml`
(`dead_thruster_por.py --no-rc --no-position`, ALT_HOLD, **disarankan untuk
BANYU**). Isi parameternya:

| Node | Parameter | Nilai |
|---|---|---|
| `banyu_mission` | `autonomy_mode` | `ALT_HOLD` (mode yang diminta `ArmAndSetGuided`) |
| `vehicle_execution_node` | `control.autonomy_mode` / `control.output_mode` | `ALT_HOLD` / `rc_override` |
| `vehicle_execution_node` | `rc_override.channel_forward/lateral/throttle/yaw` | `3/4/2/1`, harus sama dengan `RCMAP_*` di Pixhawk |
| `vehicle_execution_node` | `rc_override.gain_xy/z/yaw` | 300 PWM per m/s (atau per rad/s) |
| `banyu_vehicle_status` | `autonomy_mode`, `bypass_rc` | `ALT_HOLD`, `true` |

Cara kerjanya:
- Kecepatan menjadi **open-loop**: perintah 0,354 m/s berarti stick sekitar
  1500 + 106 PWM, bukan kecepatan terukur. Jarak diatur ulang lewat
  `--measured-speed` (ukur 10 s), atau perkuat atau perlemah stick dengan
  `--rc-gain-xy`.
- Heading (dan batas IMU untuk putaran serta orbit) diambil dari
  `/mavros/imu/data` saat pose tidak ada.
- Saat mission berhenti atau ada fault, stick ditahan **netral** selama robot
  masih arm di ALT_HOLD, lalu **dilepas** ke transmitter saat disarm. Kalau
  stack mati, ArduSub membuang override sendiri setelah `RC_OVERRIDE_TIME`
  (3 s).
- RC override hanya diterima dari GCS dengan sysid = `SYSID_MYGCS` (255). MAVROS
  sudah memakai `system_id: 255`.

Hasil cek di Pixhawk BANYU (disarm, 20 Okt):
- **BARO2 = `MS5837_30BA` (Bar30) di I2C 0x76**. Driver ini bertipe `BARO_TYPE_WATER`,
  jadi ArduSub menganggap ada sensor kedalaman. Pembacaannya hidup (1030 hPa di udara).
- Ganti mode ke **ALT_HOLD diterima** dan STABILIZE diterima. GUIDED/AUTO/POSHOLD
  ditolak karena butuh posisi (`ArduSub/mode.cpp`, `position_ok()`).
- ALT_HOLD butuh sensor kedalaman tipe water (`Sub::control_check_barometer`).
  Kalau Bar30 lepas, pakai `--no-depth-sensor` (profil `..._pixhawk_only.yaml`,
  STABILIZE, kedalaman open-loop dengan `--depth-trim`).

Kompensasi stick (generator menghitungnya dari parameter Pixhawk, lihat
`ArduSub/Attitude.cpp` dan `RC_Channel::pwm_to_angle_dz_trim`):

| Sumbu | ArduSub | Setting (BANYU) |
|---|---|---|
| Yaw | rate = 4500 cdeg × `ACRO_YAW_P` per s pada 400 − DZ PWM, DZ = `RC1_DZ` × `JS_GAIN_DEFAULT` | DZ 20, gain 143,4 PWM per rad/s (`--acro-yaw-p 3.375 --rc-yaw-dz 40 --js-gain 0.5`) |
| Throttle (ALT_HOLD) | climb = `PILOT_SPEED` × (stick − `THR_DZ` × gain) | DZ 50, gain 200 PWM per m/s (`--thr-dz 100 --pilot-speed 200`) |
| Maju / samping | dorongan langsung (`norm_input`, tanpa DZ) | gain 300 PWM per m/s, kalibrasi dengan `--measured-speed` |

Tanpa kompensasi deadzone, yaw orbit 0,217 rad/s (33 PWM) tertelan oleh deadzone
20 PWM, dan robot hampir tidak berbelok.

Putaran 45° di awal **dibatasi IMU** di segmen `pass_gate`
(`timed_pass_gate_target_angle_rad`). Dive turun lurus tanpa yaw, karena yaw
berbasis waktu lewat stick tidak akurat.

Cek channel dengan `ros2 topic echo /mavros/rc/in` atau QGC Radio. Kalau
`RCMAP_*` di Pixhawk berbeda, buat ulang profil dengan
`--rc-channels forward,lateral,throttle,yaw`.

Launch dan jalankan:

```bash
python3 ~/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX/tools/scripts/esp32_kill_cmd.py auto --hold   # T2
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por enable_teleop:=False \
  por_params_file:=$HOME/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_rc_no_pos.yaml
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

Berhenti: `/banyu/mission/abort`, lalu
`ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"`,
lalu `esp32_kill_cmd.py kill`.

### Versi dengan RC (TX16S): `por_params_dead_thruster_2_3_no_pos.yaml`

Sama dengan profil no-position (ALT_HOLD + RC override), tetapi **RC tetap
dipakai**:
- SB atas = autonomous.
- SB tengah = takeover manual. Override dari stack **langsung dilepas**, jadi
  stick langsung berfungsi.
- SB bawah = kill.
- Mission tetap di-start dari terminal.

Mode uji darat dari teleop wajib dimatikan (`enable_mock_auto:=False`). Kalau
tidak, SB atas mengaktifkan mode uji darat (MANUAL + override 1540), bukan
mission:

```bash
ros2 launch banyu_bringup sub_bringup.launch.py mode:=por enable_mock_auto:=False \
  por_params_file:=$HOME/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_pos.yaml
# TX16S: SB atas (autonomous), lalu:
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"
```

Buat ulang profil ini dengan `dead_thruster_por.py --no-position` (tanpa `--no-rc`).
