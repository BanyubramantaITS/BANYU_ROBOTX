# POR: Tuning

Cara menjalankan dan memilih profil ada di [`README.md`](README.md). Hasil sim dan model gerak
ada di [`sim_results.md`](sim_results.md).

File profil ada di `src/BANYU_ROBOTX/banyu_bringup/config/`. Workspace memakai
`--symlink-install`, jadi perubahan YAML langsung berlaku di launch berikutnya tanpa build.

## 1. Konsep yang perlu dipahami dulu

- **Kecepatan = kekuatan stick, bukan m/s.** Di ALT_HOLD + RC override,
  `speed_x_mps: 0.5` artinya stick forward 1500 + 0,5 × `gain_xy_pwm_per_mps` (300) = 1650 µs.
  Kecepatan aslinya tergantung robot, baterai, dan air. Di sim, kecepatan asli ≈ 1,34·√cmd maju
  dan ≈ 1,39·√cmd geser. Karena itu kecepatan harus **diukur** di kolam (§3).
- **Robot meluncur 0,7–1 m** setelah segmen berhenti (tidak ada rem aktif).
- **Rute = urutan segmen.** Setiap segmen punya `*_duration_s`, `*_speed_x/y/z_mps`,
  `*_yaw_rate_rad_s`, dan beberapa punya `*_target_angle_rad`.
- **Konvensi arah (badan robot, FLU):** `speed_x` + maju / − mundur; `speed_y` + kiri / − kanan;
  `speed_z` + naik / − turun; `yaw_rate` + belok kiri / − belok kanan.
- **`*_duration_s: 0.0`** melewati segmen itu.
- **`*_target_angle_rad` > 0** menghentikan putaran saat IMU mencapai sudut itu (durasi jadi
  batas maksimum). Isi sedikit kurang dari target untuk menyerap overshoot.

### Urutan segmen (tree timer)

| # | Segmen | Fungsi |
|---|---|---|
| 1 | `dive` | Turun ke `gate_depth_m`, selesai saat kedalaman tercapai (atau durasi habis) |
| 2 | `timed_pass_gate` | Maju lewat gate |
| 3 | `approach_pole` | Maju sampai jarak tertentu sebelum marker |
| 4 | `circle_turn_1` … `circle_forward_4` | Delapan slot loop di sekitar marker. Nama "turn/forward" hanya nama slot: di strafe box `circle_turn_*` adalah geser, di diamond semuanya diagonal |
| 5 | `return_turn` | Putar balik (profil `turn`, `vision`) atau dilewati |
| 6 | `return_to_gate` | Kembali lewat gate (mundur atau maju) |
| 7 | `timed_surface` | Naik ke permukaan |

### Saklar sensor (`banyu_mission.mission`)

| Parameter | Arti |
|---|---|
| `use_sensors` | `false` = murni timer, semua sensor diabaikan (profil `blind/`) |
| `use_baro_depth` | Tahan `gate_depth_m` pakai baro di semua segmen |
| `use_imu_yaw_cap` | Pakai `*_target_angle_rad` untuk menghentikan putaran |
| `gate_depth_m` | Kedalaman jalan: di tengah antara bawah palang gate dan lantai (1,8 m di kolam 2,5 m; maksimal 2,0) |

`vehicle_execution_node.timed_move.heading_hold.enabled: true` menahan heading awal di segmen
lurus (mengoreksi drift karena thruster tidak seimbang).

## 2. Parameter yang biasa diubah

| Tujuan | Parameter | Tempat |
|---|---|---|
| Panjang tiap segmen | `*_duration_s` | `banyu_mission.mission` |
| Arah dan kekuatan tiap segmen | `*_speed_x/y/z_mps`, `*_yaw_rate_rad_s` | `banyu_mission.mission` |
| Sudut putaran | `*_target_angle_rad` (terutama `return_turn_target_angle_rad`) | `banyu_mission.mission` |
| Kedalaman | `gate_depth_m` | `banyu_mission.mission` |
| Lama koreksi kamera | `assist_gate_align_max_s`, `assist_marker_align_max_s` (0 = mati) | `banyu_mission.mission` (vision) |
| Warna gate/marker | `gate_*`, `marker_*`, `min_area` | `banyu_detection/config/por_detection_params.yaml` (§5) |
| Batas kecepatan horizontal | `timed_move.max_horizontal_speed_mps` (0 = tanpa batas) | `vehicle_execution_node` |
| Kekuatan stick | `rc_override.gain_*` | `vehicle_execution_node` |
| Kanal stick | `rc_override.channel_*` = 2/1/3/4 (harus = `RCMAP_*` Pixhawk) | `vehicle_execution_node` |

Yang **tidak** perlu diubah untuk tuning: `autonomy_mode` (harus `ALT_HOLD` di tiga node),
`output_mode: rc_override`, `fallback_mode: DISARM`, topic, action, `esp32_*`,
`sim_rc_deadman_node`, `ekf_source_switch`.

Kalau satu angka diubah di satu profil, profil lain tidak ikut berubah (setiap profil berdiri
sendiri). Profil `blind/` juga harus diubah terpisah.

## 3. Alur tuning

1. **Sim dulu.** Ubah YAML, jalankan `launch_por.sh --sim --profile <x>`, start, lihat di
   Gazebo apakah robot lewat gate, mengitari marker, dan kembali.
2. **Kolam: kalibrasi kecepatan** (paling penting, §4.2).
3. **Hitung ulang durasi** dari kecepatan asli (§4.3).
4. **Tuning segmen satu per satu** (§4.4).
5. **Arah start** (§4.6).

## 4. Tuning di kolam

### 4.1 Persiapan

- Stopwatch, meteran 20 m, tali atau garis di lantai kolam ke arah marker, satu orang merekam
  dari pinggir.
- Laptop dengan `ROS_DOMAIN_ID` sama dengan robot untuk melihat topic.
- Abort selalu siap (SB, atau `/banyu/mission/abort`).
- Tuning di level baterai yang sama dengan saat lomba, catat voltasenya.
- Mulai dengan `use_sensors: true` supaya kedalaman ditahan baro.

### 4.2 Kalibrasi kecepatan

Salin profil, set semua `*_duration_s` ke 0 kecuali `dive` dan **satu** segmen yang mau diukur.
Contoh maju:

```yaml
dive_duration_s: 8.0
timed_pass_gate_duration_s: 10.0
timed_pass_gate_speed_x_mps: 0.6     # kekuatan stick yang dipakai profil
# segmen lain: *_duration_s: 0.0
```

Jalankan dengan `--profile /path/ke/file_tes.yaml`, ukur jarak, lalu:

**`v_asli = jarak / (durasi − dead_time)`**

Dead time awal: maju 0,55 s, mundur 0,85 s, geser 0,6–0,75 s. Ulangi setiap gerakan 3 kali.
Hasil harus konsisten sampai **±5%**; lebih dari itu loop bisa menabrak dinding belakang
(marker hanya 3,5 m dari dinding). Ukur semua gerakan yang dipakai profil: maju, mundur, geser
kiri, geser kanan, dan diagonal (untuk diamond, ukur jarak X dan Y terpisah).

### 4.3 Hitung durasi

**`durasi = jarak_rencana / v_asli + dead_time`**

Strafe box (`safe`, `fast`, `default`, `slow`, `hybrid`):

| Segmen | Jarak rencana |
|---|---|
| `dive` + `timed_pass_gate` + `approach_pole` | sampai 2 m sebelum marker |
| `circle_turn_1` | geser kanan 2 m |
| `circle_forward_1` | maju 3,5 m |
| `circle_turn_2` | geser kiri 4 m |
| `circle_forward_2` | mundur 3,5 m |
| `circle_turn_3` | geser kanan 2 m |
| `return_to_gate` | mundur sampai 2 m lewat gate |

Diamond (`fastest`, `turn`, `vision`):

| Segmen | Jarak rencana |
|---|---|
| `dive` + `approach_pole` | sampai 3,3 m sebelum marker |
| `circle_turn_1` (+x, −y) | 2,6 m maju, 2,7 m kanan |
| `circle_forward_1` (+x, +y) | 2,6 m maju, 2,7 m kiri (lewat samping marker) |
| `circle_turn_2` (−x, +y) | 2,6 m mundur, 2,7 m kiri |
| `circle_forward_2` (−x, −y) | sampai kembali ke garis tengah (sedikit lebih lama) |
| `return_to_gate` | sekitar 9,7 m, sampai 2 m lewat gate |

### 4.4 Segmen satu per satu

Setiap kali, segmen berikutnya diset durasi 0. Lanjut hanya kalau segmen ini pas 2 kali
berturut-turut.

1. **Dive.** Robot harus sudah di bawah palang gate sebelum sampai gate.
2. **Approach.** Berhenti 2 m (strafe box) atau 3,3 m (diamond) sebelum marker.
3. **Loop.** Kembali ke garis tengah di depan marker.
4. **Return.** Lewat tengah gate, lalu naik.

Urutan profil yang disarankan: strafe box sampai 3 kali sukses berturut-turut, lalu diamond
(kalibrasi lagi di kecepatan diamond), lalu `turn` hanya kalau putarannya konsisten ±3°.

### 4.5 Putar balik (`turn`, `vision`)

Putaran 180° harus akurat ±3°: di sim, kelebihan 8° membuat robot meleset 2 m dari gate.

1. Tes `return_turn` saja di tempat dan ukur heading akhir (QGC).
2. Kurang dari 180° / lewat gate ke **kiri**: naikkan `return_turn_target_angle_rad`
   (+0,03 rad ≈ 2°). Lebih / lewat ke **kanan**: turunkan.
3. `return_turn_duration_s` hanya batas waktu; biarkan sedikit lebih lama dari putaran asli.
4. Di profil `blind/`, putaran hanya diatur durasi; jauh lebih sulit.

### 4.6 Arah start

Error arah start adalah sumber error kedua terbesar: 4° di jarak 13 m ≈ 0,9 m ke samping.
Sejajarkan badan robot dengan tali atau garis lantai yang menunjuk ke marker, maksimal ±4°.

### 4.7 Troubleshooting

| Gejala | Perbaikan |
|---|---|
| Menabrak marker / loop terlalu dekat | kurangi `approach_pole_duration_s` |
| Loop tidak mengitari marker | tambah `approach_pole_duration_s` |
| Menabrak dinding belakang | kurangi `approach_pole_duration_s`, atau kalibrasi ulang (kecepatan asli lebih tinggi) |
| Loop tidak menutup (kembali ke kiri/kanan garis tengah) | atur `circle_forward_2` (diamond) atau `circle_turn_3` (strafe box) |
| Lewat gate melenceng ke samping | cek arah start, lalu keseimbangan geser kiri = kanan |
| Belum lewat gate saat naik | tambah `return_to_gate_duration_s` |
| Menabrak dinding start | kurangi `return_to_gate_duration_s` |
| Menyentuh palang gate | tambah `gate_depth_m` (maksimal 2,0) |
| Hasil berubah-ubah antar run | baterai turun; kalibrasi ulang di voltase lomba |

## 5. Kamera (`vision`, `hybrid`)

### 5.1 Cara kerja

`vision` = timer diamond + koreksi kamera. Setiap langkah kamera dibatasi waktu dan jatuh ke
timer kalau gagal:

| Langkah | Kamera | Fallback |
|---|---|---|
| `AlignGate` di start (geser + kedalaman ke tengah gate) | gate | lewati setelah `assist_gate_align_max_s` |
| dive + approach | – | – |
| `AlignMarker` geser ke samping sampai marker di tengah (memperbaiki error heading start) | marker | lewati setelah `assist_marker_align_max_s` |
| diamond, putar 180° (IMU) | – | – |
| `SearchGate`, `TurnAround` mengunci heading ke gate, `PassGate` maju sambil steering | gate | gate tidak terlihat: `return_to_gate` timer |

`hybrid` = strafe box timer, lalu pulang pakai kamera (fallback `return_to_gate` timer).

Jarak tetap diatur timer: estimasi jarak dari kamera tidak bisa dipercaya (di sim, sisi pole
yang gelap tidak lolos HSV). Kalibrasi kecepatan tetap wajib.

### 5.2 Tuning warna (HSV)

Gate kuning, marker oranye; HSV terpisah per objek (`color_mode: "per_object"`). Default di
`banyu_detection/config/por_detection_params.yaml`: gate H 20–35, S 80–255, V 80–255; marker
H 5–18, S 120–255, V 100–255; `min_area` 300. Hue OpenCV 0–179; kalau `h_min > h_max` range
melingkar lewat 0 (untuk merah).

Tool slider (robot menjalankan stack seperti biasa; laptop dengan `ROS_DOMAIN_ID` sama):

Di robot: jalankan profil apa saja **dengan kamera** (jangan `--no-cam`); `por_detection` selalu
jalan di mode POR. Di laptop (satu jaringan dengan NUC, `ROS_DOMAIN_ID` robot = 30):

```bash
export ROS_DOMAIN_ID=30                                          # wajib, default laptop 0
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py          # lewat WiFi (compressed)
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py --raw    # sim / kabel LAN
python3 src/BANYU_ROBOTX/tools/scripts/por_hsv_tuner.py --source rekaman.mp4   # offline
```

- Tiga jendela slider: **Gate HSV (yellow)**, **Marker HSV (orange)**, dan Filter; slider
  langsung mengubah parameter detector.
- Selama tuner jalan, `debug_rate_hz` detector dinaikkan ke 15 Hz (`--debug-rate`, 0 = tidak
  diubah) dan dikembalikan saat keluar (juga dengan Ctrl+C). Gambar yang sampai sekitar 3–8 Hz:
  batasnya kecepatan detector di NUC, bukan jaringan, jadi `--raw` tidak lebih cepat.
- Jendela utama menampilkan gambar annotated + mask gate + mask marker.
- `s` simpan (`~/por_detection_tuned.yaml` di robot, dan snippet YAML di terminal), `r` baca
  ulang dari robot, `q` keluar.

Target: mask gate putih hanya di tiang dan palang gate, mask marker putih hanya di pole marker,
lantai dan pantulan hitam. Urutan: persempit `h_min`/`h_max` → naikkan `s_min` (buang air/lantai
pucat) → atur `v_min` (bayangan) → naikkan `min_area` (buang bintik).

Alternatif: `ros2 run rqt_reconfigure rqt_reconfigure` (pilih `/por_detection`) +
`rqt_image_view` di `/banyu/detection/debug/annotated/compressed`.

Simpan permanen: `s` menulis `~/por_detection_tuned.yaml` **di NUC** dan mencetak snippet YAML
di terminal laptop. Salin nilainya ke `banyu_detection/config/por_detection_params.yaml` (satu-
satunya sumber HSV; profil tidak meng-override), commit, lalu pull di NUC (symlink-install: tanpa build).
Tanpa itu nilai slider hilang saat stack di-restart. Cek dengan
`ros2 topic echo /banyu/detection --once`.

### 5.3 Kalibrasi jarak

`AlignMarker` menjaga jarak 3,0–3,6 m (`assist_marker_standoff_*`) dari estimasi ukuran objek.
Estimasi itu memakai focal length dari `/camera/camera_info`. `config/front_camera_info.yaml`
saat ini berisi intrinsik ideal dari sim (640×480, f = 554 px), bukan hasil kalibrasi kamera
robot. Kalibrasi kamera depan (`ros2 run camera_calibration cameracalibrator`) atau setidaknya
lakukan koreksi reference width di bawah dengan kamera robot.

1. Taruh robot 3,0 m (meteran) di depan marker.
2. Baca jarak estimasi: `ros2 topic echo /banyu/detection/marker --once` (`distance`).
3. `buoy_reference_width_m` (bagian `por_detection` di profil):
   **baru = lama × 3,0 / jarak_terbaca**. Sama untuk gate dengan `gate_reference_width_m` dan
   `gate_reference_height_m`.

## 6. Dead thruster (thruster 2 + 3 mati)

Frame Vectored-6DOF: motor 1–4 horizontal (1 depan-kanan, 2 depan-kiri, 3 belakang-kanan,
4 belakang-kiri), 5–8 vertikal. Kalau 2 + 3 mati, pasangan 1 + 4 hanya mendorong sepanjang satu
diagonal badan (depan-kiri) dan masih bisa yaw. Rutenya:

| Segmen | Gerakan |
|---|---|
| `timed_pass_gate` | putar 45° (dibatasi IMU) supaya diagonal menghadap lintasan |
| `approach_pole` | maju sepanjang diagonal sampai radius orbit sebelum marker |
| `circle_turn_1` | yaw 90° (IMU) |
| `circle_turn_2` | orbit: maju sepanjang diagonal sambil yaw, berhenti di 360° (IMU) |
| `circle_turn_3` | yaw 90° kembali |
| `return_to_gate` | mundur sepanjang diagonal lewat gate |

Jangan edit angka satu per satu; buat ulang profil dengan generator dari hasil ukur:

```bash
python3 src/BANYU_ROBOTX/tools/scripts/dead_thruster_por.py --no-position \
  --measured-speed 0.42 --start-to-gate 3.0 --gate-to-marker 10.0 \
  --out src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_pos.yaml
```

Tambah `--no-rc` untuk versi tanpa TX16S. Opsi penting:

| Opsi | Arti |
|---|---|
| `--measured-speed` | kecepatan asli sepanjang diagonal pada `--speed-cmd` (ukur 10 s dengan meteran) |
| `--speed-cmd` | kekuatan stick diagonal (default 0,5) |
| `--start-to-gate`, `--gate-to-marker`, `--orbit-radius`, `--past-gate` | geometri lintasan |
| `--live-diagonal` | `forward-left` (2+3 mati) atau `forward-right` (1+4 mati) |
| `--turn-cap`, `--orbit-cap` | sudut berhenti IMU (rad); sedikit kurang dari target |
| `--rc-channels` | kanal forward,lateral,throttle,yaw (default 2,1,3,4 = `RCMAP_*` robot) |
| `--no-depth-sensor`, `--depth-trim` | Bar30 mati: STABILIZE dengan trim kedalaman open-loop |

Urutan tes di kolam:
1. **Arah diagonal**: approach saja, robot harus bergerak ke depan-kiri badan. Kalau ke
   depan-kanan, pakai `--live-diagonal forward-right`.
2. **Kecepatan**: ukur jarak 10 s, isi `--measured-speed`, buat ulang.
3. **Orbit**: bandingkan heading akhir orbit dengan heading awal (QGC). Kurang dari 360°:
   naikkan `--orbit-cap` (+0,05 rad ≈ 3°); lebih: turunkan. Error 1° ≈ 0,17 m di gate.
4. Full run, ulangi 2–3 sampai lewat tengah gate.

Di sim, `--measured-speed 1.15 --orbit-cap 6.29` memusatkan return (profil
`config/sim_dead_thruster/`, khusus sim).

**Bar30 mati** (`--no-depth-sensor`, STABILIZE): tidak ada satu trim yang menahan kedalaman.
Di sim yang berhasil hanya dithering per segmen antara dua nilai trim yang berbeda 1 µs (detail
di [`sim_results.md`](sim_results.md)). Ukur laju naik/turun di 1496/1497 µs di robot sebelum
memakainya. Solusi yang lebih aman: perbaiki Bar30 atau buat daya apung hampir netral.

## 7. Profil blind (`config/blind/`)

Sama dengan profil normal, dengan perbedaan:
- `use_sensors: false`: kedalaman dari durasi dive (open-loop, ~1,32 × `speed_z` m/s di sim),
  putaran dari durasi saja.
- Heading hold mati, kamera tidak dipakai.

Pakai kalau baro, IMU, atau kamera tidak bisa dipercaya. Setiap perubahan durasi di profil
normal harus disalin manual ke versi blind-nya.

## 8. Stick, kanal, dan deadzone (`rc_override`)

| Sumbu | Di ArduSub | Setting sekarang |
|---|---|---|
| Maju / geser | dorongan langsung, tanpa deadzone | gain 300 PWM per m/s, kalibrasi dengan kecepatan ukur |
| Throttle (ALT_HOLD) | laju naik = `PILOT_SPEED` × (stick − `THR_DZ` × gain) | deadzone 50, gain 400 (profil normal) / 200 (dead-thruster) |
| Yaw | rate sebanding stick di luar deadzone (`RC4_DZ` 40 × `JS_GAIN_DEFAULT` 0,5) | deadzone 20, gain 143,4 PWM per rad/s |

Tanpa kompensasi deadzone, yaw kecil (mis. 33 PWM) tertelan deadzone dan robot hampir tidak
berbelok. Di sim, yaw asli ≈ 0,98 × perintah di 0,6 rad/s dan 0,86 × di 0,9 rad/s.

`rc_override.channel_*` harus sama dengan `RCMAP_*` Pixhawk (robot: forward CH2, lateral CH1,
throttle CH3, yaw CH4), dan tidak boleh sama dengan kanal switch SB/SC ([`README.md`](README.md)
§4.2). RC override hanya diterima dari sysid yang cocok dengan `MAV_GCS_SYSID` Pixhawk; MAVROS
memakai `system_id: 255`.

Saat misi berhenti, stick ditahan netral selama robot masih arm di ALT_HOLD dengan mode
autonomous, lalu dilepas ke transmitter saat takeover, disarm, atau ganti mode.
