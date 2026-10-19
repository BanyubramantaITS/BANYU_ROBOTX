# Panduan Tuning POR Timer di Kolam (Real World)

Panduan ini untuk strategi POR tanpa kamera (timer + kecepatan per segmen).
Hasil simulasi dan model gerak ada di [`por_timed_tuning.md`](por_timed_tuning.md).

| Profil | File | Waktu sim (lewat gate kembali) | Toleransi sim |
|---|---|---|---|
| Strafe box (**paling aman**) | `por_params_timed_fast.yaml` | 55 s | kecepatan −8%..+5%, heading ±6° |
| Diamond mundur (**tercepat**) | `por_params_timed_fastest.yaml` | 33,3 s | kecepatan ±6%, heading +6° (gagal −6°) |
| Diamond putar balik | `por_params_timed_fastest_turn.yaml` | 39,1 s | putaran harus akurat ±3° |

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

## Catatan: mode kamera

- `por_params_hybrid.yaml` (`use_vision_return: true`) berangkat dan loop pakai
  timer, lalu pulang lewat gate pakai kamera.
- `use_vision: true` menjalankan full vision (`bt/por_mission.xml`).
- Semua aksi vision baru bergerak kalau `/mavros/local_position/pose` tersedia.
  Cek dulu di robot dengan `ros2 topic hz /mavros/local_position/pose`.
- Kalau sim dijalankan di laptop yang satu jaringan dengan robot, pakai
  `ROS_DOMAIN_ID` yang berbeda. Kalau tidak, traffic robot memicu
  "Vehicle safety or run authorization fault" di sim.
