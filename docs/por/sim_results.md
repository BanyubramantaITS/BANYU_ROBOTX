# POR: Hasil Simulasi

Cara menjalankan ada di [`README.md`](README.md), cara tuning di [`tuning.md`](tuning.md).

## Lintasan dan kriteria lulus

- Start x = -7 (permukaan), gate x = -4 (tiang di y = ±1, palang 1 m di bawah air), marker
  x = +6. Kolam 19 × 10 × 2,5 m; marker hanya 3,5 m dari dinding belakang.
- **Lulus** = lewat gate keluar dan kembali (|y| ≤ 0,66 m, di bawah palang, di atas lantai),
  satu putaran penuh mengitari marker dengan jarak pusat robot ke marker ≥ 0,8 m (celah fisik
  ±0,5 m), dan ≥ 0,6 m dari semua dinding. Dinilai dari ground truth Gazebo.
- Semua run di bawah: ALT_HOLD + RC override, 8 thruster, Bar30, kompas, kamera depan,
  **tanpa GPS sama sekali** (`use_gps:=False`: GPS SITL dihapus sejak boot, `nogps_odom_beacon`
  jalan seperti di robot), daya apung positif.
- Waktu = waktu sim. Gazebo berjalan ±0,7× real time di laptop dev.
- Setiap baris adalah **satu run nominal**. Toleransi (error kecepatan dan heading) belum diukur
  ulang di ALT_HOLD.
- **Peta RC robot (24 Okt 2026):** SITL memakai `RCMAP` yang sama dengan Pixhawk robot
  (forward CH2, lateral CH1, throttle CH3, yaw CH4) dan semua profil mengirim override ke
  2/1/3/4. Diulang 14 skenario (8 profil normal, `vision` kamera mati, `blind/` fast dan
  fastest_turn, 3 dead-thruster): **semua lulus**, waktu lewat gate kembali sama dengan tabel di
  bawah (±1,5 s), jarak ke marker ≥ 0,9 m. Lewat gate kembali dead-thruster bergeser ke
  y = -0,25…-0,41 m (sebelumnya -0,1 m), masih di dalam ±0,66 m.

## Profil normal

| Profil | Hasil | Lewat gate kembali | Jarak ke marker |
|---|---|---|---|
| `por_params_timed_fastest.yaml` | ✅ | ~25 s | 0,98 m |
| `por_params_timed_fastest_turn.yaml` | ✅ `return_turn_target_angle_rad` 2,96 | ~28 s | 0,96 m |
| `por_params_timed_fast.yaml` | ✅ segmen loop +0,2 s untuk margin | ~32 s | 1,13 m |
| `por_params_timed_safe.yaml` | ✅ | ~37 s | 1,13 m |
| `por_params.yaml` (default) | ✅ sekarang strafe box SAFE (turn box lama tidak pernah mengitari marker) | ~37 s | 1,12 m |
| `por_params_vision_assisted.yaml` | ✅ AlignMarker kamera + pulang pakai kamera | ~36 s | 1,03 m |
| `por_params_hybrid.yaml` | ✅ pulang pakai kamera (`turn_max_angle_rad` 0,1) | ~44 s | 1,03 m |
| `por_params_timed_slow.yaml` | ✅ (stick 0,12 → ±0,47 m/s) | ~71 s | 1,05 m |

## Skenario kerusakan

- Kamera mati = `por_detection` tidak menerima gambar.
- Thruster mati = `extra_sitl_params:=.../config/sim_faults/dead_thrusters_2_3.parm`.
  `SERVOx_FUNCTION 135` (k_trim) menahan motor 2 dan 3 di 1500 µs. `SERVOx_FUNCTION 0` tidak
  bekerja di ArduSub (frame me-assign ulang motornya), dan `1` (RCPassThru) meneruskan RC
  override sehingga motor tetap jalan.
- Blind = `config/blind/*.yaml` dengan kamera juga mati.

| Skenario | Profil | Hasil | Lewat gate kembali |
|---|---|---|---|
| Kamera mati | `vision_assisted` | ✅ jatuh ke timer | ~39 s |
| Kamera mati | `hybrid` | ✅ jatuh ke return timer | ~41 s |
| Blind | `blind/..._fastest` | ✅ | ~24 s |
| Blind | `blind/..._fastest_turn` | ✅ | ~28 s |
| Blind | `blind/..._fast` | ✅ | ~32 s |
| Blind | `blind/..._vision_assisted` | ✅ | ~34 s |
| Blind | `blind/por_params`, `blind/..._safe` | ✅ | ~37 s |
| Blind | `blind/..._hybrid` | ✅ (putaran 180° berbasis waktu: 0,1 s = 0,4 m di gate) | ~37 s |
| Blind | `blind/..._slow` | ✅ | ~70 s |
| Thruster 2+3 mati | `por_params_timed_fast.yaml` (kontrol) | ❌ hanyut diagonal ke dinding samping | – |
| Thruster 2+3 mati | `sim_dead_thruster/..._no_pos` (RC) | ✅ orbit marker, y = -0,09 di gate | ~47 s |
| Thruster 2+3 mati | `sim_dead_thruster/..._no_rc_no_pos` | ✅ y = -0,10 di gate | ~46 s |
| Thruster 2+3 mati + Bar30 mati | `sim_dead_thruster/..._no_rc_pixhawk_only` (STABILIZE) | ✅ dengan dithering trim per segmen, kedalaman 1,7–2,2 m | ~44 s |

Profil di `config/sim_dead_thruster/` dibuat dengan kecepatan diagonal sim (1,15 m/s pada
`--speed-cmd 0.5`, `--orbit-cap 6.29`).

### Bar30 mati

Di STABILIZE tanpa sensor kedalaman, sweep `--depth-trim` (dive 4 s): 1500 µs naik ~0,12 m/s,
1498 µs ~0,10 m/s, 1497 µs ~0,045 m/s (semua sampai permukaan sebelum gate kembali), dan
1496 µs sudah tenggelam ke lantai. Titik seimbangnya di antara dua nilai stick yang berbeda
1 µs, jadi tidak ada satu trim yang menahan kedalaman. Yang berhasil: setiap segmen memakai
`speed_z_mps` -0,015 (1496 µs, turun ~0,05 m/s) atau -0,01 (1497 µs, naik ~0,045 m/s),
dipilih supaya kedalaman tetap di tengah air.

Laju ini tergantung daya apung persis, jadi harus diukur di robot. Sim juga tidak bisa
memodelkan kerusakan ini dengan tepat: semua baro SITL ArduSub adalah baro air, ArduPilot tidak
mau boot kalau semua baro dimatikan, dan `SIM_BAROx_FREEZE` tidak berpengaruh.

## Model gerak sim (ALT_HOLD)

| Besaran | Nilai sim |
|---|---|
| Kecepatan maju/mundur | ≈ 1,34 · √`speed_x` m/s (gain 300), mis. 0,9 → 1,28 m/s |
| Kecepatan geser | ≈ 1,39 · √`speed_y` m/s |
| Luncuran setelah segmen berhenti | 0,7–1 m |
| Kecepatan vertikal open-loop | ≈ 1,32 · `speed_z` m/s (gain_z 400, deadzone 50) |
| Yaw | ≈ 0,98 × perintah di 0,6 rad/s, 0,86 × di 0,9 rad/s |
| Overshoot putaran 180° dengan batas IMU | ±2° |

Ini model sim, bukan robot. Di kolam kecepatan harus diukur ([`tuning.md`](tuning.md) §4.2).

## Pelajaran dari era GUIDED (historis)

Sebelum ALT_HOLD, POR terbang GUIDED dengan setpoint kecepatan dari spawn di bawah air. Angka
waktunya tidak berlaku lagi, tapi kesimpulan desainnya masih berlaku:

- **Strafe box lebih baik dari turn box.** Putaran 90° dengan batas IMU di 0,6 rad/s overshoot
  ±14° karena robot tetap berputar setelah perintah berhenti. Empat putaran menumpuk error, dan
  di return 10 m, 12° berarti meleset 2,7 m. Strafe box menjaga yaw tetap dan mundur lewat gate
  di garis yang sama dengan saat keluar, jadi error heading start menggeser jalur keluar dan
  pulang bersama-sama.
- **Batas toleransi.** Error kecepatan menggeser seluruh loop sepanjang x (10% × 13 m ≈ 1,3 m);
  dengan 3,5 m di belakang marker, batasnya ±5–8%. Error heading start menggeser loop ke samping
  (tan θ × 13 m); setengah lebar loop 2 m memberi ±6°.
- **Diamond D3** (semua segmen menggabungkan sumbu, robot tidak berhenti untuk ganti arah):
  loop ±12 s dibanding 26 s strafe box. Lolos ±6% kecepatan dan +6° heading; gagal di +8%
  kecepatan karena loop bergeser ±1,3 m ke dinding belakang.
- Varian lain yang dicoba: hexagon (lebih lambat, luncuran meregangkan bentuknya), orbit
  (yaw + geser; lolos tapi drift heading menggeser return 0,3 m), dan turn box A3 (lolos tapi
  error heading menumpuk sampai ±6°).
