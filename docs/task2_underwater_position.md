# Task 2 di robot: posisi hilang saat menyelam (bahan diskusi tim)

Status: **terbuka, belum diputuskan** (Oktober 2026). Tidak ada kode yang diubah untuk ini.

## Masalah

Task 2 terbang di mode **GUIDED** (`task2_params.yaml`: `autonomy_mode: "GUIDED"`,
`control.output_mode` default `setpoint`, `control.require_guided: true`). ArduSub hanya
menerima dan menjalankan setpoint GUIDED kalau EKF punya estimasi posisi (`position_ok()`).

Robot tidak punya DVL. Sumber posisi satu-satunya adalah GPS, dan GPS tidak bekerja di bawah air.
Rantai yang terjadi di robot dengan setting sekarang (`sub_bringup.launch.py mode:=task2`,
`use_gps` default `True`):

1. Di permukaan EKF memakai source set 1 (GPS). `NavigateSurfaceGps` dan `AlignHeading` jalan.
2. Begitu kedalaman > `dive_depth` (0,5 m), `ekf_source_switch` pindah ke source set 2:
   `POSXY 0` (tidak ada), `VELXY 6` (ExternalNav). Di robot tidak ada yang mengirim ExternalNav:
   `nogps_odom_beacon` hanya jalan kalau `use_gps:=False`, dan DVL tidak ada.
3. EKF kehilangan aiding horizontal dan `position_ok()` jadi false. GUIDED tidak bisa dimasuki
   lagi (`ModeGuided::init`), dan selama masih di GUIDED, kontrol kecepatannya memakai estimasi
   kecepatan EKF yang tidak lagi dikoreksi sensor apa pun (drift inersia). Dengan
   `FS_EKF_ACTION 0` (repo) tidak ada failsafe yang menghentikannya.

Aksi BT yang jalan di bawah air dan terkena: `DiveToDepth`, `AcquirePipeline`,
`SearchPipeline`, `FollowPipeline`, `CenterOnRing`, `HoldProbeOnRing`, dan `Surface`
(`bt/banyu_mission.xml`).

## Kenapa sim tidak memperlihatkannya

- GPS SITL disintesis dari ground truth Gazebo dan **tetap mengirim data di bawah air**
  (catatan di `banyu_defaults.parm`). Sim Task 2 jalan dengan `use_gps` default `True`.
- Sim tidak menjalankan DVL secara default (`use_dvl:=False`), tapi GPS sim yang bocor ke bawah air
  menutupi kekurangan itu.

Jadi hasil sim Task 2 saat ini **tidak** membuktikan Task 2 bisa jalan di robot.

## Yang tidak boleh dipakai sebagai solusi

- **`nogps_odom_beacon` di bawah air untuk GUIDED.** Beacon mengirim perpindahan nol, jadi EKF
  selalu menganggap robot diam (kecepatan 0). Kontrol kecepatan GUIDED membandingkan setpoint
  dengan kecepatan terukur 0, sehingga terus menambah dorongan. Robot bisa lepas kendali. POR aman
  karena tidak memakai GUIDED (ALT_HOLD + RC override, kecepatan open-loop).

## Opsi untuk dibahas

| Opsi | Isi | Kelebihan | Kekurangan / kerja |
|---|---|---|---|
| A. GUIDED di permukaan, ALT_HOLD di bawah air | Transit GPS di permukaan tetap GUIDED. Setelah dive, pindah ke ALT_HOLD + RC override seperti POR (`control.output_mode: rc_override`). Kedalaman dari Bar30, heading dari kompas, follow pipeline dari kamera bawah | Memakai jalur yang sudah teruji di POR; tidak butuh DVL | Perlu ganti mode di tengah misi (BT + `vehicle_execution`), kalibrasi gain stick untuk aksi pipeline, tes sim dengan GPS dimatikan di bawah air |
| B. Pasang DVL | Source set 2 dapat kecepatan asli, GUIDED tetap jalan | Arsitektur sekarang tidak berubah (jalur DVL sudah ada di sim) | Butuh hardware DVL (tim tidak punya) |
| C. Task 2 hanya permukaan / ditunda | Fokus POR; Task 2 di robot tidak dijalankan di bawah air | Tidak ada kerja kode | Task 2 tidak bisa dikerjakan penuh |

## Yang perlu dites sebelum memilih A

1. Sim Task 2 dengan GPS SITL mati di bawah air (misalnya `SIM_GPS1_ENABLE 0` begitu menyelam,
   atau `use_gps:=False` + transit tanpa GPS) untuk memastikan sim memperlihatkan kegagalan ini.
2. Prototipe pergantian GUIDED → ALT_HOLD setelah `DiveToDepth`, lalu `FollowPipeline` dengan
   RC override.
3. Kalibrasi kecepatan stick di kolam (sama seperti POR, `docs/por/tuning.md`).
