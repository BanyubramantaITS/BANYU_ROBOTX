# Ide/Konsep Teleop menggunakan transmitter

## Komunikasi

TX16S --- Receiver (RS) --- Pixhawk --- MAVROS RCIN --- Package Teleop

## Button
Hasil cek di robot (Oktober 2026, QGC Radio + parameter Pixhawk):
- Gimbal kanan vertikal ---> surge / maju-mundur (CH2, `RCMAP_FORWARD 2`)
- Gimbal kanan horizontal ---> sway / geser (CH1, `RCMAP_LATERAL 1`)
- Gimbal kiri vertikal ---> depth / naik-turun (CH3, `RCMAP_THROTTLE 3`)
- Gimbal kiri horizontal ---> yaw (CH4, `RCMAP_YAW 4`)
- SB ---> kill / manual / autonomous (CH6), SC ---> mode terbang (CH8)

`RCMAP_PITCH` dan `RCMAP_ROLL` = 0 (tidak dipakai). Profil POR mengirim RC override ke kanal
yang sama (2/1/3/4); lihat `docs/por/README.md` §4.2.

## Data yang dibaca receiver dari transmitter TX16S di RC IN
```header:
  stamp:
    sec: 1792071768
    nanosec: 154560256
  frame_id: ''
rssi: 255
channels:
- 1503
- 1509
- 1500
- 1506
- 999
- 1503
- 1503
- 1503
- 1503
- 1503
- 1503
- 1503
- 880
- 880
- 2011
- 2011
---
```

## ROV
- Menggunakan 8 Thruster
- Frame: Vectored 6DOF
- Menggunakan ArduSub

### Mode flight Controller
Dikontrol menggunakan SC (channel 8 / index 7):
- 900: stabilize
- 1500: manual (default)
- 2000: depth hold (alt_hold)

Catatan: SC hanya aktif saat SB berada di posisi manual atau disarmed. Saat autonomous mode, input pergantian mode dari SC di-reject dan akan disinkronkan saat kembali ke manual.

### Mode Kill Switch & Arming
Kill switch ESP32 dan arming Pixhawk di-unify pada saklar SB (channel 6 / index 5):
- 900 (Up): Autonomous mode (ESP32 serial '2', operational_mode = 2) + otomatis arm Pixhawk (jika belum armed). Mengunci gimbals dan SC mode change.
- 1500 (Mid): Manual mode (ESP32 serial '1', operational_mode = 1) + otomatis arm Pixhawk. Mengizinkan kontrol gimbal dan SC mode switch.
- 2000 (Down): Disarmed / Kill mode (ESP32 serial '0', operational_mode = 0) + disarm Pixhawk.

Catatan: Saklar SF di-unmap dan tidak digunakan lagi.

### Prepare autonomy (SH)
SH (momentary, default CH5 / index 4, `prepare_channel` di `config/teleop_rcin.yaml`): ditekan
(-100 -> +100) saat SB Up memanggil `/banyu/mission/prepare_autonomy` (Task 2: datum kedalaman dan
run window baru). Ditolak kalau SB tidak Up; mission menolak saat run sedang berjalan. Hasilnya ada di
log `banyu_teleop`. Tekan hanya saat wahana mengapung bebas (datum diambil saat itu). Pixhawk
`RC5_OPTION` harus 0.

contoh program ros2:
```python
import rclpy
from rclpy.node import Node
import serial
import sys
import threading

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baud').value

        try:
            self.esp_serial = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Berhasil konek ke ESP32 di {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Gagal buka port {port}: {e}")
            sys.exit(1)

        # Timer untuk terus-menerus mengecek balasan dari ESP32 setiap 0.1 detik
        self.timer = self.create_timer(0.1, self.read_from_esp32)

        self.input_thread = threading.Thread(target=self.read_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def read_from_esp32(self):
        # Kalau ada data masuk dari ESP32, baca dan print di terminal ROS
        if self.esp_serial.in_waiting > 0:
            try:
                # Membaca data satu baris penuh
                data = self.esp_serial.readline().decode('utf-8').strip()
                if data:
                    self.get_logger().info(f"[MONITOR] {data}")
            except Exception as e:
                pass

    def read_input(self):
        self.get_logger().info("Ketik 0, 1, atau 2 lalu tekan Enter. Ketik 'q' untuk keluar.")
        while rclpy.ok():
            try:
                user_input = input("\n")
                if user_input.lower() == 'q':
                    self.get_logger().info("Program dimatikan.")
                    rclpy.shutdown()
                    break

                if user_input in ['0', '1', '2']:
                    self.esp_serial.write(user_input.encode())
                else:
                    self.get_logger().warn("Input salah! Cuma bisa 0, 1, atau 2.")
            except EOFError:
                break

    def destroy_node(self):
        if hasattr(self, 'esp_serial') and self.esp_serial.is_open:
            self.esp_serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

ya kita buat dengan cpp kode nya dan **menggunakan button SB atau channel 6**:
