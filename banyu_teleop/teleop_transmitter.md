# Ide/Konsep Teleop menggunakan transmitter

## Komunikasi

TX16S --- Receiver (RS) --- Pixhawk --- MAVROS RCIN --- Package Teleop

## Button
- Axes/Gimbal Kiri horizontal ---> sway
- Axes/Gimbal Kiri Vertical ---> surge
- Axes/Gimbal Kanan horizontal ---> Yaw
- Axes/Gimbal Kanan Vertical ---> depth/naik turun

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
Dikontroll menggunakan SA atau channel 5
900 - stabilize
1500 - manual (default)
2000 - depth hold

### Mode Kill Switch
kill switch menggunakan esp32 yang menerima perintah serial 0/1/2
900 - 0 --- autonomous
1500 - 1 --- remote
2000 - 2 --- kill

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
