import sys
import serial
import serial.tools.list_ports
from datetime import datetime
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox
)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer, Qt
import os
from pathlib import Path

base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))

class STM32GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.serial_port = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_serial)
        self.counter = 0

        # Lưu file CSV vào đường dẫn cụ thể do người dùng chỉ định
        current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = Path(fr"C:\Users\pc\OneDrive\Documents\Tài Liệu + Thực hành\HK8\KLTN\GUI_AND_DATABASE\{current_time_str}.csv")
        self.csv_file.parent.mkdir(parents=True, exist_ok=True)

        # Ghi header nếu file chưa tồn tại
        if not self.csv_file.exists():
            try:
                with open(self.csv_file, "w") as file:
                    file.write("timestamp, temperature\n")
                print(f"[INIT] Tạo file CSV tại: {self.csv_file}")
            except Exception as e:
                print(f"[INIT] Lỗi tạo file CSV: {e}")

    def initUI(self):
        self.setWindowTitle("STM32 Serial Monitor")
        self.setGeometry(100, 100, 800, 400)
        self.setStyleSheet("background-color: #303030; color: white;")
        main_layout = QHBoxLayout()

        # Left layout
        left_layout = QVBoxLayout()
        control_layout = QVBoxLayout()

        logo_layout = QHBoxLayout()
        logo1 = QLabel()
        logo1.setContentsMargins(30, 0, 0, 0)
        logo1_path = os.path.join(base_path, "CNTT.png")
        logo1.setPixmap(QPixmap(logo1_path).scaled(55, 55, Qt.AspectRatioMode.KeepAspectRatio))
        logo_layout.addWidget(logo1)
        logo2 = QLabel()
        logo2_path = os.path.join(base_path, "KTMT.png")
        logo2.setPixmap(QPixmap(logo2_path).scaled(50, 50, Qt.AspectRatioMode.KeepAspectRatio))

        logo_layout.addWidget(logo2)
        control_layout.addLayout(logo_layout)

        self.port_label = QLabel("Cổng COM:")
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.connect_button = QPushButton("Kết nối")
        self.connect_button.setStyleSheet("background-color: white; color: black;")
        self.connect_button.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.port_label)
        control_layout.addWidget(self.port_combo)
        control_layout.addWidget(self.connect_button)

        self.send_button = QPushButton("Bắt đầu")
        self.send_button.setStyleSheet("background-color: green; color: white;")
        self.send_button.clicked.connect(lambda: self.send_signal("1"))
        control_layout.addWidget(self.send_button)

        self.stop_button = QPushButton("Kết thúc")
        self.stop_button.setStyleSheet("background-color: brown; color: white;")
        self.stop_button.clicked.connect(lambda: self.send_signal("2"))
        control_layout.addWidget(self.stop_button)

        self.reset_button = QPushButton("Khởi động lại")
        self.reset_button.setStyleSheet("background-color: orange; color: black;")
        self.reset_button.clicked.connect(self.reset_values)
        control_layout.addWidget(self.reset_button)

        left_layout.addLayout(control_layout)

        self.status_label = QLabel("Trạng thái:")
        left_layout.addWidget(self.status_label)

        status_layout = QHBoxLayout()
        self.led_labels = []
        for _ in range(5):
            led = QLabel("🔲")
            led.setAlignment(Qt.AlignmentFlag.AlignCenter)
            led.setStyleSheet("font-size: 24px;")
            self.led_labels.append(led)
            status_layout.addWidget(led)
        left_layout.addLayout(status_layout)
        left_layout.addStretch()
        main_layout.addLayout(left_layout)

        # Right layout
        right_layout = QVBoxLayout()
        self.temp_display = QLabel("Temperature: --- °C")
        self.spi_display = QLabel("Temperature_SPI: --- °C")
        self.set_display = QLabel("Setpoint: --- °C")

        for label in [self.temp_display, self.spi_display, self.set_display]:
            label.setStyleSheet("font-size: 20px; background-color: #505050; padding: 12px; border-radius: 8px; margin-bottom: 10px;")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            right_layout.addWidget(label)

        main_layout.addLayout(right_layout, 3)
        self.setLayout(main_layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connect_button.setText("Kết nối")
            self.status_label.setText("Trạng thái: Chưa kết nối")
            for led in self.led_labels:
                led.setText("🔲")
            self.timer.stop()
        else:
            selected_port = self.port_combo.currentText()
            try:
                self.serial_port = serial.Serial(selected_port, 115200, timeout=1)
                self.connect_button.setText("Ngắt kết nối")
                self.status_label.setText(f"Trạng thái: Đã kết nối {selected_port}")
                self.timer.start(100)
            except Exception as e:
                print(f"Lỗi kết nối: {e}")
                self.status_label.setText("Trạng thái: Kết nối thất bại")

    def send_signal(self, signal: str):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(signal.encode())
            except Exception as e:
                print(f"Lỗi khi gửi tín hiệu: {e}")
        else:
            print("Cổng UART chưa được kết nối.")

    def reset_values(self):
        self.temp_display.setText("Temperature: --- °C")
        self.spi_display.setText("Temperature_SPI: --- °C")
        self.set_display.setText("Setpoint: --- °C")
        self.counter = 0
        self.send_signal("3")

    def read_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                data = self.serial_port.readline().decode().strip()
                updated = False

                if "Temperature:" in data:
                    value = float(data.split("Temperature:")[1].strip())
                    self.temp_display.setText(f"Temperature: {value:.1f} °C")
                    self.last_temperature = value
                    updated = True

                elif "Set:" in data:
                    value = float(data.split("Set:")[1].strip())
                    self.set_display.setText(f"Setpoint: {value:.1f} °C")
                    self.last_setpoint = value

                elif "Temperature_SPI:" in data:
                    value = float(data.split("Temperature_SPI:")[1].strip())
                    self.spi_display.setText(f"Temperature_SPI: {value:.1f} °C")
                    self.last_spi = value

                elif data.isdigit():
                    index = int(data) - 1
                    if 0 <= index < 5:
                        for i in range(5):
                            self.led_labels[i].setText("🟩" if i == index else "🔲")

                if updated:
                    timestamp = int(datetime.now().timestamp() * 1000)
                    # timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    temperature = getattr(self, 'last_temperature', "NULL")
                    spi = getattr(self, 'last_spi', "NULL")
                    with open(self.csv_file, "a") as file:
                        file.write(f"{timestamp}, {temperature}, {spi}\n")

            except Exception as e:
                print(f"Lỗi đọc dữ liệu: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = STM32GUI()
    gui.show()
    sys.exit(app.exec())
