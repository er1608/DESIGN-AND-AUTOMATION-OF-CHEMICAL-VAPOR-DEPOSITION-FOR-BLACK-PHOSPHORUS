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
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from collections import deque
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

        current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = Path(fr"C:\\Users\\pc\\OneDrive\\Documents\\Tài Liệu + Thực hành\\HK8\\KLTN\\GUI_AND_DATABASE\\{current_time_str}.csv")
        self.csv_file.parent.mkdir(parents=True, exist_ok=True)

        if not self.csv_file.exists():
            try:
                with open(self.csv_file, "w") as file:
                    file.write("timestamp, temperature, setpoint\n")
                print(f"[INIT] Tạo file CSV tại: {self.csv_file}")
            except Exception as e:
                print(f"[INIT] Lỗi tạo file CSV: {e}")

        self.temp_data = deque()
        self.setpoint_data = deque()
        self.time_data = deque()

    def initUI(self):
        self.setWindowTitle("STM32 Serial Monitor")
        self.setGeometry(100, 100, 1000, 500)
        self.setStyleSheet("background-color: #303030; color: white;")
        main_layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        control_layout = QVBoxLayout()

        logo_layout = QHBoxLayout()
        logo1 = QLabel()
        logo1.setContentsMargins(30, 0, 0, 0)
        logo1.setPixmap(QPixmap(os.path.join(base_path, "CNTT.png")).scaled(55, 55, Qt.AspectRatioMode.KeepAspectRatio))
        logo_layout.addWidget(logo1)
        logo2 = QLabel()
        logo2.setPixmap(QPixmap(os.path.join(base_path, "KTMT.png")).scaled(50, 50, Qt.AspectRatioMode.KeepAspectRatio))
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

        self.status_label = QLabel("Trạng thái:")
        control_layout.addWidget(self.status_label)

        status_layout = QHBoxLayout()
        self.led_labels = []
        for _ in range(5):
            led = QLabel("🔲")
            led.setAlignment(Qt.AlignmentFlag.AlignCenter)
            led.setStyleSheet("font-size: 24px;")
            self.led_labels.append(led)
            status_layout.addWidget(led)
        control_layout.addLayout(status_layout)

        mini_display_layout = QHBoxLayout()
        self.temp_display = QLabel("Temp: --- °C")
        self.spi_display = QLabel("SPI: --- °C")
        self.set_display = QLabel("Set: --- °C")
        for label in [self.temp_display, self.spi_display, self.set_display]:
            label.setStyleSheet("font-size: 14px; background-color: #505050; padding: 6px; border-radius: 6px;")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            mini_display_layout.addWidget(label)
        control_layout.addLayout(mini_display_layout)

        left_layout.addLayout(control_layout)
        left_layout.addStretch()

        main_layout.addLayout(left_layout)

        right_layout = QVBoxLayout()
        self.plot_canvas = FigureCanvas(Figure())
        self.ax = self.plot_canvas.figure.add_subplot(111)
        self.ax.set_title("Real-time Temperature Plot")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Temperature (°C)")
        self.temp_line, = self.ax.plot([], [], 'r-', label="Temperature")
        self.set_line, = self.ax.plot([], [], 'g-', label="Setpoint")
        self.ax.legend()
        right_layout.addWidget(self.plot_canvas)

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
        self.temp_display.setText("Temp: --- °C")
        self.spi_display.setText("SPI: --- °C")
        self.set_display.setText("Set: --- °C")
        self.counter = 0
        self.temp_data.clear()
        self.setpoint_data.clear()
        self.time_data.clear()
        self.send_signal("3")

    # def update_plot(self):
    #     self.temp_line.set_data(range(len(self.temp_data)), self.temp_data)
    #     self.set_line.set_data(range(len(self.setpoint_data)), self.setpoint_data)
    #     self.ax.relim()
    #     self.ax.autoscale_view()
    #     self.plot_canvas.draw()

    def update_plot(self):
        # Dữ liệu x tự động tăng theo số mẫu
        x_vals = list(range(len(self.temp_data)))
        
        # Cập nhật dữ liệu vào biểu đồ
        self.temp_line.set_data(x_vals, self.temp_data)
        self.set_line.set_data(x_vals, self.setpoint_data)

        # Trượt trục X theo 100 điểm gần nhất
        if len(x_vals) >= 100:
            x_min = len(x_vals) - 100
            x_max = len(x_vals)
        else:
            x_min = 0
            x_max = 100

        self.ax.set_xlim(x_min, x_max)

        # Chỉ autoscale trục Y (nhiệt độ)
        self.ax.relim()
        self.ax.autoscale_view(scalex=True, scaley=True)

        self.plot_canvas.draw()

    def read_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                data = self.serial_port.readline().decode().strip()
                updated = False

                if "Temperature:" in data:
                    value = float(data.split("Temperature:")[1].strip())
                    self.temp_display.setText(f"Temp: {value:.1f} °C")
                    self.last_temperature = value
                    updated = True

                elif "Set:" in data:
                    value = float(data.split("Set:")[1].strip())
                    self.set_display.setText(f"Set: {value:.1f} °C")
                    self.last_setpoint = value

                elif "Temperature_SPI:" in data:
                    value = float(data.split("Temperature_SPI:")[1].strip())
                    self.spi_display.setText(f"SPI: {value:.1f} °C")
                    self.last_spi = value

                elif data.isdigit():
                    index = int(data) - 1
                    if 0 <= index < 5:
                        for i in range(5):
                            self.led_labels[i].setText("🟩" if i == index else "🔲")

                if updated:
                    timestamp = int(datetime.now().timestamp() * 1000)
                    temp = getattr(self, 'last_temperature', 0)
                    setp = getattr(self, 'last_setpoint', 0)
                    self.temp_data.append(temp)
                    self.setpoint_data.append(setp)
                    self.time_data.append(timestamp)
                    self.update_plot()
                    with open(self.csv_file, "a") as file:
                        spi = getattr(self, 'last_spi', "NULL")
                        file.write(f"{timestamp}, {temp}, {setp}\n")

            except Exception as e:
                print(f"Lỗi đọc dữ liệu: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = STM32GUI()
    gui.show()
    sys.exit(app.exec())