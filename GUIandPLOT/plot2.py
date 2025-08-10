import pandas as pd
import matplotlib.pyplot as plt

# Đọc dữ liệu CSV
df = pd.read_csv("100NoK.csv", skipinitialspace=True, na_values="NULL")

# Chuyển timestamp (ms) thành giây tương đối
df['seconds'] = df['timestamp'] / 1000
df['relative_seconds'] = df['seconds'] - df['seconds'].iloc[0]

# Vẽ biểu đồ
plt.figure(figsize=(14, 6))  # Tăng kích thước ngang nếu muốn

# Vẽ nhiệt độ thực tế
plt.plot(df['relative_seconds'], df['temperature'], 
         marker='.', linestyle='-', linewidth=2, label='Nhiệt độ (°C)')

# Vẽ nhiệt độ đặt
plt.plot(df['relative_seconds'], df['setpoint'], 
         linestyle='-', linewidth=2, label='Setpoint (°C)', color='orange')

# Tùy chỉnh biểu đồ
plt.title("A")
plt.xlabel("Thời gian (s)")
plt.ylabel("Nhiệt độ (°C)")
plt.grid(True)
plt.legend()  # Thêm chú thích
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
