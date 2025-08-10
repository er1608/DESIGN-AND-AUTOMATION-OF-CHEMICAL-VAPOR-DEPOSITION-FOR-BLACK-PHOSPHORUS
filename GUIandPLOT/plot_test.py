import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv("20250608_112623.csv",skipinitialspace=True, na_values="NULL")

# Chuyển cột timestamp thành datetime
df["timestamp"] = pd.to_datetime(df["timestamp"])

# Chỉ lấy các hàng có giá trị temperature không bị thiếu
df = df.dropna(subset=["temperature"])

# Vẽ biểu đồ
plt.figure(figsize=(10, 5))
plt.plot(df["timestamp"], df["temperature"], marker='o', linestyle='-')
plt.xlabel("Timestamp")
plt.ylabel("Temperature (°C)")
plt.title("Temperature over Time")
plt.grid(True)
plt.tight_layout()
plt.show()
