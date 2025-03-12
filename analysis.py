import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal  # 频域分析
from scipy.spatial.transform import Rotation  # 四元数转欧拉角
import plotly.express as px  # 交互式可视化

df = pd.read_csv('output.csv')
df = df.iloc[2:].reset_index(drop=True)
# 处理时间戳（单位为毫秒）
df['timestamp'] = pd.to_datetime(df['timestamp'], unit='ms')


# 将四元数转换为欧拉角（弧度）
def quaternion_to_euler(x, y, z, w):
    rotation = Rotation.from_quat([x, y, z, w])
    return rotation.as_euler('xyz')  # 返回 roll, pitch, yaw


# 为原始和滤波数据添加欧拉角列
df[['imu_roll', 'imu_pitch', 'imu_yaw']] = df.apply(
    lambda row: quaternion_to_euler(row['imu_quaternion_x'], row['imu_quaternion_y'],
                                    row['imu_quaternion_z'], row['imu_quaternion_w']), axis=1, result_type='expand'
)

df[['kf_roll', 'kf_pitch', 'kf_yaw']] = df.apply(
    lambda row: quaternion_to_euler(row['kf_quaternion_x'], row['kf_quaternion_y'],
                                    row['kf_quaternion_z'], row['kf_quaternion_w']), axis=1, result_type='expand'
)

# 可视化姿态角对比
plt.figure(figsize=(14, 8))
plt.plot(df['timestamp'], df['imu_roll'], label='IMU Roll', alpha=0.7)
plt.plot(df['timestamp'], df['kf_roll'], label='KF Roll', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Roll (rad)')
plt.legend()
plt.show()

# 计算加速度统计量
acc_stats = df[[
    'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z',
    'kf_acceleration_x', 'kf_acceleration_y', 'kf_acceleration_z'
]].agg(['mean', 'std', 'min', 'max'])

print("加速度统计量:\n", acc_stats)

# 绘制 X 加速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_acceleration_x'], label='IMU X')
plt.plot(df['timestamp'], df['kf_acceleration_x'], label='KF X')
plt.xlabel('Time')
plt.ylabel('X Acceleration (m/s²)')
plt.legend()
plt.title('X Acceleration Comparison')
plt.show()

# 绘制 Y 加速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_acceleration_y'], label='IMU Y')
plt.plot(df['timestamp'], df['kf_acceleration_y'], label='KF Y')
plt.xlabel('Time')
plt.ylabel('Y Acceleration (m/s²)')
plt.legend()
plt.title('Y Acceleration Comparison')
plt.show()

# 绘制 Z 加速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_acceleration_z'], label='IMU Z')
plt.plot(df['timestamp'], df['kf_acceleration_z'], label='KF Z')
plt.xlabel('Time')
plt.ylabel('Z Acceleration (m/s²)')
plt.legend()
plt.title('Z Acceleration Comparison')
plt.show()

# 绘制 X 角速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_angular_velocity_x'], label='IMU X')
plt.plot(df['timestamp'], df['kf_angular_velocity_x'], label='KF X')
plt.xlabel('Time')
plt.ylabel('X Angular Velocity (rad/s)')
plt.legend()
plt.title('X Angular Velocity Comparison')
plt.show()

# 绘制 Y 角速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_angular_velocity_y'], label='IMU Y')
plt.plot(df['timestamp'], df['kf_angular_velocity_y'], label='KF Y')
plt.xlabel('Time')
plt.ylabel('Y Angular Velocity (rad/s)')
plt.legend()
plt.title('Y Angular Velocity Comparison')
plt.show()

# 绘制 Z 角速度时间序列
plt.figure(figsize=(14, 10))
plt.plot(df['timestamp'], df['imu_angular_velocity_z'], label='IMU Z')
plt.plot(df['timestamp'], df['kf_angular_velocity_z'], label='KF Z')
plt.xlabel('Time')
plt.ylabel('Z Angular Velocity (rad/s)')
plt.legend()
plt.title('Z Angular Velocity Comparison')
plt.show()

# 基于加速度和角速度的幅值检测运动状态
df['acc_magnitude'] = np.sqrt(
    df['imu_acceleration_x']**2 +
    df['imu_acceleration_y']**2 +
    df['imu_acceleration_z']**2
)

df['gyro_magnitude'] = np.sqrt(
    df['imu_angular_velocity_x']**2 +
    df['imu_angular_velocity_y']**2 +
    df['imu_angular_velocity_z']**2
)
