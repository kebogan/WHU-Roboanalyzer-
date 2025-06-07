#!/usr/bin/env python
# coding: utf-8

# In[17]:


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ======================
# 1. 精确直线五角星轨迹
# ======================
def generate_star(center, R_outer, R_inner, num_segments=100):
    """生成精确的直线五角星轨迹"""
    # 顶点角度定义 (逆时针)
    outer_angles = np.deg2rad([0, 72, 144, 216, 288])
    inner_angles = np.deg2rad([36, 108, 180, 252, 324])
    
    # 顶点坐标计算
    vertices = []
    for i in range(5):
        # 外顶点
        x_outer = center[0] + R_outer * np.cos(outer_angles[i])
        y_outer = center[1] + R_outer * np.sin(outer_angles[i])
        vertices.append([x_outer, y_outer, center[2]])
        
        # 内顶点
        x_inner = center[0] + R_inner * np.cos(inner_angles[i])
        y_inner = center[1] + R_inner * np.sin(inner_angles[i])
        vertices.append([x_inner, y_inner, center[2]])
    
    # 闭合轨迹 (回到起点)
    vertices.append(vertices[0])
    
    # 直线段插值
    trajectory = []
    for i in range(10):  # 10条边
        start = np.array(vertices[i])
        end = np.array(vertices[i+1])
        
        # 线性插值生成直线段
        for j in range(num_segments):
            ratio = j / num_segments
            point = start * (1 - ratio) + end * ratio
            trajectory.append(point)
    
    return np.array(trajectory), np.array(vertices)

# ======================
# 2. PUMA560逆运动学
# ======================
def puma560_ik(x, y, z):
    """PUMA560逆运动学求解"""
    # D-H参数 (单位: m)
    a2, a3, d3, d4 = 0.4318, 0.0203, 0.150, 0.4318
    
    # 关节1求解
    theta1 = np.arctan2(y, x)
    
    # 计算径向距离和高度
    r = np.sqrt(x**2 + y**2)
    s = z - d3
    
    # 几何关系
    D = (r**2 + s**2 - a2**2 - a3**2 - d4**2) / (2 * a2 * np.sqrt(a3**2 + d4**2))
    D = np.clip(D, -1.0, 1.0)  # 确保在有效范围内
    
    # 选择肘部朝上的解
    phi = np.arctan2(d4, a3)
    theta3 = np.arcsin(D) - phi
    
    # 关节2计算
    k1 = a2 + np.sqrt(a3**2 + d4**2) * np.cos(theta3 + phi)
    k2 = d3 + np.sqrt(a3**2 + d4**2) * np.sin(theta3 + phi)
    theta2 = np.arctan2(s, r) - np.arctan2(k2, k1)
    
    # 后三个关节保持固定姿态 (垂直向上)
    theta4 = 0.0
    theta5 = np.pi/2  # 保持垂直
    theta6 = 0.0
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]

# ======================
# 3. 运动学计算和导出
# ======================
def generate_roboanalyzer_data():
    """生成RoboAnalyzer所需的19列数据"""
    # 参数设置
    center = np.array([0.4, 0.0, 0.5])  # 工作平面中心
    R_outer = 0.25  # 外接圆半径 (减小以确保在工作空间内)
    R_inner = 0.1   # 内接圆半径
    total_segments = 1000  # 总轨迹点数 (10的倍数)
    total_time = 10.0      # 总运动时间 (秒)
    cruise_speed = 0.15    # 巡航速度 (m/s)
    accel_frac = 0.15      # 加速段比例
    
    # 生成五角星轨迹
    trajectory, vertices = generate_star(center, R_outer, R_inner, 
                                        num_segments=total_segments//10)
    
    # 计算时间步长
    time_step = total_time / total_segments
    
    # 预分配关节数据数组
    joint_positions = np.zeros((total_segments, 6))  # 6个关节的位置 (弧度)
    joint_velocities = np.zeros((total_segments, 6)) # 6个关节的速度 (弧度/秒)
    joint_accelerations = np.zeros((total_segments, 6)) # 6个关节的加速度 (弧度/秒²)
    
    # 计算每个轨迹点的关节位置
    for i in range(total_segments):
        pos = trajectory[i]
        joints = puma560_ik(pos[0], pos[1], pos[2])
        joint_positions[i] = joints
    
    # 计算关节速度和加速度 (中心差分法)
    for j in range(6):  # 遍历6个关节
        # 速度计算 (中心差分)
        for i in range(1, total_segments-1):
            joint_velocities[i, j] = (joint_positions[i+1, j] - joint_positions[i-1, j]) / (2 * time_step)
        
        # 端点处理 (前向/后向差分)
        joint_velocities[0, j] = (joint_positions[1, j] - joint_positions[0, j]) / time_step
        joint_velocities[-1, j] = (joint_positions[-1, j] - joint_positions[-2, j]) / time_step
        
        # 加速度计算 (中心差分)
        for i in range(1, total_segments-1):
            joint_accelerations[i, j] = (joint_positions[i+1, j] - 2*joint_positions[i, j] + joint_positions[i-1, j]) / (time_step**2)
        
        # 端点处理
        joint_accelerations[0, j] = (joint_positions[2, j] - 2*joint_positions[1, j] + joint_positions[0, j]) / (time_step**2)
        joint_accelerations[-1, j] = (joint_positions[-1, j] - 2*joint_positions[-2, j] + joint_positions[-3, j]) / (time_step**2)
    
    # 生成19列的CSV文件
    with open('puma560_star_19cols.csv', 'w') as f:
        # 写入文件头
        headers = ["Time(sec)"]
        for j in range(1, 7):
            headers.extend([
                f"Joint{j}Pos(deg)",
                f"Joint{j}Vel(deg/s)",
                f"Joint{j}Acc(deg/s2)"
            ])
        f.write(",".join(headers) + "\n")
        
        # 写入数据行
        for i in range(total_segments):
            t = i * time_step
            row = [f"{t:.4f}"]
            
            # 添加6个关节的3个数据 (位置,速度,加速度)
            for j in range(6):
                # 转换为角度单位
                pos_deg = np.rad2deg(joint_positions[i, j])
                vel_deg = np.rad2deg(joint_velocities[i, j])
                acc_deg = np.rad2deg(joint_accelerations[i, j])
                
                row.extend([
                    f"{pos_deg:.6f}",
                    f"{vel_deg:.6f}",
                    f"{acc_deg:.6f}"
                ])
            
            f.write(",".join(row) + "\n")
    
    return trajectory, joint_positions, joint_velocities, joint_accelerations

# ======================
# 4. 执行主程序
# ======================
if __name__ == "__main__":
    # 生成RoboAnalyzer数据
    traj, joint_pos, joint_vel, joint_acc = generate_roboanalyzer_data()
    
    # 可视化验证
    plt.figure(figsize=(15, 10))
    
    # 轨迹图
    plt.subplot(2, 2, 1)
    plt.plot(traj[:, 0], traj[:, 1])
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('XY平面轨迹')
    plt.axis('equal')
    plt.grid(True)
    
    # 关节位置
    plt.subplot(2, 2, 2)
    for j in range(6):
        plt.plot(np.rad2deg(joint_pos[:, j]), label=f'Joint {j+1}')
    plt.xlabel('时间步')
    plt.ylabel('关节角度 (度)')
    plt.title('关节位置')
    plt.legend()
    plt.grid(True)
    
    # 关节速度
    plt.subplot(2, 2, 3)
    for j in range(6):
        plt.plot(np.rad2deg(joint_vel[:, j]), label=f'Joint {j+1}')
    plt.xlabel('时间步')
    plt.ylabel('关节速度 (度/秒)')
    plt.title('关节速度')
    plt.legend()
    plt.grid(True)
    
    # 关节加速度
    plt.subplot(2, 2, 4)
    for j in range(6):
        plt.plot(np.rad2deg(joint_acc[:, j]), label=f'Joint {j+1}')
    plt.xlabel('时间步')
    plt.ylabel('关节加速度 (度/秒²)')
    plt.title('关节加速度')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('joint_data_validation.png', dpi=300)
    plt.show()
    
    print("="*60)
    print("RoboAnalyzer 19列数据文件已生成: puma560_star_19cols.csv")
    print("文件格式: Time(sec), Joint1Pos, Joint1Vel, Joint1Acc, ... Joint6Pos, Joint6Vel, Joint6Acc")
    print("="*60)


# In[11]:





# In[12]:





# In[ ]:




