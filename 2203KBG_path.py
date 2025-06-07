#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
import csv
from mpl_toolkits.mplot3d import Axes3D  # 添加3D投影支持
# 文字和字体设置
text = "2203KBG"
font = FontProperties(family='DejaVu Sans', size=160)  # 大字体确保清晰

# 提取字符路径
paths = []
total_width = 0
for ch in text:
    tp = TextPath((0, 0), ch, prop=font)
    width = tp.get_extents().width
    paths.append((tp, width))
    total_width += width

# 圆弧形分布控制
R = 800.0  # 减小半径使拱形更明显
center_offset = total_width / 2.0

# 保存轨迹点
coords = []

# 起始点坐标
start_x, start_y, start_z = 432, 150, 1038

# 起始 X
current_x = 0
for tp, width in paths:
    verts = tp.vertices.copy()
    verts[:, 0] += current_x - center_offset
    current_x += width

    for x, y in verts:
        # 计算圆弧的切向角度（绕Y轴旋转的角度）
        theta = np.arctan2(x, np.sqrt(max(R**2 - x**2, 1e-6)))
        b_angle = -np.degrees(theta)  # 转换为角度
        
        # 下拱效果：在Z轴方向弯曲
        dz = R - np.sqrt(max(R**2 - x**2, 0))
        X = x + start_x
        Y = y + start_y
        Z = start_z - dz  # 下拱：中心高，两侧低
        
        # 姿态设置：匹配初始姿态 A:180, C:180，B角根据路径调整
        a, b, c = 180.0, b_angle, 180.0
        coords.append([X, Y, Z, a, b, c])

# 保存为 CSV 文件
with open("2203KBG_wrapped_trajectory.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(coords)

# 用 Matplotlib 可视化路径
coords_np = np.array(coords)
fig = plt.figure(figsize=(15, 10))

# XY平面视图
ax1 = fig.add_subplot(221)
ax1.plot(coords_np[:, 0], coords_np[:, 1], 'r-', linewidth=2)
ax1.set_title("XY Plane View")
ax1.set_xlabel("X (mm)")
ax1.set_ylabel("Y (mm)")
ax1.grid(True)
ax1.axis('equal')

# XZ平面视图（显示拱形）
ax2 = fig.add_subplot(222)
ax2.plot(coords_np[:, 0], coords_np[:, 2], 'b-', linewidth=2)
ax2.set_title("XZ Plane View (Arch)")
ax2.set_xlabel("X (mm)")
ax2.set_ylabel("Z (mm)")
ax2.grid(True)
ax2.axis('equal')

# 3D视图
ax3 = fig.add_subplot(223, projection='3d')
ax3.plot(coords_np[:, 0], coords_np[:, 1], coords_np[:, 2], 'g-', linewidth=2)
ax3.set_title("3D Trajectory View")
ax3.set_xlabel("X (mm)")
ax3.set_ylabel("Y (mm)")
ax3.set_zlabel("Z (mm)")
ax3.grid(True)

# 姿态可视化（B角变化）
ax4 = fig.add_subplot(224)
ax4.plot(coords_np[:, 0], coords_np[:, 4], 'm-', linewidth=2)
ax4.set_title("B Angle Variation Along Path")
ax4.set_xlabel("X (mm)")
ax4.set_ylabel("B Angle (degrees)")
ax4.grid(True)

plt.tight_layout()
plt.show()

print(f"轨迹已生成并保存为 '2203KBG_wrapped_trajectory.csv'")
print(f"起点坐标: X={coords[0][0]:.2f}, Y={coords[0][1]:.2f}, Z={coords[0][2]:.2f}")
print(f"起点姿态: A={coords[0][3]}, B={coords[0][4]:.2f}, C={coords[0][5]}")


# In[ ]:




