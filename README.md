第一个五角星仿真较为成功。
但第二个学号姓名的仿真利用基于文本路径的轨迹规划方法，将文本字符的轮廓点提取出来，然后进行空间变换（主要是弯曲成弧形）生成机器人末端轨迹，但最终机器人描述轨迹时始终无法正视于屏幕，作者尚未找到解决方案。
利用我的两个python文件导出csv文件时，记得删除第一行的变量名再保存导入到roboanalyzer中进行逆运动求解。
姓名学号可在python文件中的一行直接进行修改，操作十分简便。
![image](https://github.com/user-attachments/assets/19948ae1-9f90-4677-a0d1-a6ea060e9e8c)
![image](https://github.com/user-attachments/assets/b7dfb9d6-11c2-42ee-ac54-6d9ca8ce280b)
结果如上图所示
