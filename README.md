第一个五角星仿真较为成功。

第一部分操作：点击下方软件中File input位置输入19*n维数据csv文件，点击右上角fkin和播放按钮进行运动规划

但第二个学号姓名的仿真利用基于文本路径的轨迹规划方法，将文本字符的轮廓点提取出来，然后进行空间变换（主要是弯曲成弧形）生成机器人末端轨迹，但最终机器人描述轨迹时始终无法正视于屏幕，作者尚未找到解决方案。
利用两个python文件导出csv文件时，记得删除第一行的变量名再保存导入到roboanalyzer中进行逆运动求解。（注：roboanalyzer中似乎只接受纯数字内容，第一行的变量名称务必删除）
姓名学号可在python文件中的text = "***"直接进行修改，操作十分简便。

第二部分操作：在Virtual robots → Cartesian Control → Motion → File，可以直接输入末端轨迹笛卡尔坐标的6*n维数据csv文件，无需额外逆运动学求解，点击Start播放

结果如下图所示


![image](https://github.com/user-attachments/assets/19948ae1-9f90-4677-a0d1-a6ea060e9e8c)
![image](https://github.com/user-attachments/assets/b7dfb9d6-11c2-42ee-ac54-6d9ca8ce280b)

