# 关键实现说明
## 数据输入  
PosePair 结构体存储机器人末端位姿和标定板位姿

实际应用中需替换为真实数据采集：

机器人末端位姿通过机器人控制器API获取

标定板位姿通过PCL点云配准（如ICP）或OpenCV棋盘格检测获取

## 优化核心
1. 误差定义：通过 AX = XB 方程构建残差，其中：

   - A：机器人末端运动变换
   - B：标定板在相机中的观测变换
   - X：待优化的相机到末端的变换

2. 参数化：

   - 四元数表示旋转（避免万向节锁问题）
   - 平移直接使用三维向量

3. 自动微分：Ceres的 AutoDiffCostFunction 自动计算雅可比矩阵

### 注意事项
- 数据质量：建议至少采集20组不同位姿数据

- 初值设置：可通过粗略测量提供初始值

- 坐标系方向：需确保机器人、相机、标定板的坐标系定义一致

## 扩展改进建议
   加入鲁棒核函数：在 AddResidualBlock 中增加Huber或Cauchy核函数抑制异常值

多传感器融合：结合IMU数据约束旋转

可视化：使用PCL可视化标定结果

如果需要更完整的实现（包括数据采集和验证模块），建议参考以下开源项目：

ROS industrial/calibration

ethz-asl/kalibr