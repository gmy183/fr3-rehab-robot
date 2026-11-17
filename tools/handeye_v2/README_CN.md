# FR3 + D435i 手眼标定：非覆盖式 V2 视觉负载与采样安全补丁

## 适用工程与边界

目标为用户现有 Ubuntu 24.04 + ROS2 Jazzy + FR3 + D435i + XHand 工程，工作空间 `~/rehab_robot_ws`。本包**不修改**原有 `checkerboard_pose_probe.py`、`fr3_d435i_handeye_collect.py`、MoveIt/ROS 控制文件、负载参数、Watchman、安全球和已有样本；新增独立的 `~/rehab_robot_ws/tools/handeye_v2/`。

V2 是单一视觉进程，取消独立棋盘检查程序与采集程序的重复运行；使用 OpenCV 单线程、5 Hz 棋盘检测、只缓存最新帧的 ROS QoS、显示约 10 Hz，不启动、恢复、切换 ROS 控制器，不发送任何 FR3/XHand 命令。**并不宣称已经解决 `communication_constraints_violation` 的系统根因。**

## 安装（在当前 Ubuntu 机器上）

将整个 `fr3_handeye_safe_patch_v1` 文件夹放入 `~/Downloads`，然后：

```bash
cd ~/Downloads/fr3_handeye_safe_patch_v1
bash install.sh
```

只新增以下文件：

- `tools/handeye_v2/handeye_capture_v2.py`：图像、棋盘角点、PnP、采样、硬件状态监控；不控制机器人。
- `tools/handeye_v2/handeye_math.py`：变换/重投影辅助计算。
- `tools/handeye_v2/run_capture_v2.sh`：单一视觉进程、1 线程计算库环境、重复程序检查。
- `tools/handeye_v2/handeye_solve_v2.py`：纯离线手眼矩阵候选求解。

如果该目标目录已存在，安装程序直接拒绝覆盖。不要删除或覆盖原 Sample 001。

## 运行前置条件

1. **仅在机械臂经过操作规程确认处于可由 FCI 安全保持的状态、负载/碰撞包络经核实、工作空间无人、急停可用后**，才考虑重新启动官方控制会话。此包**不会**重新启动任何控制会话；如果三个控制器或硬件未激活，采集会拒绝保存。
2. 官方 MoveIt 会话必须由已有经核实的入口建立；此时禁止 Desk 手动引导、拖动机械臂。FR3 运动不在本包中实现。
3. D435i 现有节点运行，彩色图像与相机内参已发布。
4. 12×9 格、内角点 11×8、单格 25 mm 的棋盘标定板**刚性固定**，相机**刚性固定**于 FR3 末端，检查实际尺寸。
5. 原 `checkerboard_pose_probe.py` 和 `fr3_d435i_handeye_collect.py` 必须退出（当前用户已经执行）。不要重新同时启动。
6. 实时控制稳定性尚未通过验证，且 XHand+D435i 的负载质心/惯量、支架碰撞包络仍须核实。**本补丁绝不授权机械臂运动或载人交互。**

## 仅在上述条件满足时开启新采集程序

```bash
source ~/robot_env/rehab.sh
~/rehab_robot_ws/tools/handeye_v2/run_capture_v2.sh
```

可选的视觉 CPU 亲和性（不是 FR3 线程的隔离，务必结合 CPU 拓扑与 IRQ 分布评估后再启用）：

```bash
HAND_EYE_CPUS=0-3 ~/rehab_robot_ws/tools/handeye_v2/run_capture_v2.sh
```

默认不开启 CPU 绑核，避免盲目改变系统调度。原配置的 RealSense 相机不会被重启。

GUI 按 `s` 请求保存、`q` 或 `Esc` 退出。保存操作同时要求：控制器 3 个 active；FrankaHardwareInterface active；控制服务响应及时；相机图像、关节时间戳与系统时钟相差 <0.5s；接收关节状态 <0.3s；机械臂静止至少 0.5s；角点检测和 RMSE ≤0.8 px；在图像时刻有 `fr3_link0 -> fr3_link8` TF；新位姿与历史样本有足够差异。未满足会显示 `[REJECT] NO SAMPLE SAVED`；已经健康的会话一旦丢失硬件状态会锁定 `HALT`，不再允许继续保存。不要试图绕过这些门禁。

**重要**：目前控制器已经是 inactive，**在现状运行 V2 不会有正式样本**。先安全恢复并验证外部控制会话与真实负载/碰撞边界。

## 数据保护与结果

新候选样本路径：

```
~/rehab_robot_ws/calibration/fr3_d435i_handeye_v2/sample_001.json
```

不会碰原路径 `calibration/fr3_d435i_handeye/sample_001.json`。V2 记录两个变换 `base_T_gripper` 与 `camera_T_board`、机器人关节状态、相机内参和各自消息时间戳，附有 `status=candidate`，不假装已经标定成功。

采集至少 15 组、且姿态旋转有足够变化后可离线求解（推荐 20–30 组，需检查视角覆盖与碰撞安全）：

```bash
cd ~/rehab_robot_ws/tools/handeye_v2
python3 handeye_solve_v2.py ~/rehab_robot_ws/calibration/fr3_d435i_handeye_v2
```

程序计算 TSAI、PARK、HORAUD、ANDREFF、DANIILIDIS，使用标定板在机器人基坐标中的**内部一致性**指标排序，输出 `handeye_candidate_report_v2.json`。它是 `CANDIDATE_NOT_VALIDATED`，不得未经独立姿态验证就发布为正式静态 TF，或用于临床/患者接触动作。相机 TF 与机器人 TF 暂时断开是预期状态。

## 运行后工程验收

1. 运行 V2 后的 CPU、线程数应明显低于之前并行两个 30+ 线程、各 ~250% CPU 的视觉进程（需实测）。
2. 不运行 V2 时先用**获授权且安全的程序**进行 FR3 静止保持稳定性测试，再运行 V2 重复；要求两个场景均没有 `communication_constraints_violation`，并记录完整 1 kHz RT 性能和网卡中断负载。仅凭 ping 0% 丢包不可判定安全。
3. 若仍发生 FCI 中断，则继续检查 `ros2_control` 控制回调延时、调度、网卡中断/驱动和系统隔离，不自动 ErrorRecovery 或重启控制器。
4. 末端负载参数历史配置 1.7 kg、CoM=(0,0,0)、惯量对角 .005 不是独立测量报告，不可仅凭 Desk profile active 作为实机安全验证。

## 限制

这个补丁基于已经确认的系统接口重新实现非运动的视觉采样通路；由于没有拿到你本机两份原始 Python 文件，**没有修改原文件内部逻辑**。本包不自动执行机器人运动和手动引导，不切换机器人控制权，也不会绕过 Watchman 安全规则。请在正式采样前对本机实际主题、节点和 TF 帧名确认一致。
