# 康复机械臂

当前统一工作空间：

`~/rehab_robot_ws`

## 三设备

FR3  
XHand  
Intel RealSense D435i

## VSCode 操作

按：

`Ctrl + Shift + P`

输入：

`Tasks: Run Task`

主要任务：

- `00｜三设备软件总验收`
- `01｜识别三个设备（只读）`
- `10｜XHand｜真实只读启动`
- `11｜D435i｜Color+Depth`
- `12｜FR3｜官方状态 Bringup`
- `20｜三设备｜一起启动（无运动）`
- `21｜三设备｜查看状态`
- `22｜FR3｜检查 PTP Action（不发送Goal）`
- `90｜停止全部设备`

## 当前硬件安全阶段

XHand：

`vendor_readonly`

不提供 command subscriber。

FR3：

只运行官方 bringup，不发送 PTPMotion goal。

D435i：

Color + Depth。

IMU 暂时关闭。

## 唯一硬件配置文件

`config/hardware_profile.env`

真实设备连接后确认：

`FR3_ROBOT_IP`

`D435I_SERIAL`

在确认前不要猜测。

## 后续运动恢复顺序

三设备只读稳定

→ XHand 单关节 0.005 rad

→ XHand 整手 0.005 rad

→ FR3 PTP 小运动

→ FR3 + XHand empty grasp

→ D435i 参与感知
