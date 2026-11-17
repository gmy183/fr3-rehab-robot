# 项目目录说明

## 主工程

~/rehab_robot_ws


## src/

正式 ROS2 源码。

包括：

rehab_robot_interfaces

rehab_robot_bringup

rehab_robot_control


## config/

唯一正式运行配置。

硬件接口、运动安全参数、XHand 参数和 Empty-Grasp
均以该目录为准。


## tools/

正式工程入口与验收脚本。

不要把临时测试脚本散落到主目录。


## validation_logs/

实机验证、软件验收和恢复过程日志。


## releases/

已经封版的稳定工程快照。

Release 目录只用于回退和查阅。

不要直接在 Release 内开发。


## archive/

恢复系统期间产生的安装、修复和历史日志。

不参与机器人运行。


## build/ install/ log/

colcon 自动生成目录。

不是源码。


## ~/franka_ros2_ws

Franka ROS2 稳定底层环境。

与 rehab_robot_ws 分离。

未经明确诊断不要修改。
