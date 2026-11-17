# 康复机械臂 FR3 + XHand 工程基线

版本：

rehab_robot_fr3_xhand_baseline_r1_20260906

日期：

2026-09-06


## 1. 当前工程定位

本版本用于恢复并固定：

Franka Research 3 + XHand 1

空载微动、状态读取和联合控制的稳定工程基线。

D435i 不属于本次基线。


## 2. 主工作空间

~/rehab_robot_ws


## 3. Franka 依赖工作空间

~/franka_ros2_ws

该工作空间是稳定底层依赖。

禁止在无明确故障证据的情况下重新配置、删除或覆盖。


## 4. 系统环境

Ubuntu 24.04 LTS

ROS 2 Jazzy

PREEMPT_RT:
6.8.1-1015-realtime

libfranka:
0.20.4

franka_ros2:
v3.3.0-era

FR3 ROS2 source commit:
d812aabfab939317bd16515a392ad2cf1928fa83


## 5. FR3

PC interface:
enp4s0

PC IP:
192.168.1.100/24

Robot IP:
192.168.1.25

arm_id:
fr3

PTP Action:

/action_server/ptp_motion

type:

franka_msgs/action/PTPMotion

已实机验证：

- FR3 live joint state
- PTP Action server
- relative pregrasp
- relative grasp
- rehab micro motion
- XHand hold during FR3 PTP

FR3 最大验证速度：

0.020 rad/s


## 6. XHand

interface:

enx6c1ff7641594

device:

EV_HAND_V001

serial:

012R3201B1217006

当前 vendor runtime:

xhand 1.1.15.0-release

repo:

v1.5.1

commit:

bc7a851694fb30e89b50b21a83099473b4170fe2


## 7. XHand 已验证微动参数

mode = 3

kp = 225

ki = 0

kd = 0

tor_max = 150 mA

delta = 0.005 rad

refresh = 20 Hz

watchdog = 0.5 s

disable frames = 3


## 8. XHand 参数状态

formal active_gains.verified = FALSE

该值禁止因为当前空载微动测试通过而改成 TRUE。

当前结论仅为：

bounded empty-load micro-validation passed

不是完整生产参数标定。


## 9. 已通过实机项目

- XHand 12关节只读状态
- XHand single-joint +0.005 rad
- XHand 9-joint bounded micro motion
- XHand 20 Hz hold
- XHand 20 Hz hold 3 s专项测试
- FR3 PTP real motion
- FR3 pregrasp segment
- FR3 grasp segment
- FR3 rehab segment
- FR3 + XHand simultaneous hold/motion
- XHand fault/finish powerless x3
- EtherCAT safe close


## 10. 2026-09-06 最终恢复段

FR3 live q_start joint1:

约 0.035968 rad

target joint1:

约 0.040968 rad

planned relative delta:

+0.005 rad

pre-send pose error:

0.000003 rad

final max error:

0.001283 rad

RESUME_RESULT:

0


## 11. 当前正式工程结论

核心 FR3 + XHand 联合控制链路：

PASS

分段实机联合验证：

PASS

完整 run_empty_grasp.sh --execute
从开始到结束单次不中断验收：

NOT YET FORMALLY ACCEPTED


## 12. 安全原则

- 不使用 synthetic midpoint 作为真实 FR3 PTP 起点
- 实机目标必须基于 live FR3 q
- XHand torque 字段按当前 SDK 的 signed mA 读取
- 不再次进行 9-bit torque 解码
- XHand active hold 限制为 20 Hz
- watchdog = 0.5 s
- XHand 故障执行 powerless
- FR3 不自动 Recovery
- FR3 故障后不自动回原位
- 不自动累计重复 FR3 相对运动
- D435i 暂不进入本基线


## 13. 已建立但未接入正式 Executor 的模块

empty_grasp_execution_state.py

empty_grasp_state.py

这些文件属于独立 checkpoint 软件合同。

目前没有接入正式 run_empty_grasp.sh --execute。

不得误认为当前已有自动断点恢复。


## 14. 推荐日常入口

加载环境：

source ~/rehab_robot_ws/env.sh

软件检查：

./tools/run_empty_grasp.sh --check

只读实机检查：

./tools/run_empty_grasp.sh --preflight


## 15. 禁止事项

不要：

- 删除 ~/franka_ros2_ws
- apt autoremove
- 无故重装 ROS2
- 无故重装 libfranka
- 无故替换 XHand SDK
- 把厂商示例增益当成历史参数
- 把 active_gains.verified 改成 true
- 为通过测试而放大运动阈值
- 在未经重新验收情况下扩大 FR3/XHand 动作范围
