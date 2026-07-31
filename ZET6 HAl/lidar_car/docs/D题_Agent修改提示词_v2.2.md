# D题 Agent 修改提示词 v2.2（历史协议背景，不是当前运行流程）

> 当前实现以 `D题_通用通信与接口规范_v2.3.docx` 和
> [D题任务一二雷达协同联调.md](D题任务一二雷达协同联调.md) 为准。本文仅保留
> 角色边界和协议演进背景；不要按其中旧的按键/任务时序操作现场设备。

本文件记录 V2.2 角色边界的历史背景，不能作为当前通信或现场操作依据。当前唯一通信
规范是 `D题_通用通信与接口规范_v2.3.docx`，当前现场流程见
`D题任务一二雷达协同联调.md`；修改前仍应阅读现有工程并保留既有安全保护。

## 小车端（树莓派 4 + MCU）

树莓派以 115200 8N1、10 Hz 向 MCU 发送 V2.2 `CAR_POSE`。将雷达/SLAM 原点换算到
平台同心圆十字中心，并通过起飞前 `CALIBRATION_SET` 得到 `FIELD_GLOBAL`；前为 `+X`、
左为 `+Y`、yaw 逆时针为正，坐标单位 cm、yaw 单位 0.1 度。MCU 校验后在车端时隙
通过 LoRa 广播 `CAR_POSE`。两个物理按键只发起抛投或动态起降的
`CAR_TASK_REQUEST`，同一序列最多重发三次，ACK 不得重复触发任务或停车。

MCU 仅在小车未运行时转发地面站 `CALIBRATION_SET` 给 Pi；Pi 内存应用成功后才对
地面站确认。校准重启失效，任务中不得修改。任务期间小车必须持续循线一整圈，不依赖
地面站、LoRa ACK 或空中状态改变自身运动。

## MaixCam Pro 相机边界

飞控接受小车任务后创建 `ModeSeq`，再通过 `CAMERA_MODE` 建立相机会话。MaixCam 仅在
会话已建立、`TaskType + MissionId + ModeSeq` 全部匹配时输出 18-byte
`CAMERA_TARGET`，或接受 `CAMERA_ACTION` 并回送 12-byte RESULT；重复
`Seq + RETRANSMISSION` 只回 ACK，不能重复初始化或驱动投放舵机。IDLE 不发送 TARGET。

相机检测 60 x 60 cm 平台上的 30/50 cm 黑色同心圆十字，15-20 Hz 输出机体系
`ErrX/ErrY`，不以画面自主选择任务或投放，不使用 LoRa 或 RTSP 图传。小车端不创建
`ModeSeq`，不解析 TARGET/RESULT，也不直接驱动 MaixCam；雷达/Pi `CAR_POSE` 仍是空地
协同主闭环，视觉只在飞控末段纠偏。
