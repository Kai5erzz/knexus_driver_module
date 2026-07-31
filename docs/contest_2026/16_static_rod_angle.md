# 底盘静止杆角控制模式

## 作用

在底盘左右轮保持失能的情况下，用杆上的 DM-IMU-L1 测量 `roll`，控制
FDCAN2 上的 C620/M3508，使杆角闭环到 `0°`。该模式目前只支持 STM32，
暂不接收上下位机目标。

## 启用

在 `common/User/config/knexus_config.h` 中只启用：

```c
#define KNEXUS_MODE_STATIC_ROD_ANGLE
```

上电后，DM-IMU-L1 姿态有效且 M3508 反馈新鲜时自动进入闭环。LED1 亮表示
杆角闭环正在使能，LED2 亮表示杆角数据有效。任一 CAN 反馈超时都会停止
M3508；底盘通信命令不会使左右轮动作。

## 已标定极性与限位

- `+rpm`：丝杆下沉，`roll` 增大。
- `-rpm`：丝杆上升，`roll` 减小。
- 手动丝杆测试仍使用 `-12°～+7°` 单向软限位。
- 当前静止杆角闭环设置 `KNEXUS_ROD_SOFT_LIMIT_ENABLE=0`，不屏蔽越界时
  的电机命令，确保D项在高速回中时能够反向制动。

因此闭环误差为 `target - roll`，默认电机符号为 `+1`。

## 初始参数

- `KNEXUS_ROD_ANGLE_KP_DEFAULT = 70 rpm/deg`
- `KNEXUS_ROD_ANGLE_KI_DEFAULT = 2 rpm/(deg*s)`，仅在目标附近启用
- `KNEXUS_ROD_ANGLE_KD_DEFAULT = 16 rpm/(deg/s)`
- 积分限幅 `30 rpm`，积分区间 `|误差|<=3°、|角速度|<=4°/s`
- 最大转速 `500 rpm`
- 目标转速斜率 `8000 rpm/s`（优先保证快速收敛和反向制动）
- 角度死区 `0.25°`

运行时可直接修改普通变量 `knexus_rod_target_deg`、
`knexus_rod_angle_kp`、`knexus_rod_angle_ki`、`knexus_rod_angle_kd`、
`knexus_rod_integral_max_rpm`、`knexus_rod_max_rpm`、
`knexus_rod_rpm_slew_rpmps` 和 `knexus_rod_deadband_deg`。

## OctoLink 关键变量

- `var900`：目标 roll（deg）
- `var901`：处理后 roll（deg）
- `var902`：角度误差（deg）
- `var903`：roll 角速度（deg/s）
- `var904/905`：P/D 输出（rpm）
- `var906/907`：限幅前/斜率限制后目标转速（rpm）
- `var908/909/910`：电机目标/斜坡目标/实测转速（rpm）
- `var913/914`：闭环就绪/电机使能
- `var915/916/917`：上限位/下限位/被屏蔽方向
- `var918/919/920/921`：Kp/Kd/最大转速/转速斜率
- `var923`：M3508 反馈年龄（ms）
- `var932~953`：DM-IMU-L1 完整诊断
- `var954/955`：Ki / 当前积分输出（rpm）
- `var956/957/958`：积分限幅 / 角度积分区 / 角速度积分区
- `var959`：当前模式软限位开关（当前为0）
- `var960`：M3508速度误差（目标斜坡转速-实际转速）
- `var961/962`：本模式M3508速度内环Kp/Ki
- `var963/964`：M3508速度积分限幅/最大电流指令
- `var965/966`：静摩擦电流前馈/前馈启用最小目标转速
