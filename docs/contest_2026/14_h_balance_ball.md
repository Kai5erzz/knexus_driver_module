# H题：车载平衡滚球巡线任务

题目赛道由两段1.5m直线和两个半径0.5m半圆组成，理论周长约6.14m。A点设有一条
与环线垂直的5cm启停线。当前实现只完成底盘巡线、整圈识别、停车计时，并为小球
控制和显示预留稳定接口。

## 操作流程

1. `KEY0`执行黑线/白底两阶段校准，流程与普通巡线一致。
2. 把车放在A点且朝顺时针方向，`KEY1`启动并开始计时。
3. 小车先处于 `START_CLEAR`，沿线驶离脚下启停线；连续80ms不再看到宽黑线后，
   才允许识别终点。
4. 行驶时间达到5s且里程达到5m后，A线识别解锁。
5. 中央区域至少有2路为黑，且满足“中央3路为黑”“至少3路连续为黑”或
   “总计至少4路为黑”之一，连续30ms后判定再次到达A点。该组合可覆盖中央3/4路、
   左右偏移的3～5路，以及横杠中某一路采样偏弱的情况。
6. 按 `STOP_AFTER_MARK` 补偿距离低速前进，然后停车、冻结计时并进入 `COMPLETE`。
7. 运行中按 `KEY1`立即停车；完成后再按 `KEY1`可重新开始一圈。

脱线保护仍然有效，连续脱线超过配置时间会停止，不会为了找A线继续盲跑。运行超过
35s仍未识别到A线也会超时停车。

## 关键配置

配置都在 `common/User/config/knexus_config.h` 的“H题环形赛道任务”区域：

| 宏 | 默认值 | 作用 |
|---|---:|---|
| `KNEXUS_H_TASK_ENABLE` | 1 | 启用H题整圈状态机；设0退回普通持续巡线 |
| `KNEXUS_H_LINE_SPEED_MPS_DEFAULT` | 0.45m/s | 直线基础速度 |
| `KNEXUS_H_START_LINE_ACTIVE_MIN` | 3 | A线判定所需最少有效通道数 |
| `KNEXUS_H_START_LINE_CONTIGUOUS_MIN` | 3 | 连续黑色通道的最小长度 |
| `KNEXUS_H_START_LINE_CENTER_ACTIVE_MIN` | 2 | 中央4路中至少需要命中的通道数 |
| `KNEXUS_H_START_LINE_CENTER_MASK` | `0x3C` | 中央区域掩码，对应CH2～CH5 |
| `KNEXUS_H_START_LINE_CHANNEL_THRESHOLD` | 0.55 | 单路归一化黑线阈值 |
| `KNEXUS_H_FINISH_ARM_MIN_TIME_MS` | 5000ms | 最早允许识别终点的时间 |
| `KNEXUS_H_FINISH_ARM_MIN_DISTANCE_M` | 5.00m | 最早允许识别终点的里程 |
| `KNEXUS_H_STOP_AFTER_MARK_M_DEFAULT` | 0m | 识别线后继续前进的补偿距离 |
| `KNEXUS_H_BALL_TARGET_CM_DEFAULT` | 0cm | 行驶时小球目标位置 |

### 底盘闭环调参流程

纵向速度使用“曲率预瞄 + S形加减速”，转向环仍使用原来的PID、最小转向补偿、
回搜角速度和角速度上限。这样可以单独把车速变化调柔，不牺牲已经接近极限的转弯力度。

第一轮保持默认参数跑完整一圈并导出OctoLink宽表CSV。后续每轮只调整一组参数：

| 参数组 | 宏 | 观察目标 |
|---|---|---|
| 直线稳定 | `STRAIGHT_ZONE/DEADBAND/MIN_GAIN` | 直线误差不持续左右换向，角速度均值接近0 |
| 入弯预瞄 | `CURVE_LOOKAHEAD/ATTACK` | 误差明显增大前开始平滑减速 |
| 弯中速度 | `CURVE_SLOWDOWN_GAIN/MIN_SPEED_SCALE` | 不丢线且不过度降速 |
| 出弯恢复 | `CURVE_RELEASE_ALPHA` | 出弯不突然加速、不二次回摆 |
| 纵向平稳 | `ACCEL/DECEL/JERK/SPEED_RESPONSE` | 实测加速度峰值小、速度曲线无尖角 |

每次采样应从按下KEY1前开始，到自动识别A点停车后结束；不要只截取弯道片段，否则无法
同时比较直线波动、弯道误差、单圈时间和停车过程。

### 停车偏差标定

先保持 `STOP_AFTER_MARK=0` 连续测试三次，测量车身唯一测试点相对A基准线的平均停车
误差。若停车点偏早，将该值增加；若停车点偏晚则降低该值或降低接近速度。运行期变量
`knexus_h_stop_after_mark_m` 可在线修改，不必重新编译。

## 小球控制接入位置

接口位于 `common/User/app/contest_2026/modes/knexus_h_hooks.h`。默认实现不驱动任何
硬件，OctoLink中的 `implemented/ready` 会保持0。小球控制组新建自己的源文件并实现：

```c
void knexus_h_ball_user_init(void);

void knexus_h_ball_user_update(
    const knexus_h_ball_request_t *request,
    const struct knx26_context *context,
    float dt_s,
    knexus_h_ball_status_t *status);
```

`request`提供保持中心、保持任意位置或静态 `+5cm -> -5cm` 测试请求；`context`提供
BMI088姿态、角速度和底盘速度，可用于转弯/加减速前馈。该接口由控制任务中的
10ms子周期调用，必须非阻塞，不允许在函数内部等待摄像头或舵机。

行驶模式会调用 `knexus_h_ball_request_hold(knexus_h_ball_target_cm)`。要求6可在线把
`knexus_h_ball_target_cm`改成指定位置。要求3的静态测试入口已经保留：

```c
knexus_h_ball_request_static_sequence();
```

显示组可实现 `knexus_h_display_time_ms()`，把冻结后的总时间显示在小于2英寸屏幕上。

## OLED计时显示

当前已经接入队友编写的SSD1306兼容128x64 OLED字库和显示接口，并按赛题计时语义重写
显示逻辑。屏幕实物尺寸必须不大于2英寸：

- 待启动：显示 `STATE READY` 和 `TIME 000.000s`；
- KEY1启动：计时与H题状态机在同一周期开始，显示 `STATE RUN`，每50ms刷新；
- 回到A点：底盘停车的同一状态转换中冻结时间；不超过20秒显示 `STATE PASS`，否则
  显示 `STATE OVER`；
- 人工停止或故障停止：显示 `STATE STOP`，保留停止时刻。

底板使用同一组 `I2C_CUSTOM` 网络。STM32使用硬件I2C1：`PB8=SCL`、`PB9=SDA`；
MSPM0使用软件I2C：`PA4=SCL`、`PA3=SDA`。MSPM0的循迹ADC已经迁到PA16，因此PA4
可安全恢复为I2C时钟；OLED不会占用目前作为左电机PWM的PB8。OLED地址默认`0x3C`，
刷新周期、通信超时和20秒判定线都在 `knexus_config.h` 中配置。

## OctoLink变量

| ID | 内容 |
|---:|---|
| 810 | 本圈计时，ms |
| 811 | 本圈里程，m |
| 812 | 当前超过A线阈值的通道数 |
| 813 / 814 | 宽黑线判定 / 已驶离起点 |
| 815 / 816 | A线确认中 / A线确认次数 |
| 817 / 818 | 识别A线后前进距离 / 停车补偿目标 |
| 819 | 小球目标位置，cm |
| 820 / 821 / 822 | 球控已实现 / 就绪 / 故障 |
| 823 / 824 | 小球实测位置cm / 球控输出 |
| 825 | 球控更新累计次数 |
| 828 | A线二值掩码，bit0～bit7对应CH0～CH7 |
| 829 | 当前最长连续黑色通道数 |
| 830 | 线置信度，0～1 |
| 831 / 832 | 滤波后的曲率指标 / 当前速度比例 |
| 833 / 834 | 基础请求速度 / 规划后的目标线速度，m/s |
| 835 / 836 | 目标角速度 / 限幅前转向控制量 |
| 837 / 838 | 规划加速度 / 滤波后的实测加速度，m/s² |
| 840 / 841 | 左/右电机速度环内部斜坡目标，m/s |
| 842 / 843 | 左/右电机速度前馈，PWM计数 |
| 844 / 845 | 左/右电机PID修正量，PWM计数 |
| 846 / 847 | 左/右电机积分项，PWM计数 |
| 848 / 849 | 左/右电机最终输出，PWM计数 |
| 850 / 851 | 速度前馈系数 / 静摩擦补偿，PWM计数 |
| 852 / 853 | OLED就绪状态 / I2C累计错误次数 |
| 854 / 855 | OLED当前显示时间ms / 显示状态（0就绪、1运行、2通过、3超时、4停止） |

状态变量仍为730：`10=START_CLEAR`、`6=RUNNING`、`11=FINISH_APPROACH`、
`12=COMPLETE`。停车原因746中，`8=完成一圈`、`9=运行超时`、`10=球控未就绪`。
