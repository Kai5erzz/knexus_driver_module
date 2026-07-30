# 任务调度设计

2026 底盘程序在 STM32 和 MSPM0 上使用同一套周期和职责划分。原则是：电机与
安全逻辑永远不等待串口、CAN、ADC 推理或调试输出；耗时功能只能放在更低优先级。

## 默认任务表

| 任务 | 周期 | 启动相位 | 主要职责 | STM32优先级 | MSPM0优先级 |
|---|---:|---:|---|---|---|
| `knx26_fast` | 1 ms | 0 ms | 电机速度环、安全、蜂鸣器；验收模式下含3508快环 | High | Idle+5 |
| `knx26_ctrl` | STM32 5 ms / MSPM0 10 ms | 0 ms | BMI088、零飘学习、里程计、底盘外环 | AboveNormal2 | Idle+4 |
| `knx26_track` | 10 ms | 1 ms | 8路ADC扫描、归一化、线误差 | AboveNormal1 | Idle+3 |
| `knx26_app` | 10 ms | 2 ms | KEY0/KEY1、工作模式状态机、巡线策略 | AboveNormal | Idle+3 |
| `knx26_comm` | 5 ms | 3 ms | 上下位机、双板/双车通信、CAN轮询和心跳 | Normal1 | Idle+2 |
| `knx26_percept` | 10 ms | 4 ms | 可选路口窗口和TinyCNN/规则识别 | Normal | Idle+2 |
| `knx26_debug` | 20 ms | 7 ms | OctoLink周期输出 | Low | Idle+1 |

当前 `KNEXUS_INTERSECTION_ENABLE=0U`，因此不会创建 `knx26_percept`，也不会运行
TinyCNN。需要路口判断时改为 `1U` 即可；推理仍独立于灰度采样任务，不会拖慢循迹。
H题球控以10ms子周期挂在现有 `knx26_ctrl` 中，不额外分配任务栈，也不占用巡线策略任务。

## 为什么这样分配

- 原通信任务每1 ms以较高优先级轮询，既没有必要，也可能与控制任务争抢CPU；现在为
  5 ms，并低于控制、采样和策略任务。
- 巡线策略由20 ms改为10 ms，控制输出从50 Hz提升到100 Hz。
- 灰度ADC只负责生成完整采样帧；路口推理和用户回调不再占用采样任务。
- 各任务错开1～7 ms启动，减少同一个系统tick内集中唤醒。
- STM32的CubeMX空默认任务启动后立即删除，不再每1 ms产生无意义的上下文切换。
- 所有周期任务错过截止时间后都会记录超期，并从当前时刻重新建立周期；不会连续
  追赶旧周期、持续饿死低优先级任务。

## 调整规则

周期和相位只在 `common/User/config/knexus_config.h` 修改。相位必须小于对应周期，
配置错误会在编译阶段直接报错。

不要在 `fast`、`control`、`track` 或 `app` 中直接打印串口，也不要使用带等待的CAN
发送。调试输出统一放进 `knx26_debug_update()`；路口模型统一放进
`knx26_perception_update()`。

## OctoLink调度诊断

调度诊断每100 ms发送一次：

| ID | 内容 |
|---:|---|
| 780～785 | app、fast、control、track、comm、debug 心跳累计值 |
| 786～791 | 上述六任务的超期累计值 |
| 792～797 | 上述六任务的单次最大执行时间，单位ms |
| 798 / 799 / 800 | perception 心跳、超期、最大执行时间 |

正常运行一秒，心跳增量应约为：`100 / 1000 / 200(STM32)或100(MSPM0) / 100 /
200 / 50`。超期计数应长期不增长。路口功能关闭时，798～800保持0是正常现象。
