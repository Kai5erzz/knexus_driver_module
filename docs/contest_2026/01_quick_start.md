# 快速开始

## 构建

```powershell
cube-cmake --preset stm32-debug
cube-cmake --build --preset stm32-debug

cube-cmake --preset mspm0-debug
cube-cmake --build --preset mspm0-debug
```

两个预设均指定 `KNEXUS_APP=contest_2026`。STM32 产物是 `knexus_st.elf/hex/bin`，
MSPM0 产物是 `knexus_mspm0.elf/hex/bin`。

## 第一次运行

1. 架空车轮，上电后确认 LED0 每 500 ms 翻转。
2. 在 `knx26_user_init()` 中调用 `knx_chassis_enable()`。
3. 使用 `knx_chassis_set_velocity()` 给出低速命令。
4. OctoLink 观察变量 `700..716`。
5. KEY1 或安全故障应立即使目标速度归零。

默认速度命令有超时保护；上层必须周期刷新运动命令。
