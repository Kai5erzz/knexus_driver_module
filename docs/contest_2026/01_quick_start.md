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
2. 在 `common/User/config/knexus_config.h` 中选择工作模式。
3. 默认巡线模式下用 KEY0 完成黑线/白底校准，再按 KEY1 启动。
4. 需要调试时打开 `KNEXUS_DEBUG_OCTOLINK_ENABLE`，观察对应模式变量。
5. KEY1 或安全故障应立即使目标速度归零。

默认速度命令有超时保护；上层必须周期刷新运动命令。
