# 存档一、二高速流畅度修复（2026-11-16）

使用手动存档一 `save.json`、存档二 `save-02.json` 的原始副本，每份保持 864 辆小车、72 辆线路公交，不修改用户存档。副本及测量数据位于 `dist/frame-smoothness-2026-11-16/`。

## 原因与修改

1. 旧窗口帧控制器发现一帧放不下两次更新时，会直接跳过引擎请求的补帧更新。仪表记录的是仿真速度，车辆位移却少走了一段现实时间，造成体感变慢与表速不符。现在每次请求都执行完整的固定步长更新，不跳步，也不按帧率修改仪表读数。
2. 原先直接绘制最近一次 60 Hz 的物理位置；刷新时机变化时，高速车和跟随镜头的位移会呈现不同大小的台阶。现在使用单调显示时钟，将车辆、拖挂、镜头、转向、车门及仪表速度统一在约 25 ms 前的时间点绘制，在相邻快照间插值。五帧历史缓冲和绘制副本复用，稳定运行时捕获及准备显示数据为零分配。碰撞、AI、站点与存档仍读取真实物理状态。换车、重定位、切地图、暂停、恢复及角度跨越 ±π 分别处理，避免画出跨地图的错误移动。
3. 性能采样发现左转出口选择会重复生成相同的整车扫掠几何，产生大量临时对象。改为按路径坐标、车长、车宽的精确浮点位比较后复用几何，并复用临时缓冲。对周围车辆的碰撞、让行和出口检查仍每次实时执行。

单独移除跳步并加插值的中间版本，在存档二曾长期落至约 30 FPS，因此没有将该版本作为完成结果；加入几何缓存后重新进行原生窗口验收。

## 验证

原生 Ebitengine 窗口 1470×915，开启垂直同步，窗口保持不聚焦但继续运行，使用与应用一致的 `GOGC=200`、`GOMEMLIMIT=1GiB`。每个存档运行 3720 个固定步，前 120 步预热，统计后续 60 秒；测试期间不并发运行回归测试。验收检查仿真至少 59 TPS、绘制至少 55 FPS、零跳步，仿真时间与实际时间差小于 0.1 秒。

| 场景 | 仿真 TPS | 绘制 FPS | 仿真时间 | 实际时间 | 跳过更新 | 帧间隔 P99 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 存档一 | 59.988 | 59.671 | 60.000 s | 60.012 s | 0 | 18.803 ms |
| 存档二 | 60.003 | 59.970 | 60.000 s | 59.997 s | 0 | 17.948 ms |

原始报告：

- `dist/frame-smoothness-2026-11-16/qualification-slot1/native-metrics.json`
- `dist/frame-smoothness-2026-11-16/qualification-slot2/native-metrics.json`
- 同目录的 `native-frames.csv` 记录逐帧间隔及每帧更新次数，不能仅用平均 FPS 判断卡顿。存档一此次最大帧间隔 93.728 ms，存档二 33.235 ms；不据此宣称所有偶发长帧已经消失。

针对存档一的孤立长帧，在完整回归结束、没有并发测试时再次单独运行相同 3720 步，并打开 CPU profile、运行时 trace 和 GC 日志。结果为 **59.994 TPS / 59.960 FPS**，仿真 60.000 秒、实际 60.006 秒，零跳步；帧间隔 P99 为 **17.539 ms**、最大 **32.320 ms**，单次更新最大 **16.426 ms**。城市哈希与前一次存档一回放完全相同，为 `1668d02eecc7d66c2bb74376c82104db50438d0528b6dafe87dd9619e88c35c9`。本轮没有重现约 94 ms 的孤立长帧，不能将前一轮停顿确定归因于某段驾驶逻辑或 GC。保留两轮结果，不把平均 FPS 接近 60 当成零长帧证明。相关数据位于 `slot1-profile/`，GC 日志为 `slot1-profile.log`。

存档二相同预热与 1200 步回放，纯仿真内存分配从每步约 100,498 B 降至 51,806 B，下降约 48.5%。两次完整序列化城市状态的 SHA-256 都是 `204602c73dc7c453954593678331ddc5b46cfaf6d04977c771e3c54e4d451b85`。另两次原生窗口 3720 步回放的城市哈希也相同：`caece838e04a50551f612aa59c2cff348a472ae0eba0ba194c3c3bc1af2caae4`。因此缓存没有通过减少车辆、跳过碰撞或改变交通行为来提高流畅度。CPU 均值受运行环境影响，不将不同采样批次的均值当成可靠提升幅度。

自动测试覆盖：

- 30、50、60、90、120、144 Hz 下，180 km/h 恒速行驶的显示位移与实际时间一致；包含更新相位变化、绘制准备耗时变化及一次多步补帧。
- 车辆、镜头与仪表保持同一时间，显示副本不改动物理状态。
- 拖挂、换车、重定位、暂停、切地图、长时间挂起恢复和旋转过 ±π。
- 稳态显示准备、几何缓存命中零分配；缓存输出与原扫掠算法逐项完全一致。

复测命令（输出目录及存档使用绝对路径）：

```sh
go test -c ./game -o dist/frame-smoothness-2026-11-16/smooth-native.test
GOGC=200 GOMEMLIMIT=1GiB \
BUS_NATIVE_SAVE=/absolute/path/slot2.json \
BUS_NATIVE_OUTPUT=/absolute/path/qualification-slot2 \
BUS_NATIVE_TICKS=3720 BUS_NATIVE_REQUIRE_REALTIME=1 \
dist/frame-smoothness-2026-11-16/smooth-native.test
```

可额外设置 `BUS_NATIVE_PROFILE=1` 输出 CPU profile 与运行时 trace，追查单帧耗时。

## 回归及应用构建

```text
go test ./game ./internal/autopilot ./internal/view ./internal/ui ./tests/integration -short -count=1 -timeout=20m
game                327.175s  PASS
internal/autopilot  245.180s  PASS
internal/view         1.978s  PASS
internal/ui           0.514s  PASS
tests/integration    64.026s  PASS
```

`./build.sh app` 成功；根目录 `busgame` 与 `2D Bus Simulator.app/Contents/MacOS/busgame` 的 SHA-256 均为 `c5c706226a2e025784d8b80a7f6a8591361628fcab5d2603e1df72ca7890509b`。两份手动存档均与本次测试前的副本逐字节一致，重启新应用后加载即可。
