# 铰接公交高车流性能优化（2026-10-23）

## 改动与行为边界

CPU 采样显示，大量小车会在同一帧内反复调用铰接公交的 `BodyEnvelope`，重复计算前后两节的八个角点。新增按前节位置、车头方向、尾节方向、车长、车宽校验的运行期缓存；姿态或车型变化后重新计算，保留原角点算法与浮点结果。缓存不进入存档，读档和直接修改车辆几何也会重新校验。

路口排队检查原先会对全地图小车执行转向、信号、路权等计算。新增保守距离预筛：使用停止线、最大道路半宽、当前车速的刹车距离和车长推导候选范围；已开始转弯的车辆无论距离多远都保留。候选仍按原顺序执行完整规则，不改变仿真频率、车辆数量或铰接物理。

## 本机测量

Apple M4、macOS 27.0.1，前后程序分别运行。两组测试均保持 864 辆小车、8 辆 AI 铰接公交及玩家铰接公交，车况保持 100%。这些是有限时间的本机场景测量，仍存在超过 16.7 ms 的慢帧，不代表所有线路都稳定达到 60 FPS。

### 固定帧 CPU 更新基准

随机种子 20261023，都会地图，AI 限速 40–60 km/h。先运行 600 帧预热，再测量 1,200 帧，仅计仿真 Update，不包含 Draw 或 GPU 时间。

| 指标 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 平均更新耗时 | 13.50 ms | 10.41 ms |
| 每次更新分配 | 538,068 B | 538,038 B |
| 每次更新分配次数 | 2,022 | 2,022 |

平均更新耗时减少约 23%。CPU 采样中的 `BodyEnvelope` 从主要热点中消失；路口排队判断累计采样耗时也下降。内存分配没有明显改善，本轮收益主要来自减少重复计算。

### 实际 Update / Draw 回调

随机种子 5，都会地图，AI 限速 30–80 km/h，4 倍视野（长宽各 4 倍）。前后均运行 1,800 次更新，前 600 次预热，随后计量 1,200 次更新及对应的实际绘制回调。与上方固定基准的种子、限速不同，不合并计算百分比。

| 指标 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 平均更新耗时 | 11.87 ms | 10.72 ms |
| 更新耗时 P50 | 12.30 ms | 11.27 ms |
| 更新耗时 P95 | 16.64 ms | 15.52 ms |
| 更新耗时 P99 | 17.15 ms | 16.04 ms |
| 最慢更新帧 | 37.40 ms | 22.95 ms |
| 实际平均绘制回调 | 53.72 FPS | 58.29 FPS |
| 实际平均更新回调 | 59.91 TPS | 59.94 TPS |

绘制数来自 Ebitengine 的真实回调计数，Update 耗时不包含 GPU 执行时间。原始数据见 [JSON 记录](articulated-performance-2026-10-23.json)。

## 行为验证

- 优化前后分别回放 1,800 帧，逐帧记录公交和全部小车的位置、速度、朝向，以及公交铰接角、转向、门状态、车况、乘客数、线路进度和候车客流；全部记录的哈希逐帧一致。
- 车身缓存覆盖位置、车头、尾节、长宽、几何版本、车型变化、实际连续转弯及存档往返，结果与未缓存算法精确一致。
- 24 万组随机车位、朝向和速度，覆盖多种路宽和不对称路口，验证预筛保留所有原规则判断具有优先权的车辆；另有远距离已承诺转弯保护。
- 六张地图逐帧对照启用／关闭路口缓存和距离预筛，比较完整小车状态。
- `go test -json ./... -count=1 -timeout=15m`：8,711 项测试及子测试通过，6 项跳过，0 项失败。
- 864 小车、8 辆 AI 铰接车及玩家铰接车连续 7,200 帧城市长测通过：所有公交车况保持 100%，行驶进度门槛通过。
- 之前绿灯卡点的原存档副本连续回放 3,600 帧通过，原卡点恢复，车况未下降。
- 缓存、路口预筛、绿灯起步和六地图缓存对照的专项 `-race` 检测通过；`go vet ./...` 与 `git diff --check` 通过。
- `./build.sh app` 构建成功，生成的 plist 通过 `plutil -lint`，游戏类别和游戏模式支持声明均已核对。

复测命令：

```sh
go test ./game -run '^$' -bench '^BenchmarkArticulatedCongestedCity$' -benchtime=1200x -count=1
go run ./cmd/release-check -only-scene 31 -buses 8 -bus-model 2 -frames 1800 -warmup 600 -output /tmp/bus-articulated-check
BUS_PERF_TRACE=/tmp/bus-trajectory.txt go test ./game -run '^TestArticulatedPerformanceTrace$' -count=1
```

## macOS 游戏模式

打包脚本现在写入 `LSApplicationCategoryType=public.app-category.simulation-games` 与 `LSSupportsGameMode=true`。后者是 macOS 26 起的明确支持声明；旧系统通过游戏类别识别。Ebitengine 当前版本使用 macOS 原生全屏窗口接口。

在 Apple 芯片 Mac、macOS Sonoma 14 或更新系统，从生成的 `.app` 启动，点击绿色窗口按钮进入原生全屏后，由 macOS 自动管理游戏模式。此前对本游戏手动关闭过游戏模式时，需在游戏菜单／游戏叠层重新开启一次；系统会记住该选择。此次验证了构建出的 plist 和原生全屏代码路径，未将应用声明等同于已观察到系统游戏模式处于开启状态。

参考：[Apple 游戏模式说明](https://support.apple.com/105118)、[LSSupportsGameMode 官方文档](https://developer.apple.com/documentation/bundleresources/information-property-list/lssupportsgamemode)、[应用类别官方文档](https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/LaunchServicesKeys.html)。
