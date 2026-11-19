# 存档二玩家跟车进站修复（2026-11-16）

使用本次最新存档二，保存时间 `2026-11-16T11:03:47.897007-07:00`。原始备份为 `dist/slot2-station-follow-2026-11-16/original-slot2.json`，SHA-256 为 `f434a48e3759e7c832087b39112210ca236a4a558014bf1bf898d370b0c75c90`。

玩家车在 `(-15813.9084, -6440.8165)`，跟随正在 13 号站上下客的 92 号铰接公交。玩家车距动态后泊位仅约 19.57 像素，横向偏差约 15.21 像素，车头相对站台偏转约 0.201 弧度。

原来的局部规划要求短曲线精确到达后泊位中心，这段距离无法满足曲率要求，于是退回普通道路跟车，停在站口。原始回放中，要等前车离站后，约第 31 秒才开始靠站；如果前车持续服务，即使后泊位可用，后车仍不会开门。

修复允许在前车占用站台、后泊位可用时，检查真实转向弧线，完成回正并停在实际站台范围内，不强求短距离横移到中心点。仅这一后泊位场景将所需前进距离从 25 像素降为 15 像素，并将最后刹车余量从 3 像素改为 0.5 像素，以完成最后的回正。泊位目标本身仍保留前车车尾后的正常安全间距；开门前车头偏角必须小于 0.075 弧度。全程保留车身、拖挂、障碍物和站台边界检查。

验证：

- 四个道路朝向均可在前车持续上下客时停入后泊位，保持 20–40 像素车头至前车车尾间隔，无碰撞或硬边界接触。
- 静止障碍、移动障碍、站台外位置、前进空间不足、车速过高、缺少道路和缺少前车的边界测试均拒绝短弧线；进站途中保存后恢复的四方向回归也已通过。
- 固定存档现场：约 0.85 秒后在前车后方靠站开门，约 47.48 秒后完成服务并驶离站台区域，两车无新增损伤。
- 完整原始存档：约 0.85 秒后靠站开门，约 40.48 秒后完成服务并跟车驶离站台区域，两车无新增损伤、无碰撞、无重定位，前车服务期间不超越前车。

完整存档验证命令：

```sh
BUS_STATION_FOLLOW_SAVE=/absolute/path/original-slot2.json \
go test ./game -run '^TestCurrentSlot2PlayerFollowsAIIntoStation$' -count=1 -v -timeout=5m
```

新回归为 `TestSavedSlot2PlayerFollowsAIIntoStation`、`TestSavedAngledFollowerBoardsBehindServingArticulatedBus` 和 `TestShortRearBerthAlignmentKeepsSafetyGuards`。原始场景与验证日志保存在 `dist/slot2-station-follow-2026-11-16/`。

完整回归已通过：

```text
go test ./internal/autopilot ./game ./tests/integration -short -count=1 -timeout=20m
internal/autopilot  244.226s  PASS
game                329.997s  PASS
tests/integration    64.819s  PASS
```

最后再次联合运行固定现场及完整原始存档专项回放，两者均通过，并显式禁止玩家车重定位、前车重定位或前车死锁重置。保存/恢复及安全边界测试单独重跑通过。

根目录可执行文件与 macOS 应用包已重新构建，二者 SHA-256 一致：`a6248065cd7ff83fa3b0dbda337f05b7805fe61311bc36a12f9b97b2f36bb889`。原始手动存档校验和未变，无需覆盖存档或手动挪车；重新启动新应用后加载存档二即可。
