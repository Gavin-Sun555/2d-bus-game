# 游戏层

`game` 包负责衔接菜单、驾驶、路线规划、存档和每帧更新。`Game` 的入口在 `game.go`。

| 要查找的逻辑 | 位置 |
| --- | --- |
| 驾驶更新、玩家输入和安全处理 | `driving.go`、`driver_controls.go`、`driver_safety.go` |
| 规划、站名搜索和线路车队 | `planning.go`、`station_search.go`、`route_lifecycle.go`、`line_fleets.go` |
| 城市选择和画面布局 | `menu.go`、`city.go`、`display.go`、`draw.go` |
| 手动存档、自动存档和状态恢复 | `save_slots.go`、`autosave.go`、`save.go` |
| 存档文件格式、校验与原子写入 | [`internal/savefile`](../internal/savefile) |
| 触控板事件、鼠标滚轮与剪贴板 | [`internal/platforminput`](../internal/platforminput) |

`replay_*_test.go` 按存档槽位或交通场景归类。可以用 `go test ./game -list .` 查找测试，或用 `go test ./game -run '^测试函数名$'` 单独运行。性能探针和基准集中在 `performance_test.go`。
