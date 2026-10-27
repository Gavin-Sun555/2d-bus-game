# 顶部信息栏布局验证

顶部改为 96 px 两行布局。第一行提供模式、营收、载客量及换图、视角、设置；第二行将站名、距离与候车人数、下车铃分配到独立区域。文本按实际字体测量宽度，超长站名以省略号结尾，距离不受站名长度影响。去除顶部按钮中字体不支持的表情图标。提示消息移到信息栏下方。

规划模式下信息栏阻止点击穿透，驾驶及规划模式下违章状态按钮可打开设置，原有 ESC 快捷键保留。

验证：

- `go test ./internal/ui -count=1 -timeout=5m` 通过。
- `go test ./internal/ui ./game -run 'Viewport|Draw|HUD|Camera|WideDrivingView|RouteLifecycle' -count=1 -timeout=5m` 通过。
- `go vet ./internal/ui ./game` 通过。
- 新增矩形边界与互不重叠测试，覆盖 1280、1440、1707、2560 逻辑宽度；中英文长站名、距离保留及点击区域检查通过。
- `go run ./cmd/hud-preview` 使用实际 HUD 渲染生成 16 张预览：中英文、驾驶/规划、铃声开/关、1280/1707 宽度。人工检查最小宽度中英文截图，顶部各区域无重叠，消息位于下方。证据在 `dist/hud-layout/`。
- `bash build.sh app` 重新生成本机程序与 macOS 应用包。
