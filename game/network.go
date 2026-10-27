package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (g *Game) activeLines() []*sim.Route {
	if len(g.Lines) == 0 || g.LineCount < 1 {
		return []*sim.Route{g.Route}
	}
	n := g.LineCount
	if n > len(g.Lines) {
		n = len(g.Lines)
	}
	// Tests and tools may replace Route directly; retain that supported behavior.
	if g.PlayerLine < n && g.Lines[g.PlayerLine] != g.Route {
		lines := append([]*sim.Route(nil), g.Lines[:n]...)
		lines[g.PlayerLine] = g.Route
		return lines
	}
	return g.Lines[:n]
}
func (g *Game) SetTransitLineCount(n int) {
	if n < 1 {
		n = 1
	}
	if n > 6 {
		n = 6
	}
	if len(g.Lines) == 0 {
		g.Lines = sim.DefaultTransitLines(g.World)
		g.Lines[0] = g.Route
	}
	if g.IsPlanMode && !g.finishPlannedRoute() {
		return
	}
	if g.IsPlanMode && g.planRoute != nil {
		for i, route := range g.Lines {
			if route == g.planRoute && i >= n {
				g.planRoute = nil
			}
		}
	}
	g.LineCount = n
	g.NetworkSelected = min(g.NetworkSelected, n-1)
	if g.PlayerLine >= n {
		g.SelectPlayerLine(n - 1)
	}
	if g.AIBusMgr != nil {
		g.AIBusMgr.Routes = g.activeLines()
		g.syncCurrentLineSettings()
		g.buildLineFleets(false)
	}
	g.SyncStopsWithRoute()
	if g.IsPlanMode {
		g.planModeInitialStops = append([]int(nil), g.planningRoute().StopIDs...)
	}
}

// Selecting a service is an explicit dispatch at its terminus. Other buses keep
// their current journeys, passengers and lifetime statistics.
func (g *Game) SelectPlayerLine(index int) bool {
	if index < 0 || index >= g.LineCount || index >= len(g.Lines) {
		return false
	}
	if len(g.Lines[index].StopIDs) == 0 {
		g.HUD.AddWarningToast(locale.T("请先为这条线路选择站点", "Select stops for this line first"))
		return false
	}
	if g.PlayerLine == index && g.Route == g.Lines[index] {
		return true
	}
	if g.PlayerLine < len(g.Lines) {
		g.Lines[g.PlayerLine] = g.Route
	}
	if g.IsPlanMode && !g.finishPlannedRoute() {
		return false
	}
	g.PlayerLine = index
	g.Route = g.Lines[index]
	g.showPlayerLineSettings()
	fleet := g.AIBusMgr.Buses
	operations := g.Operations
	g.resetBusesToRouteStart(false)
	g.Bus.SetModel(g.LineFleets[index].PlayerModel)
	g.AIBusMgr.Buses = fleet

	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos
	g.Operations = operations
	if operations != nil {
		operations.InterruptJourney(0)
		operations.Distances = operations.DistancesFor(g.Route)
	}
	g.operationsRoute = append([]int(nil), g.Route.StopIDs...)
	g.AIBusMgr.Operations = g.Operations
	g.planModeInitialStops = append([]int(nil), g.planningRoute().StopIDs...)
	g.planModeRouteModified = false
	if g.IsPlanMode {
		g.Camera.Zoom = .04
		g.Camera.TargetZoom = .04
		g.Camera.Pos = sim.Vec2{}
		g.Camera.TargetPos = sim.Vec2{}
	}
	return true
}
func (g *Game) updateNetworkPanel() bool {
	if inpututil.IsKeyJustPressed(ebiten.KeyT) && !g.Settings.IsOpen {
		g.NetworkOpen = !g.NetworkOpen
		g.NetworkSelected = g.PlayerLine
		if g.IsPlanMode {
			for i, route := range g.activeLines() {
				if route == g.planningRoute() {
					g.NetworkSelected = i
				}
			}
		}
	}
	if !g.NetworkOpen {
		return false
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		g.NetworkOpen = false
		return true
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyMinus) {
		g.SetTransitLineCount(g.LineCount - 1)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEqual) {
		g.SetTransitLineCount(g.LineCount + 1)
	}
	for i, key := range []ebiten.Key{ebiten.Key1, ebiten.Key2, ebiten.Key3, ebiten.Key4, ebiten.Key5, ebiten.Key6} {
		if inpututil.IsKeyJustPressed(key) && i < g.LineCount {
			g.NetworkSelected = i
		}
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyTab) {
		g.PlanTransitLine(g.NetworkSelected)
		return true
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) {
		g.SelectPlayerLine(g.NetworkSelected)
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		p := sim.Vec2{X: float64(x), Y: float64(y)}
		for i, b := range g.networkButtons {
			if b.Contains(p) {
				g.handleNetworkAction(i)
				break
			}
		}
	}
	return true
}
func (g *Game) handleNetworkAction(i int) {
	c := g.LineFleets[g.NetworkSelected].Normalize()
	switch i {
	case 0:
		g.SetTransitLineCount(g.LineCount - 1)
	case 1:
		g.SetTransitLineCount(g.LineCount + 1)
	case 8:
		g.NetworkOpen = false
	case 9:
		c = c.ChangeModelCount(c.Model, -1)
		g.SetLineFleet(g.NetworkSelected, c)
	case 10:
		c = c.ChangeModelCount(c.Model, 1)
		g.SetLineFleet(g.NetworkSelected, c)
	case 11:
		c.SpeedRange.MinKPH = max(30, c.SpeedRange.MinKPH-10)
		g.SetLineFleet(g.NetworkSelected, c)
	case 12:
		c.SpeedRange.MinKPH = min(c.SpeedRange.MaxKPH, c.SpeedRange.MinKPH+10)
		g.SetLineFleet(g.NetworkSelected, c)
	case 13:
		c.SpeedRange.MaxKPH = max(c.SpeedRange.MinKPH, c.SpeedRange.MaxKPH-10)
		g.SetLineFleet(g.NetworkSelected, c)
	case 14:
		c.SpeedRange.MaxKPH = min(80, c.SpeedRange.MaxKPH+10)
		g.SetLineFleet(g.NetworkSelected, c)
	case 17, 18, 19, 20, 21, 22:
		delta := -1
		if i%2 == 0 {
			delta = 1
		}
		c = c.ChangeModelCount(sim.BusModel((i-17)/2), delta)
		g.SetLineFleet(g.NetworkSelected, c)
	case 23:
		c.Model = (c.Model + 1) % sim.BusModelCount
		c.MixedModels = false
		g.SetLineFleet(g.NetworkSelected, c)
	case 24:
		c.PlayerModel = (c.PlayerModel + 1) % sim.BusModelCount
		g.SetLineFleet(g.NetworkSelected, c)
	case 16:
		g.PlanTransitLine(g.NetworkSelected)
	case 15:
		g.SelectPlayerLine(g.NetworkSelected)
	default:
		if i >= 2 && i < 8 && i-2 < g.LineCount {
			g.NetworkSelected = i - 2
		}
	}
}
func (g *Game) drawNetworkPanel(screen *ebiten.Image) {
	if !g.NetworkOpen {
		if !g.Settings.IsOpen && !g.OperationsPanel.IsOpen {
			textui.DrawText(screen, fmt.Sprintf("%s · [T] %s", g.Route.Name, locale.T("线路管理", "Transit lines")), float64(screen.Bounds().Dx())-250, float64(screen.Bounds().Dy())-24, 12, sim.RGBA(125, 211, 252, 255))
		}
		return
	}
	sw, sh := float64(screen.Bounds().Dx()), float64(screen.Bounds().Dy())
	w, h := 760.0, 690.0
	g.networkButtons = [25]sim.Rect{}
	x, y := (sw-w)/2, (sh-h)/2
	vector.DrawFilledRect(screen, 0, 0, float32(sw), float32(sh), sim.RGBA(0, 0, 0, 170), true)
	vector.DrawFilledRect(screen, float32(x), float32(y), float32(w), float32(h), sim.RGBA(15, 23, 42, 255), true)
	txt := func(s string, dx, dy float64, size float64) {
		textui.DrawText(screen, s, x+dx, y+dy, size, sim.RGBA(226, 232, 240, 255))
	}
	txt(locale.T("公交线路管理 · 已暂停", "Transit network · Paused"), 24, 20, 20)
	total := 0
	for i := 0; i < g.LineCount; i++ {
		total += g.LineFleets[i].BusCount
	}
	txt(locale.Tf("%d 条运营线路 · 共 %d 辆 AI 公交", "%d active lines · %d AI buses", g.LineCount, total), 24, 64, 15)
	button := func(i int, b sim.Rect, label string, active bool) {
		g.networkButtons[i] = b
		col := sim.RGBA(51, 65, 85, 255)
		if active {
			col = sim.RGBA(3, 105, 161, 255)
		}
		vector.DrawFilledRect(screen, float32(b.X), float32(b.Y), float32(b.W), float32(b.H), col, true)
		textui.DrawText(screen, label, b.X+12, b.Y+10, 13, sim.RGBA(248, 250, 252, 255))
	}
	button(0, sim.Rect{X: x + 526, Y: y + 52, W: 56, H: 36}, "-", false)
	button(1, sim.Rect{X: x + 600, Y: y + 52, W: 56, H: 36}, "+", false)
	for i := 0; i < 6; i++ {
		bx, by := x+24+float64(i%2)*326, y+110+float64(i/2)*64
		title := fmt.Sprintf("[%d] Line %d", i+1, i+1)
		if i == g.PlayerLine {
			title += locale.T(" · 玩家", " · Player")
		}
		if i >= g.LineCount {
			title += locale.T(" · 未启用", " · Inactive")
		}
		button(i+2, sim.Rect{X: bx, Y: by, W: 316, H: 56}, title, i == g.NetworkSelected)
		if i < g.LineCount {
			c := g.LineFleets[i].Normalize()
			stops := len(g.Lines[i].StopIDs)
			if stops > 1 && g.Lines[i].StopIDs[0] == g.Lines[i].StopIDs[stops-1] {
				stops--
			}
			textui.DrawText(screen, locale.Tf("%d站 · AI %d 辆 · %d–%d km/h", "%d stops · %d AI · %d–%d km/h", stops, c.BusCount, c.SpeedRange.MinKPH, c.SpeedRange.MaxKPH), bx+12, by+34, 12, sim.RGBA(186, 230, 253, 255))
		}
	}
	c := g.LineFleets[g.NetworkSelected].Normalize()
	txt(locale.Tf("Line %d 车型配置 · AI 总数 %d / 8", "Line %d models · AI total %d / 8", g.NetworkSelected+1, c.BusCount), 24, 316, 16)
	row := func(title string, value, minus int, dy float64) {
		txt(title, 24, dy+10, 14)
		button(minus, sim.Rect{X: x + 520, Y: y + dy, W: 44, H: 34}, "-", false)
		txt(fmt.Sprint(value), 592, dy+10, 14)
		button(minus+1, sim.Rect{X: x + 650, Y: y + dy, W: 44, H: 34}, "+", false)
	}
	counts := c.Counts()
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		row(locale.Tf("%s · %d 人", "%s · %d passengers", locale.T(model.Name(), model.EnglishName()), model.Capacity()), counts[model], 17+int(model)*2, 348+float64(model)*40)
	}
	row(locale.T("每车最高车速下限（km/h）", "Minimum top speed (km/h)"), c.SpeedRange.MinKPH, 11, 474)
	row(locale.T("每车最高车速上限（km/h）", "Maximum top speed (km/h)"), c.SpeedRange.MaxKPH, 13, 514)
	button(23, sim.Rect{X: x + 24, Y: y + 558, W: 340, H: 34}, locale.Tf("统一 AI 车型：%s", "Uniform AI: %s", locale.T(c.Model.Name(), c.Model.EnglishName())), !c.MixedModels)
	button(24, sim.Rect{X: x + 380, Y: y + 558, W: 350, H: 34}, locale.Tf("玩家用车：%s", "Player bus: %s", locale.T(c.PlayerModel.Name(), c.PlayerModel.EnglishName())), true)
	txt(locale.T("三种车型可混编；双层 / 铰接加速度为普通车的 90% / 85%。", "Mix models; double-decker / articulated acceleration: 90% / 85%."), 24, 602, 12)
	pending, retiring := 0, 0
	for _, b := range g.AIBusMgr.Buses {
		if b.LineIndex == g.NetworkSelected {
			if b.Retiring {
				retiring++
			} else if b.Bus.PendingModel != nil {
				pending++
			}
		}
	}
	if g.NetworkSelected == g.PlayerLine && g.Bus.PendingModel != nil {
		pending++
	}
	txt(locale.Tf("终点站下客后换车；待换 %d 辆，待退役 %d 辆。", "Replace after terminal alighting; %d replacements, %d retirements pending.", pending, retiring), 24, 621, 12)
	button(15, sim.Rect{X: x + 24, Y: y + 648, W: 210, H: 30}, locale.T("设为玩家运行线路", "Drive this line"), g.NetworkSelected == g.PlayerLine)
	button(16, sim.Rect{X: x + 246, Y: y + 648, W: 300, H: 30}, locale.T("规划该线路 [TAB]", "Plan this line [TAB]"), true)
	button(8, sim.Rect{X: x + 570, Y: y + 648, W: 160, H: 30}, locale.T("完成 [T]", "Done [T]"), true)
}
