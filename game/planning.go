package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) updatePlanningMode(dt float64) {
	// Always lock rotation to 0 in planning mode for upright North-Up map orientation
	g.Camera.Rotation = 0
	g.Camera.TargetRotation = 0

	// Perspective toggle hotkey (V key)
	if inpututil.IsKeyJustPressed(ebiten.KeyV) {
		newMode := g.Camera.ToggleMode()
		g.Settings.CameraViewMode = newMode
		switch newMode {
		case sim.CameraModeHeadingUp:
			g.HUD.AddToast("视角预设: 【车头朝上 (跟随)】 - 将在返回驾驶模式时生效", false)
		case sim.CameraModeSouthUp:
			g.HUD.AddToast("视角预设: 【固定正南 (倒置)】 - 将在返回驾驶模式时生效", false)
		default:
			g.HUD.AddToast("视角预设: 【固定正北 (标准)】 - 将在返回驾驶模式时生效", false)
		}
	}

	// Mouse Zoom with expanded range for large map
	_, wheelY := ebiten.Wheel()
	if wheelY != 0 {
		g.Camera.TargetZoom = sim.Clamp(g.Camera.TargetZoom+wheelY*0.03, 0.04, 1.4)
		g.Camera.Zoom = g.Camera.TargetZoom
	}

	// Mouse Pan / Drag
	mx, my := ebiten.CursorPosition()
	mouseScreen := sim.Vec2{X: float64(mx), Y: float64(my)}

	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		// Check top-bar mode badge click to exit plan mode
		if g.HUD.HandleModeBtnClick(mouseScreen) {
			g.TogglePlanMode()
			return
		}

		// Check top-bar map theme button click
		if toggled := g.HUD.HandleMapThemeBtnClick(mouseScreen, g.Settings); toggled {
			g.SetMapTheme(g.Settings.MapTheme)
			return
		}

		// Check top-bar camera button click
		if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, false)
			return
		}

		// Check interactive buttons on planning panel first
		if g.HUD.HandlePlanUndoClick(mouseScreen) {
			g.UndoLastPlannedStop()
			return
		}
		if g.HUD.HandlePlanClearClick(mouseScreen) {
			g.ClearPlannedRoute()
			return
		}
		if g.HUD.IsInsidePlanPanel(mouseScreen) {
			// Clicked inside planning panel: do not initiate world map dragging
			return
		}

		// Check if clicked on a bus stop
		mouseWorld := g.Camera.ScreenToWorld(mouseScreen)
		clickedStopID := -1
		for _, s := range g.World.Stops {
			if s.Pos.Distance(mouseWorld) < 65 {
				clickedStopID = s.ID
				break
			}
		}

		if clickedStopID >= 0 {
			prevLen := len(g.Route.StopIDs)
			g.Route.AddStop(clickedStopID)
			if len(g.Route.StopIDs) > prevLen {
				g.planModeRouteModified = true
			}
			stopName := g.World.Stops[clickedStopID].LocalizedName()
			g.HUD.AddToast(fmt.Sprintf(locale.T("已将站点 [%s] 添加至路线", "Added stop [%s] to route"), stopName), true)
		} else {
			// Start dragging map
			g.isDragging = true
			g.dragStartPos = mouseScreen
			g.camStartPos = g.Camera.Pos
		}
	}

	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) && g.isDragging {
		delta := mouseScreen.Sub(g.dragStartPos)
		newPos := g.camStartPos.Sub(delta.Mul(1.0 / g.Camera.Zoom))
		g.Camera.TargetPos = newPos
		g.Camera.Pos = newPos
	} else {
		g.isDragging = false
	}

	// Arrow keys / WASD for map panning in plan mode
	panSpd := 750.0 * dt / g.Camera.Zoom
	panned := false
	if ebiten.IsKeyPressed(ebiten.KeyA) || ebiten.IsKeyPressed(ebiten.KeyLeft) {
		g.Camera.TargetPos.X -= panSpd
		panned = true
	}
	if ebiten.IsKeyPressed(ebiten.KeyD) || ebiten.IsKeyPressed(ebiten.KeyRight) {
		g.Camera.TargetPos.X += panSpd
		panned = true
	}
	if ebiten.IsKeyPressed(ebiten.KeyW) || ebiten.IsKeyPressed(ebiten.KeyUp) {
		g.Camera.TargetPos.Y -= panSpd
		panned = true
	}
	if ebiten.IsKeyPressed(ebiten.KeyS) || ebiten.IsKeyPressed(ebiten.KeyDown) {
		g.Camera.TargetPos.Y += panSpd
		panned = true
	}
	if panned {
		g.Camera.Pos = g.Camera.TargetPos
	}

	// Undo / Cancel last station with Backspace, Z, Delete, or U key
	if inpututil.IsKeyJustPressed(ebiten.KeyBackspace) || inpututil.IsKeyJustPressed(ebiten.KeyZ) ||
		inpututil.IsKeyJustPressed(ebiten.KeyDelete) || inpututil.IsKeyJustPressed(ebiten.KeyU) {
		g.UndoLastPlannedStop()
	}

	// Clear route with C key
	if inpututil.IsKeyJustPressed(ebiten.KeyC) {
		g.ClearPlannedRoute()
	}
}

// UndoLastPlannedStop cancels/removes the last station added to the planned route.
func (g *Game) UndoLastPlannedStop() bool {
	if g.Route == nil {
		return false
	}
	if removedID, ok := g.Route.RemoveLastStop(); ok {
		g.planModeRouteModified = true
		stopName := fmt.Sprintf("#%d", removedID)
		if g.World != nil && removedID >= 0 && removedID < len(g.World.Stops) {
			stopName = g.World.Stops[removedID].LocalizedName()
		}
		if len(g.Route.StopIDs) > 0 {
			g.HUD.AddToast(fmt.Sprintf(locale.T("已撤销最后一个站点 [%s] (路线剩余 %d 站)", "Removed last stop [%s] (%d remaining)"), stopName, len(g.Route.StopIDs)), false)
		} else {
			g.HUD.AddWarningToast(fmt.Sprintf(locale.T("已撤销最后一个站点 [%s]，当前路线已为空！", "Removed last stop [%s], route is now empty!"), stopName))
		}
		return true
	}
	g.HUD.AddWarningToast(locale.T("当前路线中已无站点可撤销！", "No stops to undo in current route!"))
	return false
}

// ClearPlannedRoute clears all stations from the planned route.
func (g *Game) ClearPlannedRoute() {
	if g.Route == nil {
		return
	}
	g.Route.Clear()
	g.planModeRouteModified = true
	g.HUD.AddToast(locale.T("已清空规划路线，请重新规划！", "Route cleared, please click stations to replan!"), false)
}

func intSlicesEqual(a, b []int) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// TogglePlanMode toggles between Drive Mode and Route Planning Mode.
// When returning to Drive Mode after replanning the route, both the player's bus
// and all AI buses are reset to the first stop of the replanned route.
func (g *Game) TogglePlanMode() {
	if !g.IsPlanMode {
		// Enter Plan Mode
		g.IsPlanMode = true
		g.planModeInitialStops = append([]int(nil), g.Route.StopIDs...)
		g.planModeRouteModified = false

		// Instant transition to panoramic planning view
		g.Camera.TargetPos = sim.Vec2{X: 0, Y: 0}
		g.Camera.Pos = sim.Vec2{X: 0, Y: 0}
		g.Camera.TargetZoom = 0.08
		g.Camera.Zoom = 0.08
		g.Camera.Rotation = 0
		g.Camera.TargetRotation = 0
		g.HUD.AddToast(locale.T("已进入【线路规划模式】：游戏运行与客流生成已暂停 (点击站点加入路线，按TAB返回)", "[Plan Mode] Simulation & passenger spawning paused (Click stops to add to route, press TAB to return)"), false)
	} else {
		// Exit Plan Mode -> Return to Driver Mode
		if len(g.Route.StopIDs) == 0 {
			g.HUD.AddWarningToast(locale.T("【路线未完成】当前路线为空，请至少点击选择 1 个站点规划路线后再返回驾驶！", "[Empty Route] Route is empty. Please select at least 1 stop before driving!"))
			return
		}

		g.IsPlanMode = false

		// Check if route was replanned
		isReplanned := g.planModeRouteModified || !intSlicesEqual(g.Route.StopIDs, g.planModeInitialStops)
		if isReplanned {
			g.ResetBusesToRouteStart()
			firstStopID := g.Route.StopIDs[0]
			firstStopName := ""
			if firstStopID >= 0 && firstStopID < len(g.World.Stops) {
				firstStopName = g.World.Stops[firstStopID].LocalizedName()
			}
			g.HUD.AddToast(fmt.Sprintf(locale.T("已进入【驾驶模式】：路线重新规划完成！车辆与 AI 车队已就位始发站 [%s]，开始运营发车！", "[Drive Mode] Route replanned! Bus and AI fleet reset to starting station [%s]!"), firstStopName), true)
		} else {
			// Instant snap back to driver mode without resetting vehicle positions
			g.Camera.FollowBus(g.Bus)
			g.Camera.Pos = g.Camera.TargetPos
			g.Camera.TargetZoom = 1.0
			g.Camera.Zoom = 1.0
			g.Camera.Rotation = g.Camera.TargetRotation
			g.HUD.AddToast(locale.T("已进入【驾驶模式】：开始运营发车！", "[Drive Mode] Starting operation!"), true)
		}
	}
}
