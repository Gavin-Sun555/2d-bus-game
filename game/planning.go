package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/platforminput"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) updatePlanningMode(dt float64) {
	if inpututil.IsKeyJustPressed(ebiten.KeyF) && (ebiten.IsKeyPressed(ebiten.KeyControl) || ebiten.IsKeyPressed(ebiten.KeyMeta)) {
		g.openStationSearch()
		return
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonRight) || (ebiten.IsKeyPressed(ebiten.KeyShift) && inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft)) {
		x, y := ebiten.CursorPosition()
		point := sim.Vec2{X: float64(x), Y: float64(y)}
		if !g.HUD.IsInsideTopBar(point) && !g.HUD.IsInsidePlanPanel(point) {
			if id := g.World.StopAtScreen(g.Camera, point, 12); id >= 0 {
				g.openStopNameEditor(id)
				return
			}
		}
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyH) {
		g.HideOtherRoutes = !g.HideOtherRoutes
		if g.HideOtherRoutes {
			g.HUD.AddToast(locale.T("已隐藏其他线路 [H]", "Other lines hidden [H]"), false)
		} else {
			g.HUD.AddToast(locale.T("已显示其他线路 [H]", "Other lines shown [H]"), false)
		}
	}
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

	// Mouse Scroll Wheel Zoom & Trackpad Two-Finger Pinch Zoom
	minZoom := 0.04
	if g.World != nil && g.World.LargeGrid {
		minZoom = 0.01
	}
	if wheelY := platforminput.TakeMouseWheelY(); wheelY != 0 {
		g.Camera.TargetZoom = sim.Clamp(g.Camera.TargetZoom+wheelY*0.03, minZoom, 1.4)
		g.Camera.Zoom = g.Camera.TargetZoom
	}
	if pinchDelta := g.consumePlanningPinchDelta(); pinchDelta != 0 {
		g.Camera.TargetZoom = applyPlanningPinchZoom(g.Camera.TargetZoom, pinchDelta, minZoom, 1.4)
		g.Camera.Zoom = g.Camera.TargetZoom
	}

	// Mouse Pan / Drag
	mx, my := ebiten.CursorPosition()
	mouseScreen := sim.Vec2{X: float64(mx), Y: float64(my)}

	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		if g.HUD.PlanSearchBtnBounds.Contains(mouseScreen) {
			g.openStationSearch()
			return
		}
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

		if g.HUD.SettingsBtnBounds.Contains(mouseScreen) {
			g.Settings.IsOpen = true
			return
		}
		if g.HUD.IsInsideTopBar(mouseScreen) {
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
		clickedStopID := g.World.StopAtScreen(g.Camera, mouseScreen, 12)

		if clickedStopID >= 0 {
			prevLen := len(g.planningRoute().StopIDs)
			g.planningRoute().AddStop(clickedStopID)
			if len(g.planningRoute().StopIDs) > prevLen {
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
	if g.planningRoute() == nil {
		return false
	}
	if removedID, ok := g.planningRoute().RemoveLastStop(); ok {
		g.planModeRouteModified = true
		stopName := fmt.Sprintf("#%d", removedID)
		if g.World != nil && removedID >= 0 && removedID < len(g.World.Stops) {
			stopName = g.World.Stops[removedID].LocalizedName()
		}
		if len(g.planningRoute().StopIDs) > 0 {
			g.HUD.AddToast(fmt.Sprintf(locale.T("已撤销最后一个站点 [%s] (路线剩余 %d 站)", "Removed last stop [%s] (%d remaining)"), stopName, len(g.planningRoute().StopIDs)), false)
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
	if g.planningRoute() == nil {
		return
	}
	g.planningRoute().Clear()
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

// planningRoute keeps route editing independent of the service the player drives.
func (g *Game) planningRoute() *sim.Route {
	if g.IsPlanMode && g.planRoute != nil {
		return g.planRoute
	}
	return g.Route
}

// PlanTransitLine opens any enabled service without changing the player's line.
func (g *Game) PlanTransitLine(index int) bool {
	if index < 0 || index >= g.LineCount || index >= len(g.Lines) {
		return false
	}
	route := g.activeLines()[index]
	if g.IsPlanMode && g.planningRoute() != route && !g.finishPlannedRoute() {
		return false
	}
	if !g.IsPlanMode {
		g.TogglePlanMode()
	}
	if g.planRoute != route {
		g.planRoute = route
		g.rememberPlannedRoute(route)
	}
	g.Camera.Pos = sim.Vec2{}
	g.Camera.TargetPos = sim.Vec2{}
	planZoom := .04
	if g.World != nil && g.World.LargeGrid {
		planZoom = .012
	}
	g.Camera.Zoom = planZoom
	g.Camera.TargetZoom = planZoom
	g.NetworkSelected = index
	g.NetworkOpen = false
	return true
}

func (g *Game) rememberPlannedRoute(route *sim.Route) {
	g.planModeInitialStops = append([]int(nil), route.StopIDs...)
	g.planModeInitialIndex = route.CurrentIndex
	g.planModeRouteModified = false
}

// finishPlannedRoute dispatches changed services while keeping unrelated buses running.
func (g *Game) finishPlannedRoute() bool {
	route := g.planningRoute()
	if len(route.StopIDs) == 0 {
		g.HUD.AddWarningToast(locale.T("【路线未完成】请至少选择 1 个站点后再完成规划！", "[Empty Route] Select at least one stop before finishing!"))
		return false
	}
	if !intSlicesEqual(route.StopIDs, g.planModeInitialStops) {
		if !g.routeWithinTurnLimit(route.StopIDs) {
			return false
		}
		g.commitRouteChange(route)
		return true
	}
	// Clearing and rebuilding the same itinerary may have moved its cursor.
	route.CurrentIndex = g.planModeInitialIndex
	route.InvalidateDescription()
	g.rememberPlannedRoute(route)
	return true
}

// TogglePlanMode toggles between driving and editing the selected service.
func (g *Game) TogglePlanMode() {
	if !g.IsPlanMode {
		g.syncCurrentLineSettings()
		g.IsPlanMode = true
		g.planRoute = g.Route
		g.rememberPlannedRoute(g.Route)
		g.Camera.TargetPos = sim.Vec2{}
		g.Camera.Pos = sim.Vec2{}
		planZoom := 0.04
		if g.World != nil && g.World.LargeGrid {
			planZoom = 0.012
		}
		g.Camera.TargetZoom = planZoom
		g.Camera.Zoom = planZoom
		g.Camera.Rotation = 0
		g.Camera.TargetRotation = 0
		platforminput.ResetScrollAndPinch()
		g.pinchTouchActive = false
		g.pinchLastTouchDist = 0
		g.HUD.AddToast(locale.T("路线规划：左键加站，右键改名，[H] 其他线路，[TAB] 完成", "Planning: click to add, right-click to rename, [H] other lines, [TAB] finish"), false)
		return
	}
	if !g.finishPlannedRoute() {
		return
	}
	g.IsPlanMode = false
	g.closeStationSearch()
	g.planRoute = nil
	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Camera.TargetPos
	g.Camera.RestoreDrivingZoom()
	g.Camera.Rotation = g.Camera.TargetRotation
	g.HUD.AddToast(locale.T("线路规划已完成，恢复运营。", "Route planning complete. Service resumed."), true)
}

func (g *Game) consumePlanningPinchDelta() float64 {
	delta := platforminput.TakePinchDelta()

	// Also support two-finger touch pinch on touch-enabled devices.
	g.touchIDsBuf = ebiten.AppendTouchIDs(g.touchIDsBuf[:0])
	if len(g.touchIDsBuf) == 2 {
		x0, y0 := ebiten.TouchPosition(g.touchIDsBuf[0])
		x1, y1 := ebiten.TouchPosition(g.touchIDsBuf[1])
		dist := math.Hypot(float64(x1-x0), float64(y1-y0))
		if g.pinchTouchActive && g.pinchLastTouchDist > 1.0 && dist > 1.0 {
			delta += (dist - g.pinchLastTouchDist) / 220.0
		}
		g.pinchLastTouchDist = dist
		g.pinchTouchActive = true
	} else {
		g.pinchTouchActive = false
		g.pinchLastTouchDist = 0
	}

	return delta
}

func applyPlanningPinchZoom(currentZoom, pinchDelta, minZoom, maxZoom float64) float64 {
	if pinchDelta == 0 || math.IsNaN(pinchDelta) || math.IsInf(pinchDelta, 0) {
		return sim.Clamp(currentZoom, minZoom, maxZoom)
	}
	scale := math.Exp(pinchDelta * 1.5)
	next := currentZoom*scale + pinchDelta*0.12
	return sim.Clamp(next, minZoom, maxZoom)
}
