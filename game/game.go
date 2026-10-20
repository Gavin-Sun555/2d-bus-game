package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

type GameState int

const (
	StateMainMenu GameState = iota
	StatePlaying
)

type Game struct {
	Width  int
	Height int

	State            GameState
	SelectedMapTheme sim.MapTheme
	menuCardBounds   [6]sim.Rect
	menuStartBounds  sim.Rect
	menuLangBounds   sim.Rect

	Bus           *sim.Bus
	World         *sim.World
	Route         *sim.Route
	Camera        *sim.Camera
	HUD           *ui.HUD
	TrafficLights *sim.TrafficLightSystem
	TrafficMgr    *traffic.TrafficManager
	Settings      *ui.Settings

	// AI Autopilot & Autonomous Fleet
	AutopilotDriver        *autopilot.AutopilotDriver
	AIBusMgr               *traffic.AIBusManager
	AutopilotDeadlockTimer float64
	DeadlockTracker        sim.StationaryTracker
	obstacles              []autopilot.VehicleObstacle

	IsPlanMode bool
	Cash       int

	// Track lane width changes
	lastLaneWidth float64

	// Mouse dragging for planning mode
	isDragging   bool
	dragStartPos sim.Vec2
	camStartPos  sim.Vec2

	// Route planning tracking
	planModeInitialStops  []int
	planModeRouteModified bool

	// Boarding tracking
	boardedThisStop bool

	// Stop Request Bell & Skip Stop (飞站) tracking
	stopBellRung        bool
	passengersAlighting int
	bellToastShown      bool
	skipCooldown        float64
	lastSkippedStopID   int
}

func NewGame(width, height int) *Game {
	settings := ui.NewSettings()
	initLaneWidth := settings.GetLaneWidth()

	world := sim.NewWorld(initLaneWidth)
	route := sim.GetStandardRouteForMap(settings.MapTheme, world)

	g := &Game{
		Width:                 width,
		Height:                height,
		State:                 StateMainMenu,
		SelectedMapTheme:      sim.MapThemeMetropolis,
		World:                 world,
		Route:                 route,
		Camera:                sim.NewCamera(width, height),
		HUD:                   ui.NewHUD(),
		TrafficLights:         sim.NewTrafficLightSystem(),
		TrafficMgr:            traffic.NewTrafficManager(initLaneWidth),
		Settings:              settings,
		AutopilotDriver:       autopilot.NewAutopilotDriver(),
		AIBusMgr:              traffic.NewAIBusManager(),
		IsPlanMode:            false,
		Cash:                  100, // Starting budget
		lastLaneWidth:         initLaneWidth,
		planModeInitialStops:  make([]int, 0),
		planModeRouteModified: false,
		lastSkippedStopID:     -1,
	}

	// Adapt traffic lights and civilian AI traffic to active world network
	g.TrafficMgr.AIBusMgr = g.AIBusMgr
	g.TrafficLights.BuildForWorld(g.World)
	g.TrafficMgr.BuildForWorld(g.World, traffic.MaxTrafficCars, initLaneWidth)

	// Spawn bus parked inside the first station's bay box
	firstStop := g.World.Stops[0]
	if len(g.Route.StopIDs) > 0 {
		startID := g.Route.StopIDs[0]
		if startID >= 0 && startID < len(g.World.Stops) {
			firstStop = g.World.Stops[startID]
		}
	}
	g.Bus = sim.NewBus(firstStop.Pos.X, firstStop.Pos.Y, firstStop.Heading)
	g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos

	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.BuildFleet(g.World, g.Settings.AIBusCount, initLaneWidth, g.Bus)
	}

	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()

	g.HUD.AddToast("欢迎来到超大都市 2D 公交模拟器！12 大特色站点已上线", true)
	g.HUD.AddToast("【全新机制】路线仅指引至下一站；站内无人且车内无下车铃时可按 [F] 飞站！", false)
	g.HUD.AddToast("【始发站已就位】公交车已停靠在中央枢纽黄色框内，按 [E] 开门上下客！", false)

	return g
}

func (g *Game) Update() error {
	if g.State == StateMainMenu {
		g.updateMainMenu()
		return nil
	}

	// Playing state: Check if returning to menu was requested via HUD or Settings
	if g.HUD.ReturnToMenuRequested {
		g.HUD.ReturnToMenuRequested = false
		g.State = StateMainMenu
		return nil
	}
	if g.Settings.ReturnToMenuRequested {
		g.Settings.ReturnToMenuRequested = false
		g.State = StateMainMenu
		return nil
	}

	dt := 1.0 / 60.0

	// 1. Settings menu takes priority
	g.Settings.Update(g.Camera)
	if g.Settings.ReturnToMenuRequested {
		g.Settings.ReturnToMenuRequested = false
		g.State = StateMainMenu
		return nil
	}

	// Check if lane width customized
	currentLaneW := g.Settings.GetLaneWidth()
	if currentLaneW != g.lastLaneWidth {
		g.lastLaneWidth = currentLaneW
		g.World.UpdateLaneWidth(currentLaneW)
		if g.AIBusMgr != nil {
			g.AIBusMgr.BuildFleet(g.World, g.Settings.AIBusCount, currentLaneW, g.Bus)
		}
		g.HUD.AddToast(fmt.Sprintf("全城道路标线与车道宽度已动态切换为: %.0f px", currentLaneW), true)
	}

	// Check if AI bus fleet counts were updated in Settings
	if g.Settings.AIBusFleetChanged {
		if g.AIBusMgr != nil {
			g.AIBusMgr.BuildFleet(g.World, g.Settings.AIBusCount, currentLaneW, g.Bus)
		}
		g.Settings.AIBusFleetChanged = false
		g.HUD.AddToast(fmt.Sprintf("路线AI公交车队已调度就绪: 共 %d 辆 (环线循环)", g.Settings.AIBusCount), true)
	}

	if g.Settings.IsOpen {
		// Game paused while in settings menu
		return nil
	}

	// Hotkey L: Toggle interface language (Chinese / English)
	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		newLang := locale.ToggleLanguage()
		g.Settings.Language = newLang
		g.HUD.AddToast(locale.T("已切换语言: 简体中文", "Language switched: English"), true)
	}

	// Hotkey M: Inform player to use Main Menu
	if inpututil.IsKeyJustPressed(ebiten.KeyM) {
		g.HUD.AddToast(locale.T("【地图更换提示】请点击右上角 [主菜单/换图] 或按 [ESC] 退入主菜单更换城市！", "[Map Change] Click top-right [Menu/Maps] or press [ESC] to switch city!"), false)
	}

	// Hotkey R: Reload custom map from custom_map.json
	if inpututil.IsKeyJustPressed(ebiten.KeyR) {
		if g.SelectedMapTheme == sim.MapThemeCustom {
			_ = g.ReloadCustomMap()
		} else {
			g.HUD.AddToast(locale.T("提示: custom_map.json 支持热重载！按 ESC 退入主菜单按 [6] 可进入自定义地图", "Tip: custom_map.json supports hot-reload! Press ESC to return to menu and press [6] for Custom Map"), false)
		}
	}

	// 2. Mode switching (TAB)
	if inpututil.IsKeyJustPressed(ebiten.KeyTab) {
		g.TogglePlanMode()
	}

	// 3. Game simulation (Traffic lights, AI traffic, AI buses, passenger arrivals, violations)
	// Fully PAUSED during Planning Mode so the city stops and no new passengers generate while planning!
	if !g.IsPlanMode {
		// Traffic lights update
		g.TrafficLights.Update(dt)

		// AI Traffic vehicles update with dynamic lane width and active world for roundabout navigation
		g.TrafficMgr.Update(dt, g.Bus, g.TrafficLights, g.Settings.TrafficDensity, currentLaneW, g.World)

		// AI Route buses update
		if g.AIBusMgr != nil {
			g.AIBusMgr.Update(dt, g.Bus, g.World, g.TrafficLights, g.TrafficMgr, currentLaneW, g.Settings.TrafficDensity)
		}

		// Red light & lane discipline violation check on player bus
		g.recordTrafficViolation(g.TrafficLights.CheckPlayerViolation(g.Bus, currentLaneW))

		// Straight road surveillance camera wrong-way violation check
		g.recordTrafficViolation(g.TrafficLights.CheckRoadCameraViolation(g.Bus))

		// Collision check between player bus and AI cars
		g.checkBusCarCollisions()

		// World update (passenger arrivals ONLY at bus stops on the active route)
		g.World.Update(dt, g.Route)
	}

	// 7. Planning / Driver Mode inputs
	if g.IsPlanMode {
		g.updatePlanningMode(dt)
	} else {
		g.updateDriverMode(dt)
	}

	// 8. Camera update
	g.Camera.Update(dt)

	// 9. HUD toasts update
	g.HUD.Update(dt)

	return nil
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return g.Width, g.Height
}

func (g *Game) recordTrafficViolation(violated bool, notice string) {
	if !violated {
		return
	}
	if g.Settings.TrafficFinesEnabled {
		g.Cash -= g.Settings.FineAmount
		g.HUD.AddWarningToast(fmt.Sprintf("%s 扣除罚金 -$%d", notice, g.Settings.FineAmount))
		return
	}
	g.HUD.AddWarningToast(fmt.Sprintf("%s (当前设置已豁免罚款)", notice))
}
