package game

import (
	"bytes"
	_ "embed"
	"fmt"
	"image"
	"image/color"
	_ "image/png"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

//go:embed assets/logo_512.png
var menuLogoBytes []byte
var menuLogoImg *ebiten.Image

type GameState int

const (
	StateMainMenu GameState = iota
	StatePlaying
)

type Game struct {
	Width  int
	Height int

	State            GameState
	SelectedMapTheme MapTheme
	menuCardBounds   [6]Rect
	menuStartBounds  Rect
	menuLangBounds   Rect

	Bus           *Bus
	World         *World
	Route         *Route
	Camera        *Camera
	HUD           *HUD
	TrafficLights *TrafficLightSystem
	TrafficMgr    *TrafficManager
	Settings      *Settings

	// AI Autopilot & Autonomous Fleet
	AutopilotDriver        *AutopilotDriver
	AIBusMgr               *AIBusManager
	AutopilotDeadlockTimer float64

	IsPlanMode bool
	Cash       int

	// Track lane width changes
	lastLaneWidth float64

	// Mouse dragging for planning mode
	isDragging   bool
	dragStartPos Vec2
	camStartPos  Vec2

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
	settings := NewSettings()
	initLaneWidth := settings.GetLaneWidth()

	world := NewWorld(initLaneWidth)
	route := GetStandardRouteForMap(settings.MapTheme, world)

	g := &Game{
		Width:                 width,
		Height:                height,
		State:                 StateMainMenu,
		SelectedMapTheme:      MapThemeMetropolis,
		World:                 world,
		Route:                 route,
		Camera:                NewCamera(width, height),
		HUD:                   NewHUD(),
		TrafficLights:         NewTrafficLightSystem(),
		TrafficMgr:            NewTrafficManager(initLaneWidth),
		Settings:              settings,
		AutopilotDriver:       NewAutopilotDriver(),
		AIBusMgr:              NewAIBusManager(),
		IsPlanMode:            false,
		Cash:                  100, // Starting budget
		lastLaneWidth:         initLaneWidth,
		planModeInitialStops:  make([]int, 0),
		planModeRouteModified: false,
		lastSkippedStopID:     -1,
	}

	// Adapt traffic lights and civilian AI traffic to active world network
	g.TrafficMgr.AIBusMgr = g.AIBusMgr
	g.TrafficMgr.OnPlayerBusDeadlockReset = func() {
		if g.Settings.AutopilotEnabled {
			g.ResetToStartingStation()
		}
	}
	g.TrafficLights.BuildForWorld(g.World)
	g.TrafficMgr.BuildForWorld(g.World, MaxTrafficCars, initLaneWidth)

	// Spawn bus parked inside the first station's bay box
	firstStop := g.World.Stops[0]
	if len(g.Route.StopIDs) > 0 {
		startID := g.Route.StopIDs[0]
		if startID >= 0 && startID < len(g.World.Stops) {
			firstStop = g.World.Stops[startID]
		}
	}
	g.Bus = NewBus(firstStop.Pos.X, firstStop.Pos.Y, firstStop.Heading)
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

func (g *Game) EnterCity(theme MapTheme) {
	g.SetMapTheme(theme)
	g.State = StatePlaying
	g.IsPlanMode = false
}

func (g *Game) SetMapTheme(theme MapTheme) {
	g.Settings.MapTheme = theme
	g.SelectedMapTheme = theme
	currentLaneW := g.Settings.GetLaneWidth()
	g.World.BuildMap(theme, currentLaneW)

	// Rebuild traffic lights and AI civilian vehicles for active map
	g.TrafficLights.BuildForWorld(g.World)
	g.TrafficMgr.BuildForWorld(g.World, MaxTrafficCars, currentLaneW)
	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
	}

	// Clear Dijkstra memoized path cache because roads/roundabouts differ
	ClearRoutePathCache()

	// Update starting route to standard route for this map
	g.Route = GetStandardRouteForMap(theme, g.World)
	g.Route.CurrentIndex = 0
	g.Route.cachedDescDirty = true

	// Reposition bus cleanly to the first stop of the route
	firstStop := g.World.Stops[0]
	if len(g.Route.StopIDs) > 0 {
		startID := g.Route.StopIDs[0]
		if startID >= 0 && startID < len(g.World.Stops) {
			firstStop = g.World.Stops[startID]
		}
	}
	g.Bus.Pos = firstStop.Pos
	g.Bus.Heading = firstStop.Heading
	g.Bus.Speed = 0
	g.Bus.SteeringAngle = 0
	g.Bus.DoorsOpen = true
	g.Bus.IsBraking = false
	g.Bus.OnShoulder = false
	g.Bus.ClearPassengers()

	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.BuildFleet(g.World, g.Settings.AIBusCount, currentLaneW, g.Bus)
	}

	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos

	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()
	g.HUD.AddToast(fmt.Sprintf("已进入城市地图: 【%s】！", theme.DisplayName()), true)
	g.HUD.AddToast(fmt.Sprintf("风貌特色: %s", theme.Subtitle()), false)
}

// ReloadCustomMap reads custom_map.json and refreshes the world if playing on custom map
func (g *Game) ReloadCustomMap() error {
	customMap, err := LoadCustomMapFromFile("custom_map.json")
	if err != nil {
		g.HUD.AddWarningToast(fmt.Sprintf("读取 custom_map.json 失败: %v", err))
		return err
	}
	g.World.CustomMap = customMap
	if g.SelectedMapTheme == MapThemeCustom {
		g.SetMapTheme(MapThemeCustom)
		g.HUD.AddToast(fmt.Sprintf("已成功重载自定义地图: 【%s】！", customMap.Name), true)
	} else {
		g.HUD.AddToast(fmt.Sprintf("已成功校验自定义地图: 【%s】(可按[6]或[C]切换)", customMap.Name), true)
	}
	return nil
}

func (g *Game) rollAlightingForNextStop() {
	g.bellToastShown = false
	if g.Bus == nil || g.Route == nil {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}
	targetStopID := g.Route.GetCurrentTargetStopID()
	if targetStopID < 0 || g.Bus.PassengerCount == 0 {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}

	alighting := g.Bus.CountPassengersAlightingAt(targetStopID)
	// If heading to the terminus of the route, all remaining passengers on board will alight
	if g.Route.IsTerminusIndex(g.Route.CurrentIndex) {
		if g.Bus.PassengerCount > alighting {
			alighting = g.Bus.PassengerCount
		}
	}

	g.passengersAlighting = alighting
	g.stopBellRung = (g.passengersAlighting > 0)
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
		newLang := ToggleLanguage()
		g.Settings.Language = newLang
		g.HUD.AddToast(T("已切换语言: 简体中文", "Language switched: English"), true)
	}

	// Hotkey M: Inform player to use Main Menu
	if inpututil.IsKeyJustPressed(ebiten.KeyM) {
		g.HUD.AddToast(T("【地图更换提示】请点击右上角 [主菜单/换图] 或按 [ESC] 退入主菜单更换城市！", "[Map Change] Click top-right [Menu/Maps] or press [ESC] to switch city!"), false)
	}

	// Hotkey R: Reload custom map from custom_map.json
	if inpututil.IsKeyJustPressed(ebiten.KeyR) {
		if g.SelectedMapTheme == MapThemeCustom {
			_ = g.ReloadCustomMap()
		} else {
			g.HUD.AddToast(T("提示: custom_map.json 支持热重载！按 ESC 退入主菜单按 [6] 可进入自定义地图", "Tip: custom_map.json supports hot-reload! Press ESC to return to menu and press [6] for Custom Map"), false)
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
		if violated, notice := g.TrafficLights.CheckPlayerViolation(g.Bus, currentLaneW); violated {
			if g.Settings.TrafficFinesEnabled {
				g.Cash -= g.Settings.FineAmount
				msg := fmt.Sprintf("%s 扣除罚金 -$%d", notice, g.Settings.FineAmount)
				g.HUD.AddWarningToast(msg)
			} else {
				msg := fmt.Sprintf("%s (当前设置已豁免罚款)", notice)
				g.HUD.AddWarningToast(msg)
			}
		}

		// Straight road surveillance camera wrong-way violation check
		if violated, notice := g.TrafficLights.CheckRoadCameraViolation(g.Bus); violated {
			if g.Settings.TrafficFinesEnabled {
				g.Cash -= g.Settings.FineAmount
				msg := fmt.Sprintf("%s 扣除罚金 -$%d", notice, g.Settings.FineAmount)
				g.HUD.AddWarningToast(msg)
			} else {
				msg := fmt.Sprintf("%s (当前设置已豁免罚款)", notice)
				g.HUD.AddWarningToast(msg)
			}
		}

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

func (g *Game) checkBusCarCollisions() {
	var activeCars []*TrafficCar
	if g.TrafficMgr != nil {
		activeCars = g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	}
	if len(activeCars) == 0 && (g.AIBusMgr == nil || len(g.AIBusMgr.Buses) == 0) {
		return
	}

	busOBB := OBB{
		Center:  g.Bus.Pos,
		HalfLen: g.Bus.Length / 2, // 48.0
		HalfWid: g.Bus.Width / 2,  // 18.0
		Heading: g.Bus.Heading,
	}

	for _, car := range activeCars {
		carOBB := OBB{
			Center:  car.Pos,
			HalfLen: car.Length / 2, // 23.0 ~ 25.0
			HalfWid: car.Width / 2,  // 11.0
			Heading: car.Heading,
		}

		collides, normal, depth := CheckOBBCollision(busOBB, carOBB)
		if collides && depth > 0 {
			// Firm separation buffer to ensure zero visual overlap
			sepDepth := depth + 2.0

			// normal points from Car to Bus
			toBus := g.Bus.Pos.Sub(car.Pos)
			carForward := Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}
			busForward := Vec2{X: math.Cos(g.Bus.Heading), Y: math.Sin(g.Bus.Heading)}

			isCarFacingBus := carForward.Dot(toBus) > 0
			isBusFacingCar := busForward.Dot(car.Pos.Sub(g.Bus.Pos)) > 0

			isBusStopped := math.Abs(g.Bus.Speed) < 8.0
			isWrongWay := IsBusFacingWrongWay(g.Bus)

			// 1. Positional Separation:
			// Normal points from Car to Bus.
			if isBusStopped {
				// Heavy stopped bus remains firmly planted; car absorbs almost all separation pushback
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.1))
				car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.9))
			} else if car.Speed < 5.0 {
				// Stationary car: bus absorbs pushback so it cannot penetrate
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth))
			} else {
				// Both moving: bus takes 70%, car takes 30%
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.7))
				car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.3))
			}

			// Update busOBB center for subsequent car checks
			busOBB.Center = g.Bus.Pos

			// 2. Velocity & Momentum Resolution:
			// Relative velocity along collision normal
			busVel := Vec2{X: math.Cos(g.Bus.Heading) * g.Bus.Speed, Y: math.Sin(g.Bus.Heading) * g.Bus.Speed}
			carVel := Vec2{X: math.Cos(car.Heading) * car.Speed, Y: math.Sin(car.Heading) * car.Speed}
			relVelAlongNormal := busVel.Sub(carVel).Dot(normal)

			// Normal points from Car to Bus. When moving into each other, relVelAlongNormal < 0
			if relVelAlongNormal < 0 {
				impactSpeed := math.Abs(relVelAlongNormal)

				if isCarFacingBus && (!isBusFacingCar || car.Speed > math.Abs(g.Bus.Speed)) {
					// Car impacted the bus (e.g. car rear-ended or broadsided the bus):
					// Car MUST immediately brake and stop, not accelerate forward!
					car.Speed = 0
					car.IsBraking = true
				} else if isBusFacingCar && g.Bus.Speed > 5.0 {
					// Bus actively rammed the car from behind: push car ahead
					car.Speed = math.Max(car.Speed+20.0, math.Abs(g.Bus.Speed)*0.8+15.0)
					car.IsBraking = false
				} else {
					// Side glance / mutual contact: kill car's excess forward velocity
					car.Speed = math.Min(car.Speed, 10.0)
					car.IsBraking = true
				}

				busSpeedBeforeImpact := g.Bus.Speed

				// 3. Accident Responsibility & Fines Determination:
				// Bus is at fault ONLY IF:
				//  a) Bus is driving wrong-way (逆行) into traffic, OR
				//  b) Player is manually driving and actively rear-ends / reverses into another vehicle.
				// In Autopilot mode, the player is not driving manually; autonomous cruise is never player fault.
				// In cross-traffic/roundabouts, vehicles failing to yield or broadsiding the bus are at fault.
				isBusInRoundabout := g.World != nil && g.World.IsAtRoundabout(g.Bus.Pos)
				toCar := car.Pos.Sub(g.Bus.Pos)
				distToCar := toCar.Length()
				busDotCar := 0.0
				if distToCar > 0.001 {
					busDotCar = busForward.Dot(toCar.Mul(1.0 / distToCar))
				}
				headAngleDiff := math.Abs(NormalizeAngle(g.Bus.Heading - car.Heading))

				isPlayerAtFault := false
				if !g.Settings.AutopilotEnabled {
					if isWrongWay {
						isPlayerAtFault = true
					} else if !isBusStopped {
						// In-lane rear-end collision: bus directly rams the rear of a vehicle heading same direction
						isRearEndRam := busDotCar > 0.70 && busSpeedBeforeImpact > 18.0 && headAngleDiff < 0.85
						isReverseRam := busDotCar < -0.70 && busSpeedBeforeImpact < -12.0
						if isRearEndRam || isReverseRam {
							isPlayerAtFault = true
						}
					}
				}

				// Clamp and stop the bus from ramming through the car
				if isBusFacingCar && g.Bus.Speed > 0 {
					if impactSpeed > 45.0 {
						// High-speed collision: bounce recoil
						g.Bus.Speed = -g.Bus.Speed * 0.25
					} else {
						// Low-speed contact: clamp bus speed to 0
						g.Bus.Speed = 0
					}
				}

				if g.Bus.CrashCooldown <= 0 {
					g.Bus.CrashCooldown = 2.0
					g.Camera.AddShake(math.Min(12.0, 4.0+impactSpeed*0.15))

					if isPlayerAtFault {
						// Player fault: normal damage and fine
						damage := 8.0
						if impactSpeed > 80.0 {
							damage = 18.0
						}
						g.Bus.Durability = math.Max(0, g.Bus.Durability-damage)

						if g.Settings.TrafficFinesEnabled {
							fine := 50
							g.Cash -= fine
							if isWrongWay {
								g.HUD.AddWarningToast(fmt.Sprintf("[严重违章] 逆行引发交通事故！车况 -%.0f%%，罚款 -$%d", damage, fine))
							} else {
								g.HUD.AddWarningToast(fmt.Sprintf("[事故责任] 主动撞击社会车辆！车况 -%.0f%%，赔偿 -$%d", damage, fine))
							}
						} else {
							g.HUD.AddWarningToast(fmt.Sprintf("[事故碰撞] 撞击社会车辆！车况 -%.0f%% (已豁免罚金)", damage))
						}
					} else {
						// Victim (car struck stopped, roundabout right-of-way, or AI autopilot driving): ZERO FINE!
						damage := 3.0
						if impactSpeed > 80.0 {
							damage = 6.0
						}
						g.Bus.Durability = math.Max(0, g.Bus.Durability-damage)
						if g.Settings.AutopilotEnabled {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通碰撞] AI巡航已紧急制动避险！车况 -%.0f%% (免除事故赔偿)", damage))
						} else if isBusInRoundabout {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通事故] 环岛内社会车辆未按规定让行！车况 -%.0f%% (对方全责，免予罚款)", damage))
						} else {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通事故] 被社会车辆碰撞！车况 -%.0f%% (对方全责，免予罚款)", damage))
						}
					}

					if g.Bus.Durability <= 0 {
						g.Bus.Durability = 50.0
						g.Cash -= 200
						g.HUD.AddWarningToast("[严重损毁] 车况归零！已紧急呼叫拖车抢修至 50% (维修费 -$200)")
					}
				}
			} else {
				// Static pushing / rubbing
				if isCarFacingBus {
					car.Speed = 0
					car.IsBraking = true
				}
				if isBusFacingCar && g.Bus.Speed > car.Speed {
					g.Bus.Speed = math.Max(0, car.Speed*0.5)
				}
			}
		}
	}

	// Collision check between player bus and route AI buses
	if g.AIBusMgr != nil {
		for _, ai := range g.AIBusMgr.Buses {
			if ai.Bus == nil {
				continue
			}
			aiOBB := OBB{
				Center:  ai.Bus.Pos,
				HalfLen: ai.Bus.Length / 2,
				HalfWid: ai.Bus.Width / 2,
				Heading: ai.Bus.Heading,
			}
			collides, normal, depth := CheckOBBCollision(busOBB, aiOBB)
			if collides && depth > 0 {
				sepDepth := depth + 2.0
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.5))
				ai.Bus.Pos = ai.Bus.Pos.Sub(normal.Mul(sepDepth * 0.5))
				busOBB.Center = g.Bus.Pos

				g.Bus.Speed = -g.Bus.Speed * 0.25
				ai.Bus.Speed = -ai.Bus.Speed * 0.25
			}
		}

		// Collision separation between AI buses and civilian traffic cars
		for _, ai := range g.AIBusMgr.Buses {
			if ai.Bus == nil {
				continue
			}
			for _, car := range activeCars {
				ResolveBusCarSeparation(ai.Bus, car)
			}
		}

		// Collision separation between AI buses themselves
		for i := 0; i < len(g.AIBusMgr.Buses); i++ {
			for j := i + 1; j < len(g.AIBusMgr.Buses); j++ {
				ResolveBusBusSeparation(g.AIBusMgr.Buses[i].Bus, g.AIBusMgr.Buses[j].Bus)
			}
		}
	}

	// Collision separation between civilian traffic cars themselves
	for i := 0; i < len(activeCars); i++ {
		for j := i + 1; j < len(activeCars); j++ {
			ResolveCarCarSeparation(activeCars[i], activeCars[j])
		}
	}
}

func (g *Game) updatePlanningMode(dt float64) {
	// Always lock rotation to 0 in planning mode for upright North-Up map orientation
	g.Camera.Rotation = 0
	g.Camera.TargetRotation = 0

	// Perspective toggle hotkey (V key)
	if inpututil.IsKeyJustPressed(ebiten.KeyV) {
		newMode := g.Camera.ToggleMode()
		g.Settings.CameraViewMode = newMode
		switch newMode {
		case CameraModeHeadingUp:
			g.HUD.AddToast("视角预设: 【车头朝上 (跟随)】 - 将在返回驾驶模式时生效", false)
		case CameraModeSouthUp:
			g.HUD.AddToast("视角预设: 【固定正南 (倒置)】 - 将在返回驾驶模式时生效", false)
		default:
			g.HUD.AddToast("视角预设: 【固定正北 (标准)】 - 将在返回驾驶模式时生效", false)
		}
	}

	// Mouse Zoom with expanded range for large map
	_, wheelY := ebiten.Wheel()
	if wheelY != 0 {
		g.Camera.TargetZoom = Clamp(g.Camera.TargetZoom+wheelY*0.03, 0.04, 1.4)
		g.Camera.Zoom = g.Camera.TargetZoom
	}

	// Mouse Pan / Drag
	mx, my := ebiten.CursorPosition()
	mouseScreen := Vec2{X: float64(mx), Y: float64(my)}

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
			g.HUD.AddToast(fmt.Sprintf(T("已将站点 [%s] 添加至路线", "Added stop [%s] to route"), stopName), true)
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
			g.HUD.AddToast(fmt.Sprintf(T("已撤销最后一个站点 [%s] (路线剩余 %d 站)", "Removed last stop [%s] (%d remaining)"), stopName, len(g.Route.StopIDs)), false)
		} else {
			g.HUD.AddWarningToast(fmt.Sprintf(T("已撤销最后一个站点 [%s]，当前路线已为空！", "Removed last stop [%s], route is now empty!"), stopName))
		}
		return true
	}
	g.HUD.AddWarningToast(T("当前路线中已无站点可撤销！", "No stops to undo in current route!"))
	return false
}

// ClearPlannedRoute clears all stations from the planned route.
func (g *Game) ClearPlannedRoute() {
	if g.Route == nil {
		return
	}
	g.Route.Clear()
	g.planModeRouteModified = true
	g.HUD.AddToast(T("已清空规划路线，请重新规划！", "Route cleared, please click stations to replan!"), false)
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
		g.Camera.TargetPos = Vec2{X: 0, Y: 0}
		g.Camera.Pos = Vec2{X: 0, Y: 0}
		g.Camera.TargetZoom = 0.08
		g.Camera.Zoom = 0.08
		g.Camera.Rotation = 0
		g.Camera.TargetRotation = 0
		g.HUD.AddToast(T("已进入【线路规划模式】：游戏运行与客流生成已暂停 (点击站点加入路线，按TAB返回)", "[Plan Mode] Simulation & passenger spawning paused (Click stops to add to route, press TAB to return)"), false)
	} else {
		// Exit Plan Mode -> Return to Driver Mode
		if len(g.Route.StopIDs) == 0 {
			g.HUD.AddWarningToast(T("【路线未完成】当前路线为空，请至少点击选择 1 个站点规划路线后再返回驾驶！", "[Empty Route] Route is empty. Please select at least 1 stop before driving!"))
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
			g.HUD.AddToast(fmt.Sprintf(T("已进入【驾驶模式】：路线重新规划完成！车辆与 AI 车队已就位始发站 [%s]，开始运营发车！", "[Drive Mode] Route replanned! Bus and AI fleet reset to starting station [%s]!"), firstStopName), true)
		} else {
			// Instant snap back to driver mode without resetting vehicle positions
			g.Camera.FollowBus(g.Bus)
			g.Camera.Pos = g.Camera.TargetPos
			g.Camera.TargetZoom = 1.0
			g.Camera.Zoom = 1.0
			g.Camera.Rotation = g.Camera.TargetRotation
			g.HUD.AddToast(T("已进入【驾驶模式】：开始运营发车！", "[Drive Mode] Starting operation!"), true)
		}
	}
}

// ResetBusesToRouteStart resets both the player's bus and all AI fleet buses
// to the first stop of the currently planned route.
func (g *Game) ResetBusesToRouteStart() {
	if g.Route == nil || len(g.Route.StopIDs) == 0 || g.World == nil || len(g.World.Stops) == 0 {
		return
	}

	firstStopID := g.Route.StopIDs[0]
	if firstStopID < 0 || firstStopID >= len(g.World.Stops) {
		firstStopID = 0
	}
	firstStop := g.World.Stops[firstStopID]

	// 1. Reset player bus
	if g.Bus != nil {
		g.Bus.Pos = firstStop.Pos
		g.Bus.Heading = firstStop.Heading
		g.Bus.Speed = 0
		g.Bus.SteeringAngle = 0
		g.Bus.Gear = GearDrive
		g.Bus.TransmissionMode = TransmissionAuto
		g.Bus.IsHandbraking = false
		g.Bus.IsBraking = false
		g.Bus.DoorsOpen = true // Ready at the starting station bay for passenger boarding
		g.Bus.OnShoulder = false
		g.Bus.ClearPassengers()
	}

	g.boardedThisStop = false
	g.skipCooldown = 0
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.bellToastShown = false
	g.Route.CurrentIndex = 0
	g.Route.cachedDescDirty = true

	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
	}

	// 2. Ensure initial stop has passengers for immediate departure
	if firstStop.WaitingPassengers < 4 {
		firstStop.WaitingPassengers = 4 + simRand.Intn(4)
	}

	// 3. Reset AI bus fleet queuing behind player bus at first stop
	currentLaneW := g.Settings.GetLaneWidth()
	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.RebuildFleet(g.World, currentLaneW, g.Bus)
	}

	// 4. Follow player bus with camera
	if g.Camera != nil && g.Bus != nil {
		g.Camera.FollowBus(g.Bus)
		g.Camera.Pos = g.Bus.Pos
		g.Camera.TargetPos = g.Bus.Pos
		g.Camera.TargetZoom = 1.0
		g.Camera.Zoom = 1.0
		g.Camera.Rotation = g.Camera.TargetRotation
	}

	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()
}

// ResetToStartingStation resets the player's bus back to the starting station (起点站) of the active route.
func (g *Game) ResetToStartingStation() {
	if g.Route == nil || len(g.Route.StopIDs) == 0 || g.World == nil || len(g.World.Stops) == 0 {
		return
	}

	g.AutopilotDeadlockTimer = 0

	startID := g.Route.StopIDs[0]
	if startID < 0 || startID >= len(g.World.Stops) {
		startID = 0
	}
	startStop := g.World.Stops[startID]

	if startStop.WaitingPassengers < 4 {
		startStop.WaitingPassengers = 4 + simRand.Intn(4)
	}

	g.Route.CurrentIndex = 0
	g.Route.cachedDescDirty = true

	if g.Bus != nil {
		g.Bus.Pos = startStop.Pos
		g.Bus.Heading = startStop.Heading
		g.Bus.Speed = 0
		g.Bus.SteeringAngle = 0
		g.Bus.Gear = GearDrive
		g.Bus.TransmissionMode = TransmissionAuto
		g.Bus.IsHandbraking = false
		g.Bus.IsBraking = false
		g.Bus.DoorsOpen = false
		g.Bus.ClearPassengers()
	}

	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
		g.AutopilotDriver.State = AutopilotStateDocked
		g.AutopilotDriver.DockedStopID = startStop.ID
		g.AutopilotDriver.DwellTimer = 4.0
		g.AutopilotDriver.DwellFinished = false
	}

	g.boardedThisStop = true
	g.skipCooldown = 0
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.bellToastShown = false

	if g.Camera != nil && g.Bus != nil {
		g.Camera.Pos = g.Bus.Pos
		g.Camera.TargetPos = g.Bus.Pos
	}

	g.HUD.AddToast(T("【⚠️ 道路严重拥堵超时】AI公交车已重置回起点站，重新投入运营", "【⚠️ Severe Gridlock Timeout】AI Bus reset to starting station to resume route"), true)
}

// SyncStopsWithRoute ensures that ONLY bus stops included in the planned route
// have waiting passengers and can generate passengers.
// Any stops not on the planned route are cleared (waiting = 0, timer = 0).
func (g *Game) SyncStopsWithRoute() {
	if g.World == nil || g.Route == nil {
		return
	}
	for _, s := range g.World.Stops {
		if !g.Route.ContainsStop(s.ID) {
			s.WaitingPassengers = 0
			s.SpawnTimer = 0
		} else if s.WaitingPassengers == 0 {
			// Ensure route stops have passengers ready for service
			s.WaitingPassengers = 4 + simRand.Intn(4)
		}
	}
}

// ReplanRoute updates the active route to the given stop sequence and resets
// the player bus and AI fleet to the first station.
func (g *Game) ReplanRoute(stopIDs []int) {
	if len(stopIDs) == 0 {
		return
	}
	g.Route.Clear()
	for _, id := range stopIDs {
		g.Route.AddStop(id)
	}
	g.ResetBusesToRouteStart()
}

func (g *Game) updateDriverMode(dt float64) {
	currentLaneW := g.Settings.GetLaneWidth()

	// 0. AI Autopilot Toggle (P key) & Berserk Mode Toggle (B key or Shift+P)
	if inpututil.IsKeyJustPressed(ebiten.KeyP) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) {
			// Shift+P: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
			} else {
				g.HUD.AddToast(T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		} else {
			// Normal P key: Toggle Autopilot ON/OFF
			g.Settings.AutopilotEnabled = !g.Settings.AutopilotEnabled
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
				if g.Settings.AutopilotBerserk {
					g.HUD.AddToast(T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
				} else {
					g.HUD.AddToast(T("【🤖 AI自动驾驶已开启】AI已接管车辆，遵循交规循迹靠站 (按 B 开启狂暴，按 P 接管)", "【🤖 AI Autopilot ON】AI controls vehicle, serving stops (Press B for Berserk, P to exit)"), true)
				}
			} else {
				g.HUD.AddToast(T("【👤 已恢复手动驾驶】您已接管公交车控制权 (按 P 键可重新开启 AI)", "【👤 Manual Mode Restored】You have taken control (Press P to enable AI)"), false)
			}
		}
	} else if inpututil.IsKeyJustPressed(ebiten.KeyB) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) || ebiten.IsKeyPressed(ebiten.KeyShiftLeft) || ebiten.IsKeyPressed(ebiten.KeyShiftRight) {
			// Shift+B: Cycle Max Speed Limit (60 -> 80 -> 100 -> 120 -> 60)
			newSpd := g.Settings.CycleBerserkMaxSpeed()
			g.AutopilotDriver.BerserkMaxSpeed = g.Settings.GetBerserkCruiseSpeed()
			if g.Settings.AutopilotBerserk {
				g.Bus.MaxSpeedFwd = math.Max(g.Settings.GetBusMaxSpeed(), g.AutopilotDriver.GetBerserkCruiseSpeed()+20.0)
			} else {
				g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
			}
			g.HUD.AddToast(fmt.Sprintf(T("【⚡ 公交最高限速已切换】%d km/h (手动/狂暴模式均生效)", "【⚡ Top Speed Limit Set】%d km/h (Active in Manual & Berserk)"), newSpd), true)
		} else {
			// Dedicated B key: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(fmt.Sprintf(T("【⚡ AI自动驾驶·狂暴模式已开启】直行最高时速 %d km/h (Shift+B可切速)，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed %d km/h (Shift+B to cycle speed), multi-lane overtaking enabled!"), g.Settings.AutopilotBerserkMaxSpeed), true)
			} else {
				g.HUD.AddToast(T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		}
	}

	var throttle, brake, steer float64
	var handbrake, toggleDoors bool

	if g.Settings.AutopilotEnabled {
		// Collect dynamic obstacles (civilian traffic cars + route AI buses)
		activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
		obs := make([]VehicleObstacle, 0, len(activeCars)+len(g.AIBusMgr.Buses))
		for _, c := range activeCars {
			obs = append(obs, VehicleObstacle{
				Pos:     c.Pos,
				Heading: c.Heading,
				Speed:   c.Speed,
				Length:  c.Length,
				Width:   c.Width,
			})
		}
		if g.AIBusMgr != nil {
			for _, ab := range g.AIBusMgr.Buses {
				obs = append(obs, VehicleObstacle{
					Pos:     ab.Bus.Pos,
					Heading: ab.Bus.Heading,
					Speed:   ab.Bus.Speed,
					Length:  ab.Bus.Length,
					Width:   ab.Bus.Width,
				})
			}
		}

		g.AutopilotDriver.BerserkMode = g.Settings.AutopilotBerserk
		g.AutopilotDriver.BerserkMaxSpeed = g.Settings.GetBerserkCruiseSpeed()
		if g.Settings.AutopilotBerserk {
			g.Bus.MaxSpeedFwd = math.Max(g.Settings.GetBusMaxSpeed(), g.AutopilotDriver.GetBerserkCruiseSpeed()+20.0)
		} else {
			g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
		}
		throttle, brake, steer, handbrake, toggleDoors, _ = g.AutopilotDriver.ComputeControls(
			g.Bus, g.Route, g.World, g.TrafficLights, obs, currentLaneW, dt,
		)

		// Deadlock detection for AI Autopilot: if motionless and blocked by obstacle for > 1 traffic light cycle,
		// reset the bus back to the route's starting station (起点站)
		if math.Abs(g.Bus.Speed) < 8.0 {
			isJammed := false
			for _, ob := range obs {
				if math.Abs(ob.Speed) >= 8.0 {
					continue
				}
				dist := g.Bus.Pos.Distance(ob.Pos)
				if dist < (g.Bus.Length+ob.Length)*0.5+40.0 {
					toOb := ob.Pos.Sub(g.Bus.Pos)
					cosH := math.Cos(g.Bus.Heading)
					sinH := math.Sin(g.Bus.Heading)
					fwd := toOb.X*cosH + toOb.Y*sinH
					headDiff := math.Cos(ob.Heading - g.Bus.Heading)
					if headDiff > 0.5 && fwd > 10.0 {
						continue // Queuing behind
					}
					if fwd > -15.0 || dist < (g.Bus.Length+ob.Length)*0.5+15.0 {
						isJammed = true
						break
					}
				}
			}
			if isJammed {
				g.AutopilotDeadlockTimer += dt
				if g.AutopilotDeadlockTimer >= DeadlockDurationThreshold {
					g.AutopilotDeadlockTimer = 0
					oldBusPos := g.Bus.Pos
					g.ResetToStartingStation()
					// Also reset the jammed AI car(s) so both vehicles are cleared
					if g.TrafficMgr != nil {
						activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
						for cIdx, c := range activeCars {
							if c == nil {
								continue
							}
							dist := oldBusPos.Distance(c.Pos)
							if dist < (g.Bus.Length+c.Length)*0.5+40.0 {
								g.TrafficMgr.DespawnAndResetCar(cIdx, g.World, currentLaneW, g.Bus)
								break
							}
						}
					}
				}
			} else {
				if g.AutopilotDeadlockTimer > 0 {
					g.AutopilotDeadlockTimer = math.Max(0, g.AutopilotDeadlockTimer-dt*2.0)
				}
			}
		} else {
			g.AutopilotDeadlockTimer = 0
		}
	} else {
		g.AutopilotDeadlockTimer = 0
		g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
		// Driver controls: Separated accelerator throttle and service foot brake
		if ebiten.IsKeyPressed(ebiten.KeyW) || ebiten.IsKeyPressed(ebiten.KeyUp) {
			throttle = 1.0
		}

		if ebiten.IsKeyPressed(ebiten.KeyS) || ebiten.IsKeyPressed(ebiten.KeyDown) {
			brake = 1.0
		}

		if ebiten.IsKeyPressed(ebiten.KeyA) || ebiten.IsKeyPressed(ebiten.KeyLeft) {
			steer -= 1.0
		}
		if ebiten.IsKeyPressed(ebiten.KeyD) || ebiten.IsKeyPressed(ebiten.KeyRight) {
			steer += 1.0
		}

		handbrake = ebiten.IsKeyPressed(ebiten.KeySpace)
		toggleDoors = inpututil.IsKeyJustPressed(ebiten.KeyE)

		// Gear Shifting inputs:
		// 1. Shift Up (升档): Shift key
		if inpututil.IsKeyJustPressed(ebiten.KeyShift) || inpututil.IsKeyJustPressed(ebiten.KeyShiftLeft) || inpututil.IsKeyJustPressed(ebiten.KeyShiftRight) {
			if g.Bus.ShiftUp() {
				g.HUD.AddToast("已升档: "+g.Bus.Gear.Name(), false)
			}
		}
		// 2. Shift Down (降档): Control key or Q key
		if inpututil.IsKeyJustPressed(ebiten.KeyControl) || inpututil.IsKeyJustPressed(ebiten.KeyControlLeft) || inpututil.IsKeyJustPressed(ebiten.KeyControlRight) || inpututil.IsKeyJustPressed(ebiten.KeyQ) {
			if g.Bus.ShiftDown() {
				g.HUD.AddToast("已降档: "+g.Bus.Gear.Name(), false)
			}
		}
		// 3. Direct gear hotkeys:
		// 'R': Directly shift to Reverse (倒挡) / Toggle between D and R
		if inpututil.IsKeyJustPressed(ebiten.KeyR) {
			if g.Bus.Gear == GearReverse {
				if g.Bus.ShiftTo(GearDrive) {
					g.HUD.AddToast("已切回前进挡: "+g.Bus.Gear.Name(), false)
				}
			} else {
				if g.Bus.ShiftTo(GearReverse) {
					g.HUD.AddToast("已挂入倒挡: "+g.Bus.Gear.Name()+" (按 W/↑ 倒车，S/↓ 刹车)", false)
				} else {
					g.HUD.AddWarningToast("车速过快，变速箱保护锁止！请先踩刹车减速")
				}
			}
		}
		// 'N': Directly shift to Neutral (空挡)
		if inpututil.IsKeyJustPressed(ebiten.KeyN) {
			if g.Bus.ShiftTo(GearNeutral) {
				g.HUD.AddToast("已挂入空挡: "+g.Bus.Gear.Name(), false)
			}
		}
		// 'G': Directly shift to Drive (前进挡)
		if inpututil.IsKeyJustPressed(ebiten.KeyG) {
			if g.Bus.ShiftTo(GearDrive) {
				g.HUD.AddToast("已挂入前进挡: "+g.Bus.Gear.Name(), false)
			} else {
				g.HUD.AddWarningToast("倒车速度过快，无法直接切入前进挡！请先踩刹车减速")
			}
		}
	}

	// 4. Perspective toggle hotkey (V key)
	if inpututil.IsKeyJustPressed(ebiten.KeyV) {
		newMode := g.Camera.ToggleMode()
		g.Settings.CameraViewMode = newMode
		switch newMode {
		case CameraModeHeadingUp:
			g.HUD.AddToast("已切换视角: 【车头朝上 (跟随视角)】 - 车头始终朝上，视野随转向旋转", true)
		case CameraModeSouthUp:
			g.HUD.AddToast("已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上", true)
		default:
			g.HUD.AddToast("已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上", true)
		}
	}

	// 5. Mouse click on HUD buttons (Mode badge, Map theme badge, Perspective badge, Gear selectors & Autopilot)
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mouseScreen := Vec2{X: float64(mx), Y: float64(my)}
		if g.HUD.HandleModeBtnClick(mouseScreen) {
			g.TogglePlanMode()
			return
		} else if toggled := g.HUD.HandleMenuBtnClick(mouseScreen); toggled {
			g.State = StateMainMenu
			return
		} else if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, true)
		} else if shifted, notice := g.HUD.HandleGearClick(mouseScreen, g.Bus); shifted {
			g.HUD.AddToast(notice, false)
		} else if toggled, notice := g.HUD.HandleAutopilotClick(mouseScreen, g.Settings); toggled {
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
			}
			g.HUD.AddToast(notice, g.Settings.AutopilotEnabled)
		} else if notice != "" {
			g.HUD.AddWarningToast(notice)
		}
	}

	// Update bus kinematics
	g.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)
	if g.skipCooldown > 0 {
		g.skipCooldown -= dt
	}

	// Road Boundaries Check: Soft Shoulder (slowdown) & Hard Barriers (collision, damage & fine)
	bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, currentLaneW)
	switch bRes.Zone {
	case ZoneRoad:
		g.Bus.OnShoulder = false
	case ZoneShoulder:
		// Official bus bays are designated parking areas off the main roadway, not a shoulder violation
		inBusBay := false
		for _, s := range g.World.Stops {
			if s.ContainsBus(g.Bus) || s.Pos.Distance(g.Bus.Pos) < s.Length/2 {
				inBusBay = true
				break
			}
		}
		if inBusBay {
			g.Bus.OnShoulder = false
		} else {
			g.Bus.OnShoulder = true
		}
	case ZoneHardBarrier:
		g.Bus.OnShoulder = false
		// Repulsion pushback from hard obstacle
		g.Bus.Pos = g.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 5.0))
		impactSpeed := math.Abs(g.Bus.Speed)
		g.Bus.Speed = -g.Bus.Speed * 0.35 // bounce recoil

		// Damage & Fine event
		if g.Bus.CrashCooldown <= 0 {
			g.Bus.CrashCooldown = 1.4
			g.Camera.AddShake(10.0) // visceral camera impact shake

			damageAmount := 15.0
			if impactSpeed > 120 {
				damageAmount = 25.0
			}
			g.Bus.Durability = math.Max(0, g.Bus.Durability-damageAmount)

			fine := 100
			g.Cash -= fine

			msg := fmt.Sprintf("[事故碰撞] 撞击%s！车况 -%.0f%%，赔偿罚金 -$%d",
				bRes.ObstacleName, damageAmount, fine)
			g.HUD.AddWarningToast(msg)

			if g.Bus.Durability <= 0 {
				g.Bus.Durability = 50.0
				g.Cash -= 200
				g.HUD.AddWarningToast("[严重损毁] 车况归零！已紧急呼叫拖车抢修至 50% (维修费 -$200)")
			}
		}
	}

	// Collision check between player bus and AI cars (resolves penetration immediately after bus moved)
	g.checkBusCarCollisions()

	// Camera follows bus
	g.Camera.FollowBus(g.Bus)

	// Target stop navigation & Stop Bell / Skip Stop (飞站) processing
	targetStopID := g.Route.GetCurrentTargetStopID()
	var targetStop *BusStop
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		targetStop = g.World.Stops[targetStopID]
	}

	if targetStop != nil {
		distToTarget := g.Bus.Pos.Distance(targetStop.Pos)

		headingVec := Vec2{X: math.Cos(targetStop.Heading), Y: math.Sin(targetStop.Heading)}
		perpVec := Vec2{X: -headingVec.Y, Y: headingVec.X}
		dVec := g.Bus.Pos.Sub(targetStop.Pos)
		forwardProj := dVec.X*headingVec.X + dVec.Y*headingVec.Y

		// 1. Heading alignment: Bus must be traveling in the same general direction as the stop
		// (Prevents oncoming vehicles driving past on the opposite side of the road from triggering)
		cosHeading := math.Cos(g.Bus.Heading - targetStop.Heading)
		isAlignedHeading := cosHeading > 0.55

		// 2. Road side: Bus must be on the stop's side of the road (or borrowing oncoming lane on 2-lane road while overtaking)
		// Oncoming lanes of 4/6-lane roads, opposite carriageways, or parallel roads must NOT match
		toCenter := g.Bus.Pos.Sub(targetStop.RoadCenter)
		sideDist := toCenter.X*perpVec.X + toCenter.Y*perpVec.Y
		curbDist := targetStop.Pos.Distance(targetStop.RoadCenter)
		minSideDist := -25.0
		if targetStop.RoadLanes <= 2 {
			// On 2-lane roads, bus may borrow opposing lane during overtake
			minSideDist = -curbDist - 20.0
		}
		maxSideDist := curbDist + targetStop.Width*0.5 + 35.0
		isOnStopRoad := sideDist >= minSideDist && sideDist <= maxSideDist

		// 3. Autopilot route progress:
		// If Autopilot is driving, verify it is on the route approach to targetStop,
		// not on an earlier leg or opposite road that happens to pass near targetStop.
		isArrivingAtStop := true
		if g.Settings.AutopilotEnabled && g.AutopilotDriver != nil && g.AutopilotDriver.pathToID == targetStop.ID {
			if forwardProj > 55.0 {
				// For overshooting past the stop: must have reached the end of the leg
				if g.AutopilotDriver.LastRemaining > 200.0 {
					isArrivingAtStop = false
				}
			} else {
				// For approaching/skipping the stop: must be on the approach to the stop (not 450+ px away on earlier leg)
				if g.AutopilotDriver.LastRemaining > 450.0 && g.AutopilotDriver.LastRemaining > distToTarget+100.0 {
					isArrivingAtStop = false
				}
			}
		}

		isValidStopApproach := isAlignedHeading && isOnStopRoad && isArrivingAtStop

		// 1. Chime bell notification when approaching within 550px
		if distToTarget < 550 && g.stopBellRung && !g.bellToastShown && isAlignedHeading && isOnStopRoad && isArrivingAtStop {
			g.bellToastShown = true
			g.HUD.AddToast(fmt.Sprintf("[🔔 下车铃响] 叮咚~ 车内有 %d 位乘客按下车铃，将在下一站下车！", g.passengersAlighting), false)
		}

		// 2. Skip Stop (飞站 / 漏站) check when approaching or past within 380px
		if distToTarget <= 380.0 && isValidStopApproach {
			isOccupied := g.IsStopOccupiedByOtherVehicle(targetStop)
			canSkip := !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)

			// Proactive AI Autopilot Skip when station can be skipped and no bell rung (both empty and occupied station)
			if g.Settings.AutopilotEnabled && canSkip && forwardProj < 55.0 && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 {
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.Name
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					prevOffset := g.AutopilotDriver.LaneOffset
					g.AutopilotDriver.Reset()
					if math.Abs(prevOffset) > 5.0 {
						g.AutopilotDriver.LaneOffset = prevOffset
						g.AutopilotDriver.TargetLaneOffset = 0.0
						g.AutopilotDriver.OvertakeState = OvertakeReturning
					}
					g.AutopilotDriver.State = AutopilotStateCruising
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if g.Settings.AutopilotEnabled && math.Abs(g.Bus.Speed) < 25.0 {
					g.Bus.Speed = 30.0
				}
				if isOccupied {
					g.HUD.AddToast(fmt.Sprintf(T("【🤖 AI自主飞站】[%s] 检测到前方已有车靠站且无下车乘客，自动飞站提速！下一站: [%s]",
						"[🤖 AI Skip] [%s] Station occupied & no alighting pax, skipping stop! Next: [%s]"), oldName, newTarget.Name), true)
				} else {
					g.HUD.AddToast(fmt.Sprintf(T("【🤖 AI自主飞站】[%s] 站台无人且车内无下车乘客，自动飞站提速！下一站: [%s]",
						"[🤖 AI Skip] [%s] Station empty & no alighting pax, skipping stop! Next: [%s]"), oldName, newTarget.Name), true)
				}
			}

			// Manual skip with F key
			if inpututil.IsKeyJustPressed(ebiten.KeyF) {
				if canSkip {
					g.lastSkippedStopID = targetStop.ID
					g.skipCooldown = 0.5
					oldName := targetStop.Name
					g.Route.AdvanceToNextStop()
					newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
					g.rollAlightingForNextStop()
					if g.AutopilotDriver != nil {
						prevOffset := g.AutopilotDriver.LaneOffset
						g.AutopilotDriver.Reset()
						if math.Abs(prevOffset) > 5.0 {
							g.AutopilotDriver.LaneOffset = prevOffset
							g.AutopilotDriver.TargetLaneOffset = 0.0
							g.AutopilotDriver.OvertakeState = OvertakeReturning
						}
						g.AutopilotDriver.State = AutopilotStateCruising
					}
					g.Bus.IsBraking = false
					g.Bus.IsHandbraking = false
					if isOccupied {
						g.HUD.AddToast(fmt.Sprintf("【⚡ 飞站成功】[%s] 前方站台已有车靠站且本车无人下车，已飞站！下一站: [%s]", oldName, newTarget.Name), true)
					} else {
						g.HUD.AddToast(fmt.Sprintf("【⚡ 飞站成功】[%s] 站内无人且无人按铃，已飞站！下一站: [%s]", oldName, newTarget.Name), true)
					}
				} else {
					reasons := ""
					if g.stopBellRung {
						reasons += fmt.Sprintf("车内有 %d 人按铃下车 ", g.passengersAlighting)
					}
					if targetStop.WaitingPassengers > 0 && !isOccupied {
						reasons += fmt.Sprintf("站台有 %d 人候车且前方无其他车辆靠站 ", targetStop.WaitingPassengers)
					}
					g.HUD.AddWarningToast(fmt.Sprintf("【不可飞站】[%s] %s，必须进站停靠！", targetStop.Name, reasons))
				}
			}

			// Drive-by auto skip / AI missed stop check when driving past the stop
			isDocked := targetStop.ContainsBus(g.Bus) || (g.AutopilotDriver != nil && g.AutopilotDriver.State == AutopilotStateDocked && g.AutopilotDriver.DockedStopID == targetStop.ID)

			isAIMissed := !canSkip && g.Settings.AutopilotEnabled && !isDocked && (
				(forwardProj > 55.0 && distToTarget < 350.0) ||
				(g.Settings.AutopilotBerserk && forwardProj > 15.0 && distToTarget < 350.0 && g.AutopilotDriver != nil && g.AutopilotDriver.LaneOffset < -currentLaneW*0.3) ||
				(math.Abs(g.Bus.Speed) < 5.0 && distToTarget < 120.0 && !targetStop.ContainsBus(g.Bus)))

			if forwardProj > 55.0 && distToTarget < 350.0 && !isDocked && canSkip && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 && (math.Abs(g.Bus.Speed) > 15.0 || g.Settings.AutopilotEnabled) {
				// Lawful skip when station is empty or occupied with no bell rung
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.Name
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					prevOffset := g.AutopilotDriver.LaneOffset
					g.AutopilotDriver.Reset()
					if math.Abs(prevOffset) > 5.0 {
						g.AutopilotDriver.LaneOffset = prevOffset
						g.AutopilotDriver.TargetLaneOffset = 0.0
						g.AutopilotDriver.OvertakeState = OvertakeReturning
					}
					g.AutopilotDriver.State = AutopilotStateCruising
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if g.Settings.AutopilotEnabled && math.Abs(g.Bus.Speed) < 25.0 {
					g.Bus.Speed = 30.0
				}
				if isOccupied {
					g.HUD.AddToast(fmt.Sprintf(T("【⚡ 快速飞站】顺利掠过 [%s]（站台已有车靠站）！已自动飞站，下一站: [%s]", "【⚡ Fast Skip】Glided past [%s] (occupied)! Next stop: [%s]"), oldName, newTarget.Name), true)
				} else {
					g.HUD.AddToast(fmt.Sprintf(T("【⚡ 快速飞站】顺利掠过 [%s]！已自动飞站，下一站: [%s]", "【⚡ Fast Skip】Glided past [%s]! Next stop: [%s]"), oldName, newTarget.Name), true)
				}
			} else if isAIMissed && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 {
				// AI Autopilot missed the station after overtaking / overshooting:
				// Directly skip this station and deduct $50 fine (only in AI mode), and immediately continue driving!
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.Name
				if g.Settings.TrafficFinesEnabled {
					g.Cash -= 50
				}
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				// Reassign destination for passengers on board destined for the missed stop to next stop
				for i := range g.Bus.Passengers {
					if g.Bus.Passengers[i].TargetStopID == targetStop.ID {
						g.Bus.Passengers[i].TargetStopID = newTarget.ID
					}
				}
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					prevOffset := g.AutopilotDriver.LaneOffset
					g.AutopilotDriver.Reset()
					if math.Abs(prevOffset) > 5.0 {
						g.AutopilotDriver.LaneOffset = prevOffset
						g.AutopilotDriver.TargetLaneOffset = 0.0
						g.AutopilotDriver.OvertakeState = OvertakeReturning
					}
					g.AutopilotDriver.State = AutopilotStateCruising
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if g.Settings.AutopilotEnabled && math.Abs(g.Bus.Speed) < 25.0 {
					g.Bus.Speed = 30.0
				}
				if g.Settings.TrafficFinesEnabled {
					g.HUD.AddWarningToast(fmt.Sprintf(T("【⚠️ 站台过头·漏站罚款 -$50】AI未能及时靠站，已越过 [%s] 站！扣除罚款 -$50，下一站: [%s]",
						"[⚠️ Missed Stop Fine -$50] AI failed to dock in time and overshot [%s]! Fined -$50, next stop: [%s]"), oldName, newTarget.Name))
				} else {
					g.HUD.AddWarningToast(fmt.Sprintf(T("【⚠️ 站台过头·漏站免罚】AI未能及时靠站，已越过 [%s] 站 (已豁免罚款)！下一站: [%s]",
						"[⚠️ Missed Stop Waived] AI failed to dock in time and overshot [%s] (fines waived)! Next stop: [%s]"), oldName, newTarget.Name))
				}
			}
		}
	}

	// Check if parked at any bus stop
	var activeStop *BusStop
	var misalignedStop *BusStop
	for _, s := range g.World.Stops {
		isAutopilotDockedAtThisStop := g.Settings.AutopilotEnabled && g.AutopilotDriver != nil &&
			g.AutopilotDriver.State == AutopilotStateDocked && g.AutopilotDriver.DockedStopID == s.ID &&
			(s.ContainsBus(g.Bus) || s.Pos.Distance(g.Bus.Pos) < s.Length/2+65.0)
		if s.ContainsBus(g.Bus) || isAutopilotDockedAtThisStop {
			activeStop = s
			break
		} else if s.Pos.Distance(g.Bus.Pos) < s.Length/2+65 || s.RoadCenter.Distance(g.Bus.Pos) < 80 {
			misalignedStop = s
		}
	}

	// Feedback if driver tries to open doors near a stop from wrong side or reverse orientation
	if toggleDoors && activeStop == nil && misalignedStop != nil {
		angleDiff := math.Abs(NormalizeAngle(g.Bus.Heading - misalignedStop.Heading))
		if angleDiff >= math.Pi/4 {
			g.HUD.AddWarningToast(fmt.Sprintf("【靠站方向错误】[%s] 车门背离站台！公交车只能靠马路右侧顺行停靠", misalignedStop.Name))
		} else {
			g.HUD.AddWarningToast(fmt.Sprintf("【未准确进站】[%s] 请驶入马路右侧黄色停靠港湾方可上下客", misalignedStop.Name))
		}
	}

	// Boarding logic when parked at active stop
	if activeStop != nil {
		if !g.Bus.DoorsOpen {
			// Doors closed while parked: reset flag so opening doors at this same stop allows boarding again
			g.boardedThisStop = false
		} else {
			// Doors are open
			boardSpace := g.Bus.Capacity - g.Bus.PassengerCount

			if !g.boardedThisStop {
				g.boardedThisStop = true

				// Passengers getting off:
				// Remove passengers whose target destination is this stop
				offCount := g.Bus.RemovePassengersAlightingAt(activeStop.ID)

				// If this active stop is the terminus of the route, all remaining passengers must alight
				if g.Route.IsTerminusIndex(g.Route.CurrentIndex) && activeStop.ID == g.Route.GetCurrentTargetStopID() {
					offCount += g.Bus.ClearPassengers()
				}
				g.stopBellRung = false
				g.passengersAlighting = 0

				// Passengers getting on
				boardCount := activeStop.WaitingPassengers
				if boardCount > boardSpace {
					boardCount = boardSpace
				}

				if boardCount > 0 {
					candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
					if len(candidates) == 0 && len(g.Route.StopIDs) > 0 {
						for _, id := range g.Route.StopIDs {
							if id != activeStop.ID {
								candidates = append(candidates, id)
							}
						}
					}
					actualBoarded := g.Bus.BoardPassengers(boardCount, candidates)
					activeStop.WaitingPassengers -= actualBoarded
					fareEarned := actualBoarded * 3
					g.Cash += fareEarned
					toastMsg := Tf("[%s] 下客 %d 人，上客 %d 人，营收 +$%d！",
						"[%s] %d alighted, %d boarded, revenue +$%d!",
						activeStop.CleanName(), offCount, actualBoarded, fareEarned)
					g.HUD.AddToast(toastMsg, true)
				} else if boardSpace <= 0 && activeStop.WaitingPassengers > 0 {
					var toastMsg string
					if offCount > 0 {
						toastMsg = Tf("[%s] 下客 %d 人，当前车厢已满载，无法上客！",
							"[%s] %d alighted, bus is at full capacity!",
							activeStop.CleanName(), offCount)
					} else {
						toastMsg = Tf("[%s] 当前车厢已满载，无法上客！",
							"[%s] Bus is at full capacity!",
							activeStop.CleanName())
					}
					g.HUD.AddToast(toastMsg, false)
				} else if offCount > 0 {
					toastMsg := Tf("[%s] 下客 %d 人，本站无乘客上车！",
						"[%s] %d alighted, no passengers waiting!",
						activeStop.CleanName(), offCount)
					g.HUD.AddToast(toastMsg, false)
				} else {
					toastMsg := Tf("[%s] 本站无乘客上下车！",
						"[%s] No passengers boarding or alighting!",
						activeStop.CleanName())
					g.HUD.AddToast(toastMsg, false)
				}

				// Advance to next stop if this was the planned target stop
				if activeStop.ID == g.Route.GetCurrentTargetStopID() {
					g.Route.AdvanceToNextStop()
				}
				g.rollAlightingForNextStop()
			} else {
				// Doors remain open: if new passengers arrived while parked, board them as long as there is space!
				if activeStop.WaitingPassengers > 0 && boardSpace > 0 {
					boardCount := activeStop.WaitingPassengers
					if boardCount > boardSpace {
						boardCount = boardSpace
					}
					candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
					if len(candidates) == 0 && len(g.Route.StopIDs) > 0 {
						for _, id := range g.Route.StopIDs {
							if id != activeStop.ID {
								candidates = append(candidates, id)
							}
						}
					}
					actualBoarded := g.Bus.BoardPassengers(boardCount, candidates)
					activeStop.WaitingPassengers -= actualBoarded
					fareEarned := actualBoarded * 3
					g.Cash += fareEarned
					toastMsg := Tf("[%s] 站台新增乘客，上客 %d 人，营收 +$%d！",
						"[%s] Additional passengers: %d boarded, revenue +$%d!",
						activeStop.CleanName(), actualBoarded, fareEarned)
					g.HUD.AddToast(toastMsg, true)
				}
			}
		}
	} else {
		// Left the stop bay, reset boarding flag
		g.boardedThisStop = false
	}
}

// IsStopOccupiedByOtherVehicle checks if a bus stop bay is currently occupied by another stopped or docking vehicle
func (g *Game) IsStopOccupiedByOtherVehicle(stop *BusStop) bool {
	if stop == nil {
		return false
	}
	// 1. Check AI route buses
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus != nil {
				// Docked strictly inside yellow bay box
				if stop.ContainsBus(ab.Bus) {
					return true
				}
				// Autopilot driver is in docked state at this stop
				if ab.Driver != nil && ab.Driver.State == AutopilotStateDocked && ab.Driver.DockedStopID == stop.ID {
					return true
				}
				// Close to bay center and stopped or crawling, facing the stop direction
				if ab.Bus.Pos.Distance(stop.Pos) < (stop.Length*0.5+25.0) && math.Abs(ab.Bus.Speed) < 8.0 &&
					math.Cos(ab.Bus.Heading-stop.Heading) > 0.4 {
					return true
				}
			}
		}
	}
	// 2. Check civilian traffic cars that might be stalled or parked in the bay
	if g.TrafficMgr != nil {
		for _, c := range g.TrafficMgr.Cars {
			if c.Pos.Distance(stop.Pos) < 55.0 && math.Abs(c.Speed) < 8.0 &&
				math.Cos(c.Heading-stop.Heading) > 0.4 {
				return true
			}
		}
	}
	return false
}

func (g *Game) Draw(screen *ebiten.Image) {
	if g.State == StateMainMenu {
		g.drawMainMenu(screen)
		return
	}

	targetStopID := g.Route.GetCurrentTargetStopID()
	laneWidth := g.Settings.GetLaneWidth()

	// 1. Draw World (Roads, Buildings, Trees, Rivers, Bus Stops)
	g.World.Draw(screen, g.Camera, targetStopID, laneWidth, g.IsPlanMode)

	// 2. Draw Traffic Lights and Stop Lines (Only in Driver Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		g.TrafficLights.Draw(screen, g.Camera, laneWidth)
	}

	// 3. Draw Active Route lines on map (offset into driving lanes)
	g.Route.DrawRouteLine(screen, g.Camera, g.World, g.IsPlanMode, laneWidth)

	// 4. Draw AI Traffic Vehicles (Hidden during Planning Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		g.TrafficMgr.Draw(screen, g.Camera, g.Settings.TrafficDensity)
	}

	// 4.5 Draw Route AI Buses (Hidden during Planning Mode)
	if !g.IsPlanMode && g.AIBusMgr != nil {
		g.AIBusMgr.Draw(screen, g.Camera)
	}

	// 5. Draw Bus
	g.Bus.Draw(screen, g.Camera)

	// 6. Draw Navigation Guide Arrow towards target stop in Driver Mode
	if !g.IsPlanMode && targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		targetStop := g.World.Stops[targetStopID]
		g.Route.DrawNavArrow(screen, g.Camera, g.Bus, targetStop)
	}

	// 7. Draw HUD Overlay
	var currentStop *BusStop
	for _, s := range g.World.Stops {
		if s.ContainsBus(g.Bus) {
			currentStop = s
			break
		}
	}
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		g.HUD.TargetStopOccupied = g.IsStopOccupiedByOtherVehicle(g.World.Stops[targetStopID])
	} else {
		g.HUD.TargetStopOccupied = false
	}
	g.HUD.Draw(screen, g.Bus, g.Route, g.World, g.IsPlanMode, g.Cash, currentStop, g.Settings, g.stopBellRung, g.passengersAlighting, g.Camera, g.TrafficLights)

	// 8. Draw Settings Modal (on top of everything)
	g.Settings.Draw(screen, g.Camera)
}

func (g *Game) updateMainMenu() {
	// 1. Keyboard shortcuts: [1]..[6] / [C]
	if inpututil.IsKeyJustPressed(ebiten.Key1) {
		g.SelectedMapTheme = MapThemeMetropolis
	} else if inpututil.IsKeyJustPressed(ebiten.Key2) {
		g.SelectedMapTheme = MapThemeCoastalBay
	} else if inpututil.IsKeyJustPressed(ebiten.Key3) {
		g.SelectedMapTheme = MapThemeRiverValley
	} else if inpututil.IsKeyJustPressed(ebiten.Key4) {
		g.SelectedMapTheme = MapThemeEmeraldLake
	} else if inpututil.IsKeyJustPressed(ebiten.Key5) {
		g.SelectedMapTheme = MapThemeTwinCities
	} else if inpututil.IsKeyJustPressed(ebiten.Key6) || inpututil.IsKeyJustPressed(ebiten.KeyC) {
		g.SelectedMapTheme = MapThemeCustom
	}

	// Hotkey L: Toggle interface language
	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		newLang := ToggleLanguage()
		g.Settings.Language = newLang
	}

	// Hotkey R: reload custom map from custom_map.json
	if inpututil.IsKeyJustPressed(ebiten.KeyR) {
		_ = g.ReloadCustomMap()
	}

	// Arrow keys / A / D
	if inpututil.IsKeyJustPressed(ebiten.KeyLeft) || inpututil.IsKeyJustPressed(ebiten.KeyA) {
		g.SelectedMapTheme = (g.SelectedMapTheme + 5) % 6
	} else if inpututil.IsKeyJustPressed(ebiten.KeyRight) || inpututil.IsKeyJustPressed(ebiten.KeyD) {
		g.SelectedMapTheme = (g.SelectedMapTheme + 1) % 6
	}

	// Enter or Space: Enter city!
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeySpace) {
		g.EnterCity(g.SelectedMapTheme)
		return
	}

	// Mouse click detection
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mPos := Vec2{X: float64(mx), Y: float64(my)}

		if g.menuLangBounds.Contains(mPos) {
			newLang := ToggleLanguage()
			g.Settings.Language = newLang
			return
		}

		if g.menuStartBounds.Contains(mPos) {
			g.EnterCity(g.SelectedMapTheme)
			return
		}

		for i := 0; i < 6; i++ {
			if g.menuCardBounds[i].Contains(mPos) {
				if g.SelectedMapTheme == MapTheme(i) {
					g.EnterCity(g.SelectedMapTheme)
					return
				}
				g.SelectedMapTheme = MapTheme(i)
				return
			}
		}
	}
}

func (g *Game) drawMainMenu(screen *ebiten.Image) {
	sw := float64(screen.Bounds().Dx())
	sh := float64(screen.Bounds().Dy())

	// 1. Deep slate background with ambient grid
	screen.Fill(RGBA(15, 23, 42, 255))
	gridSpacing := 60.0
	for gx := 0.0; gx < sw; gx += gridSpacing {
		vector.StrokeLine(screen, float32(gx), 0, float32(gx), float32(sh), 1.0, RGBA(30, 41, 59, 120), true)
	}
	for gy := 0.0; gy < sh; gy += gridSpacing {
		vector.StrokeLine(screen, 0, float32(gy), float32(sw), float32(gy), 1.0, RGBA(30, 41, 59, 120), true)
	}

	// Language toggle button at top right
	langBtnW := 190.0
	langBtnH := 32.0
	langBtnX := sw - langBtnW - 24.0
	langBtnY := 18.0
	g.menuLangBounds = Rect{X: langBtnX, Y: langBtnY, W: langBtnW, H: langBtnH}
	vector.DrawFilledRect(screen, float32(langBtnX), float32(langBtnY), float32(langBtnW), float32(langBtnH), RGBA(30, 41, 59, 220), true)
	vector.StrokeRect(screen, float32(langBtnX), float32(langBtnY), float32(langBtnW), float32(langBtnH), 1.5, RGBA(56, 189, 248, 255), true)
	DrawText(screen, T("🌐 语言: 简体中文 [L]", "🌐 Lang: English [L]"), langBtnX+18, langBtnY+8, 12, RGBA(56, 189, 248, 255))

	// 2. Title and Header
	titleY := 36.0
	if menuLogoImg == nil && len(menuLogoBytes) > 0 {
		if raw, _, err := image.Decode(bytes.NewReader(menuLogoBytes)); err == nil {
			menuLogoImg = ebiten.NewImageFromImage(raw)
		}
	}

	titleText := T("2D BUS SIMULATOR · 城市公交模拟器", "2D BUS SIMULATOR · Urban Bus Simulator")
	tw, _ := MeasureText(titleText, 24)
	targetSize := 64.0
	gap := 16.0
	totalHeaderW := tw
	if menuLogoImg != nil {
		totalHeaderW = targetSize + gap + tw
	}
	headerStartX := (sw - totalHeaderW) / 2.0

	if menuLogoImg != nil {
		op := &ebiten.DrawImageOptions{}
		scale := targetSize / float64(menuLogoImg.Bounds().Dx())
		op.GeoM.Scale(scale, scale)
		op.GeoM.Translate(headerStartX, titleY-18)
		op.Filter = ebiten.FilterLinear // Ultra-crisp HD bilinear filtering for Retina displays
		screen.DrawImage(menuLogoImg, op)
		headerStartX += targetSize + gap
	}

	DrawText(screen, titleText, headerStartX, titleY, 24, RGBA(248, 250, 252, 255))

	subText := T("请选择营运城市与地图线路 (6 大独立地理风貌与拓扑路网 · 支持玩家自定义地图及热重载)", "Select city & route (6 distinct maps with road networks · Custom map & hot-reload supported)")
	subW, _ := MeasureText(subText, 14)
	DrawText(screen, subText, (sw-subW)/2.0, titleY+42, 14, RGBA(148, 163, 184, 255))

	// 3. 6 Map Cards
	cardW := 192.0
	cardH := 380.0
	cardGap := 12.0
	totalW := 6*cardW + 5*cardGap
	startX := (sw - totalW) / 2.0
	startY := 115.0

	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	accents := []color.RGBA{
		RGBA(59, 130, 246, 255), // Metropolis: Royal Blue
		RGBA(6, 182, 212, 255),  // Coastal Bay: Ocean Cyan
		RGBA(16, 185, 129, 255), // River Valley: River Emerald
		RGBA(20, 184, 166, 255), // Emerald Lake: Teal
		RGBA(168, 85, 247, 255), // Twin Cities: High-tech Violet
		RGBA(245, 158, 11, 255), // Custom Map: Amber Gold
	}

	type mapCardInfo struct {
		tag      string
		name     string
		sub      string
		topo     string
		landmark string
		stops    string
	}

	cardData := []mapCardInfo{
		{
			tag:      T("【01 都会 CBD】", "【01 Metropolis】"),
			name:     T("都会核心 (Metropolis)", "Metropolis CBD"),
			sub:      T("经典都会 · 商务高层群", "Classic Skyline & CBD"),
			topo:     T("• 5×5 经典棋盘快速干道", "• 5×5 High-density grid"),
			landmark: T("• 运河双子桥与喷泉环岛", "• Canal bridges & fountain"),
			stops:    T("• 12 座核心枢纽客运站", "• 12 Primary bus stops"),
		},
		{
			tag:      T("【02 海滨港湾】", "【02 Coastal Bay】"),
			name:     T("海滨港湾 (Coastal Bay)", "Coastal Bay Port"),
			sub:      T("海景风情 · 国际深水港", "Scenic coast & seaport"),
			topo:     T("• 黄金海岸与高密港口路网", "• Ocean drive & freight grid"),
			landmark: T("• 1200m 跨海悬索特大桥", "• 1200m Suspension bridge"),
			stops:    T("• 12 座海滨国际客运站", "• 12 Coastal bay stations"),
		},
		{
			tag:      T("【03 溪谷江城】", "【03 River Valley】"),
			name:     T("溪谷江城 (River Valley)", "River Valley City"),
			sub:      T("天然大江 · 山城两岸立体拓扑", "River canyon & mountain city"),
			topo:     T("• 跨江斜拉双桥与沿江干道", "• Dual cable-stayed bridges"),
			landmark: T("• 滨江大道与观景高地", "• Riverside expressway"),
			stops:    T("• 12 座立体山水客运站", "• 12 River valley stations"),
		},
		{
			tag:      T("【04 湖畔水乡】", "【04 Emerald Lake】"),
			name:     T("湖岛水乡 (Emerald Lake)", "Emerald Lake Town"),
			sub:      T("巨型翠湖 · 湖心绿岛古镇水网", "Vast lake & historic canals"),
			topo:     T("• 环湖干道与东西向通航河渠", "• Lakeside loop road network"),
			landmark: T("• 四方长堤与汉白玉石拱桥", "• Causeway & stone bridges"),
			stops:    T("• 12 座水乡古镇名胜站", "• 12 Historic town stations"),
		},
		{
			tag:      T("【05 高新双城】", "【05 Twin Districts】"),
			name:     T("高新双城 (Twin Districts)", "Twin Tech Districts"),
			sub:      T("临空制造 · 硅谷软件双城融合", "Aerospace & software hub"),
			topo:     T("• 东西双城干道与中央快速路", "• Inter-district corridors"),
			landmark: T("• 跨区中央高架特大桥", "• Grade-separated overpass"),
			stops:    T("• 12 座高新智造枢纽站", "• 12 Hi-tech district stops"),
		},
		{
			tag:      T("【06 自定义地图】", "【06 Custom Map】"),
			name:     T("自定义地图 (Custom)", "Custom Map (JSON)"),
			sub:      T("custom_map.json 动态载入", "Dynamic custom_map.json"),
			topo:     T("• 自由扩展道路/站点/水系", "• User-defined roads & stops"),
			landmark: T("• 支持随时按 [R] 热重载", "• Hot reload with [R] anytime"),
			stops:    T("• 12 座自由规划客运站", "• Fully customizable stops"),
		},
	}

	for i, th := range themes {
		cx := startX + float64(i)*(cardW+cardGap)
		cy := startY
		isSelected := (g.SelectedMapTheme == th)
		if isSelected {
			cy -= 8.0 // Lift up
		}

		g.menuCardBounds[i] = Rect{X: cx, Y: cy, W: cardW, H: cardH}
		accent := accents[i]
		info := cardData[i]

		// Card background
		bgCol := RGBA(30, 41, 59, 230)
		borderW := float32(1.5)
		borderCol := RGBA(71, 85, 105, 200)
		if isSelected {
			bgCol = RGBA(30, 41, 59, 255)
			borderW = 3.0
			borderCol = accent
		}

		// Draw card shadow & card body
		vector.DrawFilledRect(screen, float32(cx+4), float32(cy+6), float32(cardW), float32(cardH), RGBA(2, 6, 23, 160), true)
		vector.DrawFilledRect(screen, float32(cx), float32(cy), float32(cardW), float32(cardH), bgCol, true)
		vector.StrokeRect(screen, float32(cx), float32(cy), float32(cardW), float32(cardH), borderW, borderCol, true)

		// Top accent line
		vector.DrawFilledRect(screen, float32(cx), float32(cy), float32(cardW), 6, accent, true)

		// Tag & Titles
		DrawText(screen, info.tag, cx+12, cy+22, 12, accent)
		DrawText(screen, info.name, cx+12, cy+52, 12, RGBA(248, 250, 252, 255))
		DrawText(screen, info.sub, cx+12, cy+78, 10, RGBA(148, 163, 184, 255))

		// Separator line
		vector.StrokeLine(screen, float32(cx+12), float32(cy+105), float32(cx+cardW-12), float32(cy+105), 1.0, RGBA(71, 85, 105, 180), true)

		// Features
		DrawText(screen, T("【地理与路网特色】", "【Map Features】"), cx+12, cy+125, 11, accent)
		DrawText(screen, info.topo, cx+12, cy+155, 10, RGBA(226, 232, 240, 255))
		DrawText(screen, info.landmark, cx+12, cy+185, 10, RGBA(226, 232, 240, 255))
		DrawText(screen, info.stops, cx+12, cy+215, 10, RGBA(226, 232, 240, 255))

		// Button inside card
		btnBoxY := cy + cardH - 48
		btnBoxW := cardW - 24
		btnBoxH := 32.0
		if isSelected {
			vector.DrawFilledRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), accent, true)
			DrawText(screen, T("★ 当前选定 ★", "★ Selected ★"), cx+32, btnBoxY+8, 12, color.White)
		} else {
			vector.DrawFilledRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), RGBA(15, 23, 42, 220), true)
			vector.StrokeRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), 1.0, RGBA(100, 116, 139, 200), true)
			DrawText(screen, Tf("点击选择 [%d]", "Select [%d]", i+1), cx+36, btnBoxY+8, 12, RGBA(148, 163, 184, 255))
		}
	}

	// 4. Large Action Button: Start Playing
	btnW := 380.0
	btnH := 48.0
	btnX := (sw - btnW) / 2.0
	btnY := startY + cardH + 24.0
	g.menuStartBounds = Rect{X: btnX, Y: btnY, W: btnW, H: btnH}

	vector.DrawFilledRect(screen, float32(btnX+3), float32(btnY+5), float32(btnW), float32(btnH), RGBA(2, 6, 23, 180), true)
	vector.DrawFilledRect(screen, float32(btnX), float32(btnY), float32(btnW), float32(btnH), RGBA(16, 185, 129, 255), true)
	vector.StrokeRect(screen, float32(btnX), float32(btnY), float32(btnW), float32(btnH), 2.0, RGBA(167, 243, 208, 255), true)
	DrawText(screen, T("▶ 开始营运 / 进入所选城市 (回车 / 空格)", "▶ Start Operation / Enter City (Enter / Space)"), btnX+24, btnY+14, 15, color.White)

	// 5. Bottom Operation Tips
	bottomY := btnY + btnH + 18.0
	tips := T("【操作提示】按数字键 [1-6/C] 或 [← / →] 切换城市 | 按 [L] 切换语言 | 按 [R] 重载自定义地图 | 驾驶途中点击右上角【主菜单】或按 ESC 退回更换", "【Controls】Keys [1-6/C] or [←/→] Select City | [L] Toggle Language | [R] Hot Reload Map | Click [Menu] or ESC to Change")
	tipW, _ := MeasureText(tips, 12)
	DrawText(screen, tips, (sw-tipW)/2.0, bottomY, 12, RGBA(148, 163, 184, 255))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return g.Width, g.Height
}
