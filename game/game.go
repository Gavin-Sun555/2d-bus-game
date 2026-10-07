package game

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

type Game struct {
	Width  int
	Height int

	Bus           *Bus
	World         *World
	Route         *Route
	Camera        *Camera
	HUD           *HUD
	TrafficLights *TrafficLightSystem
	TrafficMgr    *TrafficManager
	Settings      *Settings

	IsPlanMode bool
	Cash       int

	// Track lane width changes
	lastLaneWidth float64

	// Mouse dragging for planning mode
	isDragging   bool
	dragStartPos Vec2
	camStartPos  Vec2

	// Boarding tracking
	boardedThisStop bool

	// Stop Request Bell & Skip Stop (飞站) tracking
	stopBellRung        bool
	passengersAlighting int
	bellToastShown      bool
	skipCooldown        float64
}

func NewGame(width, height int) *Game {
	settings := NewSettings()
	initLaneWidth := settings.GetLaneWidth()

	world := NewWorld(initLaneWidth)

	g := &Game{
		Width:         width,
		Height:        height,
		World:         world,
		Route:         NewDefaultRoute(),
		Camera:        NewCamera(width, height),
		HUD:           NewHUD(),
		TrafficLights: NewTrafficLightSystem(),
		TrafficMgr:    NewTrafficManager(initLaneWidth),
		Settings:      settings,
		IsPlanMode:    false,
		Cash:          100, // Starting budget
		lastLaneWidth: initLaneWidth,
	}

	// Spawn bus parked inside the first station's bay box (Stop 0: Central Terminal)
	firstStop := g.World.Stops[0]
	g.Bus = NewBus(firstStop.Pos.X, firstStop.Pos.Y, firstStop.Heading)
	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos

	g.rollAlightingForNextStop()

	g.HUD.AddToast("欢迎来到超大都市 2D 公交模拟器！12 大特色站点已上线", true)
	g.HUD.AddToast("【全新机制】路线仅指引至下一站；站内无人且车内无下车铃时可按 [F] 飞站！", false)
	g.HUD.AddToast("【始发站已就位】公交车已停靠在中央枢纽黄色框内，按 [E] 开门上下客！", false)

	return g
}

func (g *Game) rollAlightingForNextStop() {
	g.bellToastShown = false
	if g.Bus.PassengerCount == 0 {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}

	if g.Bus.PassengerCount <= 3 {
		if rand.Float64() < 0.5 {
			g.passengersAlighting = 1
			g.stopBellRung = true
		} else {
			g.passengersAlighting = 0
			g.stopBellRung = false
		}
	} else {
		if rand.Float64() < 0.35 {
			g.passengersAlighting = 0
			g.stopBellRung = false
		} else {
			maxA := g.Bus.PassengerCount / 2
			if maxA < 1 {
				maxA = 1
			}
			if maxA > 5 {
				maxA = 5
			}
			g.passengersAlighting = 1 + rand.Intn(maxA)
			g.stopBellRung = true
		}
	}
}

func (g *Game) Update() error {
	dt := 1.0 / 60.0

	// 1. Settings menu takes priority
	g.Settings.Update(g.Camera)

	// Check if lane width customized
	currentLaneW := g.Settings.GetLaneWidth()
	if currentLaneW != g.lastLaneWidth {
		g.lastLaneWidth = currentLaneW
		g.World.UpdateLaneWidth(currentLaneW)
		g.HUD.AddToast(fmt.Sprintf("全城道路标线与车道宽度已动态切换为: %.0f px", currentLaneW), true)
	}

	if g.Settings.IsOpen {
		// Game paused while in settings menu
		return nil
	}

	// 2. Mode switching (TAB)
	if inpututil.IsKeyJustPressed(ebiten.KeyTab) {
		g.IsPlanMode = !g.IsPlanMode
		if g.IsPlanMode {
			// Instant transition to panoramic planning view (no slow progressive animation)
			g.Camera.TargetPos = Vec2{X: 0, Y: 0}
			g.Camera.Pos = Vec2{X: 0, Y: 0}
			g.Camera.TargetZoom = 0.30
			g.Camera.Zoom = 0.30
			g.Camera.Rotation = 0
			g.Camera.TargetRotation = 0
			g.HUD.AddToast("已进入【线路规划模式】：点击站点可加入路线 (滚轮可缩放/拖拽漫游)", false)
		} else {
			// Instant snap back to driver mode
			g.Camera.FollowBus(g.Bus)
			g.Camera.Pos = g.Camera.TargetPos
			g.Camera.TargetZoom = 1.0
			g.Camera.Zoom = 1.0
			g.Camera.Rotation = g.Camera.TargetRotation
			g.HUD.AddToast("已进入【驾驶模式】：开始运营发车！", true)
		}
	}

	// 3. Traffic lights update
	g.TrafficLights.Update(dt)

	// 4. AI Traffic & Violations (Paused during Planning Mode to eliminate stutter and lag)
	if !g.IsPlanMode {
		// AI Traffic vehicles update with dynamic lane width
		g.TrafficMgr.Update(dt, g.Bus, g.TrafficLights, g.Settings.TrafficDensity, currentLaneW)

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
	}

	// 7. Planning / Driver Mode inputs
	if g.IsPlanMode {
		g.updatePlanningMode(dt)
	} else {
		g.updateDriverMode(dt)
	}

	// 8. Camera update
	g.Camera.Update(dt)

	// 9. World update (passenger arrivals)
	g.World.Update(dt)

	// 10. HUD toasts update
	g.HUD.Update(dt)

	return nil
}

func (g *Game) checkBusCarCollisions() {
	activeCount := 24
	if g.Settings.TrafficDensity == 1 {
		activeCount = 12
	} else if g.Settings.TrafficDensity == 3 {
		activeCount = 36
	}
	if activeCount > len(g.TrafficMgr.Cars) {
		activeCount = len(g.TrafficMgr.Cars)
	}

	busOBB := OBB{
		Center:  g.Bus.Pos,
		HalfLen: g.Bus.Length / 2, // 48.0
		HalfWid: g.Bus.Width / 2,  // 18.0
		Heading: g.Bus.Heading,
	}

	for i := 0; i < activeCount; i++ {
		car := g.TrafficMgr.Cars[i]
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

			// 1. Positional Separation:
			// Normal points from Car to Bus.
			// Bus is pushed back along +normal, car is pushed forward along -normal.
			if car.Speed < 5.0 {
				// Stationary car (e.g. stopped at intersection / red light):
				// Bus absorbs 100% of separation so it is completely blocked from penetrating
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth))
			} else {
				// Moving car: bus takes 75% separation pushback, car pushed 25% ahead
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.75))
				car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.25))
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

				// Momentum transfer: Accelerate the car forward away from the bus!
				car.Speed = math.Max(car.Speed+30.0, math.Abs(g.Bus.Speed)*0.9+20.0)
				car.IsBraking = false

				// Clamp and stop the bus from ramming through the car
				if impactSpeed > 45.0 {
					// High-speed collision: bounce recoil
					g.Bus.Speed = -g.Bus.Speed * 0.25
				} else {
					// Low-speed contact: clamp bus speed to 0 or match/trail car
					if g.Bus.Speed > 0 {
						g.Bus.Speed = 0
					}
				}

				// 3. Collision Damage, Shake & HUD Feedback
				if g.Bus.CrashCooldown <= 0 {
					g.Bus.CrashCooldown = 1.0
					g.Camera.AddShake(math.Min(12.0, 4.0+impactSpeed*0.15))

					damage := 8.0
					if impactSpeed > 80.0 {
						damage = 18.0
					}
					g.Bus.Durability = math.Max(0, g.Bus.Durability-damage)

					if g.Settings.TrafficFinesEnabled {
						fine := 50
						g.Cash -= fine
						g.HUD.AddWarningToast(fmt.Sprintf("[事故碰撞] 撞击社会车辆！车况 -%.0f%%，赔偿 -$%d", damage, fine))
					} else {
						g.HUD.AddWarningToast(fmt.Sprintf("[事故碰撞] 撞击社会车辆！车况 -%.0f%% (已豁免罚金)", damage))
					}

					if g.Bus.Durability <= 0 {
						g.Bus.Durability = 50.0
						g.Cash -= 200
						g.HUD.AddWarningToast("[严重损毁] 车况归零！已紧急呼叫拖车抢修至 50% (维修费 -$200)")
					}
				}
			} else {
				// Even if not moving towards each other (e.g. static pushing / holding W),
				// firmly clamp bus speed so throttle cannot overpower the car
				if g.Bus.Speed > car.Speed {
					g.Bus.Speed = math.Max(0, car.Speed*0.5)
				}
			}
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
		g.Camera.TargetZoom = Clamp(g.Camera.TargetZoom+wheelY*0.06, 0.16, 1.4)
		g.Camera.Zoom = g.Camera.TargetZoom
	}

	// Mouse Pan / Drag
	mx, my := ebiten.CursorPosition()
	mouseScreen := Vec2{X: float64(mx), Y: float64(my)}

	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		// Check top-bar camera button click
		if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, false)
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
			g.Route.AddStop(clickedStopID)
			stopName := g.World.Stops[clickedStopID].Name
			g.HUD.AddToast(fmt.Sprintf("已将站点 [%s] 添加至路线", stopName), true)
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

	// Clear route with C key
	if inpututil.IsKeyJustPressed(ebiten.KeyC) {
		g.Route.Clear()
		g.HUD.AddToast("已清空规划路线，请重新规划！", false)
	}
}

func (g *Game) updateDriverMode(dt float64) {
	// Driver controls: Separated accelerator throttle and service foot brake
	throttle := 0.0
	if ebiten.IsKeyPressed(ebiten.KeyW) || ebiten.IsKeyPressed(ebiten.KeyUp) {
		throttle = 1.0
	}

	brake := 0.0
	if ebiten.IsKeyPressed(ebiten.KeyS) || ebiten.IsKeyPressed(ebiten.KeyDown) {
		brake = 1.0
	}

	steer := 0.0
	if ebiten.IsKeyPressed(ebiten.KeyA) || ebiten.IsKeyPressed(ebiten.KeyLeft) {
		steer -= 1.0
	}
	if ebiten.IsKeyPressed(ebiten.KeyD) || ebiten.IsKeyPressed(ebiten.KeyRight) {
		steer += 1.0
	}

	handbrake := ebiten.IsKeyPressed(ebiten.KeySpace)
	toggleDoors := inpututil.IsKeyJustPressed(ebiten.KeyE)

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

	// 5. Mouse click on HUD buttons (Perspective badge & Gear selectors)
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mouseScreen := Vec2{X: float64(mx), Y: float64(my)}
		if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, true)
		} else if shifted, notice := g.HUD.HandleGearClick(mouseScreen, g.Bus); shifted {
			g.HUD.AddToast(notice, false)
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
	currentLaneW := g.Settings.GetLaneWidth()
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

		// 1. Chime bell notification when approaching within 550px
		if distToTarget < 550 && g.stopBellRung && !g.bellToastShown {
			g.bellToastShown = true
			g.HUD.AddToast(fmt.Sprintf("[🔔 下车铃响] 叮咚~ 车内有 %d 位乘客按下车铃，将在下一站下车！", g.passengersAlighting), false)
		}

		// 2. Skip Stop (飞站) check when approaching within 280px
		if distToTarget <= 280.0 {
			canSkip := targetStop.WaitingPassengers == 0 && !g.stopBellRung

			// Manual skip with F key
			if inpututil.IsKeyJustPressed(ebiten.KeyF) {
				if canSkip {
					oldName := targetStop.Name
					g.Route.AdvanceToNextStop()
					newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
					g.rollAlightingForNextStop()
					g.HUD.AddToast(fmt.Sprintf("【⚡ 飞站成功】[%s] 站内无人且无人按铃，已飞站！下一站: [%s]", oldName, newTarget.Name), true)
				} else {
					reasons := ""
					if targetStop.WaitingPassengers > 0 {
						reasons += fmt.Sprintf("站台有 %d 人候车 ", targetStop.WaitingPassengers)
					}
					if g.stopBellRung {
						reasons += fmt.Sprintf("车内有 %d 人按铃下车 ", g.passengersAlighting)
					}
					g.HUD.AddWarningToast(fmt.Sprintf("【不可飞站】[%s] %s，必须进站停靠！", targetStop.Name, reasons))
				}
			}

			// Drive-by auto skip when driving past the stop
			dVec := g.Bus.Pos.Sub(targetStop.Pos)
			headingVec := Vec2{X: math.Cos(targetStop.Heading), Y: math.Sin(targetStop.Heading)}
			forwardProj := dVec.X*headingVec.X + dVec.Y*headingVec.Y

			if forwardProj > 50 && distToTarget < 220 && math.Abs(g.Bus.Speed) > 15 {
				if canSkip && g.skipCooldown <= 0 {
					g.skipCooldown = 2.5
					oldName := targetStop.Name
					g.Route.AdvanceToNextStop()
					newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
					g.rollAlightingForNextStop()
					g.HUD.AddToast(fmt.Sprintf("【⚡ 快速飞站】顺利掠过 [%s]！已自动飞站，下一站: [%s]", oldName, newTarget.Name), true)
				}
			}
		}
	}

	// Check if parked at any bus stop
	var activeStop *BusStop
	var misalignedStop *BusStop
	for _, s := range g.World.Stops {
		if s.ContainsBus(g.Bus) {
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
		if g.Bus.DoorsOpen && !g.boardedThisStop {
			// Embark / Disembark passengers
			g.boardedThisStop = true

			// Passengers getting off
			offCount := 0
			if g.passengersAlighting > 0 {
				offCount = g.passengersAlighting
				if offCount > g.Bus.PassengerCount {
					offCount = g.Bus.PassengerCount
				}
				g.Bus.PassengerCount -= offCount
				g.stopBellRung = false
				g.passengersAlighting = 0
			} else if g.Bus.PassengerCount > 0 {
				offCount = 1 + rand.Intn(g.Bus.PassengerCount)
				if offCount > g.Bus.PassengerCount {
					offCount = g.Bus.PassengerCount
				}
				g.Bus.PassengerCount -= offCount
			}

			// Passengers getting on
			boardSpace := g.Bus.Capacity - g.Bus.PassengerCount
			boardCount := activeStop.WaitingPassengers
			if boardCount > boardSpace {
				boardCount = boardSpace
			}

			if boardCount > 0 {
				activeStop.WaitingPassengers -= boardCount
				g.Bus.PassengerCount += boardCount
				fareEarned := boardCount * 3
				g.Cash += fareEarned
				toastMsg := fmt.Sprintf("[%s] 下客 %d 人，上客 %d 人，营收 +$%d！",
					activeStop.Name, offCount, boardCount, fareEarned)
				g.HUD.AddToast(toastMsg, true)
			} else {
				toastMsg := fmt.Sprintf("[%s] 下客 %d 人，当前车厢已满载！", activeStop.Name, offCount)
				g.HUD.AddToast(toastMsg, false)
			}

			// Advance to next stop if this was the planned target stop
			if activeStop.ID == g.Route.GetCurrentTargetStopID() {
				g.Route.AdvanceToNextStop()
				g.rollAlightingForNextStop()
			}
		}
	} else {
		// Left the stop bay, reset boarding flag
		g.boardedThisStop = false
	}
}

func (g *Game) Draw(screen *ebiten.Image) {
	targetStopID := g.Route.GetCurrentTargetStopID()
	laneWidth := g.Settings.GetLaneWidth()

	// 1. Draw World (Roads, Buildings, Trees, Rivers, Bus Stops)
	g.World.Draw(screen, g.Camera, targetStopID, laneWidth)

	// 2. Draw Traffic Lights and Stop Lines with current lane width
	g.TrafficLights.Draw(screen, g.Camera, laneWidth)

	// 3. Draw Active Route lines on map (offset into driving lanes)
	g.Route.DrawRouteLine(screen, g.Camera, g.World, g.IsPlanMode, laneWidth)

	// 4. Draw AI Traffic Vehicles (Hidden during Planning Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		g.TrafficMgr.Draw(screen, g.Camera, g.Settings.TrafficDensity)
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
	g.HUD.Draw(screen, g.Bus, g.Route, g.World, g.IsPlanMode, g.Cash, currentStop, g.Settings, g.stopBellRung, g.passengersAlighting, g.Camera, g.TrafficLights)

	// 8. Draw Settings Modal (on top of everything)
	g.Settings.Draw(screen, g.Camera)
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return g.Width, g.Height
}
