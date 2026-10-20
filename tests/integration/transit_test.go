package integration_test

import (
	"math"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestPlayerBusAutopilotToggle verifies that AI autopilot for the player's bus can be toggled on and off cleanly.
func TestPlayerBusAutopilotToggle(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)

	// Initially disabled
	if g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot should be disabled by default")
	}

	// 1. Toggle via Settings
	g.Settings.AutopilotEnabled = true
	if !g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot failed to enable via Settings")
	}

	g.Settings.AutopilotEnabled = false
	if g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot failed to disable via Settings")
	}

	// 2. Toggle via HUD Dashboard Click
	btnPos := sim.Vec2{
		X: g.HUD.AutopilotBounds.X + g.HUD.AutopilotBounds.W/2,
		Y: g.HUD.AutopilotBounds.Y + g.HUD.AutopilotBounds.H/2,
	}

	toggled, notice := g.HUD.HandleAutopilotClick(btnPos, g.Settings)
	if !toggled || !g.Settings.AutopilotEnabled {
		t.Fatalf("HandleAutopilotClick failed to toggle ON, notice=%s", notice)
	}

	// Click again to turn off
	toggled, notice = g.HUD.HandleAutopilotClick(btnPos, g.Settings)
	if !toggled || g.Settings.AutopilotEnabled {
		t.Fatalf("HandleAutopilotClick failed to toggle OFF, notice=%s", notice)
	}

	// Click outside bounds
	outsidePos := sim.Vec2{X: 10, Y: 10}
	toggled, _ = g.HUD.HandleAutopilotClick(outsidePos, g.Settings)
	if toggled {
		t.Fatalf("Click outside bounds should not trigger toggle")
	}
}

// TestPlayerBusAutopilotDockingAndDeparture tests that when autopilot is active,
// the driver steers to the stop, docks in the yellow bay, opens doors, dwells, closes doors, and advances stop.
func TestPlayerBusAutopilotDockingAndDeparture(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.AutopilotEnabled = true

	// Route starts at Stop 0, target is Stop 2
	g.Route.StopIDs = []int{0, 2, 10, 8, 0}
	g.Route.CurrentIndex = 1
	targetStop := g.World.Stops[2] // Stop 2

	// Position bus 60px ahead of Stop 2 aligned with heading, approaching at moderate speed
	g.Bus.Pos = targetStop.Pos.Sub(sim.Vec2{X: math.Cos(targetStop.Heading) * 60, Y: math.Sin(targetStop.Heading) * 60})
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 20.0
	g.Bus.DoorsOpen = false

	driver := autopilot.NewAutopilotDriver()
	driver.DwellTotal = 0.2 // Short dwell for fast test

	dt := 0.05
	docked := false
	doorsOpened := false
	doorsClosed := false

	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			g.Bus, g.Route, g.World, g.TrafficLights, nil, 52.0, dt,
		)
		g.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if isDocked {
			docked = true
		}
		if g.Bus.DoorsOpen {
			doorsOpened = true
		}
		if doorsOpened && !g.Bus.DoorsOpen && g.Bus.DoorProgress <= 0.05 {
			doorsClosed = true
			if driver.State != autopilot.AutopilotStateDocked {
				break
			}
		}
	}

	if !docked {
		t.Fatalf("Autopilot failed to dock inside the bus stop bay")
	}
	if !doorsOpened {
		t.Fatalf("Autopilot failed to open doors while docked")
	}
	if !doorsClosed {
		t.Fatalf("Autopilot failed to close doors after dwelling")
	}
	if driver.State != autopilot.AutopilotStateDeparting && driver.State != autopilot.AutopilotStateCruising {
		t.Fatalf("Autopilot state should transition to Departing/Cruising after dwell, got %v", driver.State)
	}
}

// TestCivilianCarsZeroShoulderDrivingAllMaps verifies Bug 3:
// Civilian AI vehicles driving on all city maps stay centered in lanes and 0 cars ever enter ZoneShoulder.
func TestCivilianCarsZeroShoulderDrivingAllMaps(t *testing.T) {
	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	laneW := 52.0
	dt := 1.0 / 60.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			world := sim.NewWorld(laneW, theme)
			tm := traffic.NewTrafficManager(laneW)
			tm.BuildForWorld(world, 24, laneW)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(world)
			bus := sim.NewBus(5000, 5000, 0) // Park dummy bus far away

			shoulderViolations := 0
			var firstViolationPos sim.Vec2
			var firstViolationName string

			for frame := 0; frame < 200; frame++ {
				tm.Update(dt, bus, tls, 2, laneW, world)

				for _, car := range tm.Cars {
					if car.InRoundabout {
						continue // Roundabout interior has distinct circular geometry
					}
					bRes := world.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone == sim.ZoneShoulder || bRes.Zone == sim.ZoneHardBarrier {
						shoulderViolations++
						if firstViolationPos == (sim.Vec2{}) {
							firstViolationPos = car.Pos
							firstViolationName = bRes.ObstacleName
						}
					}
				}
			}

			if shoulderViolations > 0 {
				t.Fatalf("[%s] Detected %d instances of civilian AI cars driving on shoulder/barrier! First at %v (%s)",
					theme.DisplayName(), shoulderViolations, firstViolationPos, firstViolationName)
			}
		})
	}
}

// TestIntegrationFullGameplayAllMaps simulates full end-to-end gameplay across all 6 maps:
// verifies single-route architecture, fleet dispatch, station auto-launch, obstacle avoidance,
// zero-shoulder civilian traffic, and smooth simulation without NaN or panics.
func TestIntegrationFullGameplayAllMaps(t *testing.T) {
	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			g := game.NewGame(1280, 720)
			g.EnterCity(theme)

			// 1. Verify single standard route architecture
			if g.Route == nil || len(g.Route.StopIDs) < 2 {
				t.Fatalf("[%s] Standard loop route must have >= 2 stops", theme.DisplayName())
			}
			if g.Route.StopIDs[0] != g.Route.StopIDs[len(g.Route.StopIDs)-1] {
				t.Fatalf("[%s] Route must be a closed circular loop", theme.DisplayName())
			}

			// 2. Dispatch 2 AI route buses
			g.Settings.AIBusCount = 2
			g.Settings.AIBusFleetChanged = true

			// 3. Enable player autopilot while parked at initial station
			g.Settings.AutopilotEnabled = true

			initPos := g.Bus.Pos

			// 4. Simulate 350 game ticks of live gameplay (dwell cycle takes ~208 frames, then vehicle cruises down the street)
			for frame := 0; frame < 350; frame++ {
				err := g.Update()
				if err != nil {
					t.Fatalf("[%s] Frame %d: g.Update() returned error: %v", theme.DisplayName(), frame, err)
				}

				// Player bus position and speed validity
				if math.IsNaN(g.Bus.Pos.X) || math.IsNaN(g.Bus.Pos.Y) {
					t.Fatalf("[%s] Frame %d: Player bus position is NaN", theme.DisplayName(), frame)
				}
				if math.IsNaN(g.Bus.Speed) {
					t.Fatalf("[%s] Frame %d: Player bus speed is NaN", theme.DisplayName(), frame)
				}

				// Check player bus doesn't collide with barriers
				bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, g.Settings.GetLaneWidth())
				if bRes.Zone == sim.ZoneHardBarrier {
					t.Fatalf("[%s] Frame %d: Player bus collided with hard barrier at %v (%s)",
						theme.DisplayName(), frame, g.Bus.Pos, bRes.ObstacleName)
				}

				// Check fleet buses
				if g.AIBusMgr != nil {
					for _, ab := range g.AIBusMgr.Buses {
						if math.IsNaN(ab.Bus.Pos.X) || math.IsNaN(ab.Bus.Pos.Y) {
							t.Fatalf("[%s] Frame %d: AI bus #%d position is NaN", theme.DisplayName(), frame, ab.ID)
						}
					}
				}

				// Check civilian cars do not drive on shoulder
				if g.TrafficMgr != nil {
					for _, c := range g.TrafficMgr.Cars {
						if c.InRoundabout {
							continue
						}
						cRes := g.World.CheckBoundaries(c.Pos, c.Width/2, g.Settings.GetLaneWidth())
						if cRes.Zone == sim.ZoneShoulder || cRes.Zone == sim.ZoneHardBarrier {
							t.Fatalf("[%s] Frame %d: Civilian car #%d driven on shoulder/barrier at %v",
								theme.DisplayName(), frame, c.ID, c.Pos)
						}
					}
				}
			}

			// 5. Verify the player bus successfully launched and moved away from the spawn station
			distMoved := g.Bus.Pos.Distance(initPos)
			if distMoved < 15.0 {
				t.Fatalf("[%s] Player bus failed to launch and move away from the stop (distMoved=%.2f < 15px)",
					theme.DisplayName(), distMoved)
			}
		})
	}
}

// TestAutopilotDockingWithLateralOffset verifies that a bus pulling in with a slight lateral offset
// (e.g. 20px off center) is correctly recognized by ContainsBus and activeStop, boarding passengers.
func TestAutopilotDockingWithLateralOffset(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	s0 := g.World.Stops[0]
	s0.WaitingPassengers = 5

	// Position bus parked in bay with 20px lateral offset
	sinH := math.Sin(s0.Heading)
	cosH := math.Cos(s0.Heading)
	perpX := -sinH
	perpY := cosH
	g.Bus.Pos = s0.Pos.Add(sim.Vec2{X: perpX * 20.0, Y: perpY * 20.0})
	g.Bus.Heading = s0.Heading
	g.Bus.Speed = 0.0
	g.Bus.DoorsOpen = false
	g.Bus.DoorProgress = 0.0
	g.Bus.Passengers = nil
	g.Bus.PassengerCount = 0

	if !s0.ContainsBus(g.Bus) {
		t.Fatalf("Bus with 20px lateral offset in bay should be accepted by ContainsBus")
	}

	// Press E to open doors in manual mode
	g.Bus.Update(0.016, 0, 0, 0, false, true)
	if !g.Bus.DoorsOpen {
		t.Fatalf("Doors should open when speed is 0 and toggleDoors is true")
	}

	// Run boarding logic
	var activeStop *sim.BusStop
	for _, s := range g.World.Stops {
		if s.ContainsBus(g.Bus) {
			activeStop = s
			break
		}
	}
	if activeStop == nil {
		t.Fatalf("activeStop should be detected for 20px lateral offset bus in bay")
	}

	candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
	actual := g.Bus.BoardPassengers(activeStop.WaitingPassengers, candidates)
	if actual != 5 {
		t.Fatalf("Expected 5 passengers boarded, got %d", actual)
	}
}

func TestBusStopBayStrictContainmentAndSpawn(t *testing.T) {
	g := game.NewGame(1280, 720)
	s0 := g.World.Stops[0]

	// 1. Bus must spawn directly inside the yellow bay box of Stop 0 (not in the middle of the road)
	if g.Bus.Pos.Distance(s0.Pos) > 1.0 {
		t.Fatalf("Bus should spawn at Stop 0 bay center %v, got %v", s0.Pos, g.Bus.Pos)
	}
	if !s0.ContainsBus(g.Bus) {
		t.Fatalf("Initial bus spawn must be considered properly parked inside Stop 0 bay")
	}

	// 2. Bus parked in the middle of the road adjacent to the stop MUST NOT be accepted
	// Road center is at s0.RoadCenter. On 4-lane road, driving lane is at Y=31.5
	busOnRoad := sim.NewBus(s0.RoadCenter.X, s0.RoadCenter.Y+31.5, s0.Heading)
	if s0.ContainsBus(busOnRoad) {
		t.Fatalf("Bus stopped in the middle of the road must NOT be accepted as parked in the stop bay")
	}

	// 3. Bus parked in rightmost lane on road surface (outside yellow box) MUST NOT be accepted
	busOnOuterLane := sim.NewBus(s0.RoadCenter.X, s0.RoadCenter.Y+70.0, s0.Heading)
	if s0.ContainsBus(busOnOuterLane) {
		t.Fatalf("Bus on road lane surface must NOT be accepted as parked in the stop bay")
	}

	// 4. Bus pulled squarely into the yellow bay box (within ±15 px laterally) MUST be accepted
	busInBay := sim.NewBus(s0.Pos.X+10.0, s0.Pos.Y+5.0, s0.Heading)
	if !s0.ContainsBus(busInBay) {
		t.Fatalf("Bus pulled inside the yellow stop bay box must be accepted")
	}
}

func TestCleanStationName(t *testing.T) {
	stop := &sim.BusStop{
		Name: "中央枢纽 (Central Terminal)",
	}

	locale.CurrentLanguage = locale.LanguageZH
	if clean := stop.CleanName(); clean != "中央枢纽" {
		t.Fatalf("Expected Chinese CleanName to be '中央枢纽', got %q", clean)
	}

	locale.CurrentLanguage = locale.LanguageEN
	if clean := stop.CleanName(); clean != "Central Terminal" {
		t.Fatalf("Expected English CleanName to be 'Central Terminal', got %q", clean)
	}

	// Reset language back to Chinese
	locale.CurrentLanguage = locale.LanguageZH
}

func TestFormattedRouteLinesNoOverflow(t *testing.T) {
	world := sim.NewWorld(52.0, sim.MapThemeMetropolis)

	// 1. Default 5-stop route
	route := sim.NewDefaultRoute()
	lines := route.GetFormattedRouteLines(world, 1100.0, 12.0)
	if len(lines) == 0 {
		t.Fatalf("Expected at least 1 formatted route line")
	}
	for i, l := range lines {
		w, _ := textui.MeasureText(l, 12.0)
		if w > 1100.0 {
			t.Fatalf("Line %d exceeds maxWidth 1100.0: w=%.1f text=%q", i, w, l)
		}
	}

	// 2. Constrained width test: ensure wrapping onto multiple lines
	narrowLines := route.GetFormattedRouteLines(world, 350.0, 12.0)
	if len(narrowLines) < 2 {
		t.Fatalf("Expected route to wrap onto multiple lines under 350px width, got %d lines: %v", len(narrowLines), narrowLines)
	}
	for i, l := range narrowLines {
		w, _ := textui.MeasureText(l, 12.0)
		if w > 350.0 {
			t.Fatalf("Narrow Line %d exceeds maxWidth 350.0: w=%.1f text=%q", i, w, l)
		}
	}

	// 3. Extremely long route (30 stops): must wrap and truncate safely without overflowing maxWidth
	longRoute := sim.NewRoute()
	for i := 0; i < 30; i++ {
		longRoute.AddStop(i % len(world.Stops))
	}
	longLines := longRoute.GetFormattedRouteLines(world, 1000.0, 12.0)
	if len(longLines) > 3 {
		t.Fatalf("Expected at most 3 wrapped lines for long route, got %d", len(longLines))
	}
	for i, l := range longLines {
		w, _ := textui.MeasureText(l, 12.0)
		if w > 1000.0 {
			t.Fatalf("Long route Line %d exceeds maxWidth 1000.0: w=%.1f text=%q", i, w, l)
		}
	}
}

// TestIsStopOccupiedIgnoresOpposingVehicles verifies that stopped or crawling vehicles in the
// oncoming lane facing the opposite direction do NOT count as occupying the bus stop bay.
func TestIsStopOccupiedIgnoresOpposingVehicles(t *testing.T) {
	g := game.NewGame(1280, 720)
	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]

	// 1. Initially empty
	g.AIBusMgr.Buses = nil
	g.TrafficMgr.Cars = nil
	if g.IsStopOccupiedByOtherVehicle(targetStop) {
		t.Fatalf("Empty stop should not be occupied")
	}

	// 2. An oncoming AI bus stopped in the opposing lane near the stop
	oppHeading := sim.NormalizeAngle(targetStop.Heading + math.Pi)
	oppBus := sim.NewBus(targetStop.Pos.X+10.0, targetStop.Pos.Y+10.0, oppHeading)
	oppBus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{{Bus: oppBus}}

	if g.IsStopOccupiedByOtherVehicle(targetStop) {
		t.Fatalf("Oncoming bus facing opposite direction should NOT count as occupying the stop bay")
	}

	// 3. A civilian car crawling in opposing direction near the stop
	oppCar := &sim.TrafficCar{
		Pos:     targetStop.Pos.Add(sim.Vec2{X: 15.0, Y: 15.0}),
		Heading: oppHeading,
		Speed:   2.0,
		Length:  48.0,
		Width:   22.0,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{oppCar}

	if g.IsStopOccupiedByOtherVehicle(targetStop) {
		t.Fatalf("Oncoming civilian car facing opposite direction should NOT count as occupying the stop bay")
	}

	// 4. Same car/bus facing the correct stop heading DOES count as occupying
	alignedBus := sim.NewBus(targetStop.Pos.X+10.0, targetStop.Pos.Y+10.0, targetStop.Heading)
	alignedBus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{{Bus: alignedBus}}

	if !g.IsStopOccupiedByOtherVehicle(targetStop) {
		t.Fatalf("Vehicle stopped facing the stop heading SHOULD count as occupying the stop bay")
	}
}

func TestAIMissedStopOvertakeContinuesDriving(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.State = game.StatePlaying
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficFinesEnabled = true

	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]
	targetStop.WaitingPassengers = 4

	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	laneW := g.Settings.GetLaneWidth()

	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())

	// Bus is 60px past stop in the passing lane, speed was 20px/s
	g.Bus.Pos = targetStop.Pos.Add(sim.Vec2{X: cosH * 60.0, Y: sinH * 60.0}).Add(toRoadDir.Mul(laneW))
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 20.0
	g.AutopilotDriver.LaneOffset = -laneW
	g.AutopilotDriver.TargetLaneOffset = -laneW
	g.AutopilotDriver.OvertakeState = autopilot.OvertakePassing

	// Place active car in the curb lane alongside the bus
	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if len(activeCars) > 0 {
		activeCars[0].Pos = targetStop.Pos.Add(sim.Vec2{X: cosH * 30.0, Y: sinH * 30.0})
		activeCars[0].Heading = targetStop.Heading
		activeCars[0].Speed = 0.0
		activeCars[0].Length = 32.0
		activeCars[0].Width = 18.0
	}

	// Trigger update
	_ = g.Update()

	// Verify $50 fine was applied and route advanced
	if g.Route.GetCurrentTargetStopID() == targetStopID {
		t.Fatalf("Expected route to advance past missed stop %d", targetStopID)
	}

	// Now run for 60 frames (1 second)
	for frame := 0; frame < 60; frame++ {
		_ = g.Update()
		// Advancing the route must preserve a physical through passage. The
		// previous bay-relative offset could steer sharply into the median.
		yaw := sim.NormalizeAngle(g.Bus.Heading - targetStop.Heading)
		right := sim.Vec2{X: -sinH, Y: cosH}
		clearance := g.Bus.Pos.Sub(targetStop.RoadCenter).Dot(right) - math.Abs(math.Sin(yaw))*g.Bus.Length/2 - math.Abs(math.Cos(yaw))*g.Bus.Width/2
		if clearance < 0 || math.Abs(yaw) > .3 {
			t.Fatalf("missed-stop continuation left its road corridor at frame%d: clearance%.2f yaw%.2f", frame, clearance, yaw)
		}
	}

	// Ensure the bus is actively driving forward along the road, not stuck motionless
	if g.Bus.Speed < 30.0 {
		t.Fatalf("Expected bus to continue driving at cruise speed, got speed=%.2f, TargetSpd=%.2f",
			g.Bus.Speed, g.AutopilotDriver.LastTargetSpeed)
	}
	if g.Bus.IsBraking || g.Bus.IsHandbraking {
		t.Fatalf("Expected bus brakes to be released, got IsBraking=%v, IsHandbraking=%v",
			g.Bus.IsBraking, g.Bus.IsHandbraking)
	}
}

// TestBerserkModePlayerBusOuterLaneDockingAndPassengerExchange verifies that:
// 1. In Berserk mode, player bus can dock directly from an outer travel lane alongside the station and exchange passengers.
// 2. In Standard mode (non-Berserk), outer-lane boarding is rejected and directs player to use the bay.
func TestBerserkModePlayerBusOuterLaneDockingAndPassengerExchange(t *testing.T) {
	// --- A. Berserk Mode: Allowed to dock and exchange passengers from outer lane ---
	g := game.NewGame(1280, 720)
	g.State = game.StatePlaying
	g.Settings.AutopilotBerserk = true
	g.HUD.Toasts = nil

	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]
	laneW := g.Settings.GetLaneWidth()

	// Ensure stop has passengers waiting
	targetStop.WaitingPassengers = 8

	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())

	// Position player bus in the travel lane alongside the station bay (lat offset = laneW, center squarely alongside)
	g.Bus.Pos = targetStop.Pos.Add(toRoadDir.Mul(laneW))
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 0.0
	g.Bus.DoorsOpen = false
	g.Cash = 100

	// Verify that ContainsBusInOuterLane is true, but ContainsBus is false
	if targetStop.ContainsBus(g.Bus) {
		t.Fatalf("Bus in outer lane should not qualify for standard bay ContainsBus")
	}
	if !targetStop.ContainsBusInOuterLane(g.Bus, laneW) {
		t.Fatalf("Bus alongside bay should qualify for ContainsBusInOuterLane")
	}

	// Open doors to board passengers
	g.Bus.DoorsOpen = true
	_ = g.Update()

	// Should have boarded passengers and earned fare
	if g.Bus.PassengerCount == 0 {
		t.Fatalf("Berserk mode player bus should have boarded passengers in outer lane, got %d", g.Bus.PassengerCount)
	}
	if g.Cash <= 100 {
		t.Fatalf("Berserk mode player bus should have collected fare, got Cash=%d", g.Cash)
	}

	// Verify toast includes berserk outer-lane branding
	hasBerserkToast := false
	for _, toast := range g.HUD.Toasts {
		if strings.Contains(toast.Text, "狂暴外道靠站") || strings.Contains(toast.Text, "Berserk Outer-Lane") {
			hasBerserkToast = true
			break
		}
	}
	if !hasBerserkToast {
		t.Fatalf("Expected Berserk outer-lane dock toast, got toasts: %v", g.HUD.Toasts)
	}

	// --- B. Standard Mode (Non-Berserk): Outer lane docking is NOT allowed ---
	gStd := game.NewGame(1280, 720)
	gStd.State = game.StatePlaying
	gStd.Settings.AutopilotBerserk = false
	if gStd.AutopilotDriver != nil {
		gStd.AutopilotDriver.BerserkMode = false
	}
	gStd.HUD.Toasts = nil

	targetStopStd := gStd.World.Stops[targetStopID]
	targetStopStd.WaitingPassengers = 8

	gStd.Bus.Pos = targetStopStd.Pos.Add(toRoadDir.Mul(laneW))
	gStd.Bus.Heading = targetStopStd.Heading
	gStd.Bus.Speed = 0.0
	gStd.Bus.DoorsOpen = false
	gStd.Cash = 100

	// In standard mode, opening doors in outer lane should NOT board passengers
	gStd.Bus.DoorsOpen = true
	_ = gStd.Update()

	if gStd.Bus.PassengerCount != 0 {
		t.Fatalf("Standard mode bus should NOT board passengers from outer lane, got %d", gStd.Bus.PassengerCount)
	}
	if gStd.Cash != 100 {
		t.Fatalf("Standard mode bus should not collect fare in outer lane, got %d", gStd.Cash)
	}
}

// TestBerserkModePlayerBusOppositeLaneDockingOnTwoLaneRoad verifies that:
//  1. On two-way two-lane roads (双向双车道), player bus in Berserk mode can dock and exchange passengers
//     when temporarily stopping in the opposite-direction travel lane (both forward and reverse headings).
//  2. On 4-lane roads, opposing carriageway docking is strictly disallowed.
//  3. In Standard mode (non-Berserk), opposing-lane docking is disallowed.
func TestBerserkModePlayerBusOppositeLaneDockingOnTwoLaneRoad(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.State = game.StatePlaying
	g.Settings.AutopilotBerserk = true
	laneW := g.Settings.GetLaneWidth()

	// In Metropolis, Stop 2 is on a 2-lane road (RoadLanes = 2)
	stop2L := g.World.Stops[2]
	stop2L.WaitingPassengers = 10
	stop4L := g.World.Stops[3] // Stop 3 is on a 4-lane road (RoadLanes = 4)

	g.Route.StopIDs = []int{2, 3}
	g.Route.CurrentIndex = 0
	g.Cash = 100
	g.HUD.Toasts = nil

	toOpposite := stop2L.RoadCenter.Sub(stop2L.Pos)
	toOppositeDir := toOpposite.Mul(1.0 / toOpposite.Length())

	// Opposite-direction lane is across the road center
	oppositeLanePos := stop2L.RoadCenter.Add(toOppositeDir.Mul(laneW * 0.5))

	// --- A. Berserk Mode Forward in Opposite Lane on 2-Lane Road ---
	g.Bus.Pos = oppositeLanePos
	g.Bus.Heading = stop2L.Heading
	g.Bus.Speed = 0.0

	if stop2L.ContainsBus(g.Bus) {
		t.Fatalf("Bus in opposite lane should NOT match standard bay ContainsBus")
	}
	if !stop2L.ContainsBusInOuterLane(g.Bus, laneW) {
		t.Fatalf("Bus in opposite lane on 2-lane road MUST match ContainsBusInOuterLane in Berserk mode")
	}

	// Open doors -> boards passengers and earns revenue
	g.Bus.DoorsOpen = true
	_ = g.Update()

	if g.Bus.PassengerCount == 0 {
		t.Fatalf("Berserk mode bus should have boarded passengers from opposite lane on 2-lane road, got 0")
	}
	if g.Cash <= 100 {
		t.Fatalf("Berserk mode bus should have collected fare, got Cash=%d", g.Cash)
	}

	// Verify opposing-lane branding toast
	hasOpposeToast := false
	for _, toast := range g.HUD.Toasts {
		if strings.Contains(toast.Text, "狂暴对向借道靠站") || strings.Contains(toast.Text, "Berserk Opposing-Lane") {
			hasOpposeToast = true
			break
		}
	}
	if !hasOpposeToast {
		t.Fatalf("Expected opposing-lane dock toast, got: %v", g.HUD.Toasts)
	}

	// --- B. Berserk Mode Reverse Heading in Opposite Lane on 2-Lane Road ---
	busRev := sim.NewBus(oppositeLanePos.X, oppositeLanePos.Y, sim.NormalizeAngle(stop2L.Heading+math.Pi))
	if !stop2L.ContainsBusInOuterLane(busRev, laneW) {
		t.Fatalf("Bus in opposite lane with reverse heading on 2-lane road should qualify for Berserk temporary dock")
	}

	// --- C. 4-Lane Road Stop: Opposing Carriageway is NOT Permitted ---
	toOpposite4L := stop4L.RoadCenter.Sub(stop4L.Pos)
	toOppositeDir4L := toOpposite4L.Mul(1.0 / toOpposite4L.Length())
	busOncoming4L := sim.NewBus(stop4L.RoadCenter.X+toOppositeDir4L.X*laneW*0.5, stop4L.RoadCenter.Y+toOppositeDir4L.Y*laneW*0.5, stop4L.Heading)
	if stop4L.ContainsBusInOuterLane(busOncoming4L, laneW) {
		t.Fatalf("Opposing carriageway on 4-lane road must NOT be accepted by ContainsBusInOuterLane")
	}

	// --- D. Standard Mode (Non-Berserk) on 2-Lane Road: Opposing lane docking is NOT Permitted ---
	gStd := game.NewGame(1280, 720)
	gStd.State = game.StatePlaying
	gStd.Settings.AutopilotBerserk = false
	if gStd.AutopilotDriver != nil {
		gStd.AutopilotDriver.BerserkMode = false
	}
	gStd.Route.StopIDs = []int{2, 3}
	gStd.Route.CurrentIndex = 0
	gStd.Cash = 100
	stop2L.WaitingPassengers = 10

	gStd.Bus.Pos = oppositeLanePos
	gStd.Bus.Heading = stop2L.Heading
	gStd.Bus.Speed = 0.0
	gStd.Bus.DoorsOpen = true
	_ = gStd.Update()

	if gStd.Bus.PassengerCount != 0 {
		t.Fatalf("Standard mode bus must NOT board passengers from opposite lane, got %d", gStd.Bus.PassengerCount)
	}
	if gStd.Cash != 100 {
		t.Fatalf("Standard mode bus must NOT collect fare from opposite lane, got Cash=%d", gStd.Cash)
	}
}
