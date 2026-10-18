package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestRoadCameraAutonomousClearance verifies that the bus in AI cruise mode drives normally past
// mid-block road surveillance cameras without stopping (Bug #2).
func TestRoadCameraAutonomousClearance(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.AutopilotEnabled = true
	g.Settings.TrafficDensity = 2 // Medium density (24 active cars out of 36)

	// Ensure no spawn definitions coincide directly with road camera locations
	for _, cam := range g.TrafficLights.RoadCameras {
		for _, c := range g.TrafficMgr.Cars {
			if c.Pos.Distance(cam.Pos) < 10.0 {
				t.Fatalf("Car %d spawns directly on top of road camera at (%.1f, %.1f)", c.ID, cam.Pos.X, cam.Pos.Y)
			}
		}
	}

	// Inactive cars (index >= 24) must never be added to active obstacle list
	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if len(activeCars) != 24 {
		t.Fatalf("Expected 24 active cars for density level 2, got %d", len(activeCars))
	}

	// Set active leg Stop 9 -> Stop 10 Eastbound along South Ring (Y=1200*MapGridScale)
	g.Route.StopIDs = []int{9, 10}
	g.Route.CurrentIndex = 1
	camPos := sim.Vec2{X: -650 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}
	g.Bus.Pos = sim.Vec2{X: camPos.X - 200, Y: 1200*sim.MapGridScale + 52.0*1.5}
	g.Bus.Heading = 0 // Eastbound
	g.Bus.Speed = 100.0
	g.AutopilotDriver.Reset()

	// Advance simulation past the camera gantry
	passedCamera := false
	for frame := 0; frame < 180; frame++ {
		_ = g.Update()
		if g.Bus.Pos.X > camPos.X+80.0 {
			passedCamera = true
			break
		}
		// Bus must never brake to a halt at the camera
		if g.Bus.Pos.X > camPos.X-100.0 && g.Bus.Pos.X < camPos.X+50.0 {
			if g.Bus.Speed < 30.0 {
				t.Fatalf("[Frame %d] Bus stopped near road camera: Pos=(%.1f, %.1f), Spd=%.1f",
					frame, g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Speed)
			}
		}
	}
	if !passedCamera {
		t.Fatalf("Bus failed to drive past road camera within 180 frames; final pos=(%.1f, %.1f)", g.Bus.Pos.X, g.Bus.Pos.Y)
	}
}

// TestTrafficDensity72And128 verifies the 4th (72 cars) and 5th (128 cars) density options across all 6 maps.
func TestTrafficDensity72And128(t *testing.T) {
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
			lw := 52.0
			w := sim.NewWorld(lw, theme)
			tm := traffic.NewTrafficManager(lw)
			tm.BuildForWorld(w, traffic.MaxTrafficCars, lw)

			if len(tm.Cars) != 128 {
				t.Fatalf("[%s] Expected 128 total cars generated, got %d", theme.DisplayName(), len(tm.Cars))
			}

			// Verify density levels: 1:12, 2:24, 3:36, 4:72, 5:128
			expectedCounts := map[int]int{
				1: 12,
				2: 24,
				3: 36,
				4: 72,
				5: 128,
			}
			for lvl, exp := range expectedCounts {
				active := tm.GetActiveCars(lvl)
				if len(active) != exp {
					t.Fatalf("[%s] Density level %d: expected %d active cars, got %d",
						theme.DisplayName(), lvl, exp, len(active))
				}
			}

			// Verify simulation step with 128 cars runs smoothly without NaN or panic
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			playerBus := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, w.Stops[0].Heading)

			dt := 0.05
			for frame := 0; frame < 30; frame++ {
				tls.Update(dt)
				tm.Update(dt, playerBus, tls, 5, lw, w)
				for idx, c := range tm.GetActiveCars(5) {
					if math.IsNaN(c.Pos.X) || math.IsNaN(c.Pos.Y) || math.IsNaN(c.Speed) {
						t.Fatalf("[%s] Car #%d has NaN values at frame %d", theme.DisplayName(), idx, frame)
					}
				}
			}
		})
	}
}

func TestAITraffic(t *testing.T) {
	tm := traffic.NewTrafficManager(42.0)
	if len(tm.Cars) < 8 {
		t.Fatalf("Expected pre-allocated AI traffic cars, got %d", len(tm.Cars))
	}

	tls := sim.NewTrafficLightSystem()
	bus := sim.NewBus(0, 0, 0)

	// Step simulation
	for i := 0; i < 30; i++ {
		tm.Update(0.016, bus, tls, 2, 42.0)
	}

	for _, car := range tm.Cars[:8] {
		if math.IsNaN(car.Pos.X) || math.IsNaN(car.Pos.Y) {
			t.Fatalf("Car position resulted in NaN")
		}
	}
}

func TestInitialCarSpawnsClearOfIntersectionsAndRoadCenter(t *testing.T) {
	tm := traffic.NewTrafficManager(42.0)
	if len(tm.Cars) == 0 {
		t.Fatalf("Expected initialized cars")
	}

	interCenters := make([]sim.Vec2, 25)
	rawInters := []sim.Vec2{
		{X: -2600, Y: -1200}, {X: -1300, Y: -1200}, {X: 0, Y: -1200}, {X: 1300, Y: -1200}, {X: 2600, Y: -1200},
		{X: -2600, Y: -600}, {X: -1300, Y: -600}, {X: 0, Y: -600}, {X: 1300, Y: -600}, {X: 2600, Y: -600},
		{X: -2600, Y: 0}, {X: -1300, Y: 0}, {X: 0, Y: 0}, {X: 1300, Y: 0}, {X: 2600, Y: 0},
		{X: -2600, Y: 600}, {X: -1300, Y: 600}, {X: 0, Y: 600}, {X: 1300, Y: 600}, {X: 2600, Y: 600},
		{X: -2600, Y: 1200}, {X: -1300, Y: 1200}, {X: 0, Y: 1200}, {X: 1300, Y: 1200}, {X: 2600, Y: 1200},
	}
	for idx, ic := range rawInters {
		interCenters[idx] = ic.Mul(sim.MapGridScale)
	}

	// Central Terminal bay position
	centralTerminal := sim.Vec2{X: -650 * sim.MapGridScale, Y: 0}

	for i, car := range tm.Cars {
		// Verify no car spawns inside or within 220px of ANY intersection center
		for _, ic := range interCenters {
			dist := car.Pos.Distance(ic)
			if dist < 220.0 {
				t.Fatalf("Car #%d at %v spawned too close to intersection %v (dist=%.1f < 220)", i, car.Pos, ic, dist)
			}
		}

		// Verify no car spawns directly inside Central Terminal departure bay
		if car.Pos.Distance(centralTerminal) < 120.0 {
			t.Fatalf("Car #%d spawned conflicting with Central Terminal departure bay at %v", i, car.Pos)
		}
	}
}

func TestAICarStoppingBufferClearOfZebraCrossing(t *testing.T) {
	inter := &sim.TrafficIntersection{
		ID:             0,
		Center:         sim.Vec2{X: 0, Y: 0},
		Phase:          sim.PhaseNSGreen, // EW is Red
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
	laneW := 42.0
	stopLineDist := laneW*1.6 + 18.0     // 85.2
	crosswalkDist := stopLineDist - 25.0 // 60.2

	// AI Car sedan length = 46, halfLen = 23.
	car := &sim.TrafficCar{
		ID:          0,
		Pos:         sim.Vec2{X: -150, Y: 30},
		Heading:     0,
		Speed:       120,
		TargetSpeed: 150,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}

	tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
	bus := sim.NewBus(1000, 1000, 0) // bus far away

	// Simulate AI car updating for 3 seconds approaching the red light
	dt := 0.016
	for step := 0; step < 180; step++ {
		tm.Update(dt, bus, tls, 1, laneW)
	}

	// Front bumper position
	halfLen := car.Length * 0.5
	bumperX := car.Pos.X + math.Cos(car.Heading)*halfLen

	// Verify bumper stopped BEFORE the stop line:
	stopLineX := -stopLineDist
	if bumperX > stopLineX {
		t.Fatalf("AI Car bumper (%.2f) must be strictly behind stop line (%.2f)", bumperX, stopLineX)
	}

	// Verify bumper is well clear (>20px) of the zebra crossing (crosswalk outer edge is at -crosswalkDist - 10 = -70.2):
	crosswalkOuterEdgeX := -crosswalkDist - 10.0
	bufferDistance := crosswalkOuterEdgeX - bumperX
	if bufferDistance < 15.0 {
		t.Fatalf("AI Car bumper (%.2f) stopped too close to zebra crossing (edge: %.2f, buffer: %.2f < 15px)",
			bumperX, crosswalkOuterEdgeX, bufferDistance)
	}
}

func TestAIVehicleLaneComplianceAndTurnExecution(t *testing.T) {
	tls := sim.NewTrafficLightSystem()
	laneW := 42.0

	// Major intersection at (0,0)
	var majorInter *sim.TrafficIntersection
	for _, inter := range tls.Intersections {
		if inter.Center.X == 0 && inter.Center.Y == 0 {
			majorInter = inter
			break
		}
	}
	if majorInter == nil {
		t.Fatalf("Expected major intersection at (0,0)")
	}
	majorInter.Phase = sim.PhaseEWLeftGreen // Left turn is Green!

	// AI Car in Lane 0 (inner lane, Eastbound, approaching (0,0))
	car := &sim.TrafficCar{
		ID:          0,
		Pos:         sim.Vec2{X: -15, Y: 21.0}, // Right at intersection center
		Heading:     0,
		Speed:       80,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
		LaneIndex:   0, // Inner left-turn lane
	}

	tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
	bus := sim.NewBus(1000, 1000, 0)

	// Update 1 step
	tm.Update(0.016, bus, tls, 1, laneW)

	// Car must have executed its left turn: heading North (-Pi/2), IsEW=false, RoadCenter=0
	if car.IsEW {
		t.Fatalf("AI Car in left-turn lane must turn onto cross street (IsEW=false)")
	}
	if math.Abs(sim.NormalizeAngle(car.Heading-(-math.Pi/2))) > 0.1 {
		t.Fatalf("AI Car must turn left towards North (-Pi/2), got heading: %f", car.Heading)
	}
	if car.LaneIndex != 1 {
		t.Fatalf("AI Car must join through lane (LaneIndex=1) on the new road, got %d", car.LaneIndex)
	}
}

func TestYellowBoxIntersectionClearanceForTrafficCars(t *testing.T) {
	lw := 52.0
	inter := &sim.TrafficIntersection{
		Center:            sim.Vec2{X: 0, Y: 0},
		Phase:             sim.PhaseNSGreen, // NS green for northbound vehicle
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := sim.NewTrafficLightSystem()
	tls.Intersections = []*sim.TrafficIntersection{inter}

	tm := traffic.NewTrafficManager(lw)

	// Stop line distance for northbound approach (~126px from center)
	stopLineDist := inter.GetStopLineDistForHeading(-math.Pi/2, lw)

	// AI Car approaching northbound (South of intersection, moving North)
	// Position car so bumper is 25px before stop line, in lane 1 (straight lane)
	carPos := sim.Vec2{
		X: 78.0,
		Y: stopLineDist + 25.0 + 23.0,
	}
	car := &sim.TrafficCar{
		ID:          99,
		Pos:         carPos,
		Heading:     -math.Pi / 2, // Northbound
		Speed:       80.0,
		TargetSpeed: 120.0,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        false,
		LaneIndex:   1, // Straight lane
	}
	tm.Cars = []*sim.TrafficCar{car}

	// 1. When intersection is clear, car does not brake on green
	tm.StepCar(0.1, car, 0, nil, tls, 1, lw, nil)
	if car.IsBraking {
		t.Fatalf("Car should not brake when intersection box is clear on green light!")
	}

	// 2. Place an AI bus inside the intersection box (crossing East-West)
	aiBus := sim.NewBus(0, 0, 0) // Centered in intersection box, heading East
	aiBus.Speed = 10.0           // Traversing slowly or turning

	// Update car: Yellow Box clearance must activate!
	tm.StepCar(0.1, car, 0, aiBus, tls, 1, lw, nil)

	if !car.IsBraking {
		t.Fatalf("Car must brake and yield before stop line when bus is occupying intersection box (Yellow Box rule)!")
	}

	// Car must decelerate towards stop line
	if car.Speed > 70.0 {
		t.Fatalf("Car speed should be decelerated towards stop line, got %.1f", car.Speed)
	}
}

// TestTrafficCarDrivesPastBusChangingLanesInAdjacentLane verifies that a civilian traffic car
// does NOT brake or freeze when a bus in an adjacent lane is angled / changing lanes or parked inside a bay.
// The car must continue driving forward unimpeded down its clear lane.
func TestTrafficCarDrivesPastBusChangingLanesInAdjacentLane(t *testing.T) {
	laneW := 52.0
	tm := traffic.NewTrafficManager(laneW)

	// Eastbound road at Y = 0 (2 lanes each direction)
	// Lane 0 (inner/passing): Y = +26
	// Lane 1 (outer/through): Y = +78
	car := &sim.TrafficCar{
		ID:          1,
		Pos:         sim.Vec2{X: 100.0, Y: 78.0}, // In outer lane 1
		Heading:     0.0,                         // Eastbound
		Speed:       70.0,
		TargetSpeed: 85.0,
		Type:        sim.CarSedan,
		Length:      48.0,
		Width:       22.0,
		RoadCenter:  0.0,
		IsEW:        true,
		LaneIndex:   1,
	}
	tm.Cars = []*sim.TrafficCar{car}

	// World with a bus stop bay at Y = 104+21 = 125
	stop := sim.NewBusStop(0, "Bay Stop", sim.Vec2{X: 0, Y: 0}, 0.0, 4, sim.RGBA(255, 255, 255, 255), laneW)
	w := &sim.World{
		Stops: []*sim.BusStop{stop},
	}

	// 1. Stopped bus inside the bus stop bay (off-road)
	bayBus := sim.NewBus(stop.Pos.X, stop.Pos.Y, 0.0)
	bayBus.Speed = 0.0

	// 2. Player bus changing lanes in adjacent lane 0 (at Y = 26.0, angled at -20 degrees, ahead and to the left of car)
	playerBus := sim.NewBus(140.0, 26.0, -0.35)
	playerBus.Speed = 0.0 // Even if momentarily stopped or slow during lane change
	tm.AIBusMgr = &traffic.AIBusManager{
		Buses: []*traffic.AIBus{{ID: 1, Bus: bayBus}},
	}

	// Update traffic car
	tm.StepCar(0.016, car, 0, playerBus, nil, 1, laneW, w)

	// Car must NOT brake for bus in adjacent lane or parked in bay
	if car.IsBraking {
		t.Fatalf("Traffic car should NOT brake for a bus in adjacent lane or inside bay! Pos=%v", car.Pos)
	}
	if car.TargetSpeed < 50.0 {
		t.Fatalf("Traffic car target speed should remain high to cruise forward, got %.2f", car.TargetSpeed)
	}

	// Deadlock resolution check: moving car or open road car must not trigger deadlock timer
	tm.ResolveDeadlocks(0.1, playerBus, 1, laneW, w)
	if car.DeadlockTimer > 0 {
		t.Fatalf("Traffic car should have 0 deadlock timer, got %.2f", car.DeadlockTimer)
	}
}
