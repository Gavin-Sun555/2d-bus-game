package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestRoundaboutAICarsNoDeadlock tests that an AI car circulating inside a roundabout
// does not brake or deadlock with another AI car waiting at an entrance branch yield line.
func TestRoundaboutAICarsNoDeadlock(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	tm := traffic.NewTrafficManager(52.0)

	// Car 0: Yellow car circulating inside roundabout on outer lane near bottom (approaching East)
	car0 := tm.Cars[0]
	car0.InRoundabout = true
	car0.RoundaboutCenter = rb.Center
	car0.RoundaboutTargetR = 214.0
	car0.RoundaboutRadius = 214.0
	car0.RoundaboutOuterRadius = rb.Radius
	car0.RoundaboutAngle = math.Pi / 4 // 45 degrees
	car0.Pos = rb.Center.Add(sim.Vec2{
		X: math.Cos(car0.RoundaboutAngle) * car0.RoundaboutRadius,
		Y: math.Sin(car0.RoundaboutAngle) * car0.RoundaboutRadius,
	})
	car0.Heading = sim.NormalizeAngle(car0.RoundaboutAngle - math.Pi/2)
	car0.Speed = 60.0
	car0.TargetSpeed = 80.0
	car0.LaneIndex = 1

	// Car 1: Blue car approaching/waiting at the South entrance branch yield line
	car1 := tm.Cars[1]
	car1.InRoundabout = false
	car1.Pos = sim.Vec2{X: rb.Center.X + 78.0, Y: rb.Center.Y + rb.Radius + 30.0}
	car1.Heading = -math.Pi / 2 // Heading North into roundabout
	car1.Speed = 0.0
	car1.TargetSpeed = 60.0
	car1.LaneIndex = 1

	bus := sim.NewBus(1000, 1000, 0) // Distant dummy bus

	dt := 0.016
	stuckCount := 0

	for step := 0; step < 100; step++ {
		tm.StepCar(dt, car0, 0, bus, nil, 2, 52.0, w)
		tm.StepCar(dt, car1, 1, bus, nil, 2, 52.0, w)

		if car0.Speed < 1.0 {
			stuckCount++
		}
	}

	if stuckCount > 5 {
		t.Fatalf("Car 0 inside roundabout got stuck/deadlocked with car waiting at entrance! stuckCount=%d, speed=%.2f, pos=(%.1f, %.1f)",
			stuckCount, car0.Speed, car0.Pos.X, car0.Pos.Y)
	}

	// Car 0 must have advanced along the circular arc (angle decreased)
	if car0.RoundaboutAngle >= math.Pi/4 {
		t.Fatalf("Car 0 failed to circulate: RoundaboutAngle=%.2f >= %.2f", car0.RoundaboutAngle, math.Pi/4)
	}
}

func TestFleetDeadlockTimerIgnoresOncomingPlayer(t *testing.T) {
	lw := 26.0
	w := sim.NewWorld(lw, sim.MapThemeTwinCities)
	route := sim.GetStandardRouteForMap(sim.MapThemeTwinCities, w)
	turningBus := sim.NewBus(3880, -10, 2.7)
	driver := autopilot.NewAutopilotDriver()
	// Also verify AIBusManager deadlock timer is not suppressed by this oncoming vehicle
	aiBus := &traffic.AIBus{
		ID:     8,
		Bus:    turningBus,
		Route:  route,
		Driver: driver,
	}
	// An opposing AI bus that is jammed against aiBus
	otherBus := sim.NewBus(3860, 10, 0.5)
	otherBus.Speed = 0.0
	otherAIBus := &traffic.AIBus{
		ID:     5,
		Bus:    otherBus,
		Route:  route,
		Driver: autopilot.NewAutopilotDriver(),
	}
	playerBus := sim.NewBus(3845, 26, 0.0)
	playerBus.Speed = 0.0

	abm := traffic.NewAIBusManager()
	abm.Buses = []*traffic.AIBus{aiBus, otherAIBus}

	// When turning bus is stationary and jammed against otherAIBus, deadlock timer should advance
	// because playerBus is oncoming traffic and not a same-direction leader blocking the bus
	turningBus.Speed = 0.0
	abm.Update(0.1, playerBus, w, nil, nil, lw)
	if aiBus.DeadlockTimer < 0.05 {
		t.Fatalf("DeadlockTimer failed to increment (got %.2f), playerBlocking falsely suppressed deadlock resolution",
			aiBus.DeadlockTimer)
	}
}

func TestNonPlayerVehiclesDeadlockTimeoutDespawn(t *testing.T) {
	// 1. Two AI civilian cars jammed head-to-head
	tm := &traffic.TrafficManager{
		Cars: make([]*sim.TrafficCar, 0),
	}
	car1 := &sim.TrafficCar{
		ID:          1,
		Pos:         sim.Vec2{X: 100, Y: 0},
		Heading:     0,
		Speed:       0,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}
	car2 := &sim.TrafficCar{
		ID:          2,
		Pos:         sim.Vec2{X: 150, Y: 0},
		Heading:     math.Pi,
		Speed:       0,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}
	tm.Cars = append(tm.Cars, car1, car2)

	playerBus := sim.NewBus(5000, 5000, 0) // Far away player bus

	// Simulate 10 seconds of motionless deadlock (less than 38s traffic light cycle threshold)
	dt := 0.5
	for elapsed := 0.0; elapsed < 10.0; elapsed += dt {
		tm.ResolveDeadlocks(dt, playerBus, len(tm.Cars), 52.0, nil)
	}

	if len(tm.Cars) != 2 {
		t.Fatalf("Expected 2 cars still present after only 10s deadlock, got %d", len(tm.Cars))
	}
	if car1.DeadlockTimer < 9.5 || car1.DeadlockTimer > 10.5 {
		t.Fatalf("Expected car1 deadlock timer ~10s, got %f", car1.DeadlockTimer)
	}

	// Moving resets deadlock timer
	car1.Speed = 50.0
	tm.ResolveDeadlocks(dt, playerBus, len(tm.Cars), 52.0, nil)
	if car1.DeadlockTimer != 0 {
		t.Fatalf("Expected moving car to reset deadlock timer to 0, got %f", car1.DeadlockTimer)
	}
	car1.Speed = 0

	// Now simulate until deadlock timer exceeds DeadlockDurationThreshold (38.0s)
	// (Simulate 39s more)
	for elapsed := 0.0; elapsed < 39.0; elapsed += dt {
		tm.ResolveDeadlocks(dt, playerBus, len(tm.Cars), 52.0, nil)
		if len(tm.Cars) < 2 {
			break
		}
	}

	if len(tm.Cars) != 1 {
		t.Fatalf("Expected one car to directly disappear after > 38s deadlock, remaining: %d", len(tm.Cars))
	}

	// 2. Verify AI car stopped behind player bus NEVER despawns
	soloCar := tm.Cars[0]
	soloCar.Pos = sim.Vec2{X: 100, Y: 0}
	soloCar.Heading = 0
	soloCar.Speed = 0
	soloCar.DeadlockTimer = 0

	// Place player bus right in front of soloCar
	playerBus.Pos = sim.Vec2{X: 150, Y: 0}
	playerBus.Heading = 0
	playerBus.Speed = 0

	for elapsed := 0.0; elapsed < 45.0; elapsed += dt {
		tm.ResolveDeadlocks(dt, playerBus, len(tm.Cars), 52.0, nil)
	}

	if len(tm.Cars) != 1 {
		t.Fatalf("Car should not disappear when stopped because of the player bus!")
	}
	if soloCar.DeadlockTimer != 0 {
		t.Fatalf("Deadlock timer should remain 0 when blocked by player bus, got %f", soloCar.DeadlockTimer)
	}

	// 3. Two AI buses jammed head-to-head
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	abm := traffic.NewAIBusManager()
	abm.SetLineCount(2, w, 52.0)

	if len(abm.Buses) >= 2 {
		bus1 := abm.Buses[0]
		bus2 := abm.Buses[1]
		bus1.Bus.Pos = sim.Vec2{X: 200, Y: 0}
		bus1.Bus.Heading = 0
		bus1.Bus.Speed = 0
		bus2.Bus.Pos = sim.Vec2{X: 270, Y: 0}
		bus2.Bus.Heading = math.Pi
		bus2.Bus.Speed = 0

		oldPos1 := bus1.Bus.Pos

		// Simulate 40s (> DeadlockDurationThreshold)
		for elapsed := 0.0; elapsed < 40.0; elapsed += dt {
			abm.Update(dt, playerBus, w, nil, tm, 52.0, 2)
		}

		// At least one AI bus must have repositioned to its next route stop to clear the jam
		if bus1.Bus.Pos == oldPos1 && bus2.Bus.Pos.Distance(oldPos1) < 100.0 {
			t.Fatalf("Expected AI buses to resolve deadlock after > 38s by advancing stop")
		}
	}
}

func TestIntersectionDeadlockResolutionCarDespawnsAndAIBusResetsToStart(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)
	abm := traffic.NewAIBusManager()
	abm.ActiveRoute = route
	abm.SetLineCount(1, w, lw)

	if len(abm.Buses) == 0 {
		t.Fatalf("Expected at least 1 AI bus in manager")
	}
	aiBus := abm.Buses[0]
	startStopID := route.StopIDs[0]
	startStop := w.Stops[startStopID]

	// Advance AI bus to stop index 1 and add passengers
	aiBus.Route.CurrentIndex = 1
	aiBus.Bus.PassengerCount = 15
	aiBus.Bus.Passengers = make([]sim.Passenger, 15)

	// Position AI bus on Central Ave Eastbound lane heading East
	aiBus.Bus.Pos = sim.Vec2{X: -400.0, Y: 26.0}
	aiBus.Bus.Heading = 0.0
	aiBus.Bus.Speed = 0.0

	// Position an AI car directly on the road ahead of the bus, facing head-to-head (Westbound)
	tm := traffic.NewTrafficManager(lw)
	carPos := sim.Vec2{X: -340.0, Y: 26.0}
	car := &sim.TrafficCar{
		ID:          42,
		Pos:         carPos,
		Heading:     math.Pi, // Facing West towards Eastbound bus
		Speed:       0.0,
		TargetSpeed: 100.0,
		Length:      46,
		Width:       22,
		RoadCenter:  52.0,
		IsEW:        true,
	}
	tm.Cars = []*sim.TrafficCar{car}
	tm.AIBusMgr = abm

	// Verify initial state
	if aiBus.Route.CurrentIndex == 0 {
		t.Fatalf("Initial AI bus route index should not be 0 for this test")
	}

	// Simulate motionless deadlock for > 38.0s (DeadlockDurationThreshold)
	dt := 0.2
	for elapsed := 0.0; elapsed < 42.0; elapsed += dt {
		aiBus.Bus.Speed = 0.0
		car.Speed = 0.0
		tm.ResolveDeadlocks(dt, nil, len(tm.Cars), lw, w)
		if car.DeadlockTimer == 0 && len(tm.Cars) > 0 && tm.Cars[0].Pos.Distance(carPos) > 10.0 {
			break
		}
	}

	// 1. AI Car must have disappeared/reset from the stuck position
	if len(tm.Cars) > 0 {
		if tm.Cars[0].Pos.Distance(carPos) < 10.0 {
			t.Fatalf("Deadlocked AI car failed to disappear/reset from the deadlock spot!")
		}
		if tm.Cars[0].DeadlockTimer != 0 {
			t.Fatalf("Reset car must have DeadlockTimer=0, got %.2f", tm.Cars[0].DeadlockTimer)
		}
	}

	// 2. AI Bus must have reset back to the route's starting station (StopIDs[0])
	if aiBus.Route.CurrentIndex != 0 {
		t.Fatalf("AI Bus should be reset to starting station index 0, got %d", aiBus.Route.CurrentIndex)
	}
	if aiBus.Bus.Pos.Distance(startStop.Pos) > 1.0 {
		t.Fatalf("AI Bus position should be reset to starting station Pos %v, got %v", startStop.Pos, aiBus.Bus.Pos)
	}
	if aiBus.Bus.PassengerCount != 0 {
		t.Fatalf("AI Bus passengers should be cleared upon reset to starting station, got %d", aiBus.Bus.PassengerCount)
	}
	if aiBus.DeadlockTimer != 0 {
		t.Fatalf("AI Bus DeadlockTimer should be reset to 0, got %.2f", aiBus.DeadlockTimer)
	}
}

func TestAutopilotDeadlockPlayerBusResetsToStartingStation(t *testing.T) {
	g := game.NewGame(1920, 1080)
	g.State = game.StatePlaying
	g.Settings.AutopilotEnabled = true

	startStopID := g.Route.StopIDs[0]
	startStop := g.World.Stops[startStopID]

	// Advance route index and board passengers
	g.Route.CurrentIndex = 1
	g.Bus.PassengerCount = 20
	g.Bus.Passengers = make([]sim.Passenger, 20)

	// Place player bus along Central Ave Eastbound lane (Y=26) heading East
	g.Bus.Pos = sim.Vec2{X: -400.0, Y: 26.0}
	g.Bus.Heading = 0.0
	g.Bus.Speed = 0.0

	// Place an AI car directly ahead facing Westbound in same lane
	carPos := sim.Vec2{X: -340.0, Y: 26.0}
	car := &sim.TrafficCar{
		ID:          88,
		Pos:         carPos,
		Heading:     math.Pi, // Facing West towards Eastbound bus
		Speed:       0.0,
		TargetSpeed: 0.0,
		Length:      46,
		Width:       22,
		RoadCenter:  52.0,
		IsEW:        true,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{car}

	// Simulate motionless deadlock for > 38.0s (at 60 FPS, 40s = 2400 frames)
	totalFrames := int(40.0 * 60)
	for i := 0; i < totalFrames; i++ {
		g.Bus.Speed = 0.0
		car.Speed = 0.0
		_ = g.Update()
		if g.Route.CurrentIndex == 0 {
			break
		}
	}

	// 1. AI Car must have disappeared/reset from stuck spot
	if len(g.TrafficMgr.Cars) > 0 {
		if g.TrafficMgr.Cars[0].Pos.Distance(carPos) < 10.0 {
			t.Fatalf("Deadlocked AI car failed to disappear/reset from the deadlock spot!")
		}
	}

	// 2. Player bus in AI autopilot mode must have reset back to starting station
	if g.Route.CurrentIndex != 0 {
		t.Fatalf("Player AI Bus should be reset to starting station index 0, got %d", g.Route.CurrentIndex)
	}
	if g.Bus.Pos.Distance(startStop.Pos) > 1.0 {
		t.Fatalf("Player AI Bus position should be reset to starting station Pos %v, got %v", startStop.Pos, g.Bus.Pos)
	}
	if g.Bus.PassengerCount > g.Bus.Capacity {
		t.Fatalf("Player AI Bus passengers exceeded capacity upon reset: %d", g.Bus.PassengerCount)
	}
	if g.AutopilotDeadlockTimer != 0 {
		t.Fatalf("Player AI Bus AutopilotDeadlockTimer should be reset to 0, got %.2f", g.AutopilotDeadlockTimer)
	}
}

func TestIntersectionTurnDeadlockFastResolution(t *testing.T) {
	g := game.NewGame(1920, 1080)
	g.State = game.StatePlaying
	g.Settings.AutopilotEnabled = true

	// Find an intersection
	if len(g.World.Intersections) == 0 {
		t.Skip("No intersections in world")
	}
	inter := g.World.Intersections[0]

	// Place player bus in intersection turning right (heading ~ 24 deg from North)
	busHeading := -math.Pi/2 + 24.0*math.Pi/180.0
	g.Bus.Pos = inter.Add(sim.Vec2{X: 15.0, Y: 25.0})
	g.Bus.Heading = busHeading
	g.Bus.Speed = 0.0

	// Place an AI car stopped right at the corner/mouth facing North
	carPos := inter.Add(sim.Vec2{X: 25.0, Y: 10.0})
	car := &sim.TrafficCar{
		ID:          99,
		Pos:         carPos,
		Heading:     -math.Pi / 2, // Facing North
		Speed:       0.0,
		TargetSpeed: 0.0,
		Length:      46,
		Width:       22,
		RoadCenter:  inter.X,
		IsEW:        false,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{car}

	// Simulate for 5 seconds (which exceeds IntersectionDeadlockThreshold = 4.0s)
	totalFrames := int(5.0 * 60)
	for i := 0; i < totalFrames; i++ {
		g.Bus.Speed = 0.0
		car.Speed = 0.0
		_ = g.Update()
		if len(g.TrafficMgr.Cars) > 0 && g.TrafficMgr.Cars[0].Pos.Distance(carPos) > 10.0 {
			break
		}
	}

	// AI car must have been despawned and safely reset away from the intersection corner!
	if len(g.TrafficMgr.Cars) > 0 {
		if g.TrafficMgr.Cars[0].Pos.Distance(carPos) < 10.0 {
			t.Fatalf("Deadlocked AI car at turning corner failed to despawn within 4.0s! Car is still at %v", g.TrafficMgr.Cars[0].Pos)
		}
	}
}

// TestIntersectionAIBusJamFastResolution verifies that wedged AI traffic jams inside intersections
// are fast-resolved within 4.0s so turning buses never stay stuck.
func TestIntersectionAIBusJamFastResolution(t *testing.T) {
	g := game.NewGame(1920, 1080)
	g.State = game.StatePlaying
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true

	// Position at the first intersection
	inter := g.World.Intersections[0]

	// Position player bus inside the intersection
	g.Bus.Pos = inter.Sub(sim.Vec2{X: 30.0, Y: 0.0})
	g.Bus.Heading = 0.0
	g.Bus.Speed = 0.0

	// Position an AI bus wedged directly in front inside the intersection
	aiBusPos := inter.Add(sim.Vec2{X: 30.0, Y: 0.0})
	aiDriver := autopilot.NewAutopilotDriver()
	aiRoute := sim.GetStandardRouteForMap(g.Settings.MapTheme, g.World)
	aiRoute.CurrentIndex = 1
	aiBus := &traffic.AIBus{
		ID:     1,
		Bus:    sim.NewBus(aiBusPos.X, aiBusPos.Y, math.Pi), // Facing each other
		Route:  aiRoute,
		Driver: aiDriver,
	}
	aiBus.Bus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{aiBus}

	g.Route.CurrentIndex = 1
	g.Bus.PassengerCount = 10

	// Simulate for 5.0 seconds (> IntersectionDeadlockThreshold = 4.0s)
	totalFrames := int(5.0 * 60)
	for i := 0; i < totalFrames; i++ {
		g.Bus.Speed = 0.0
		aiBus.Bus.Speed = 0.0
		_ = g.Update()
		if aiBus.Bus.Pos.Distance(aiBusPos) > 30.0 {
			break
		}
	}

	// 1. Inside an intersection, fast deadlock resolution clears the jam after 4.0s
	if aiBus.Bus.Pos.Distance(aiBusPos) < 20.0 {
		t.Fatalf("Deadlocked AI bus inside intersection failed to reset within 4.0s! AI Bus is still at %v", aiBus.Bus.Pos)
	}

	// 2. The player bus must remain on active leg 1
	if g.Route.CurrentIndex != 1 {
		t.Fatalf("Player bus was wrongfully reset to route index %d", g.Route.CurrentIndex)
	}
}
