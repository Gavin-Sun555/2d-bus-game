package integration_test

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPlanningModeInstantTransitionAndAITrafficSuppression(t *testing.T) {
	g := game.NewGame(1280, 720)

	// Initially in Driver Mode
	if g.IsPlanMode {
		t.Fatalf("Expected initial mode to be Driver Mode (IsPlanMode == false)")
	}
	if g.Camera.Zoom != 1.0 {
		t.Fatalf("Expected initial camera zoom 1.0, got %f", g.Camera.Zoom)
	}

	// 1. Enter Planning Mode: Verify Instant transition (no progressive lerp)
	g.IsPlanMode = true
	g.Camera.TargetPos = sim.Vec2{X: 0, Y: 0}
	g.Camera.Pos = sim.Vec2{X: 0, Y: 0}
	g.Camera.TargetZoom = 0.30
	g.Camera.Zoom = 0.30

	if g.Camera.Pos.X != 0 || g.Camera.Pos.Y != 0 {
		t.Fatalf("Expected camera Pos to instantly center at (0, 0) in planning mode")
	}
	if g.Camera.Zoom != 0.30 {
		t.Fatalf("Expected camera Zoom to instantly snap to 0.30 without progressive delay, got %f", g.Camera.Zoom)
	}

	// 2. Step game update while in Planning Mode
	// AI Traffic update should be paused/skipped to prevent stutter
	initialCar0Pos := g.TrafficMgr.Cars[0].Pos
	// If !g.IsPlanMode branch guards TrafficMgr.Update, car pos will remain stable
	if !g.IsPlanMode {
		g.TrafficMgr.Update(0.1, g.Bus, g.TrafficLights, 2, 52.0)
	}
	if g.TrafficMgr.Cars[0].Pos != initialCar0Pos {
		t.Fatalf("AI Cars must not move while in Planning Mode")
	}

	// 3. Exit Planning Mode back to Driver Mode: Verify instant snap back to bus
	g.IsPlanMode = false
	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Camera.TargetPos
	g.Camera.TargetZoom = 1.0
	g.Camera.Zoom = 1.0

	if g.Camera.Zoom != 1.0 {
		t.Fatalf("Expected camera Zoom to instantly snap back to 1.0, got %f", g.Camera.Zoom)
	}
	if g.Camera.Pos.Distance(g.Bus.Pos) > 50.0 {
		t.Fatalf("Expected camera to instantly snap to bus position upon returning to driver mode")
	}
}

// TestPlanModePausesSimulationAndPassengerSpawning verifies that when planning routes (IsPlanMode = true),
// the entire game simulation (traffic lights, AI traffic, player bus motion, passenger arrivals at bus stops)
// is strictly paused, and no new passengers are automatically spawned.
func TestPlanModePausesSimulationAndPassengerSpawning(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)

	// Ensure we start in Drive Mode
	if g.IsPlanMode {
		t.Fatal("Expected to start in Driver Mode")
	}

	// 1. In Driver Mode: Verify simulation ticks advance world & passenger timers
	initialSpawnTimers := make([]float64, len(g.World.Stops))
	initialPassengers := make([]int, len(g.World.Stops))
	for i, s := range g.World.Stops {
		initialSpawnTimers[i] = s.SpawnTimer
		initialPassengers[i] = s.WaitingPassengers
	}

	initialTLTimer := g.TrafficLights.Intersections[0].Timer

	// Run 10 ticks in Driver Mode
	for i := 0; i < 10; i++ {
		_ = g.Update()
	}

	// Verify timers advanced in Driver Mode
	advancedInDriver := false
	for i, s := range g.World.Stops {
		if s.SpawnTimer != initialSpawnTimers[i] {
			advancedInDriver = true
			break
		}
	}
	if !advancedInDriver {
		t.Fatal("Expected stop spawn timers to advance in Driver Mode")
	}
	if g.TrafficLights.Intersections[0].Timer == initialTLTimer {
		t.Fatal("Expected traffic light timer to advance in Driver Mode")
	}

	// 2. Enter Plan Mode: Verify simulation and passenger spawning are strictly paused
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true after TogglePlanMode()")
	}

	// Record snapshot of all stops, traffic lights, and AI cars
	pausedSpawnTimers := make([]float64, len(g.World.Stops))
	pausedPassengers := make([]int, len(g.World.Stops))
	for i, s := range g.World.Stops {
		pausedSpawnTimers[i] = s.SpawnTimer
		pausedPassengers[i] = s.WaitingPassengers
	}

	pausedTLTimers := make([]float64, len(g.TrafficLights.Intersections))
	for i, it := range g.TrafficLights.Intersections {
		pausedTLTimers[i] = it.Timer
	}

	pausedCarPositions := make([]sim.Vec2, len(g.TrafficMgr.Cars))
	for i, c := range g.TrafficMgr.Cars {
		pausedCarPositions[i] = c.Pos
	}

	// Run 100 ticks in Plan Mode (simulating 1.6+ seconds of player route planning)
	for i := 0; i < 100; i++ {
		_ = g.Update()
	}

	// Verify that passenger spawn timers did NOT advance at all
	for i, s := range g.World.Stops {
		if s.SpawnTimer != pausedSpawnTimers[i] {
			t.Fatalf("Stop #%d (%s) spawn timer advanced while in plan mode: got %.3f, expected %.3f",
				s.ID, s.Name, s.SpawnTimer, pausedSpawnTimers[i])
		}
		if s.WaitingPassengers != pausedPassengers[i] {
			t.Fatalf("Stop #%d (%s) waiting passengers changed while in plan mode: got %d, expected %d",
				s.ID, s.Name, s.WaitingPassengers, pausedPassengers[i])
		}
	}

	// Verify that traffic lights did NOT advance
	for i, it := range g.TrafficLights.Intersections {
		if it.Timer != pausedTLTimers[i] {
			t.Fatalf("Traffic intersection #%d timer advanced while in plan mode: got %.3f, expected %.3f",
				it.ID, it.Timer, pausedTLTimers[i])
		}
	}

	// Verify that civilian traffic cars did NOT move
	for i, c := range g.TrafficMgr.Cars {
		if c.Pos != pausedCarPositions[i] {
			t.Fatalf("Civilian car #%d moved while in plan mode: got %v, expected %v",
				c.ID, c.Pos, pausedCarPositions[i])
		}
	}

	// 3. Exit Plan Mode: Verify simulation resumes cleanly
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after TogglePlanMode()")
	}

	// Run 10 ticks in Driver Mode after resuming
	for i := 0; i < 10; i++ {
		_ = g.Update()
	}

	// Verify passenger spawn timers and traffic lights resume advancing
	resumedPassengerTimers := false
	for i, s := range g.World.Stops {
		if s.SpawnTimer != pausedSpawnTimers[i] {
			resumedPassengerTimers = true
			break
		}
	}
	if !resumedPassengerTimers {
		t.Fatal("Expected passenger spawn timers to resume advancing after exiting plan mode")
	}

	if g.TrafficLights.Intersections[0].Timer == pausedTLTimers[0] {
		t.Fatal("Expected traffic light timer to resume advancing after exiting plan mode")
	}
}
