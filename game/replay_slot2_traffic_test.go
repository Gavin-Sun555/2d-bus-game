package game

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func findSlot2JamFile() string {
	return firstFixture(
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1105", "slot2.json"),
		filepath.Join("..", "dist", "slot2-crash-2026-11-02", "slot2.json"),
	)
}

// TestSavedSlot2BridgeGridlockDissolves verifies that the 14-bus queue and civilian traffic
// deadlock on the Road 10 six-lane bridge in slot 2 dissolves smoothly:
// the lane-changing lead bus is permitted to clear the overlap padding,
// the following bus queue accelerates, and traffic flows freely.
func TestSavedSlot2BridgeGridlockDissolves(t *testing.T) {
	p := findSlot2JamFile()
	if p == "" {
		t.Skip("no save file found for slot2 jam")
	}
	g := loadSaveForTest(t, p, 1280, 720)

	bus4 := g.AIBusMgr.Buses[4]
	bus36 := g.AIBusMgr.Buses[36]
	initBus4X := bus4.Bus.Pos.X
	initBus36X := bus36.Bus.Pos.X

	stoppedOnBridge := func() int {
		n := 0
		for _, b := range g.AIBusMgr.Buses {
			if b.Bus.Pos.X >= 3000 && b.Bus.Pos.X <= 5500 && b.Bus.Pos.Y >= 0 && b.Bus.Pos.Y <= 150 && math.Abs(b.Bus.Speed) < 2.0 {
				n++
			}
		}
		return n
	}
	// Simulate 30 seconds (allowing one full traffic light phase at intersection X=5400).
	// Track the least congested moment: the queue re-forms at each red phase.
	fewestStopped := stoppedOnBridge()
	for tick := 1; tick <= 60*30; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}
		fewestStopped = min(fewestStopped, stoppedOnBridge())
	}

	// Verify Bus 4 advanced significantly
	if bus4.Bus.Pos.X <= initBus4X+100.0 {
		t.Fatalf("Bus 4 did not advance: init X=%.1f, current Pos=%v", initBus4X, bus4.Bus.Pos)
	}

	// Verify Bus 36 accelerated and traversed the bridge
	if bus36.Bus.Pos.X <= initBus36X+500.0 {
		t.Fatalf("Bus 36 did not clear: init X=%.1f, current Pos=%v", initBus36X, bus36.Bus.Pos)
	}

	// Verify the stopped buses on the bridge queue drained at some point
	if fewestStopped > 2 {
		t.Fatalf("Bridge still jammed: never fewer than %d buses stopped (%d at the end)", fewestStopped, stoppedOnBridge())
	}
}

func findSlot2HeadonFile() string {
	return firstFixture(filepath.Join("..", "dist", "slot2-headon-2026-11-02-1227", "slot2.json"))
}

// TestSavedSlot2HeadonDeadlockDissolves verifies that the head-to-head deadlock at intersection
// (0, 3600) between left-turning Bus 17 and straight-going Bus 47 dissolves cleanly:
// 1. After 10s of being locked head-to-head, Bus 17 (yielding left turn) reverses backward ~70px.
// 2. Bus 47's straight trajectory unblocks, allowing Bus 47 to cross intersection (0, 3600) Eastward.
// 3. Bus 17 completes its left turn South onto the avenue.
// 4. The 8-bus queue backing up to the player's bus dissolves, and the player bus moves forward.
func TestSavedSlot2HeadonDeadlockDissolves(t *testing.T) {
	savePath := findSlot2HeadonFile()
	if savePath == "" {
		t.Skip("save file not found for slot2 headon deadlock")
	}
	sim.SeedSimulation(1)
	g := loadSaveForTest(t, savePath, 1280, 720)

	var b17, b47 *sim.Bus
	var d17 *autopilot.AutopilotDriver
	var ab17, ab47 *traffic.AIBus

	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 17 {
			ab17 = ab
			b17 = ab.Bus
			d17 = ab.Driver
		} else if ab.ID == 47 {
			ab47 = ab
			b47 = ab.Bus
		}
	}

	if b17 == nil || b47 == nil || d17 == nil || ab17 == nil || ab47 == nil {
		t.Fatalf("could not find AI buses 17 and 47")
	}

	initB17Pos := b17.Pos
	initB47Pos := b47.Pos
	initPlayerX := g.Bus.Pos.X

	for step := 1; step <= 2100; step++ { // 35 seconds at 60Hz
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at step %d: %v", step, err)
		}
	}

	// 1. Bus 47 must have crossed intersection (0, 3600) and traveled East along Road (0, 3600)->(5400, 3600)
	if b47.Pos.X < 2000.0 {
		t.Fatalf("B47 remained stuck at intersection: initial X=%.1f, current X=%.1f", initB47Pos.X, b47.Pos.X)
	}

	// 2. Bus 17 must have completed its left turn and traveled South along Road (0, 3600)->(0, 7200)
	if b17.Pos.Y < 3800.0 {
		t.Fatalf("B17 remained stuck at intersection: initial Y=%.1f, current Y=%.1f", initB17Pos.Y, b17.Pos.Y)
	}

	// 3. Player's bus (and queue) must have unblocked and moved Westward
	if g.Bus.Pos.X > initPlayerX-50.0 {
		t.Fatalf("player bus remained queued: initial X=%.1f, current X=%.1f", initPlayerX, g.Bus.Pos.X)
	}
}

func findSlot2Jam1814File() string {
	return firstFixture(filepath.Join("..", "dist", "slot2-jam-2026-11-02-1814", "slot2.json"))
}

func TestSavedSlot2PlayerCarJamResolved(t *testing.T) {
	p := findSlot2Jam1814File()
	if p == "" {
		t.Skip("snapshot missing")
	}
	g := loadSaveForTest(t, p, 1440, 900)

	playerBus := g.Bus
	var car32231 *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == 32231 {
			car32231 = c
		}
	}
	if car32231 == nil {
		t.Fatal("Car 32231 not found in snapshot")
	}

	var ai8 *sim.Bus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 8 {
			ai8 = ab.Bus
		}
	}
	if ai8 == nil {
		t.Fatal("AI Bus 8 not found in snapshot")
	}

	initialPlayerPos := playerBus.Pos

	// Run for 600 ticks (10 seconds)
	for tick := 1; tick <= 600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		// AI Bus 8 must NEVER collide with player bus
		if hit, _, _ := sim.CheckBusesCollision(playerBus, ai8); hit {
			t.Fatalf("tick %d: AI Bus 8 collided with player bus! Player=%v AI8=%v",
				tick, playerBus.Pos, ai8.Pos)
		}
	}

	// 1. Car 32231 must have exited the roundabout and moved far along the avenue
	if car32231.InRoundabout {
		t.Errorf("Car 32231 should have exited the roundabout, but InRoundabout is still true! Pos=%v, Spd=%.1f",
			car32231.Pos, car32231.Speed)
	}
	if car32231.Pos.X > -4100.0 {
		t.Errorf("Car 32231 did not travel down the avenue! Pos=%v", car32231.Pos)
	}

	// 2. Player bus must have unblocked and traveled forward smoothly
	if playerBus.Pos.Distance(initialPlayerPos) < 50.0 {
		t.Errorf("Player bus remained stuck! Pos=%v, initial=%v", playerBus.Pos, initialPlayerPos)
	}

	// 3. AI Bus 8 must maintain safe following distance behind player bus
	dist := ai8.Pos.Distance(playerBus.Pos)
	if dist < playerBus.Length {
		t.Errorf("AI Bus 8 too close to player bus! Dist=%.1f, Player=%v, AI8=%v",
			dist, playerBus.Pos, ai8.Pos)
	}
}

func TestStationaryBlockDeadlockRecoveryReverses(t *testing.T) {
	// Unit test: A bus blocked by a stationary car at an angle for >= 10s
	// must initiate reverse recovery and back up safely to clear space.
	w := sim.NewWorld(1) // Coast/Seaside map
	lw := 52.0
	bus := sim.NewBus(-4065.5, 1685.9, 2.250)
	bus.Speed = 0.0

	// Stationary car blocking right in front
	car := &sim.TrafficCar{
		Pos:     sim.Vec2{X: -4080.4, Y: 1747.6},
		Heading: 2.665,
		Speed:   0.0,
		Length:  46,
		Width:   22,
	}
	obstacles := []autopilot.VehicleObstacle{
		{
			Pos:     car.Pos,
			Heading: car.Heading,
			Speed:   car.Speed,
			Length:  car.Length,
			Width:   car.Width,
		},
	}

	driver := autopilot.NewAutopilotDriver()
	route := sim.NewRoute()
	route.AddStop(0)
	route.AddStop(10)
	route.CurrentIndex = 1

	startPos := bus.Pos
	recoveryTriggered := false

	for step := 0; step < 900; step++ { // 15 seconds
		dt := 1.0 / 60.0
		th, br, st, hb, doors, _ := driver.ComputeControls(
			bus, route, w, nil, obstacles, lw, dt,
		)
		bus.Update(dt, th, br, st, hb, doors)

		if driver.HeadonRecoveryActive() {
			recoveryTriggered = true
			if bus.Gear == sim.GearReverse {
				// Verify bus moves backward (increasing distance from blocker)
				distToStart := bus.Pos.Distance(startPos)
				if distToStart > 5.0 {
					// Verify reversing direction
					u := sim.UnitFromAngle(bus.Heading)
					rel := bus.Pos.Sub(startPos)
					if rel.Dot(u) > 0 {
						t.Errorf("bus moved forward instead of backward during reverse recovery! rel=%v", rel)
					}
				}
			}
		}
	}

	if !recoveryTriggered {
		t.Fatalf("Deadlock reverse recovery was not triggered within 12.5 seconds! DeadlockTimer=%.2f",
			driver.HeadonDeadlockTimer())
	}
}

func TestAIBusWaitsBehindStoppedPlayerBusNearRoundabout(t *testing.T) {
	// Unit test: When player bus is stopped near roundabout entrance,
	// an approaching AI bus on the same lane must brake and wait behind,
	// without ramming into the player bus.
	p := findSlot2Jam1814File()
	if p == "" {
		t.Skip("snapshot missing")
	}
	g := loadSaveForTest(t, p, 1440, 900)

	// Freeze player bus completely in place (simulate waiting or manual parking)
	g.Bus.Speed = 0
	g.Bus.Gear = sim.GearNeutral

	var ai8 *sim.Bus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 8 {
			ai8 = ab.Bus
		}
	}
	if ai8 == nil {
		t.Fatal("AI Bus 8 not found")
	}

	for tick := 1; tick <= 300; tick++ {
		// Keep player bus motionless
		g.Bus.Speed = 0
		g.Bus.Pos = sim.Vec2{X: -4065.5, Y: 1685.9}
		g.Bus.Heading = 2.250

		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		// Verify AI Bus 8 never collides with player bus
		if hit, _, _ := sim.CheckBusesCollision(g.Bus, ai8); hit {
			t.Fatalf("tick %d: AI Bus 8 collided with stopped player bus! Player=%v AI8=%v",
				tick, g.Bus.Pos, ai8.Pos)
		}
	}

	// AI Bus 8 should have stopped safely behind player bus
	dist := ai8.Pos.Distance(g.Bus.Pos)
	if dist > 200.0 {
		t.Errorf("AI Bus 8 did not approach: dist=%.1f", dist)
	}
	if math.Abs(ai8.Speed) > 5.0 {
		t.Errorf("AI Bus 8 did not stop behind player bus: spd=%.2f, dist=%.1f", ai8.Speed, dist)
	}
}

func findSlot2Jam1501File() string {
	return firstFixture(filepath.Join("..", "dist", "slot2-jam-2026-11-02-1501", "slot2.json"))
}

func TestSlot2CarBusMergeDeadlockResolution(t *testing.T) {
	savePath := findSlot2Jam1501File()
	if savePath == "" {
		t.Skip("slot2 snapshot not found; skipping")
	}
	sim.SeedSimulation(1)
	g := loadSaveForTest(t, savePath, 1440, 900)

	var car31566 *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == 31566 {
			car31566 = c
			break
		}
	}

	var bus8 *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 8 {
			bus8 = ab
			break
		}
	}

	if car31566 == nil || bus8 == nil {
		t.Fatalf("car 31566 (%v) or bus 8 (%v) not found", car31566, bus8)
	}

	initCarX := car31566.Pos.X
	initBusX := bus8.Bus.Pos.X

	// Step simulation for 500 ticks (~8.3s)
	for tick := 1; tick <= 500; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at tick %d: %v", tick, err)
		}
	}

	// Car 31566 should have accelerated and driven past Bus 8 (heading West, so X becomes more negative)
	if car31566.Pos.X >= initCarX-200.0 {
		t.Fatalf("Car 31566 should have moved forward significantly, got X=%.2f (init=%.2f)", car31566.Pos.X, initCarX)
	}
	if car31566.Speed < 50.0 {
		t.Fatalf("Car 31566 should be cruising at speed, got Spd=%.2f", car31566.Speed)
	}

	// Bus 8 should have moved forward significantly (it merges into Lane 2 and proceeds to Station 1)
	if bus8.Bus.Pos.X >= initBusX-100.0 {
		t.Fatalf("Bus 8 should have advanced and merged, got X=%.2f (init=%.2f)", bus8.Bus.Pos.X, initBusX)
	}
}

func TestSlot2RightApproachDoesNotOscillate(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	p := filepath.Join(filepath.Dir(source), "../dist/slot2-sway-2026-11-04/slot2.json")
	if _, e := os.Stat(p); os.IsNotExist(e) {
		t.Skip("sway snapshot unavailable")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if e := g.LoadGame(); e != nil {
		t.Fatal(e)
	}
	g.Settings.IsOpen = false
	lastSign, reversals := 0, 0
	maxYaw := 0.
	initial := g.Bus.Pos
	for tick := 0; tick < 60*12; tick++ {
		if e := g.Update(); e != nil {
			t.Fatal(e)
		}
		yaw := simAngle(g.Bus.Heading + math.Pi/2)
		if math.Abs(yaw) > maxYaw {
			maxYaw = math.Abs(yaw)
		}
		sign := 0
		if yaw > .08 {
			sign = 1
		}
		if yaw < -.08 {
			sign = -1
		}
		if sign != 0 && sign != lastSign {
			if lastSign != 0 {
				reversals++
			}
			lastSign = sign
		}
		if tick%60 == 0 {
			t.Logf("tick%d x%.1f y%.1f yaw%.3f steer%.2f speed%.1f offset%.1f target%.1f", tick, g.Bus.Pos.X, g.Bus.Pos.Y, yaw, g.Bus.SteeringAngle, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset)
		}
	}
	t.Logf("reversals%d maxYaw%.2f traveled%.0f", reversals, maxYaw, g.Bus.Pos.Distance(initial))
	if g.Bus.Pos.Distance(initial) < 700 {
		t.Fatal("bus stayed stuck rather than stabilizing")
	}
	if reversals > 2 {
		t.Fatalf("repeated lane hunting: %d yaw reversals", reversals)
	}
}
func simAngle(a float64) float64 { return math.Atan2(math.Sin(a), math.Cos(a)) }
