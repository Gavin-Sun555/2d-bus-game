package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func findSlot2Jam1814File() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1814", "slot2.json"),
		filepath.Join("dist", "slot2-jam-2026-11-02-1814", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-jam-2026-11-02-1814/slot2.json",
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

func TestSavedSlot2PlayerCarJamResolved(t *testing.T) {
	p := findSlot2Jam1814File()
	if p == "" {
		t.Skip("snapshot missing")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = p
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

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
	g := NewGame(1440, 900)
	g.savePathOverride = p
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

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
