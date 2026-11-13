package game

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func findSlot2CrashFile() string {
	return firstFixture(filepath.Join("..", "dist", "slot2-crash-2026-11-02", "slot2.json"))
}

// TestSavedSlot2ResumeAfterCrash verifies that loading slot 2 directly at the crash location
// recovers smoothly: the bus does not suffer further damage, follows the queue as the traffic light
// clears, and successfully continues along the route.
func TestSavedSlot2ResumeAfterCrash(t *testing.T) {
	p := findSlot2CrashFile()
	if p == "" {
		t.Skip("no save file found for slot2 crash")
	}
	g := loadSaveForTest(t, p, 1280, 720)

	initialDur := g.Bus.Durability

	for tick := 1; tick <= 60*12; tick++ { // 12 seconds
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}
		if g.Bus.Durability < initialDur {
			t.Fatalf("durability dropped from %.1f to %.1f at tick %d (pos=%v spd=%.1f)",
				initialDur, g.Bus.Durability, tick, g.Bus.Pos, g.Bus.Speed)
		}
	}

	t.Logf("Resumed safely! pos=%v spd=%.1f dur=%.1f", g.Bus.Pos, g.Bus.Speed, g.Bus.Durability)
}

// TestSavedSlot2BerserkApproachStoppedQueue tests approaching a stopped queue at high speed
// in Berserk mode on a multi-lane expressway, verifying that the bus brakes smoothly to a stop
// without crashing into the stationary rear car of the queue.
func TestSavedSlot2BerserkApproachStoppedQueue(t *testing.T) {
	p := findSlot2CrashFile()
	if p == "" {
		t.Skip("no save file found for slot2 crash")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	// Ambient respawns draw from the shared simulation RNG. Seed it so earlier
	// tests cannot place a replacement car in this approach lane.
	sim.SeedSimulation(1)

	// Move bus back to X=-4600 so it approaches the queue at high cruise speed
	g.Bus.Pos.X = -4600.0
	g.Bus.Pos.Y = -3574.0 // In right lane
	g.Bus.Heading = 0.0
	g.Bus.Speed = 440.0
	g.AutopilotDriver.LaneOffset = 0.0
	g.AutopilotDriver.TargetLaneOffset = 0.0
	g.AutopilotDriver.OvertakeState = 0
	g.AutopilotDriver.State = 0

	stopped := false
	for tick := 1; tick <= 60*26; tick++ {
		// Keep cars 19946..19949 stationary (waiting at red signal)
		for _, car := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			if car.ID >= 19678 && car.ID <= 19949 {
				car.Speed = 0.0
				car.TargetSpeed = 0.0
			}
		}

		durBefore := g.Bus.Durability
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}

		if g.Bus.Durability < durBefore {
			t.Fatalf("collision detected at tick %d! dur=%.1f pos=%v spd=%.1f",
				tick, g.Bus.Durability, g.Bus.Pos, g.Bus.Speed)
		}

		if g.Bus.Speed < 0.5 && g.Bus.Pos.X > -2750.0 {
			gap := -2498.2 - g.Bus.Pos.X - (g.Bus.Length/2 + 23.0)
			if gap < 25.0 {
				t.Fatalf("stopped too close to car: gap=%.2f px", gap)
			}
			t.Logf("Stopped safely at tick %d! pos=%v gap=%.2f px dur=%.1f",
				tick, g.Bus.Pos, gap, g.Bus.Durability)
			stopped = true
			break
		}
	}

	if !stopped {
		d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, 52)
		t.Fatalf("bus did not stop behind queue: pos=%v spd=%.1f target=%.1f limits=%v blockers=%v",
			g.Bus.Pos, g.Bus.Speed, g.AutopilotDriver.LastTargetSpeed, d.Limits, d.Blockers)
	}
}

func TestSavedSlot2RecentersLeftPass(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	path := filepath.Join(filepath.Dir(source), "../dist/slot2-left-pass-2026-11-05/slot2.json")
	if _, err := os.Stat(path); os.IsNotExist(err) {
		t.Skip("November 5 slot 2 snapshot unavailable")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	bus := g.Bus
	initialCondition := bus.Durability
	target := g.Route.GetCurrentTargetStopID()
	const innerCenter = -14426.
	centeredAt, dockedAt := -1, -1
	for tick := 0; tick < 1800; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Durability < initialCondition {
			t.Fatalf("collision at %.2fs, condition %.0f -> %.0f", float64(tick+1)/60, initialCondition, bus.Durability)
		}
		if centeredAt < 0 && math.Abs(bus.Pos.Y-innerCenter) < 4 && bus.Speed > 250 {
			centeredAt = tick + 1
		}
		if dockedAt < 0 && bus.DoorsOpen && g.AutopilotDriver.DockedStopID == target {
			dockedAt = tick + 1
		}
	}
	if centeredAt < 0 || centeredAt > 8*60 || dockedAt < 0 {
		t.Fatalf("did not promptly recenter and reach the stop: centered tick %d, docked tick %d", centeredAt, dockedAt)
	}
	t.Logf("recentered and accelerating at %.2fs, opened doors at %.2fs, condition %.0f", float64(centeredAt)/60, float64(dockedAt)/60, bus.Durability)
}

// Optional user replay complements the synthetic, portable lane-policy matrix.
func TestSavedSlot2SettledRightTurnDoesNotCrawl(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	path := filepath.Join(filepath.Dir(source), "..", "dist", "right-turn-queue-2026-11-04", "slot2-before.json")
	if _, err := os.Stat(path); err != nil {
		t.Skip("optional slot 2 right-turn replay not available")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	state := g.AutopilotDriver.Snapshot()
	if state.ApproachMerge == nil || !state.ApproachMerge.RightTurn || !state.CurbBusRight {
		t.Fatal("wrong slot 2 fixture")
	}
	g.Settings.AutopilotEnabled = true
	start, health := g.Bus.Pos, g.Bus.Durability
	maxTarget, maxSpeed := 0., math.Abs(g.Bus.Speed)
	for tick := 0; tick < 120; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		maxTarget = math.Max(maxTarget, g.AutopilotDriver.LastTargetSpeed)
		maxSpeed = math.Max(maxSpeed, g.Bus.Speed)
		if g.Bus.Durability < health {
			t.Fatal("right-turn cruise caused contact", tick)
		}
	}
	t.Logf("slot2 2s: target %.1f speed %.1f distance %.1f joined %v", maxTarget, maxSpeed, start.Distance(g.Bus.Pos), g.AutopilotDriver.Snapshot().ApproachMergeJoined)
	if maxTarget <= 160 || maxSpeed <= 120 {
		t.Fatal("settled curb lane remained at maneuver speed")
	}
	junction := state.ApproachMerge.Junction
	geometry := g.World.JunctionAt(junction, g.Settings.GetLaneWidth())
	cleared := false
	for tick := 120; tick < 7200; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatal("right-turn replay caused contact", tick)
		}
		if tick%60 == 0 {
			if hit := g.Bus.CheckBoundaries(g.World, g.Settings.GetLaneWidth()); hit.Zone == sim.ZoneHardBarrier {
				t.Fatal("right-turn replay hit barrier", hit.ObstacleName)
			}
		}
		if g.Bus.Pos.X > junction.X+geometry.HalfX+g.Bus.OverallLength()/2+80 && math.Abs(g.Bus.Heading) < .35 {
			cleared = true
			t.Logf("saved right turn cleared in %.2fs, health %.0f", float64(tick+1)/60, g.Bus.Durability)
			break
		}
	}
	if !cleared {
		t.Fatal("saved curb right turn never cleared the junction")
	}

}

// Optional local replay; the small observation-cache regression is covered by
// traffic.TestPlayerObstacleSnapshotWithoutAIBuses without an external save.
func TestSlot2BerserkCrashNovember4(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	p := filepath.Join(filepath.Dir(source), "../dist/slot2-berserk-crash-2026-11-04/slot2.json")
	if _, err := os.Stat(p); os.IsNotExist(err) {
		t.Skip("November 4 slot 2 snapshot unavailable")
	}
	for _, rewind := range []bool{false, true} {
		name := "resume"
		if rewind {
			name = "high-speed-approach"
		}
		t.Run(name, func(t *testing.T) {
			g := loadSaveForTest(t, p, 1280, 720)
			g.Settings.IsOpen = false
			if !g.Settings.AutopilotEnabled || !g.Settings.AutopilotBerserk || len(g.AIBusMgr.Buses) != 0 {
				t.Fatal("fixture no longer represents berserk driving without AI buses")
			}
			if rewind {
				g.Bus.Pos.X -= 900
				g.Bus.Speed = g.Settings.GetBerserkCruiseSpeed()
				g.Bus.CrashCooldown = 0
				g.AutopilotDriver.Reset()
			}
			initialDur := g.Bus.Durability
			initialPos := g.Bus.Pos
			slowed := false
			for tick := 0; tick < 60*20; tick++ {
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < initialDur {
					t.Fatalf("collision tick%d dur%.1f->%.1f pos%v speed%.1f", tick, initialDur, g.Bus.Durability, g.Bus.Pos, g.Bus.Speed)
				}
				if g.AutopilotDriver.LastTargetSpeed < g.Settings.GetBerserkCruiseSpeed()-20 {
					slowed = true
				}
			}
			if !slowed {
				t.Fatal("autopilot never responded to slower traffic")
			}
			if g.Bus.Pos.Distance(initialPos) < 1000 {
				t.Fatal("bus avoided crash by remaining stuck")
			}
			t.Logf("20 seconds without new damage: durability%.1f, distance%.0fpx", g.Bus.Durability, g.Bus.Pos.Distance(initialPos))
		})
	}
}

// TestSaveSlot2BerserkOvertakesSlowerAI ensures that in Save Slot 2 on the 8-lane
// bus boulevard (城市拓展大道), the player's 120 km/h berserk autopilot changes lanes to
// Lane 2 to pass the slower AI bus #10 (traveling at ~80 km/h), accelerates to cruise speed,
// passes AI #10, and cleanly returns to the reserved bus lane (Lane 3).
func TestSaveSlot2BerserkOvertakesSlowerAI(t *testing.T) {
	savePath := firstFixture(filepath.Join("..", "dist", "slot2-oct30-1116", "slot2.json"))
	if savePath == "" {
		wd, _ := os.Getwd()
		t.Skipf("slot2 save file not found, wd=%s", wd)
	}

	absPath, err := filepath.Abs(savePath)
	if err != nil {
		t.Fatal(err)
	}

	g := loadSaveForTest(t, absPath, 1280, 720)

	p := g.Bus
	ad := g.AutopilotDriver
	lw := g.Settings.GetLaneWidth()

	if !g.Settings.AutopilotBerserk || !ad.BerserkMode {
		t.Fatalf("expected berserk mode enabled, got settings=%v driver=%v", g.Settings.AutopilotBerserk, ad.BerserkMode)
	}

	var ai10 *sim.Bus
	for _, b := range g.AIBusMgr.Buses {
		if b.ID == 10 {
			ai10 = b.Bus
			break
		}
	}
	if ai10 == nil {
		t.Fatal("AI #10 not found in save slot 2")
	}

	// AI #10 must initially be ahead of the player in the same lane (heading West)
	if p.Pos.X <= ai10.Pos.X {
		t.Fatalf("expected player (X=%.1f) to be behind AI #10 (X=%.1f)", p.Pos.X, ai10.Pos.X)
	}

	laneChanged := false
	passedLeader := false
	returnedToLane := false

	for tick := 1; tick <= 350; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		// 1. Bus initiates lane change to Lane 2 (TargetLaneOffset = -lw)
		if ad.TargetLaneOffset <= -lw+1.0 {
			laneChanged = true
		}

		// 2. Bus passes AI #10 while heading West (Player X < AI X)
		if laneChanged && p.Pos.X < ai10.Pos.X {
			passedLeader = true
		}

		// 3. Bus returns to base lane (Lane 3) after completing the pass
		if passedLeader && math.Abs(ad.LaneOffset) < 1.0 && ad.OvertakeState == autopilot.OvertakeNone {
			returnedToLane = true
			break
		}
	}

	if !laneChanged {
		t.Errorf("player bus failed to initiate lane change to overtake AI #10: TargetLaneOffset=%.1f", ad.TargetLaneOffset)
	}
	if !passedLeader {
		t.Errorf("player bus failed to pass AI #10: PlayerX=%.1f AIX=%.1f", p.Pos.X, ai10.Pos.X)
	}
	if !returnedToLane {
		t.Errorf("player bus failed to return to base bus lane after passing: LaneOffset=%.1f OvertakeState=%d", ad.LaneOffset, ad.OvertakeState)
	}
}
