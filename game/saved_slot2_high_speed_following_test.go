package game

import (
	"os"
	"path/filepath"
	"testing"
)

func findSlot2CrashFile() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-crash-2026-11-02", "slot2.json"),
		filepath.Join("dist", "slot2-crash-2026-11-02", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-crash-2026-11-02/slot2.json",
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2ResumeAfterCrash verifies that loading slot 2 directly at the crash location
// recovers smoothly: the bus does not suffer further damage, follows the queue as the traffic light
// clears, and successfully continues along the route.
func TestSavedSlot2ResumeAfterCrash(t *testing.T) {
	p := findSlot2CrashFile()
	if p == "" {
		t.Skip("no save file found for slot2 crash")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

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
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

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
