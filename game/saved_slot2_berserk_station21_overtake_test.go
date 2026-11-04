package game

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestSavedSlot2PlayerBerserkDoesNotStallSlanted verifies that when the player bus in Berserk mode
// approaches Stop 32 (Coastal District 21) on multi-lane road in the passing lane while the bay is occupied,
// it does not freeze at 0 km/h diagonally slanted across the travel lanes.
func TestSavedSlot2PlayerBerserkDoesNotStallSlanted(t *testing.T) {
	savePath := "/Users/yansun/personal/2d-bus-game/dist/slot2-station21-jam-2026-11-03-0654/slot2.json"
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	// 1. Verify saved position can resume and cruise to Stop 4
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	for step := 1; step <= 60; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if g.Route.StopIDs[g.Route.CurrentIndex] != 4 {
		t.Fatalf("Expected route to advance to Stop 4, got Stop %d", g.Route.StopIDs[g.Route.CurrentIndex])
	}
	if g.Bus.Speed < 30.0 {
		t.Fatalf("Expected bus to accelerate after resuming, got speed %.1f", g.Bus.Speed)
	}

	// 2. Test approaching Stop 32 in passing lane with occupied bay
	g.Route.CurrentIndex = 3 // Stop 32
	g.Bus.Pos = sim.Vec2{X: -5100, Y: -7174}
	g.Bus.Heading = 0.0
	g.Bus.Speed = 150.0
	g.Bus.SteeringAngle = 0.0
	g.AutopilotDriver.BerserkMode = true
	g.AutopilotDriver.LaneOffset = -52
	g.AutopilotDriver.TargetLaneOffset = -52
	g.AutopilotDriver.OvertakeState = autopilot.OvertakePassing

	stalled := false
	for step := 1; step <= 250; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if step == 1 {
			targetStop := g.World.Stops[g.Route.GetCurrentTargetStopID()]
			t.Logf("Stop %d: pos=%v frontBerth=%v rearBerth=%v len=%.1f",
				targetStop.ID, targetStop.Pos, targetStop.FrontBerthPosition(), targetStop.RearBerthPosition(), targetStop.Length)
			for i, ob := range g.obstacles {
				if ob.Pos.Distance(targetStop.Pos) < 200 {
					t.Logf("  near ob %d: pos=%v spd=%.1f", i, ob.Pos, ob.Speed)
				}
			}
		}
		// Bus must never stall diagonally slanted in the travel lanes with 0 km/h speed
		isSlanted := math.Abs(g.Bus.Heading) > 0.08
		if g.Bus.Speed < 1.0 && isSlanted && g.AutopilotDriver.State != autopilot.AutopilotStateDocked {
			stalled = true
			t.Errorf("Step %d: Player bus stalled diagonally slanted on road: Pos=(%.1f, %.1f), Heading=%.4f (%.1f deg), Spd=%.1f",
				step, g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Heading, g.Bus.Heading*180/math.Pi, g.Bus.Speed)
			break
		}
	}
	if stalled {
		t.Fatal("Player bus stalled slanted in travel lanes")
	}
}
