package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"testing"
)

// Read a copied local save, never write it. This includes the occupied platform
// and the two alongside buses that held the berserk articulated player bus.
func TestSavedBerserkTurnRecovery(t *testing.T) {
	path := os.Getenv("BUS_TURN_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_TURN_REPLAY_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if !g.Settings.AutopilotEnabled || !g.Settings.AutopilotBerserk || !g.Bus.IsArticulated() {
		t.Fatal("expected saved berserk articulated scene")
	}
	initialTarget, initialHealth := g.Route.GetCurrentTargetStopID(), g.Bus.Durability
	aiHealth := map[int]float64{}
	for _, ab := range g.AIBusMgr.Buses {
		aiHealth[ab.ID] = ab.Bus.Durability
	}
	distance, resumedAt := 0., -1.
	servedTarget := false
	for tick := 0; tick < 7200; tick++ {
		before := g.Bus.Pos
		aiPositions := map[int]sim.Vec2{}
		for _, ab := range g.AIBusMgr.Buses {
			aiPositions[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		step := g.Bus.Pos.Distance(before)
		if step > g.Bus.MaxSpeedFwd/60+1 {
			t.Fatalf("player discontinuity tick%d: %.3f", tick, step)
		}
		distance += step
		if g.Bus.Durability < initialHealth {
			t.Fatalf("player contact tick%d", tick)
		}
		if math.Abs(sim.NormalizeAngle(g.Bus.Heading-g.Bus.TailHeading())) > sim.MaxArticulationAngle+1e-9 {
			t.Fatalf("jackknife tick%d", tick)
		}
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus.Pos.Distance(aiPositions[ab.ID]) > ab.Bus.MaxSpeedFwd/60+1 {
				t.Fatalf("AI%d discontinuity tick%d", ab.ID, tick)
			}
			if ab.Bus.Durability < aiHealth[ab.ID] {
				t.Fatalf("AI%d contact tick%d", ab.ID, tick)
			}
		}
		if g.AutopilotDriver.DockedStopID == initialTarget && g.Bus.DoorsOpen {
			servedTarget = true
		}
		if resumedAt < 0 && g.Route.GetCurrentTargetStopID() != initialTarget {
			if !servedTarget || g.lastSkippedStopID == initialTarget {
				t.Fatal("target advanced without actual boarding service")
			}
			resumedAt = float64(tick) / 60
		}
		if tick == 1799 && resumedAt < 0 {
			t.Fatal("did not resume service within 30 seconds")
		}
	}
	if distance < 2000 {
		t.Fatalf("insufficient travel %.1f", distance)
	}
	t.Logf("resumed service %.2fs, traveled %.0fpx in 120 simulated seconds, health %.0f", resumedAt, distance, g.Bus.Durability)
}
