package game

import (
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Read-only replay of the October 25 outer articulated berth deadlock.
func TestSavedOuterBerthDeadlockRecovery(t *testing.T) {
	path := os.Getenv("BUS_OUTER_BERTH_SAVE")
	if path == "" {
		t.Skip("BUS_OUTER_BERTH_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	target := g.Route.GetCurrentTargetStopID()
	initialAlighted := 0
	if a := g.Operations.Vehicle(0).Stations[target]; a != nil {
		initialAlighted = a.Alighted
	}
	boarded, served := false, false
	distances := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	for tick := 0; tick < 3600; tick++ {
		before := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, ab := range g.AIBusMgr.Buses {
			before[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		check := func(id int, bus *sim.Bus) {
			step := bus.Pos.Distance(before[id])
			if step > bus.MaxSpeedFwd/60+1 {
				t.Fatalf("vehicle%d jumped at tick%d: %.2f", id, tick, step)
			}
			if bus.Durability < health[id] {
				t.Fatalf("vehicle%d contact at tick%d", id, tick)
			}
			distances[id] += step
		}
		check(0, g.Bus)
		for _, ab := range g.AIBusMgr.Buses {
			check(ab.ID, ab.Bus)
		}
		if !boarded && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			boarded = true
			t.Logf("opened doors at original target%d at %.2fs", target, float64(tick)/60)
		}
		if tick == 120 && !boarded {
			t.Fatal("did not open at saved outer berth within two seconds")
		}
		if !served && g.Route.GetCurrentTargetStopID() != target {
			a := g.Operations.Vehicle(0).Stations[target]
			if !boarded || g.lastSkippedStopID == target || a == nil || a.Alighted <= initialAlighted {
				t.Fatal("advanced without actually serving original stop")
			}
			served = true
			t.Logf("passengers alighted and route advanced at %.2fs", float64(tick)/60)
		}
	}
	if !served || distances[0] < 1000 || distances[8] < 1000 {
		t.Fatalf("deadlock: served%v player%.1f AI8%.1f", served, distances[0], distances[8])
	}
	t.Logf("60-second replay: player%.1f AI8%.1f; all buses retained health", distances[0], distances[8])
}
