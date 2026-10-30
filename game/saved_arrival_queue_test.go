package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"strconv"
	"testing"
)

// Opt-in replay of the copied 19:13 slot 2, including the complete station
// queue rather than accepting a short movement followed by another deadlock.
func TestSavedArrivalQueueResumesService(t *testing.T) {
	path := os.Getenv("BUS_SAVED_ARRIVAL_QUEUE")
	if path == "" {
		t.Skip("copied local save required")
	}
	seed := int64(5)
	if value := os.Getenv("BUS_SAVED_ARRIVAL_SEED"); value != "" {
		var err error
		seed, err = strconv.ParseInt(value, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
	}
	sim.SeedSimulation(seed)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(seed)
	start := g.Bus.Pos
	target := g.Route.GetCurrentTargetStopID()
	health := g.Bus.Durability
	beginnings := map[int]sim.Vec2{}
	initialTargets := map[int]int{}
	served := map[int]bool{}
	for _, a := range g.AIBusMgr.Buses {
		beginnings[a.ID] = a.Bus.Pos
		initialTargets[a.ID] = a.Route.GetCurrentTargetStopID()
	}
	boardingStarted := false
	for tick := 0; tick < 30000; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if before.Distance(g.Bus.Pos) > g.Bus.MaxSpeedFwd/60+2 {
			t.Fatal("player relocated")
		}
		if g.Bus.Durability < health {
			t.Fatalf("player contact at %.2fs", float64(tick)/60)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Route.GetCurrentTargetStopID() != initialTargets[a.ID] && !served[a.ID] {
				served[a.ID] = true
				t.Logf("AI%d served original target %.2fs", a.ID, float64(tick)/60)
			}
		}
		if tick%1800 == 0 {
			t.Logf("%.0fs player %v distance %.1f target%d", float64(tick)/60, g.Bus.Pos, g.Bus.Pos.Distance(start), g.Route.GetCurrentTargetStopID())
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(g.Bus.Pos) < 2400 && a.Bus.Speed < 1 {
					t.Logf("AI%d pos%v h%.3f state%v targetSpeed%.2f svc%.1f health%.0f", a.ID, a.Bus.Pos, a.Bus.Heading, a.Driver.State, a.Driver.LastTargetSpeed, a.Bus.PassengerServiceRemaining(), a.Bus.Durability)
				}
			}
		}
		if !boardingStarted && g.Route.GetCurrentTargetStopID() != target && g.Bus.DoorsOpen && g.AutopilotDriver.DockedStopID == target {
			boardingStarted = true
			t.Logf("player began original stop service %.2fs", float64(tick)/60)
		}
		if os.Getenv("BUS_SAVED_ARRIVAL_EARLY") == "1" && served[3] && served[4] && served[15] && g.Bus.Pos.Distance(start) > 200 {
			t.Logf("seed%d original three-bus lock cleared %.2fs", seed, float64(tick)/60)
			return
		}
		if boardingStarted && !g.Bus.DoorsOpen && g.Bus.Pos.Distance(g.World.Stops[target].Pos) > 300 {
			if !served[3] || !served[4] || !served[15] {
				t.Fatal("original leading buses did not finish service")
			}
			for _, a := range g.AIBusMgr.Buses {
				if (a.ID == 3 || a.ID == 4 || a.ID == 15) && a.Bus.Pos.Distance(beginnings[a.ID]) < 500 {
					t.Fatal("leading bus did not leave original station zone")
				}
			}
			if g.lastSkippedStopID == target {
				t.Fatal("recovery skipped original target")
			}
			t.Logf("seed%d player completed service and departure %.2fs; moved %.1f", seed, float64(tick)/60, g.Bus.Pos.Distance(start))
			return
		}
	}
	t.Fatalf("saved queue did not resume service: position%v served%v", g.Bus.Pos, served)
}
