package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestBusSignalPrioritySavedTrafficReplay(t *testing.T) {
	path := os.Getenv("BUS_PRIORITY_SAVE")
	if path == "" {
		t.Skip("copied local traffic save required")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g.Settings.BusSignalPriority = true
	health := g.Bus.Durability
	initialTarget := g.Route.GetCurrentTargetStopID()
	served := false
	for tick := 0; tick < 5400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatalf("player contact at %.2fs", float64(tick)/60)
		}
		if g.AutopilotDriver.DockedStopID == initialTarget && g.Bus.DoorsOpen {
			served = true
		}
		for _, i := range g.TrafficLights.Intersections {
			ew, el, ns, nl := i.GetDetailedSignals()
			count := 0
			for _, s := range []sim.LightSignal{ew, el, ns, nl} {
				if s == sim.SignalGreen {
					count++
				}
			}
			limit := 1
			if !i.IsMajor {
				limit = 2
			}
			if count > limit {
				t.Fatal("conflicting protected greens")
			}
		}
	}
	if !served || g.Route.GetCurrentTargetStopID() == initialTarget {
		t.Fatal("priority replay failed to serve/depart original stop")
	}
	t.Logf("90s mixed traffic replay: original stop served and departed, health %.0f", g.Bus.Durability)
}

func TestBusSignalPriorityGameWiringAndSave(t *testing.T) {
	g := saveTestGame(t)
	if g.Settings.BusSignalPriority {
		t.Fatal("new feature unexpectedly enabled by default")
	}
	var chosen *sim.TrafficIntersection
	for _, i := range g.TrafficLights.Intersections {
		if i.ApproachBusLane[0] && i.ApproachLanes[0] >= 4 {
			chosen = i
			break
		}
	}
	if chosen == nil {
		t.Fatal("fixture has no bus road")
	}
	g.Bus.Pos = chosen.Center.Add(sim.Vec2{X: -chosen.GetStopLineDistForHeading(0, 52) - 300, Y: 182})
	g.Bus.Heading = 0
	g.Bus.DoorsOpen = false
	g.AutopilotDriver = autopilot.NewAutopilotDriver()
	g.Settings.BusSignalPriority = true
	fleet := g.AIBusMgr
	g.AIBusMgr = nil // A city without AI fleet must still support the player.
	g.updateBusSignalPriority()
	if !g.TrafficLights.BusPriorityEnabled || len(chosen.BusPriorityQueue) != 1 || chosen.BusPriorityQueue[0].BusID != 0 {
		t.Fatal("player did not request priority")
	}
	// Restore an empty manager before saving a normal game snapshot.
	g.AIBusMgr = fleet
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if !g.Settings.BusSignalPriority || !g.TrafficLights.BusPriorityEnabled {
		t.Fatal("save lost priority setting")
	}
	var retained bool
	for _, i := range g.TrafficLights.Intersections {
		if i.ID == chosen.ID && len(i.BusPriorityQueue) == 1 {
			retained = true
		}
	}
	if !retained {
		t.Fatal("save lost FIFO priority state")
	}
	g.Settings.BusSignalPriority = false
	g.updateBusSignalPriority()
	for _, i := range g.TrafficLights.Intersections {
		if len(i.BusPriorityQueue) != 0 {
			t.Fatal("disabled feature left a live request")
		}
	}
	if g.TrafficLights.BusPriorityEnabled {
		t.Fatal("disabled feature remained active")
	}
}
