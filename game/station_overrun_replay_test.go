package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"os"
	"testing"
)

func TestSavedStationOverrunReplay(t *testing.T) {
	path := os.Getenv("BUS_STATION_OVERRUN_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_OVERRUN_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	target := g.Route.GetCurrentTargetStopID()
	stop := g.World.Stops[target]
	initialHealth := g.Bus.Durability
	initialAlighted := 0
	if activity := g.Operations.Vehicle(0).Stations[target]; activity != nil {
		initialAlighted = activity.Alighted
	}
	boarded := false
	for tick := 0; tick < 3600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			if !boarded {
				t.Logf("opened doors at target at %.2fs pos%v", float64(tick)/60, g.Bus.Pos)
			}
			boarded = true
		}
		if g.Bus.Durability < initialHealth {
			t.Fatalf("contact at tick%d", tick)
		}
		if g.Route.GetCurrentTargetStopID() != target {
			if !boarded {
				t.Fatalf("advanced without boarding at %.2fs: pos%v frontBerth%v skipped%d", float64(tick)/60, g.Bus.Pos, stop.FrontBerthPosition(), g.lastSkippedStopID)
			}
			if g.lastSkippedStopID == target {
				t.Fatal("target advanced through missed-stop handling")
			}
			activity := g.Operations.Vehicle(0).Stations[target]
			if activity == nil || activity.Alighted <= initialAlighted {
				t.Fatal("passengers did not actually alight")
			}
			t.Logf("served target and advanced at %.2fs health%.0f", float64(tick)/60, g.Bus.Durability)
			return
		}
	}
	t.Fatalf("did not serve target in 60 seconds, boarded%v pos%v", boarded, g.Bus.Pos)
}
