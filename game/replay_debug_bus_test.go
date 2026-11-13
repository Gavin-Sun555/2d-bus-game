package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func TestDebugCapturedBusSceneReleases(t *testing.T) {
	path := os.Getenv("BUS_DEBUG_REPLAY")
	if path == "" {
		t.Skip("BUS_DEBUG_REPLAY not set")
	}
	envelope, err := savefile.Read[SaveMetadata](path)
	if err != nil {
		t.Fatal(err)
	}
	if envelope.Metadata == nil || envelope.Metadata.Debug == nil {
		t.Fatal("debug metadata missing")
	}
	meta := envelope.Metadata.Debug
	if meta.VehicleKind != "bus" {
		t.Skip("bus replay requires bus capture")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(copyPath, raw, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	var root *traffic.AIBus
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == meta.VehicleID {
			root = a
			break
		}
	}
	if root == nil {
		t.Fatal("trigger bus missing")
	}
	root.Driver.BeforeRecovery = func() { t.Fatal("bus repositioned instead of driving away") }
	g.AIBusMgr.BeforeRecovery = func(event sim.TrafficRecoveryEvent) {
		if event.ID == root.ID {
			t.Fatal("bus reset")
		}
	}
	start, health, passengers := root.Bus.Pos, root.Bus.Durability, root.Bus.PassengerCount
	targetStopID := root.Route.GetCurrentTargetStopID()
	nearStop := targetStopID >= 0 && targetStopID < len(g.World.Stops) && root.Bus.Pos.Distance(g.World.Stops[targetStopID].Pos) < 1000
	skippedEmptyStop := false
	for tick := 0; tick < 14400; tick++ {
		beforeStop := root.Route.GetCurrentTargetStopID()
		beforeIndex := root.Route.CurrentIndex
		wasAllowedToSkip := root.Driver.CanSkipTargetStop
		hadAlighting := nearStop && beforeStop == targetStopID && root.Bus.CountPassengersAlightingAt(targetStopID) > 0
		before := root.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if root.Bus.Pos.Distance(before) > math.Abs(root.Bus.Speed)/60+.05 || root.Bus.Durability < health {
			t.Fatal("repositioned, damaged or lost passengers")
		}
		if root.Driver.State == autopilot.AutopilotStateDocked && root.Driver.DockedStopID == targetStopID && root.Bus.DoorsOpen {
			t.Logf("bus%d reached stop%d normally in %.2fs", root.ID, targetStopID, float64(tick+1)/60)
			return
		}
		if nearStop && beforeStop == targetStopID && root.Route.GetCurrentTargetStopID() != targetStopID &&
			root.Bus.PassengerCount == passengers && !hadAlighting {
			stop := g.World.Stops[targetStopID]
			skippedEmptyStop = wasAllowedToSkip || stop.WaitingPassengers == 0 ||
				!stop.HasMatchingWaiting(root.Route.GetUpcomingCandidateStopIDs(beforeIndex))
		}
		if skippedEmptyStop && root.Bus.Pos.Distance(start) > 1000 {
			t.Logf("bus%d passed stop%d without passenger demand and cleared in %.2fs", root.ID, targetStopID, float64(tick+1)/60)
			return
		}
		if root.Bus.PassengerCount != passengers {
			t.Fatal("passengers changed before reaching a platform")
		}
		if tick == 3600 || tick == 10800 {
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(root.Bus.Pos) > 1500 {
					continue
				}
				obs := g.AIBusMgr.GetObstacles(a.ID)
				obs = append(obs, autopilot.ObserveBus(g.Bus))
				for _, c := range g.TrafficMgr.Cars {
					if c.Pos.Distance(a.Bus.Pos) < 1200 {
						obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
					}
				}
				d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, obs, 52)
				d.Path = nil
				t.Logf("at%.1fs bus%d pos%v heading%.3f tail%.3f %+v", float64(tick)/60, a.ID, a.Bus.Pos, a.Bus.Heading, a.Bus.TailHeading(), d)
			}
		}
		if !nearStop && root.Bus.Pos.Distance(start) > 1000 {
			t.Logf("bus%d released in %.2fs", root.ID, float64(tick+1)/60)
			return
		}
	}
	if output := os.Getenv("BUS_DEBUG_REPLAY_OUTPUT"); output != "" {
		g.savePathOverride = output
		if err = g.SaveGame(); err != nil {
			t.Fatal(err)
		}
	}
	t.Fatalf("bus did not clear entry within 240sec: %v", root.Bus.Pos)
}
