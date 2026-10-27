package game

import (
	"encoding/json"
	"os"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
)

// Optional local reproduction: copies/reads a save without changing its source.
func TestSavedJamReplay(t *testing.T) {
	path := os.Getenv("BUS_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_REPLAY_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	initialPos, initialHealth := g.Bus.Pos, g.Bus.Durability
	initialTarget := g.Route.GetCurrentTargetStopID()
	ticks := 3600
	if value := os.Getenv("BUS_REPLAY_TICKS"); value != "" {
		n, err := strconv.Atoi(value)
		if err != nil || n < 1 {
			t.Fatal("invalid BUS_REPLAY_TICKS")
		}
		ticks = n
	}
	for tick := 0; tick <= ticks; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if tick%300 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			d.Path = nil
			snap := g.AutopilotDriver.Snapshot()
			record := struct {
				Tick      int
				Bus       any
				Diagnosis autopilot.StopDiagnosis
				Snapshot  any
			}{tick, struct {
				Position               any
				Heading, Speed, Health float64
			}{g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.Bus.Durability}, d, struct {
				Revision      int
				DepartureRun  float64
				Ordinary      bool
				Mid, Approach any
			}{snap.PathRevision, snap.DockDepartureRun, snap.OrdinaryBusLeft, snap.MidLaneRecovery, snap.ApproachMerge}}
			b, _ := json.Marshal(record)
			t.Log(string(b))
			for _, ab := range g.AIBusMgr.Buses {
				if ab.Bus.Pos.Distance(g.Bus.Pos) > 650 || ab.Bus.Speed > 5 {
					continue
				}
				obs := make([]autopilot.VehicleObstacle, 0, len(g.obstacles)+1)
				for _, ob := range g.obstacles {
					if ob.Bus != ab.Bus {
						obs = append(obs, ob)
					}
				}
				obs = append(obs, autopilot.ObserveBus(g.Bus))
				d := ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
				d.Path = nil
				b, _ := json.Marshal(struct {
					Tick, ID  int
					Pos       any
					Heading   float64
					Diagnosis autopilot.StopDiagnosis
					Mid       any
				}{tick, ab.ID, ab.Bus.Pos, ab.Bus.Heading, d, ab.Driver.Snapshot().MidLaneRecovery})
				t.Log("fleet " + string(b))
			}
		}
	}
	if g.Bus.Pos.Distance(initialPos) < 2000 || g.Route.GetCurrentTargetStopID() == initialTarget {
		t.Fatalf("saved jam did not resume service: start%v pos%v target%d", initialPos, g.Bus.Pos, g.Route.GetCurrentTargetStopID())
	}
	if g.Bus.Durability < initialHealth {
		t.Fatalf("saved recovery lost durability: %.1f -> %.1f", initialHealth, g.Bus.Durability)
	}
}
