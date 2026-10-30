package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/parallel-departure-slot1-2026-10-30.json.gz
var parallelDepartureScene []byte

func TestSavedDepartureClearsParallelArrivalDespiteHeadway(t *testing.T) {
	var s stoppedSlotScene
	gz, err := gzip.NewReader(bytes.NewReader(parallelDepartureScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	if err = json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: s.Bus, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: s.World.Stops[s.Route.GetCurrentTargetStopID()]}
	for _, b := range s.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	if limit := ad.followingSpeed(f); limit != 0 {
		t.Fatalf("fixture lost conservative headway block: %g", limit)
	}
	desired, _, _, _ := ad.pursuitSteering(f)
	if _, ok := ad.stationDepartureEscape(f, desired); !ok {
		t.Fatal("clear physical departure rejected by adjacent headway")
	}
	start := s.Bus.Pos
	for tick := 0; tick < 1200; tick++ {
		th, br, st, hb, door, _ := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, f.obstacles, 52, 1./60)
		s.Bus.Update(1./60, th, br, st, hb, door)
		if s.Bus.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("boundary contact tick%d", tick)
		}
		for _, other := range s.OtherBuses {
			if hit, _, depth := sim.CheckBusesCollision(s.Bus, other); hit && depth > 1e-6 {
				t.Fatalf("bus contact tick%d depth%g", tick, depth)
			}
		}
		if s.Bus.Pos.Distance(start) > 250 {
			return
		}
	}
	t.Fatal("parallel arrival kept the departure blocked")
}
