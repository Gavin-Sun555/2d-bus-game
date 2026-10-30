package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/yawed-past-berth-2026-10-30.json.gz
var yawedPastBerthScene []byte

func TestYawedMissedBerthCrawlsFromFirstMovingTick(t *testing.T) {
	var s stoppedSlotScene
	gz, err := gzip.NewReader(bytes.NewReader(yawedPastBerthScene))
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
	from := s.World.Stops[s.Route.GetCurrentTargetStopID()]
	s.Route.AdvanceToNextStop()
	ad.ContinuePastSkippedStop(s.Bus, from, 52)
	if !ad.departureEscapeActive {
		t.Fatal("missed taper did not arm recovery before acceleration")
	}
	var obstacles []VehicleObstacle
	for _, bus := range s.OtherBuses {
		obstacles = append(obstacles, ObserveBus(bus))
	}
	start := s.Bus.Pos
	restored := false
	for tick := 0; tick < 1800; tick++ {
		s.Lights.Update(1. / 60)
		th, br, st, hb, door, _ := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, obstacles, 52, 1./60)
		if ad.departureEscapeActive && s.Bus.Speed > 22 {
			t.Fatal("accelerated before proven crawl cleared tail")
		}
		s.Bus.Update(1./60, th, br, st, hb, door)
		if s.Bus.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("curb/shelter contact tick%d", tick)
		}
		for _, other := range s.OtherBuses {
			if hit, _, depth := sim.CheckBusesCollision(s.Bus, other); hit && depth > 1e-6 {
				t.Fatalf("bus contact tick%d", tick)
			}
		}
		if !restored && ad.departureEscapeActive && s.Bus.Pos.Distance(start) > 10 {
			raw, err := json.Marshal(ad.Snapshot())
			if err != nil {
				t.Fatal(err)
			}
			var snapshot DriverSnapshot
			if err = json.Unmarshal(raw, &snapshot); err != nil {
				t.Fatal(err)
			}
			ad, err = snapshot.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
			if !ad.departureEscapeActive {
				t.Fatal("restoring save lost early crawl")
			}
			restored = true
		}
		if s.Bus.Pos.Distance(start) > 200 {
			if !restored || ad.departureEscapeActive {
				t.Fatal("recovery did not finish")
			}
			return
		}
	}
	t.Fatal("missed arrival became a stalled departure")
}
