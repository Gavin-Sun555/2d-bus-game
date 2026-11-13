package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestMergeObservationMatchesLiveRulesAndRefreshes(t *testing.T) {
	stop := sim.NewBusStop(731, "stop", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), 52, 10)
	w := &sim.World{Stops: []*sim.BusStop{stop}}
	b := sim.NewBus(stop.Pos.X, stop.Pos.Y, 0)
	tm := &TrafficManager{}
	for phase := 0; phase < 8; phase++ {
		b.DoorsOpen = false
		b.LaneMergeRequest = nil
		b.StationMergeRequest = nil
		switch phase {
		case 0, 1, 2, 3:
			b.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 731, Remaining: .3}
		case 4:
			b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}, Remaining: .3}
		case 5:
			b.LaneMergeRequest = &sim.BusLaneMergeRequest{Remaining: 0}
		}
		if phase == 1 {
			b.DoorsOpen = true
		}
		if phase == 2 {
			w.Stops = nil
		} else {
			w.Stops = []*sim.BusStop{stop}
		}
		if phase == 3 {
			replacement := *stop
			replacement.Heading = math.Pi
			w.Stops = []*sim.BusStop{&replacement, stop}
		}
		tm.prepareBusPerception(b, w)
		for lane := 0; lane < 4; lane++ {
			for x := -600.; x <= 100; x += 70 {
				car := sim.TrafficCar{Pos: sim.Vec2{X: x, Y: (float64(lane) + .5) * 52}, Length: 48, Width: 22, LaneIndex: lane, Speed: 80}
				cached, live := car, car
				f := carFrame{car: &live, bus: b, buses: []*sim.Bus{b}, world: w, laneWidth: 52}
				want := tm.busMergeYieldSpeed(f, 100)
				f.car = &cached
				f.mergeBuses = tm.observedMerges
				f.mergeBusesReady = true
				got := tm.busMergeYieldSpeed(f, 100)
				if got != want || cached.IsBraking != live.IsBraking || cached.YieldingToBus != live.YieldingToBus {
					t.Fatalf("phase %d lane %d x %v: cached %v live %v", phase, lane, x, got, want)
				}
			}
		}
	}
	tm.prepareBusPerception(nil, w)
	if len(tm.observedMerges) != 0 || tm.mergeStorage[0].bus != nil {
		t.Fatal("removed bus retained")
	}
}
