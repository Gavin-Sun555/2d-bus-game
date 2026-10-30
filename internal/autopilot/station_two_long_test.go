package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/two-articulated-berths-2026-10-29.json
var savedTwoLongBerths []byte

func TestSavedSecondArticulatedBoardsFromWaitingPlan(t *testing.T) {
	var s rearDepartureFixture
	if err := json.Unmarshal(savedTwoLongBerths, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	stop := s.World.Stops[s.Route.GetCurrentTargetStopID()]
	initial, length := s.Bus.Pos, stop.Length
	var obs []VehicleObstacle
	for _, b := range s.OtherBuses {
		obs = append(obs, ObserveBus(b))
	}
	_, _, _, _, doors, docked := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, obs, 52, 1./60)
	if !doors || !docked || ad.State != AutopilotStateDocked {
		t.Fatal("saved rear waiting plan did not become a boarding berth")
	}
	if s.Bus.Pos != initial || stop.Length != length {
		t.Fatal("boarding moved bus or extended platform")
	}
	if !stop.ContainsBus(s.Bus) {
		t.Fatal("partial articulated arrival rejected")
	}
	for _, b := range s.OtherBuses {
		if hit, _, _ := sim.CheckBusesCollision(s.Bus, b); hit {
			t.Fatal("boarding accepted body overlap")
		}
	}
}

func TestThirdBusWaitsForEitherModelPair(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for first := sim.BusStandard; first < sim.BusModelCount; first++ {
			for second := sim.BusStandard; second < sim.BusModelCount; second++ {
				w, u, _ := serviceTestWorld(h, 52)
				stop := w.Stops[1]
				a, b, c := sim.NewBus(0, 0, h), sim.NewBus(0, 0, h), sim.NewBus(0, 0, h)
				a.SetModel(first)
				b.SetModel(second)
				c.SetModel(sim.BusArticulated)
				sim.PlaceBusInStationQueue(a, stop, nil, 52)
				sim.PlaceBusInStationQueue(b, stop, []*sim.Bus{a}, 52)
				sim.PlaceBusInStationQueue(c, stop, []*sim.Bus{a, b}, 52)
				a.DoorsOpen, b.DoorsOpen = true, true
				obs := []VehicleObstacle{ObserveBus(a), ObserveBus(b)}
				if len(availableStationBerths(c, stop, obs)) != 0 {
					t.Fatal("third bus reserved a boarding berth")
				}
				ad := NewAutopilotDriver()
				ad.QueueAtStation(stop.ID)
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				for tick := 0; tick < 120; tick++ {
					th, br, st, hb, doors, docked := ad.ComputeControls(c, route, w, nil, obs, 52, 1./60)
					c.Update(1./60, th, br, st, hb, doors)
					if doors || docked || c.DoorsOpen {
						t.Fatal("third bus boarded while both berths occupied")
					}
					if hit, _, _ := sim.CheckBusesCollision(c, b); hit {
						t.Fatal("third bus hit queue")
					}
				}
				if c.Pos.Sub(b.Pos).Dot(u) >= 0 {
					t.Fatal("third bus overtook queue")
				}
			}
		}
	}
}
