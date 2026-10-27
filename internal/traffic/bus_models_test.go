package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestMixedModelFleetCountsAndStableSlots(t *testing.T) {
	w := sim.NewWorld(52)
	routes := sim.DefaultTransitLines(w)
	c := LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 2, 2}, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}.Normalize()
	plans := []LineFleetPlan{{routes[0], c}, {routes[1], LineFleetSettings{BusCount: 2, Model: sim.BusDoubleDecker, SpeedRange: c.SpeedRange}}}
	m := NewAIBusManager()
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	counts := [sim.BusModelCount]int{}
	var decker, other *AIBus
	for _, b := range m.Buses {
		if b.LineIndex == 0 {
			counts[b.Bus.Model]++
			if b.Bus.Model == sim.BusDoubleDecker {
				decker = b
			}
		} else {
			other = b
		}
		if b.Bus.Capacity != b.Bus.Model.Capacity() {
			t.Fatal("wrong dispatch capacity")
		}
	}
	if counts != [sim.BusModelCount]int{2, 2, 2} {
		t.Fatal(counts)
	}
	decker.Bus.BoardPassengers(60, []int{routes[0].StopIDs[1]})
	pos := decker.Bus.Pos
	plans[0].Settings = c.ChangeModelCount(sim.BusStandard, -1)
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	found := false
	for _, b := range m.Buses {
		if b.ID == decker.ID {
			found = b == decker && b.Bus.Model == sim.BusDoubleDecker && b.Bus.PassengerCount == 60 && b.Bus.Pos == pos
		}
	}
	if !found || m.Buses[len(m.Buses)-1] != other {
		t.Fatal("editing mix replaced existing decker or another service")
	}
	c = c.ChangeModelCount(sim.BusArticulated, 100)
	if c.BusCount != 8 {
		t.Fatal("fleet exceeded per-line cap")
	}
	c = c.ChangeModelCount(sim.BusStandard, -100)
	if c.ModelCounts[0] != 0 || c.BusCount != 6 {
		t.Fatal("invalid count clamp")
	}
}
