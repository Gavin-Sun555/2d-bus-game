package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestPerLineFleetCountSpeedAndIsolation(t *testing.T) {
	w := sim.NewWorld(52)
	routes := sim.DefaultTransitLines(w)
	m := NewAIBusManager()
	m.Operations = sim.NewTransitOperations(w, routes[0], 52, sim.DefaultFarePolicy())
	plans := []LineFleetPlan{{routes[0], LineFleetSettings{BusCount: 3, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 30}}}, {routes[1], LineFleetSettings{BusCount: 5, SpeedRange: sim.SpeedRange{MinKPH: 80, MaxKPH: 80}}}, {routes[2], LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}}}}
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	if len(m.Buses) != 10 {
		t.Fatalf("total=%d; per-line counts must exceed old global cap", len(m.Buses))
	}
	counts := map[int]int{}
	var saved *AIBus
	for _, b := range m.Buses {
		counts[b.LineIndex]++
		if b.LineIndex == 0 {
			if b.MaxSpeedKPH != 30 {
				t.Fatal("line0 speed")
			}
			saved = b
		}
		if b.LineIndex == 1 && b.MaxSpeedKPH != 80 {
			t.Fatal("line1 speed")
		}
		if b.LineIndex == 1 {
			b.Bus.Speed = sim.SpeedKPH(70)
		}
	}
	if counts[0] != 3 || counts[1] != 5 || counts[2] != 2 {
		t.Fatal(counts)
	}
	saved.Bus.Pos.X += 150
	saved.Bus.PassengerCount = 7
	pos := saved.Bus.Pos
	speed := saved.MaxSpeedKPH
	plans[1].Settings = LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 50, MaxKPH: 50}}
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	retiring := 0
	for _, b := range m.Buses {
		if b.Retiring {
			retiring++
		}
	}
	if len(m.Buses) != 10 || retiring != 4 {
		t.Fatal("reduction should retire buses only after this lap", len(m.Buses), retiring)
	}
	found := false
	for _, b := range m.Buses {
		if b.ID == saved.ID {
			found = true
			if b != saved || b.Bus.Pos != pos || b.Bus.PassengerCount != 7 || b.MaxSpeedKPH != speed {
				t.Fatal("changed other line")
			}
		}
		if b.LineIndex == 1 {
			cap := sim.SpeedKPH(50)
			if b.MaxSpeedKPH != 50 || b.Bus.MaxSpeedFwd != cap || b.Bus.Speed != cap || b.Driver.CruiseSpeedLimit != cap ||
				m.Operations.Vehicle(b.ID).MaxSpeedKPH != 50 {
				t.Fatalf("range update not immediately applied to bus %d (retiring %v)", b.ID, b.Retiring)
			}
		}
	}
	if !found {
		t.Fatal("removed wrong line bus")
	}
	// Disabled services retain stable slots when re-enabled, while departed buses
	// and on-board passengers on remaining services are untouched.
	m.ConfigureLineFleets(plans[:1], w, 52, nil, false)
	if len(m.Buses) != 3 {
		t.Fatal("disabled line still dispatched")
	}
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	if len(m.Buses) != 6 {
		t.Fatal("re-enable failed")
	}
}

func TestSameRouteResetKeepsAssignedSpeed(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "test", StopIDs: []int{0, 2, 10, 0}}
	speedRange := sim.SpeedRange{MinKPH: 30, MaxKPH: 120}
	plans := []LineFleetPlan{{r, LineFleetSettings{BusCount: 4, SpeedRange: speedRange}}}
	m := NewAIBusManager()
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	speeds := make(map[int]int)
	for _, b := range m.Buses {
		speeds[b.ID] = b.MaxSpeedKPH
	}
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	m.ConfigureLineFleets(plans, w, 52, nil, true)
	for _, b := range m.Buses {
		if b.MaxSpeedKPH != speeds[b.ID] || b.Bus.MaxSpeedFwd != sim.SpeedKPH(speeds[b.ID]) {
			t.Fatalf("reset rerolled bus %d from %d to %d", b.ID, speeds[b.ID], b.MaxSpeedKPH)
		}
	}
}

func TestMaximumPerLineFleetsSpawnWithoutOverlap(t *testing.T) {
	w := sim.NewWorld(52)
	routes := sim.DefaultTransitLines(w)
	m := NewAIBusManager()
	player := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, w.Stops[0].Heading)
	plans := make([]LineFleetPlan, 6)
	for i, r := range routes {
		plans[i] = LineFleetPlan{r, LineFleetSettings{BusCount: MaxBusesPerLine, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}}}
	}
	m.ConfigureLineFleets(plans, w, 52, player, false)
	if len(m.Buses) != MaxFleetBuses {
		t.Fatalf("max fleet=%d", len(m.Buses))
	}
	body := func(b *sim.Bus) sim.OBB {
		return sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}
	}
	for i, b := range m.Buses {
		if hit, _, _ := sim.CheckOBBCollision(body(b.Bus), body(player)); hit {
			t.Fatalf("bus%d overlaps player", b.ID)
		}
		for _, other := range m.Buses[i+1:] {
			if hit, _, _ := sim.CheckOBBCollision(body(b.Bus), body(other.Bus)); hit {
				t.Fatalf("buses%d/%d overlap", b.ID, other.ID)
			}
		}
	}
}
