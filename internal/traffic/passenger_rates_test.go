package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestFleetUsesConfiguredModelDoorRates(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "door rates", StopIDs: []int{0, 2, 0}}
	m := NewAIBusManager()
	m.ConfigureLineFleets([]LineFleetPlan{{r, LineFleetSettings{BusCount: 2}}}, w, 52, nil, false)
	m.PassengerRates[sim.BusStandard] = sim.PassengerRates{Boarding: .5, Alighting: .25}
	m.PassengerRates[sim.BusDoubleDecker] = sim.PassengerRates{Boarding: 1.5, Alighting: .75}
	st := w.Stops[0]
	parkInitialFleetForTest(m, st, 52)
	st.WaitingPassengers = 8
	for i, ab := range m.Buses {
		ab.Bus.SetModel(sim.BusModel(i))
		ab.Driver.Reset()
		ab.Driver.QueueAtStation(st.ID)
	}
	m.Update(0, nil, w, nil, nil, 52)
	for _, ab := range m.Buses {
		rates := m.PassengerRates[ab.Bus.Model]
		if ab.Bus.PassengerService.BoardingSeconds != rates.Boarding || ab.Bus.PassengerService.AlightingSeconds != rates.Alighting {
			t.Fatal("fleet used fixed timing instead of model configuration")
		}
	}
	for tick := 0; tick < 180; tick++ {
		m.Update(1./60, nil, w, nil, nil, 52)
	}
	if m.Buses[0].Bus.PassengerCount != 6 || m.Buses[1].Bus.PassengerCount != 2 || st.WaitingPassengers != 0 {
		t.Fatalf("model throughput incorrect: %d/%d, waiting=%d", m.Buses[0].Bus.PassengerCount, m.Buses[1].Bus.PassengerCount, st.WaitingPassengers)
	}
}
