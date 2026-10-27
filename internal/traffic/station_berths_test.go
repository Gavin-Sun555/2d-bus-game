package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestTwoBerthsBoardTogetherAndSharePassengerPool(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "berths", StopIDs: []int{0, 2, 0}}
	m := NewAIBusManager()
	m.ConfigureLineFleets([]LineFleetPlan{{r, LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}}}, w, 52, nil, false)
	st := w.Stops[0]
	st.WaitingPassengers = 100
	for i, ab := range m.Buses {
		ab.Bus.SetModel(sim.BusModel(i))
		ab.Driver.Reset()
		ab.Driver.QueueAtStation(st.ID)
	}
	m.Update(1./60, nil, w, nil, nil, 52)
	if !m.Buses[0].Bus.DoorsOpen || !m.Buses[1].Bus.DoorsOpen || m.Buses[0].Driver.State != autopilot.AutopilotStateDocked || m.Buses[1].Driver.State != autopilot.AutopilotStateDocked {
		t.Fatal("two short vehicles did not dock together")
	}
	if m.Buses[0].Bus.PassengerCount != 45 || m.Buses[1].Bus.PassengerCount != 55 || st.WaitingPassengers != 0 {
		t.Fatal("passengers duplicated/lost or rear did not board", m.Buses[0].Bus.PassengerCount, m.Buses[1].Bus.PassengerCount, st.WaitingPassengers)
	}
}
func TestLongReplacementDoesNotTrapRearBerth(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "berths", StopIDs: []int{0, 2, 0}}
	m := NewAIBusManager()
	m.ConfigureLineFleets([]LineFleetPlan{{r, LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}}}, w, 52, nil, false)
	st := w.Stops[0]
	front, rear := m.Buses[0], m.Buses[1]
	front.Route.CurrentIndex = 2
	front.Bus.RequestModel(sim.BusArticulated)
	for _, ab := range m.Buses {
		ab.Driver.Reset()
		ab.Driver.QueueAtStation(st.ID)
		ab.Bus.DoorsOpen = true
		ab.Bus.DoorProgress = 1
	}
	m.Update(1./60, nil, w, nil, nil, 52)
	if front.Bus.Model != sim.BusStandard || front.Bus.PendingModel == nil || front.Bus.TerminalReplacementHold {
		t.Fatal("replacement trapped the rear short bus or expanded inside it")
	}
	if hit, _, _ := sim.CheckBusesCollision(front.Bus, rear.Bus); hit {
		t.Fatal("terminal vehicle overlap")
	}
	// Once the rear platform is clear, the pending request still completes.
	rear.Bus.Pos = rear.Bus.Pos.Add(sim.Vec2{X: 2000, Y: 2000})
	m.Update(1./60, nil, w, nil, nil, 52)
	if front.Bus.Model != sim.BusArticulated || front.Bus.Pos.Distance(st.FrontBerthPosition()) > 1 {
		t.Fatal("pending long vehicle did not replace at front berth")
	}
}
