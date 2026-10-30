package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func serveFleetStop(m *AIBusManager, b *AIBus, w *sim.World, index, waiting int) {
	b.Route.CurrentIndex = index
	st := w.Stops[b.Route.GetCurrentTargetStopID()]
	st.WaitingPassengers = waiting
	b.Bus.Pos = st.FrontBerthPosition()
	b.Bus.Heading = st.Heading
	b.Bus.Speed = 0
	b.Bus.DoorsOpen = true
	b.Bus.DoorProgress = 1
	b.Driver.Reset()
	b.Driver.QueueAtStation(st.ID)
	b.LastExchangedStopID = -1
	m.Update(1./60, nil, w, nil, nil, 52)
	for len(m.Buses) > 0 && b.Bus.PassengerServiceRemaining() > 0 {
		m.Update(1.0, nil, w, nil, nil, 52)
	}
}

func TestFleetReplacesOnlyOnTerminalArrivalAndBoardsNewCapacity(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "test", StopIDs: []int{0, 2, 10, 0}}
	c := LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	m := NewAIBusManager()
	plans := []LineFleetPlan{{r, c}}
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	b := m.Buses[0]
	b.Bus.Durability = 60
	c.Model = sim.BusDoubleDecker
	plans[0].Settings = c
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	if b.Bus.Model != sim.BusStandard || b.Bus.PendingModel == nil {
		t.Fatal("upgraded mid-route")
	}
	serveFleetStop(m, b, w, 1, 0)
	if b.Bus.Model != sim.BusStandard || b.Bus.Durability != 60 {
		t.Fatal("empty intermediate stop replaced vehicle")
	}
	serveFleetStop(m, b, w, 3, 100)
	if len(m.Buses) != 1 || m.Buses[0] != b || b.Bus.Model != sim.BusDoubleDecker || b.Bus.PendingModel != nil || b.Bus.PassengerCount != 80 || b.Bus.Durability != 100 || w.Stops[0].WaitingPassengers != 20 {
		t.Fatalf("terminal replacement model%d passengers%d", b.Bus.Model, b.Bus.PassengerCount)
	}
}

func TestMixReductionWaitsForTerminalAndCanBecomeReplacement(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "test", StopIDs: []int{0, 2, 10, 0}}
	c := LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	plans := []LineFleetPlan{{r, c}}
	m := NewAIBusManager()
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	b := m.Buses[0]
	plans[0].Settings = c.ChangeModelCount(sim.BusStandard, -1)
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	if len(m.Buses) != 1 || !b.Retiring {
		t.Fatal("removed vehicle mid-route")
	}
	serveFleetStop(m, b, w, 1, 0)
	if len(m.Buses) != 1 {
		t.Fatal("retired at intermediate stop")
	}
	plans[0].Settings = plans[0].Settings.ChangeModelCount(sim.BusArticulated, 1)
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	if len(m.Buses) != 1 || m.Buses[0] != b || b.Retiring || b.Bus.PendingModel == nil || *b.Bus.PendingModel != sim.BusArticulated || b.Bus.Model != sim.BusStandard {
		t.Fatal("mix edit did not reuse pending departure as replacement")
	}
	serveFleetStop(m, b, w, 3, 0)
	if b.Bus.Model != sim.BusArticulated {
		t.Fatal("terminal did not hand over replacement")
	}
	plans[0].Settings = plans[0].Settings.ChangeModelCount(sim.BusArticulated, -1)
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	serveFleetStop(m, b, w, 1, 0)
	if len(m.Buses) != 1 {
		t.Fatal("retired too early")
	}
	serveFleetStop(m, b, w, 3, 100)
	if len(m.Buses) != 0 || m.BusCount != 0 || b.Route.TotalTrips != 1 || w.Stops[0].WaitingPassengers != 100 {
		t.Fatal("retiring vehicle boarded or did not disappear")
	}
}

func TestRetirementRecordsCompletedLapBeforeRemovingVehicle(t *testing.T) {
	w := sim.NewWorld(52)
	r := &sim.Route{Name: "test", StopIDs: []int{0, 2, 10, 0}}
	c := LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	plans := []LineFleetPlan{{r, c}}
	m := NewAIBusManager()
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	b := m.Buses[0]
	b.Route.CurrentIndex = 0 // This fixture measures a complete terminal-to-terminal lap.
	b.Bus.Pos = w.Stops[0].FrontBerthPosition()
	m.Operations = sim.NewTransitOperations(w, b.Route, 52, sim.DefaultFarePolicy())
	m.Operations.Track(b.ID, b.Bus, b.Route, 0)
	b.Bus.Speed = 100
	seconds := m.Operations.Distances.TotalKM * 1000 / sim.MetersPerPixel / b.Bus.Speed
	m.Operations.Track(b.ID, b.Bus, b.Route, seconds)
	m.Operations.Vehicle(b.ID).Revenue = 40
	m.Operations.Revenue = 40
	plans[0].Settings = c.ChangeModelCount(sim.BusStandard, -1)
	m.ConfigureLineFleets(plans, w, 52, nil, false)
	serveFleetStop(m, b, w, 3, 0)
	v := m.Operations.Vehicle(b.ID)
	if len(m.Buses) != 0 || v.CompletedLaps != 1 || v.Revenue != 40 || m.Operations.Revenue != 40 {
		t.Fatalf("retirement lost completed service: laps%d revenue%d", v.CompletedLaps, v.Revenue)
	}
}
