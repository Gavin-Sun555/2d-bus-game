package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPlayerFareAndStationLedger(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Route.StopIDs = []int{0, 1, 0}
	g.Route.CurrentIndex = 0
	g.Settings.Fare.Fixed = 7
	stop := g.World.Stops[0]
	stop.WaitingPassengers = 5
	g.Bus.ClearPassengers()
	g.Bus.Pos = stop.Pos
	g.Bus.Heading = stop.Heading
	g.Bus.Speed = 0
	g.Bus.DoorsOpen = true
	g.Bus.DoorProgress = 1
	cash := g.Cash
	stepExchange := func() {
		g.updatePassengerExchange(false)
		for g.Bus.PassengerServiceRemaining() > 0 {
			g.Bus.Update(1.0, 0, 0, 0, false, false)
			g.updatePassengerExchange(false)
		}
	}
	stepExchange()
	if g.Cash-cash != 35 || g.Operations.Revenue != 35 || g.Operations.Vehicle(0).Boarded != 5 {
		t.Fatal("player fare not credited exactly once")
	}
	stepExchange()
	if g.Cash-cash != 35 {
		t.Fatal("open doors charged the same passengers twice")
	}
	stop.WaitingPassengers = 2
	stepExchange()
	if g.Cash-cash != 49 || g.Operations.Stations[0].Boarded != 7 {
		t.Fatal("continuous boarding not charged")
	}
	g.Settings.Fare.Fixed = 0
	stop.WaitingPassengers = 1
	stepExchange()
	if g.Cash-cash != 49 || g.Operations.Vehicle(0).Boarded != 8 {
		t.Fatal("free boarding accounting failed")
	}
	if g.Bus.Passengers[0].PaidFare != 7 || g.Bus.Passengers[7].PaidFare != 0 {
		t.Fatal("fare edits repriced previous passengers")
	}
	old := g.Operations
	g.Route.StopIDs = []int{0, 2, 0}
	g.ensureOperations()
	if g.Operations == old || g.Operations.Revenue != 0 {
		t.Fatal("new route retained old ledger")
	}
}

func TestOperationsOnlyCountsActiveSimulationTime(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.TrafficFinesEnabled = false
	g.Settings.AIBusCount = 0
	g.AIBusMgr.BuildFleet(g.World, 0, g.Settings.GetLaneWidth(), g.Bus)
	g.trackOperations(0)
	for _, pause := range []string{"statistics", "settings", "planning"} {
		g.OperationsPanel.IsOpen = pause == "statistics"
		g.Settings.IsOpen = pause == "settings"
		g.IsPlanMode = pause == "planning"
		before := g.Operations.Vehicle(0).Seconds
		for tick := 0; tick < 3; tick++ {
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
		}
		if g.Operations.Vehicle(0).Seconds != before {
			t.Fatalf("%s time counted as service", pause)
		}
	}
	g.OperationsPanel.IsOpen = false
	g.Settings.IsOpen = false
	g.IsPlanMode = false
	if err := g.Update(); err != nil {
		t.Fatal(err)
	}
	if g.Operations.Vehicle(0).Seconds != 1.0/60 {
		t.Fatal("stationary active time must count toward service speed")
	}
}

func TestFleetFareRevenueReachesCompanyCash(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.Fare.Fixed = 5
	g.Settings.TrafficFinesEnabled = false
	g.Settings.AIBusCount = 1
	g.AIBusMgr.BuildFleet(g.World, 1, 52, g.Bus)
	ab := g.AIBusMgr.Buses[0]
	stop := g.World.Stops[ab.Route.StopIDs[0]]
	g.Bus.Pos = stop.Pos.Add(sim.Vec2{X: 600})
	g.Bus.DoorsOpen = false
	g.Bus.DoorProgress = 0
	ab.Route.CurrentIndex = 0
	ab.Driver.Reset()
	ab.Bus.Pos = stop.Pos
	ab.Bus.Heading = stop.Heading
	ab.Bus.DoorsOpen = true
	ab.Bus.DoorProgress = 1
	ab.Driver.State = autopilot.AutopilotStateDocked
	ab.Driver.DockedStopID = stop.ID
	ab.Driver.DwellTimer = 2
	ab.LastExchangedStopID = -1
	for _, s := range g.World.Stops {
		s.WaitingPassengers = 0
	}
	stop.WaitingPassengers = 4
	cash := g.Cash
	if err := g.Update(); err != nil {
		t.Fatal(err)
	}
	for ab.Bus.PassengerServiceRemaining() > 0 {
		for _, s := range g.World.Stops {
			if s.ID != stop.ID {
				s.WaitingPassengers = 0
			}
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if g.Cash-cash != 20 || g.Operations.Vehicle(ab.ID).Revenue != 20 || g.Operations.Stations[stop.ID].Boarded != 4 {
		t.Fatalf("AI income not linked: cash%d stats%+v", g.Cash-cash, g.Operations.Vehicle(ab.ID))
	}
	if err := g.Update(); err != nil {
		t.Fatal(err)
	}
	if g.Cash-cash != 20 {
		t.Fatal("AI fare duplicated during dwell")
	}
}
