package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestPassengerRatesReachFleetAndSurviveSaveLoad(t *testing.T) {
	g := saveTestGame(t)
	var rates [sim.BusModelCount]sim.PassengerRates
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		rates[m] = sim.PassengerRates{Boarding: .75, Alighting: .25}
	}
	g.Settings.PassengerRates = &rates
	g.syncPassengerRates()
	if g.AIBusMgr.PassengerRates != rates {
		t.Fatal("AI buses did not receive model timing")
	}
	g.Bus.BeginStationService(g.Route.GetCurrentTargetStopID(), g.Route.CurrentIndex, 0, false, nil, g.Settings.ModelPassengerRates(g.Bus.Model))
	g.Bus.AddPassengerService(2, 0)
	g.Bus.DoorsOpen = true
	g.Bus.Update(.25, 0, 0, 0, false, false)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		if g.Settings.ModelPassengerRates(m) != rates[m] {
			t.Fatal("saved model timing lost")
		}
	}
	if g.Bus.PassengerServiceRemaining() != 1.25 {
		t.Fatal("load changed in-progress service time")
	}
	g.syncPassengerRates()
	if g.AIBusMgr.PassengerRates != rates {
		t.Fatal("restored fleet lost timings")
	}
}
