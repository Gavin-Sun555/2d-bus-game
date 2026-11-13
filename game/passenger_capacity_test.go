package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestPassengerCapacityUpdatesExistingAndNewBuses(t *testing.T) {
	g := saveTestGame(t)
	fleet := traffic.LineFleetSettings{
		MixedModels: true,
		ModelCounts: [sim.BusModelCount]int{1, 1, 1},
	}
	g.SetLineFleet(0, fleet)
	existing := append([]*traffic.AIBus(nil), g.AIBusMgr.Buses...)
	want := [sim.BusModelCount]int{67, 120, 127}
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		if got := g.Settings.ModelPassengerCapacity(model); got != model.Capacity() {
			t.Fatalf("model %v default capacity = %d, want %d", model, got, model.Capacity())
		}
		g.Settings.AdjustPassengerOverload(model, 100)
	}
	g.syncPassengerCapacities()
	for _, vehicle := range existing {
		if got := vehicle.Bus.Capacity; got != want[vehicle.Bus.Model] {
			t.Fatalf("existing AI capacity = %d, want %d", got, want[vehicle.Bus.Model])
		}
	}

	fleet.ModelCounts[sim.BusDoubleDecker]++
	g.SetLineFleet(0, fleet)
	doubleDeckers := 0
	for _, vehicle := range g.AIBusMgr.Buses {
		if vehicle.Bus.Model == sim.BusDoubleDecker {
			doubleDeckers++
			if vehicle.Bus.Capacity != 120 {
				t.Fatalf("double-decker capacity = %d, want 120", vehicle.Bus.Capacity)
			}
		}
	}
	if doubleDeckers != 2 {
		t.Fatalf("expected two double-deckers after dispatch, got %d", doubleDeckers)
	}

	destinations := []int{g.Route.StopIDs[1]}
	if boarded := g.Bus.BoardPassengers(100, destinations); boarded != 67 {
		t.Fatalf("player boarded %d, want 67", boarded)
	}
	g.Settings.AdjustPassengerOverload(sim.BusStandard, -100)
	g.syncPassengerCapacities()
	if g.Bus.Capacity != 45 || g.Bus.PassengerCount != 67 || g.Bus.BoardPassengers(1, destinations) != 0 {
		t.Fatal("lowering capacity must preserve riders and prevent additional boarding")
	}

	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Capacity != 45 || g.Bus.PassengerCount != 67 || g.Settings.ModelPassengerCapacity(sim.BusArticulated) != 127 {
		t.Fatal("save and load did not preserve capacity settings and onboard riders")
	}
}

func TestLegacyPassengerCapacitySaveLoadsAsPercent(t *testing.T) {
	g := saveTestGame(t)
	g.Settings.PassengerCapacities = &[sim.BusModelCount]int{56, 100, 106}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Settings.PassengerCapacities != nil || g.Settings.ModelPassengerOverload(sim.BusStandard) != 30 || g.Bus.Capacity != 58 {
		t.Fatal("old capacity save did not migrate to percentage setting")
	}
}

func TestPassengerOverloadSaveValidation(t *testing.T) {
	g := saveTestGame(t)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	s, err := readSave(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	for _, percent := range []int{-10, 0, 10, 15, 50, 60} {
		s.Settings.PassengerOverloadPercents = &[sim.BusModelCount]int{percent, 0, 0}
		valid := percent >= 0 && percent <= 50 && percent%10 == 0
		if err = s.validate(); (err == nil) != valid {
			t.Fatalf("percentage %d: validation error %v", percent, err)
		}
	}
}
