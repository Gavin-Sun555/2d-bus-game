package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestDespawnKeepsJunctionScratchWithVehicle(t *testing.T) {
	cars := []*sim.TrafficCar{{ID: 1}, {ID: 2}, {ID: 3}, {ID: 4}}
	corridor := &sim.JunctionCorridor{Bodies: []sim.OBB{{HalfLen: 10}}}
	tm := &TrafficManager{Cars: cars, junctionScratch: []carJunctionScratch{{owner: cars[0]}, {owner: cars[1]}, {owner: cars[2], corridor: corridor, valid: true}}}
	retained := tm.Cars[2]
	tm.DespawnCar(1)
	if tm.Cars[1] != retained || tm.junctionScratch[1].owner != retained || tm.junctionScratch[1].corridor != corridor || !tm.junctionScratch[1].valid {
		t.Fatal("scratch did not follow shifted vehicle")
	}
	if tm.junctionScratch[2].owner != nil {
		t.Fatal("new active slot retained a previous owner")
	}
	tm.DespawnCar(2)
	if tm.junctionScratch[2].owner != nil {
		t.Fatal("tail scratch retained")
	}
	// Direct external reorder still cannot borrow another car's cached state.
	tm.Cars[0], tm.Cars[1] = tm.Cars[1], tm.Cars[0]
	s := tm.scratchForCar(tm.Cars[0], 0)
	if s.owner != tm.Cars[0] || s.valid || s.corridor != nil {
		t.Fatal("owner mismatch reused another vehicle's cache")
	}
	if corridor.Bodies[0].HalfLen != 10 {
		t.Fatal("published geometry changed")
	}
}
