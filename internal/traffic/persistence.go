package traffic

import (
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// PopulationSnapshot retains replacement requests, capacity and stable car IDs.
type PopulationSnapshot struct {
	PendingExits []sim.Vec2
	NextCarID    int
	Capacity     int `json:",omitempty"`
}

func (tm *TrafficManager) PopulationSnapshot() PopulationSnapshot {
	// Copy: the live queue keeps appending, which must not rewrite a snapshot
	// that is being saved or restored into another manager.
	return PopulationSnapshot{PendingExits: slices.Clone(tm.pendingExits), NextCarID: tm.nextCarID, Capacity: tm.populationCapacity}
}
func (tm *TrafficManager) RestorePopulation(s PopulationSnapshot) {
	tm.pendingExits = slices.Clone(s.PendingExits)
	tm.nextCarID = s.NextCarID
	capacity := s.Capacity
	if capacity <= 0 {
		// Older saves tracked surviving cars but had no population target.
		capacity = MaxTrafficCars
	}
	tm.populationCapacity = min(MaxTrafficCars, max(capacity, len(tm.Cars)+len(tm.pendingExits)))
	tm.spawnSites = nil
}
