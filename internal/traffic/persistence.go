package traffic

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

// PopulationSnapshot retains boundary replacement queues and stable car IDs.
type PopulationSnapshot struct {
	PendingExits []sim.Vec2
	NextCarID    int
}

func (tm *TrafficManager) PopulationSnapshot() PopulationSnapshot {
	return PopulationSnapshot{tm.pendingExits, tm.nextCarID}
}
func (tm *TrafficManager) RestorePopulation(s PopulationSnapshot) {
	tm.pendingExits = s.PendingExits
	tm.nextCarID = s.NextCarID
}
