package traffic

import (
	"math"
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Spatial buckets follow the original sequential moves. A monotone horizon
// bound covers both already-updated and not-yet-updated cars. Committed turns
// are included independently of position (including unusual legacy saves).
func (tm *TrafficManager) prepareJunctionCandidates(count int) {
	tm.junctionMaxHorizon = 0
	tm.junctionCommitted = tm.junctionCommitted[:0]
	if cap(tm.junctionRegistered) < count {
		tm.junctionRegistered = make([]bool, count)
	} else {
		tm.junctionRegistered = tm.junctionRegistered[:count]
		clear(tm.junctionRegistered)
	}
	for i, car := range tm.Cars[:count] {
		tm.observeJunctionCandidate(i, car)
	}
}

func (tm *TrafficManager) observeJunctionCandidate(i int, car *sim.TrafficCar) {
	horizon := car.Length/2 + math.Max(180, car.Speed*car.Speed/(2*220)+60)
	tm.junctionMaxHorizon = math.Max(tm.junctionMaxHorizon, horizon)
	if car.JunctionTurn != nil && !tm.junctionRegistered[i] {
		tm.junctionRegistered[i] = true
		tm.junctionCommitted = append(tm.junctionCommitted, i)
	}
}

func (tm *TrafficManager) junctionScanCandidates(f carFrame, inter *sim.TrafficIntersection, approachReach, exitReach float64) ([]int, bool) {
	if !tm.spatialActive || tm.disableJunctionCandidates || tm.disableJunctionBroadPhase || f.activeCount != len(tm.spatial.cells) {
		return nil, false
	}
	radius := math.Max(approachReach+tm.junctionMaxHorizon, exitReach)
	// Abnormal/custom states must not create an unbounded grid enumeration.
	if math.IsNaN(radius) || math.IsInf(radius, 0) || radius > 8192 || radius < 0 {
		return nil, false
	}
	radius += 1e-7 * (1 + radius)
	ids := tm.spatial.queryUnsorted(inter.Center, radius, tm.junctionCandidates)
	ids = append(ids, tm.junctionCommitted...)
	slices.Sort(ids)
	tm.junctionCandidates = slices.Compact(ids)
	return tm.junctionCandidates, true
}
