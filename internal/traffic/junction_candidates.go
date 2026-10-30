package traffic

import (
	"math"
	"math/bits"
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
	tm.junctionCandidates = orderedJunctionCandidates(ids, f.activeCount)
	return tm.junctionCandidates, true
}

// Keep the exact ascending slot order used by Sort + Compact. Normal city
// populations fit a bounded stack bitset; small queries and abnormal IDs keep
// the original implementation. No traffic state or query result is cached.
func orderedJunctionCandidates(ids []int, count int) []int {
	const maxSlots = 1024
	if len(ids) < 24 || count < 0 || count > maxSlots {
		slices.Sort(ids)
		return slices.Compact(ids)
	}
	var words [maxSlots / 64]uint64
	lo, hi := len(words), 0
	for _, id := range ids {
		if uint(id) >= uint(count) {
			slices.Sort(ids)
			return slices.Compact(ids)
		}
		word := id / 64
		words[word] |= uint64(1) << uint(id%64)
		lo, hi = min(lo, word), max(hi, word)
	}
	result := ids[:0]
	for word := lo; word <= hi; word++ {
		value := words[word]
		for value != 0 {
			result = append(result, word*64+bits.TrailingZeros64(value))
			value &= value - 1
		}
	}
	return result
}
