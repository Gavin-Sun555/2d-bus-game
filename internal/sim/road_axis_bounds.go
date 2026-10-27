package sim

import "math"

// RoadAxisBounds reuses the validated static map only inside a boundary-query
// batch. Standalone queries read live public roads, including direct edits.
func (w *World) RoadAxisBounds(eastWest bool) (float64, float64) {
	axis := 1
	if eastWest {
		axis = 0
	}
	if w.boundaryDepth > 0 && w.boundaryCache != nil {
		lo, hi := w.boundaryCache.roadMin[axis], w.boundaryCache.roadMax[axis]
		// Existing geometry validation compares floats with ==, so a direct
		// +0/-0 edit is equal there. Scan zero extrema to preserve their bits.
		if lo != 0 && hi != 0 {
			return lo, hi
		}
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, road := range w.Roads {
		a, b := road.P1.Y, road.P2.Y
		if eastWest {
			a, b = road.P1.X, road.P2.X
		}
		lo = math.Min(lo, math.Min(a, b))
		hi = math.Max(hi, math.Max(a, b))
	}
	return lo, hi
}
