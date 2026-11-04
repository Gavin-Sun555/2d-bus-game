package sim

import "math"

// Configure after all authored and generated stops exist, before bay clearance.
// Station IDs and route membership remain stable across the two map variants.
func (w *World) configureCenterBRT(lw float64) {
	w.CenterBRT = w.CenterBRT && w.HasBRTEligibleRoads()
	for i := range w.Roads {
		r := &w.Roads[i]
		r.MedianBusLane = w.CenterBRT && r.HasBusLane() && r.LaneCount() == 8
	}
	for _, s := range w.Stops {
		ew := math.Abs(math.Cos(s.Heading)) > .5
		coord := s.RoadCenter.X
		if ew {
			coord = s.RoadCenter.Y
		}
		road, ok := w.RoadAt(coord, ew, s.RoadCenter)
		s.CenterBRT = ok && road.HasMedianBusLane()
		s.UpdateGeometry(lw)
	}
}
