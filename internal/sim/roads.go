package sim

import (
	"math"
)

// getCurrentRoadInfo detects the road segment the vehicle is currently on and whether it is a multi-lane or 6-lane avenue
func CurrentRoadInfo(w *World, pos Vec2, heading float64, lw float64) (roadCenter float64, isEW bool, isMultiLane bool, is6Lane bool) {
	if w == nil {
		return 0, true, false, false
	}
	for _, rb := range w.Roundabouts {
		if pos.Distance(rb.Center) <= rb.Radius+15.0 {
			return 0, false, false, false
		}
	}
	cosH := math.Abs(math.Cos(heading))
	sinH := math.Abs(math.Sin(heading))
	busEW := cosH > sinH

	for pass := 0; pass < 2; pass++ {
		targetIsEW := busEW
		if pass == 1 {
			targetIsEW = !busEW
		}
		bestDistance := math.Inf(1)
		var bestRoad *RoadSegment
		for i := range w.Roads {
			r := &w.Roads[i]
			roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
			if roadIsEW != targetIsEW {
				continue
			}
			c := r.P1.Y
			if !roadIsEW {
				c = r.P1.X
			}
			along, across := pos.X, pos.Y
			start, end := r.P1.X, r.P2.X
			if !roadIsEW {
				along, across = pos.Y, pos.X
				start, end = r.P1.Y, r.P2.Y
			}
			minAlong, maxAlong := math.Min(start, end)-30.0, math.Max(start, end)+30.0
			if along < minAlong || along > maxAlong || math.Abs(across-c) > r.GetWidth(lw)*0.5+ShoulderWidth+20.0 {
				continue
			}
			if distance := math.Abs(across - c); distance < bestDistance {
				bestDistance = distance
				bestRoad = r
			}
		}
		if bestRoad != nil {
			r := bestRoad
			roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1
			c := r.P1.Y
			if !roadIsEW {
				c = r.P1.X
			}
			lanes := r.Lanes
			if lanes <= 0 {
				lanes = w.GetRoadLanes(c, roadIsEW, pos)
			}
			is6 := lanes >= 6
			isMulti := lanes >= 4
			return c, roadIsEW, isMulti, is6
		}
	}
	return 0, busEW, false, false
}
