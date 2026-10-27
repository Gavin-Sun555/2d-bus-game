package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Only the travel axis matters: lane offsets can lie outside the road-center bbox.
func outsideRoadBounds(car *sim.TrafficCar, w *sim.World, margin float64) bool {
	if w == nil || len(w.Roads) == 0 || car.InRoundabout {
		return false
	}
	ew := car.IsEW
	if turn := car.JunctionTurn; turn != nil {
		// An outward turn may leave on a different axis than its entry.
		// Remove only once the whole body clears that edge; do not wait
		// for the virtual path tail or keep navigating along the entry axis.
		if turn.LaneChange || !isMapExitAhead(w, turn.Center, turn.ExitHeading) {
			return false
		}
		ew = turn.ExitEW
	}
	lo, hi := roadAxisBounds(w, ew)
	coordinate := car.Pos.Y
	if ew {
		coordinate = car.Pos.X
	}
	return coordinate < lo-margin || coordinate > hi+margin
}

// Use the same extents for exit admission and rear-body removal. An internal
// dead end must still merge/turn; an outward-facing edge is an open map exit.
func roadAxisBounds(w *sim.World, ew bool) (float64, float64) {
	return w.RoadAxisBounds(ew)
}

func isMapExitAhead(w *sim.World, center sim.Vec2, heading float64) bool {
	if w == nil || len(w.Roads) == 0 {
		return false
	}
	ew := math.Abs(math.Cos(heading)) > .5
	lo, hi := roadAxisBounds(w, ew)
	coordinate, direction := center.Y, math.Sin(heading)
	if ew {
		coordinate, direction = center.X, math.Cos(heading)
	}
	if (direction > 0 && math.Abs(coordinate-hi) > 1) || (direction < 0 && math.Abs(coordinate-lo) > 1) {
		return false
	}
	return hasRoadInDirection(w, center, heading+math.Pi)
}

func (tm *TrafficManager) updateBoundaryTraffic(bus *sim.Bus, density int, laneWidth float64, w *sim.World) {
	if w == nil {
		return
	}
	active := len(tm.GetActiveCars(density))
	for i := active - 1; i >= 0; i-- {
		car := tm.Cars[i]
		half := (math.Abs(math.Cos(car.Heading))*car.Length + math.Abs(math.Sin(car.Heading))*car.Width) / 2
		ew := car.IsEW
		if turn := car.JunctionTurn; turn != nil && !turn.LaneChange && isMapExitAhead(w, turn.Center, turn.ExitHeading) {
			ew = turn.ExitEW
		}
		if !ew {
			half = (math.Abs(math.Sin(car.Heading))*car.Length + math.Abs(math.Cos(car.Heading))*car.Width) / 2
		}
		if outsideRoadBounds(car, w, half) {
			tm.pendingExits = append(tm.pendingExits, car.Pos)
			tm.DespawnCar(i)
		}
	}
	for attempt := 0; attempt < 4 && len(tm.pendingExits) > 0; attempt++ {
		car := tm.spawnAtOtherIntersection(tm.pendingExits[0], bus, laneWidth, w)
		if car == nil {
			break
		}
		// Insert into the active prefix, leaving the inactive density reserve intact.
		index := min(len(tm.Cars), max(0, sim.TrafficCarCount(density)-len(tm.pendingExits)))
		tm.Cars = append(tm.Cars, nil)
		copy(tm.Cars[index+1:], tm.Cars[index:])
		tm.Cars[index] = car
		tm.pendingExits = tm.pendingExits[1:]
	}
}

func (tm *TrafficManager) spawnAtOtherIntersection(exit sim.Vec2, bus *sim.Bus, lw float64, w *sim.World) *sim.TrafficCar {
	if len(w.Intersections) < 2 {
		return nil
	}
	excluded := 0
	for i, p := range w.Intersections {
		if p.Distance(exit) < w.Intersections[excluded].Distance(exit) {
			excluded = i
		}
	}
	start := sim.RandomIntn(len(w.Intersections))
	for n := 0; n < len(w.Intersections); n++ {
		idx := (start + n) % len(w.Intersections)
		center := w.Intersections[idx]
		if idx == excluded || w.IsAtRoundabout(center) {
			continue
		}
		first := sim.RandomIntn(4)
		for j := 0; j < 4; j++ {
			heading := float64((first+j)%4) * math.Pi / 2
			forward := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			ew := math.Abs(forward.X) > .5
			coord := center.X
			g := w.JunctionAt(center, lw)
			distance := g.HalfY + sim.JunctionStopInset + 100
			if ew {
				coord = center.Y
				distance = g.HalfX + sim.JunctionStopInset + 100
			}
			approach := center.Sub(forward.Mul(distance))
			road, ok := w.RoadAt(coord, ew, approach)
			if !ok {
				continue
			}
			lane := sim.RandomIntn(road.GeneralTrafficLanes())
			pos := approach.Add(right.Mul((float64(lane) + .5) * lw))
			clear := bus == nil || pos.Distance(bus.Pos) > bus.Length/2+160
			for _, c := range tm.Cars {
				if pos.Distance(c.Pos) < 160+math.Abs(c.Speed)*.5 {
					clear = false
					break
				}
			}
			if tm.AIBusMgr != nil {
				for _, ab := range tm.AIBusMgr.Buses {
					if ab.Bus != nil && pos.Distance(ab.Bus.Pos) < ab.Bus.Length/2+180 {
						clear = false
					}
				}
			}
			for _, stop := range w.Stops {
				if pos.Distance(stop.Pos) < 160 {
					clear = false
				}
			}
			for _, rb := range w.Roundabouts {
				if pos.Distance(rb.Center) < rb.YieldLineDistance()+60 {
					clear = false
				}
			}
			if !clear {
				continue
			}
			for _, c := range tm.Cars {
				tm.nextCarID = max(tm.nextCarID, c.ID+1)
			}
			id := tm.nextCarID
			tm.nextCarID++
			car := &sim.TrafficCar{ID: id, Pos: pos, Heading: heading, Length: 46, Width: 22, Color: carPalette[id%len(carPalette)], RoadCenter: coord, IsEW: ew, LaneIndex: lane}
			tm.assignCarSpeed(car)
			return car
		}
	}
	return nil
}
