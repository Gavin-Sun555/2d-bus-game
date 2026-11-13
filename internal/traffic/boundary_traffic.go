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

func (tm *TrafficManager) queueCarReplacement(i int) {
	car := tm.Cars[i]
	tm.nextCarID = max(tm.nextCarID, car.ID+1)
	tm.pendingExits = append(tm.pendingExits, car.Pos)
	tm.DespawnCar(i)
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
			tm.queueCarReplacement(i)
		}
	}
	target := min(sim.TrafficCarCount(density), tm.populationCapacity)
	for attempt := 0; attempt < 4 && len(tm.Cars) < MaxTrafficCars; attempt++ {
		replacing := len(tm.pendingExits) > 0
		if !replacing && len(tm.Cars) >= target {
			break
		}
		var car *sim.TrafficCar
		index := len(tm.Cars)
		if replacing {
			car = tm.spawnAtOtherIntersection(tm.pendingExits[0], bus, laneWidth, w)
			// Insert into the active prefix, leaving the inactive reserve intact.
			index = min(index, max(0, sim.TrafficCarCount(density)-len(tm.pendingExits)))
		}
		if car == nil && tm.populationCapacity > 0 {
			car = tm.spawnOnRoad(bus, laneWidth, w, target, replacing)
		}
		if car == nil {
			break
		}
		tm.Cars = append(tm.Cars, nil)
		copy(tm.Cars[index+1:], tm.Cars[index:])
		tm.Cars[index] = car
		if index < len(tm.junctionScratch) {
			copy(tm.junctionScratch[index+1:], tm.junctionScratch[index:len(tm.junctionScratch)-1])
			tm.junctionScratch[index] = carJunctionScratch{}
		}
		if replacing {
			tm.pendingExits = tm.pendingExits[1:]
		}
	}
}

// Fill legacy deficits using the same citywide sites as initial dispatch.
// Limit work per frame and retry when moving traffic blocks the sampled sites.
func (tm *TrafficManager) spawnOnRoad(bus *sim.Bus, lw float64, w *sim.World, target int, replacing bool) *sim.TrafficCar {
	if tm.spawnSites == nil {
		tm.spawnSites = newTrafficSpawnPool(w, lw)
	}
	counts := tm.spawnSites.counts()
	capacity := counts[0] + counts[1] + counts[2]
	if capacity == 0 || (!replacing && len(tm.Cars) >= min(target, capacity)) {
		return nil
	}
	for attempt := 0; attempt < 16; attempt++ {
		class, index := drawTrafficSite(counts)
		site := tm.spawnSites[class][index]
		if bus != nil && site.pos.Distance(bus.Pos) < bus.OverallLength()+160 {
			continue
		}
		clear := true
		if tm.AIBusMgr != nil {
			for _, ab := range tm.AIBusMgr.Buses {
				if ab.Bus != nil && site.pos.Distance(ab.Bus.Pos) < ab.Bus.OverallLength()+160 {
					clear = false
					break
				}
			}
		}
		for _, rail := range w.Railways() {
			if math.Abs(site.pos.Y-rail.Y) <= rail.HalfWidth+80 {
				clear = false
				break
			}
		}
		if !clear {
			continue
		}
		body := sim.OBB{Center: site.pos, Heading: site.h, HalfLen: 48, HalfWid: 14}
		for _, other := range tm.Cars {
			gapLength := other.Length/2 + math.Abs(other.Speed)*.5
			reach := body.HalfLen + body.HalfWid + gapLength + other.Width/2
			delta := other.Pos.Sub(site.pos)
			if delta.Dot(delta) > reach*reach {
				continue
			}
			gap := sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: gapLength, HalfWid: other.Width / 2}
			if hit, _, _ := sim.CheckOBBCollision(body, gap); hit {
				clear = false
				break
			}
		}
		if clear {
			return tm.newPopulationCar(site)
		}
	}
	return nil
}

func (tm *TrafficManager) newPopulationCar(site trafficSpawnSite) *sim.TrafficCar {
	for _, c := range tm.Cars {
		tm.nextCarID = max(tm.nextCarID, c.ID+1)
	}
	car := site.car(tm.nextCarID)
	tm.nextCarID++
	tm.assignCarSpeed(car)
	return car
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
	// Give wider approaches more chances on the first scan. If they are
	// unavailable, allow any clear approach on the fallback scan.
	for pass := 0; pass < 2; pass++ {
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
				if !ok || (pass == 0 && !preferTrafficRoad(road)) {
					continue
				}
				lane := road.FirstGeneralTrafficLane() + sim.RandomIntn(road.GeneralTrafficLanes())
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
				return tm.newPopulationCar(trafficSpawnSite{pos: pos, h: heading, rc: coord, ew: ew, laneIdx: lane})
			}
		}
	}
	return nil
}
