package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func (ad *AutopilotDriver) junctionAdmissionSpeed(f autopilotFrame) float64 {
	corridor := f.bus.JunctionCorridor
	if f.world == nil || f.tls == nil || corridor == nil || corridor.Remaining <= 0 {
		return math.Inf(1)
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	var inter *sim.TrafficIntersection
	for _, it := range f.tls.Intersections {
		if it.Center.Distance(corridor.Center) < 1 {
			inter = it
			break
		}
	}
	if inter == nil {
		return math.Inf(1)
	}
	gap := corridor.Center.Sub(f.bus.Pos).Dot(u) - inter.GetStopLineDistForHeading(heading, f.laneWidth) - f.bus.Length/2
	// A vehicle already admitted clears the box instead of waiting at its exit.
	if gap <= 0 {
		return math.Inf(1)
	}
	g := f.world.JunctionAt(corridor.Center, f.laneWidth)
	for _, ob := range f.obstacles {
		// A straight leader may be safe to follow but leave no place for
		// the whole articulated bus beyond the box.
		if f.bus.IsArticulated() && exitQueueBlocksObstacle(corridor, g, f.bus, ob) {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		if _, ok := sim.JunctionLeaderGap(corridor, ob.JunctionCorridor, body, f.bus.Length, f.bus.Width); ok {
			continue
		}
		conflict := false
		waitingAtRed := waitingOutsideRedEntry(ob, inter, g)
		if !waitingAtRed && ob.JunctionCorridor != nil && ob.JunctionCorridor.Remaining > 0 && ob.JunctionCorridor.Entered && ob.JunctionCorridor.Center.Distance(corridor.Center) < 1 {
			var otherExit float64
			if n := len(ob.JunctionCorridor.Bodies); n >= 1 {
				otherExit = ob.JunctionCorridor.Bodies[n-1].Heading
			}
			sameLeader := ad.sameDirectionMovingLeader(corridor, f, heading, u, ob, ob.JunctionCorridor.EntryHeading, otherExit)
			if !sameLeader {
				if len(corridor.Bodies) > 1 && corridorTurnAngle(corridor) > .5 &&
					math.Cos(otherExit-ob.JunctionCorridor.EntryHeading) > .98 &&
					math.Cos(ob.JunctionCorridor.EntryHeading-heading) < -.9 &&
					ob.Pos.Sub(corridor.Center).Dot(sim.UnitFromAngle(ob.Heading)) > 0 {
					boxIdx := 0
					for boxIdx < len(corridor.Bodies) && !f.world.InsideJunction(corridor.Bodies[boxIdx].Center, f.laneWidth, 0) {
						boxIdx++
					}
					conflict = boxIdx < len(corridor.Bodies) && !sim.CorridorsSeparated(corridor.Bodies[boxIdx:], ob.JunctionCorridor.Bodies)
				} else {
					conflict = !sim.SeparatedJunctionCorridors(corridor, ob.JunctionCorridor)
				}
			}
		}
		if !conflict && !corridor.Entered && sim.UTurnPrecedesEntry(ob.JunctionCorridor, corridor, inter) {
			conflict = !sim.SeparatedJunctionCorridors(corridor, ob.JunctionCorridor)
		}
		if !conflict && !corridor.Entered && len(corridor.Bodies) > 1 &&
			math.Abs(corridorTurnAngle(corridor)) >= 2.8 &&
			opposingGreenBeforeEntry(corridor, ob, inter, f.laneWidth) {
			conflict = !sim.SeparatedJunctionCorridors(corridor, ob.JunctionCorridor)
		}
		if !waitingAtRed && ob.JunctionTurn != nil && !ob.JunctionTurn.LaneChange && ob.JunctionTurn.Center.Distance(corridor.Center) < 1 {
			points := ob.JunctionTurn.Points[ob.JunctionTurn.Segment:]
			sameLeader := ad.sameDirectionMovingLeader(corridor, f, heading, u, ob, ob.JunctionTurn.EntryHeading, ob.JunctionTurn.ExitHeading)
			if !conflict && !sameLeader {
				var storage []sim.OBB
				if ad != nil {
					storage = ad.geometryScratch().admission[:0]
				}
				otherBodies := sim.AppendPathBodies(storage, points, ob.Length, ob.Width)
				if ad != nil && cap(otherBodies) <= maxTemporaryBodies {
					ad.temporaryGeometry.admission = otherBodies
				}
				ownBodies := corridor.Bodies
				if len(corridor.Bodies) > 1 && corridorTurnAngle(corridor) > .5 &&
					ob.JunctionTurn.Straight && math.Cos(ob.JunctionTurn.EntryHeading-heading) < -.9 &&
					ob.Pos.Sub(corridor.Center).Dot(sim.UnitFromAngle(ob.Heading)) > 0 {
					boxIdx := 0
					for boxIdx < len(ownBodies) && !f.world.InsideJunction(ownBodies[boxIdx].Center, f.laneWidth, 0) {
						boxIdx++
					}
					ownBodies = ownBodies[boxIdx:]
				}
				conflict = len(ownBodies) > 0 && !sim.CorridorsSeparated(ownBodies, otherBodies)
			}
		}
		if conflict {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
		if !f.bus.IsArticulated() && exitQueueBlocksObstacle(corridor, g, f.bus, ob) {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
	}
	return math.Inf(1)
}

// A moving vehicle ahead in the same approach-and-exit stream (including an
// adjacent-lane turner whose exit merges downstream once past our nose) is a
// leader to follow via headway/sweep checks, not cross-traffic.
func (ad *AutopilotDriver) sameDirectionMovingLeader(
	corridor *sim.JunctionCorridor,
	f autopilotFrame,
	heading float64,
	u sim.Vec2,
	ob VehicleObstacle,
	otherEntry, otherExit float64,
) bool {
	if ob.Speed <= 15 || corridor == nil || len(corridor.Bodies) < 2 {
		return false
	}
	delta := ob.Pos.Sub(f.bus.Pos)
	along := delta.Dot(u)
	if along <= 0 {
		return false
	}
	if math.Cos(ob.Heading-heading) > .98 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.35 {
		return true
	}
	rawExit := corridor.Bodies[len(corridor.Bodies)-1].Heading
	ourExit := rawExit
	roadExit := approachRoadHeading(rawExit)
	postJunctionTaper := math.Cos(heading-roadExit) > .98 && math.Cos(rawExit-roadExit) > .95 && math.Cos(rawExit-roadExit) < .995
	if postJunctionTaper {
		ourExit = roadExit
	}
	outU := sim.UnitFromAngle(ourExit)
	outR := sim.Vec2{X: -outU.Y, Y: outU.X}
	if along > f.bus.Length/2+ob.Length/2+20 &&
		ob.Pos.Sub(corridor.Center).Dot(outU) > 0 &&
		math.Cos(ob.Heading-ourExit) > .94 &&
		math.Cos(otherExit-ourExit) > .98 &&
		math.Abs(ob.Pos.Sub(corridor.Bodies[len(corridor.Bodies)-1].Center).Dot(outR)) < f.laneWidth*1.2 {
		return true
	}
	turning := math.Cos(ourExit-heading) < .98
	if !turning && (ad == nil || ad.fallbackJunction == nil) {
		if ob.JunctionTurn == nil || len(ob.JunctionTurn.Points) < 2 {
			return false
		}
		pts := ob.JunctionTurn.Points
		lastSeg := pts[len(pts)-1].Sub(pts[len(pts)-2])
		outU := sim.UnitFromAngle(ourExit)
		if lastSeg.Dot(outU) <= .98*lastSeg.Length() {
			return false
		}
	}
	lateral := math.Abs(delta.X*u.Y - delta.Y*u.X)
	return (along > f.bus.Length/2+ob.Length/2 || (postJunctionTaper && lateral > f.bus.Width/2+ob.Width/2+4)) &&
		math.Cos(otherEntry-heading) > .98 &&
		math.Cos(otherExit-ourExit) > .98 &&
		(ad != nil && ad.fallbackJunction != nil || lateral < f.laneWidth*4)
}

func exitQueueBlocksObstacle(corridor *sim.JunctionCorridor, g sim.JunctionGeometry, bus *sim.Bus, ob VehicleObstacle) bool {
	if math.Abs(ob.Speed) > 5 {
		return false
	}
	queue := sim.PrepareJunctionExitQueue(corridor.Bodies, corridor.Center, g, bus.OverallLength(), bus.Width)
	for _, part := range predictedObstacleBodies(ob, 0) {
		if part.HalfLen > 0 && queue.Blocks(part) {
			return true
		}
	}
	return false
}

// Reaching the painted line is not the same as occupying the junction box.
// A queue can nose past that line and publish Entered before it has a green
// signal. Reserving its untravelled crossing curve then deadlocks a lawful turn.
// Only release the future reservation: obstacle and swept-body checks still
// protect the actual vehicle, and blocked outgoing queues remain authoritative.
func waitingOutsideRedEntry(ob VehicleObstacle, inter *sim.TrafficIntersection, g sim.JunctionGeometry) bool {
	if math.Abs(ob.Speed) > 5 {
		return false
	}
	entry, exit := ob.Heading, ob.Heading
	known := false
	if c := ob.JunctionCorridor; c != nil && c.Remaining > 0 && c.Center.Distance(inter.Center) < 1 && len(c.Bodies) > 1 {
		entry, exit = c.EntryHeading, c.Bodies[len(c.Bodies)-1].Heading
		known = true
	}
	if turn := ob.JunctionTurn; turn != nil && !turn.LaneChange && turn.Center.Distance(inter.Center) < 1 {
		entry, exit = turn.EntryHeading, turn.ExitHeading
		known = true
	}
	if !known || math.Cos(ob.Heading-entry) < .98 {
		return false
	}
	angle := sim.NormalizeAngle(exit - entry)
	// Right-on-red traffic may already be starting a lawful merge.
	if angle > .35 && angle < 2.8 {
		return false
	}
	left := angle < -.35 || math.Abs(angle) >= 2.8
	if inter.GetSignalForHeading(entry, left) != sim.SignalRed {
		return false
	}
	u := sim.UnitFromAngle(entry)
	half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	yaw := ob.Heading - entry
	nose := ob.Pos.Sub(inter.Center).Dot(u) + math.Abs(math.Cos(yaw))*ob.Length/2 + math.Abs(math.Sin(yaw))*ob.Width/2
	return nose <= -half
}

// Two opposing left greens can release both approaches on the same tick.
// A U-turn yields before entering to a conflicting regular movement.
func opposingGreenBeforeEntry(c *sim.JunctionCorridor, ob VehicleObstacle, inter *sim.TrafficIntersection, laneWidth float64) bool {
	other := ob.JunctionCorridor
	if other == nil || other.Entered || other.Remaining <= 0 || other.Center.Distance(c.Center) > 1 ||
		len(other.Bodies) < 2 || math.Cos(other.EntryHeading-c.EntryHeading) > -.98 {
		return false
	}
	angle := corridorTurnAngle(other)
	u := sim.UnitFromAngle(other.EntryHeading)
	gap := inter.Center.Sub(ob.Pos).Dot(u) - inter.GetStopLineDistForHeading(other.EntryHeading, laneWidth) - ob.Length/2
	uTurn := math.Abs(angle) >= 2.8
	if gap > laneWidth || inter.GetSignalForHeading(other.EntryHeading, uTurn || angle < -.35) != sim.SignalGreen {
		return false
	}
	// Use the approach direction to break symmetry between two U-turns.
	return (!c.UTurnReady || !inter.WaitingUTurnHasPriority()) && (!uTurn || sim.UnitFromAngle(c.EntryHeading).Dot(sim.Vec2{X: 1, Y: 1}) < 0)
}
