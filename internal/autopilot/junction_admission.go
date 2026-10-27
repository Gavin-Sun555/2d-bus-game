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
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		if _, ok := sim.JunctionLeaderGap(corridor, ob.JunctionCorridor, body, f.bus.Length, f.bus.Width); ok {
			continue
		}
		conflict := false
		waitingAtRed := waitingOutsideRedEntry(ob, inter, g)
		if !waitingAtRed && ob.JunctionCorridor != nil && ob.JunctionCorridor.Remaining > 0 && ob.JunctionCorridor.Entered && ob.JunctionCorridor.Center.Distance(corridor.Center) < 1 {
			sameLeader := ob.Speed > 15 && math.Cos(ob.Heading-heading) > .98 && ob.Pos.Sub(f.bus.Pos).Dot(u) > 0 && math.Abs(ob.Pos.Sub(f.bus.Pos).X*u.Y-ob.Pos.Sub(f.bus.Pos).Y*u.X) < f.laneWidth*.35
			conflict = !sameLeader && !sim.SeparatedJunctionCorridors(corridor, ob.JunctionCorridor)
		}
		if !waitingAtRed && ob.JunctionTurn != nil && !ob.JunctionTurn.LaneChange && ob.JunctionTurn.Center.Distance(corridor.Center) < 1 {
			points := ob.JunctionTurn.Points[ob.JunctionTurn.Segment:]
			delta := ob.Pos.Sub(f.bus.Pos)
			sameLeader := ob.Speed > 15 && math.Cos(ob.Heading-heading) > .98 && delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.35
			if !conflict && !sameLeader {
				var storage []sim.OBB
				if ad != nil {
					storage = ad.geometryScratch().admission[:0]
				}
				otherBodies := sim.AppendPathBodies(storage, points, ob.Length, ob.Width)
				if ad != nil && cap(otherBodies) <= maxTemporaryBodies {
					ad.temporaryGeometry.admission = otherBodies
				}
				conflict = !sim.CorridorsSeparated(corridor.Bodies, otherBodies)
			}
		}
		if conflict {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
		if sim.ExitQueueBlocksEntry(corridor.Bodies, corridor.Center, g, f.bus.Length, f.bus.Width, body, ob.Speed) {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
	}
	return math.Inf(1)
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
