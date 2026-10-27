package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Passing offsets belong to the incoming straight road. Rotating a negative
// offset around a right turn moves the target into the outgoing oncoming lane.
// Once in the junction, track its explicit turn path without that offset until
// the whole bus has cleared the exit mouth.
func (ad *AutopilotDriver) updateRightTurnOffset(f autopilotFrame) bool {
	if f.world == nil || len(ad.drivePath) < 2 {
		return false
	}
	if ad.rightTurnJunction != nil {
		center := *ad.rightTurnJunction
		g := f.world.JunctionAt(center, f.laneWidth)
		d := f.bus.Pos.Sub(center)
		if math.Abs(d.X) > g.HalfX+f.bus.OverallLength()/2+80 || math.Abs(d.Y) > g.HalfY+f.bus.OverallLength()/2+80 {
			ad.rightTurnJunction = nil
			ad.rightTurnWaiting = false
			return false
		}
		ad.trackRightTurnWithoutPassingOffset(f)
		return true
	}
	if !ad.BerserkMode {
		return false
	}
	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.UnitFromAngle(heading)
	for _, center := range f.world.Intersections {
		if f.world.IsAtRoundabout(center) {
			continue
		}
		g := f.world.JunctionAt(center, f.laneWidth)
		delta := center.Sub(f.bus.Pos)
		if math.Abs(delta.X) > g.HalfX+f.bus.Length/2 || math.Abs(delta.Y) > g.HalfY+f.bus.Length/2 || delta.Dot(forward) < 0 {
			continue
		}
		stop := g.HalfX + sim.JunctionStopInset
		coord := center.Y
		ew := math.Abs(forward.X) > .5
		if !ew {
			stop = g.HalfY + sim.JunctionStopInset
			coord = center.X
		}
		lanes := f.world.GetRoadLanes(coord, ew, f.bus.Pos)
		threshold := math.Max(1.6, float64(lanes)/2+.2) * f.laneWidth
		turn, ok := ad.intersectionExitTurn(center, forward.X, forward.Y, stop, threshold)
		if !ok || turn != routeTurnRight {
			continue
		}
		c := center
		ad.rightTurnJunction = &c
		ad.rightTurnHeading = heading
		ad.trackRightTurnWithoutPassingOffset(f)
		ad.pathSynced = false
		return true
	}
	return false
}

func (ad *AutopilotDriver) trackRightTurnWithoutPassingOffset(f autopilotFrame) {
	ad.rightTurnWaiting = false
	heading := ad.rightTurnHeading
	withinLane := true
	right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	base := ad.pointAt(f.s)
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - heading
		extent := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
		if math.Abs(part.Center.Sub(base).Dot(right))+extent > f.laneWidth/2-.25 {
			withinLane = false
			break
		}
	}
	// A small in-lane correction needs no passing slot behind the vehicle.
	if ad.LaneOffset < -1 && !withinLane && !isLaneClearToChange(f.bus, f.obstacles, math.Cos(heading), math.Sin(heading), ad.LaneOffset, 0, f.laneWidth) {
		// Do not replace the old offset with a forced merge through traffic.
		// Hold the incoming corridor until there is room to enter the turn.
		ad.rightTurnWaiting = true
		distToStop := 0.0
		if f.world != nil && ad.rightTurnJunction != nil {
			center := *ad.rightTurnJunction
			forward := sim.UnitFromAngle(heading)
			stopLineDist := 0.0
			if f.tls != nil {
				for _, it := range f.tls.Intersections {
					if it.Center.Distance(center) < 1 {
						stopLineDist = it.GetStopLineDistForHeading(heading, f.laneWidth)
						break
					}
				}
			}
			if stopLineDist == 0 {
				g := f.world.JunctionAt(center, f.laneWidth)
				stopLineDist = g.HalfX + sim.JunctionStopInset
				if math.Abs(math.Cos(heading)) <= .5 {
					stopLineDist = g.HalfY + sim.JunctionStopInset
				}
			}
			toJunction := center.Sub(f.bus.Pos).Dot(forward)
			distToStop = toJunction - stopLineDist - f.bus.Length/2
		}
		if distToStop > 0 {
			ad.busTurnHoldDistance = math.Max(.01, distToStop-25)
		} else {
			ad.busTurnHoldDistance = .01
		}
		ad.holdLaneOffset(ad.LaneOffset)
		return
	}
	ad.holdLaneOffset(0)
	ad.OvertakeState = OvertakeNone
}
