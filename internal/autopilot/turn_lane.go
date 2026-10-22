package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A sampled turning arc has many small angles. Looking for a single sharp
// waypoint can mistake its latter half for a straight passing opportunity.
// Keep the junction's original entry axis until the rear clears the exit.
func (ad *AutopilotDriver) trackJunctionTurnLane(f autopilotFrame, dt float64) bool {
	if f.world == nil || len(ad.drivePath) < 2 {
		return false
	}
	for _, center := range f.world.Intersections {
		if f.world.IsAtRoundabout(center) {
			continue
		}
		g := f.world.JunctionAt(center, f.laneWidth)
		delta := f.bus.Pos.Sub(center)
		clearance := f.bus.Length/2 + 60
		if math.Abs(delta.X) > g.HalfX+clearance || math.Abs(delta.Y) > g.HalfY+clearance {
			continue
		}
		heading := approachRoadHeading(f.bus.Heading)
		if c := f.bus.JunctionCorridor; c != nil && c.Remaining > 0 && c.Center.Distance(center) < 1 {
			heading = c.EntryHeading
		}
		u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
		stop := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY + sim.JunctionStopInset
		if -delta.Dot(u)-stop-f.bus.Length/2 > 0 {
			continue
		}
		coord, ew := center.Y, math.Abs(u.X) > .5
		if !ew {
			coord = center.X
		}
		lanes := f.world.GetRoadLanes(coord, ew, center.Sub(u.Mul(stop+100)))
		threshold := math.Max(1.6, float64(lanes)/2+.2) * f.laneWidth
		turn, known := ad.intersectionExitTurn(center, u.X, u.Y, stop, threshold)
		if !known || turn == routeTurnStraight {
			continue
		}
		// Unwind an existing passing offset along the route rather than
		// selecting another lane around a slow vehicle while rotating.
		ad.TargetLaneOffset = 0
		ad.OvertakeState = OvertakeReturning
		ad.progressLaneOffset(dt, f.speedAbs)
		if math.Abs(ad.LaneOffset) < 1 {
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
		}
		return true
	}
	return false
}
