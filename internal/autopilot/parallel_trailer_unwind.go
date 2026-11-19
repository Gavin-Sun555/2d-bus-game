package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// An almost straight bus can be boxed beside a parallel follower: pursuing a
// small lateral correction swings the trailer into it. Center the wheels at
// rest, then creep straight until that correction has a clear sweep again.
// Signals and forward admission still come from the ordinary target speed.
func (ad *AutopilotDriver) parallelTrailerStraightClear(f autopilotFrame, desired float64) bool {
	if !ad.FleetBus || f.bus == nil || !f.bus.IsArticulated() || f.world == nil || f.nearRoundabout ||
		f.speedAbs >= 16 || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || math.Abs(desired) < .35 {
		return false
	}
	heading := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .03 || math.Abs(sim.NormalizeAngle(f.bus.TailHeading()-heading)) > .03 {
		return false
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	if !multi && !six {
		return false
	}
	if f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) || roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew) < 8 {
		return false
	}
	u := sim.UnitFromAngle(heading)
	blockedByRear := false
	for _, ob := range f.obstacles {
		if ob.Bus == nil || math.Abs(ob.Speed) > 1 || math.Cos(ob.Heading-heading) < .98 {
			continue
		}
		d := ob.Pos.Sub(f.bus.Pos)
		if d.Dot(u) >= -f.bus.Length/2 || d.Dot(u) < -f.bus.OverallLength()-ob.Length || math.Abs(d.X*u.Y-d.Y*u.X) > f.laneWidth*1.6 {
			continue
		}
		isolated := f
		isolated.obstacles = []VehicleObstacle{ob}
		if ad.articulatedSweptSpeed(isolated, desired, true) <= .5 || ad.controlSweptSpeed(isolated, desired) <= .5 {
			blockedByRear = true
			break
		}
	}
	if !blockedByRear {
		return false
	}
	centered := *f.bus
	centered.SteeringAngle = 0
	probe := f
	probe.bus = &centered
	return ad.controlSweptSpeed(probe, 0) >= 16 && ad.articulatedSweptSpeed(probe, 0, true) >= 16
}
