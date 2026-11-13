package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Stay on the current line until the bus in the right lane has cleared the
// merge. The actual bodies matter even on a straight navigation segment.
func (ad *AutopilotDriver) holdOccupiedRightLane(f autopilotFrame, steer float64) float64 {
	if len(ad.drivePath) < 2 || ad.uTurn != nil || ad.rightTurnJunction != nil || f.nearRoundabout {
		return steer
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	tangent := ad.pointAt(f.s + 140).Sub(ad.pointAt(f.s))
	if (steer <= 0 && yaw <= .04 && f.bus.SteeringAngle <= 0) || math.Abs(yaw) > .35 || tangent.Normalize().Dot(u) < .98 {
		return steer
	}
	rear, nose := stationBodyRange(f.bus, u)
	for _, ob := range f.obstacles {
		if ob.Bus == nil || ob.Bus == f.bus || math.Cos(ob.Bus.Heading-heading) < .98 {
			continue
		}
		lateral := ob.Bus.Pos.Sub(f.bus.Pos).Dot(r)
		if lateral < f.bus.Width/2 || lateral > 1.3*f.laneWidth {
			continue
		}
		otherRear, otherNose := stationBodyRange(ob.Bus, u)
		// Keep enough room for the trailer and for a bus closing from behind.
		if otherRear <= nose+20+math.Max(0, f.bus.Speed-ob.Speed)*.8 &&
			otherNose >= rear-22-math.Max(0, ob.Speed-f.bus.Speed)*.8 {
			return math.Min(steer, sim.Clamp(-4*yaw, -1, 1))
		}
	}
	return steer
}

// Only the straight bus on the right retreats; the bus already angled into
// its lane waits for the gap. Check the real rear sweep before committing.
func (ad *AutopilotDriver) parallelMergeRetreatDistance(f autopilotFrame) float64 {
	if !ad.FleetBus || f.world == nil || f.speedAbs > 1 || f.bus.DoorsOpen || isArticulatedAndBent(f.bus) ||
		ad.uTurn != nil || ad.rightTurnJunction != nil || f.nearRoundabout || len(ad.drivePath) < 2 ||
		ad.State == AutopilotStateDocked || f.bus.PassengerServiceRemaining() > 0 {
		return 0
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .08 ||
		ad.pointAt(f.s+100).Sub(ad.pointAt(f.s)).Normalize().Dot(u) < .98 {
		return 0
	}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	rear, nose := stationBodyRange(f.bus, u)
	for _, ob := range f.obstacles {
		other := ob.Bus
		if other == nil || other == f.bus || other.DoorsOpen || math.Abs(other.Speed) > 1 {
			continue
		}
		yaw := sim.NormalizeAngle(other.Heading - heading)
		lateral := f.bus.Pos.Sub(other.Pos).Dot(r)
		if yaw < .12 || yaw > .5 || lateral < f.bus.Width/2 || lateral > 1.2*f.laneWidth {
			continue
		}
		otherRear, otherNose := stationBodyRange(other, u)
		if otherRear < nose && otherNose > rear && other.Pos.Sub(f.bus.Pos).Dot(u) > -f.bus.Length/2 &&
			ad.trafficLightSpeed(f, 30) > .5 {
			return math.Min(f.bus.OverallLength(), nose-otherRear+12)
		}
	}
	return 0
}
