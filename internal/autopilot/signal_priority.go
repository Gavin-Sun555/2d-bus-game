package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// SignalPriorityRequest uses the same road axis and route-exit classifier as
// braking. It does not infer a left turn merely from lane-change steering.
func (ad *AutopilotDriver) SignalPriorityRequest(bus *sim.Bus, tls *sim.TrafficLightSystem, laneWidth float64) *sim.BusSignalRequest {
	if bus == nil || tls == nil || bus.DoorsOpen || bus.PassengerServiceRemaining() > 0 || bus.Speed < 0 {
		return nil
	}
	heading := approachRoadHeading(bus.Heading)
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	var nearest *sim.TrafficIntersection
	distance := math.Inf(1)
	for _, inter := range tls.Intersections {
		d := inter.Center.Sub(bus.Pos)
		stop := inter.GetStopLineDistForHeading(heading, laneWidth)
		toLine := d.Dot(u) - stop - bus.Length/2*math.Cos(bus.Heading-heading)
		if toLine < 0 || toLine > math.Min(900, 600+math.Abs(bus.Speed)*1.5) || math.Abs(d.Dot(r)) > laneWidth*4.8 {
			continue
		}
		if toLine < distance {
			nearest, distance = inter, toLine
		}
	}
	if nearest == nil {
		return nil
	}
	idx := nearest.GetApproachIndexForHeading(heading)
	if !nearest.ApproachBusLane[idx] || nearest.ApproachLanes[idx] < 4 {
		return nil
	}
	threshold := laneWidth * 1.6
	if nearest.ApproachLanes[idx] >= 6 {
		threshold = laneWidth * 2.8
	}
	if nearest.ApproachLanes[idx] >= 8 {
		threshold = laneWidth * 4.2
	}
	turn, found := ad.intersectionExitTurn(nearest.Center, u.X, u.Y, nearest.GetStopLineDistForHeading(heading, laneWidth), threshold)
	left := found && turn == routeTurnLeft
	if ad.approachMerge != nil {
		left = ad.approachMerge.LeftTurn
	}
	phase := sim.PhaseEWGreen
	if math.Abs(u.Y) > .5 {
		phase = sim.PhaseNSGreen
	}
	if left && nearest.ApproachHasDedicatedLanesForHeading(heading) {
		if phase == sim.PhaseEWGreen {
			phase = sim.PhaseEWLeftGreen
		} else {
			phase = sim.PhaseNSLeftGreen
		}
	}
	return &sim.BusSignalRequest{IntersectionID: nearest.ID, Heading: heading, Phase: phase}
}
