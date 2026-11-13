package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Reserve time and distance for the lateral move, then approach that zone
// with a braking envelope instead of crawling from the first planning frame.
// Gap selection, signal braking and the actual-body sweeps remain separate.
func (ad *AutopilotDriver) approachManeuverSpeed(f autopilotFrame, request *sim.BusLaneMergeRequest) float64 {
	if !ad.BerserkMode || f.world == nil || request.Is2Lane {
		return 80
	}
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	shift := math.Abs(request.Target.Sub(f.bus.Pos).Dot(right))
	reserve := math.Max(360, shift*5+f.bus.OverallLength()*2)
	runway := math.Max(0, approachMergeStopDistance(f, request)-reserve)
	limit := math.Sqrt(80*80 + 2*130*runway)
	if shift > 4 && ad.heldApproachLane(f, request) == nil {
		limit = math.Min(limit, 240)
	}
	if shift < f.laneWidth*.3 && !mergePoseSettled(f, request) {
		limit = math.Min(limit, 100)
	}
	// A vehicle still yawed across lanes must finish that move before
	// accelerating. Smooth, shallow merges can proceed substantially faster.
	curvature := math.Abs(math.Tan(f.bus.SteeringAngle)) / f.bus.WheelBase
	if curvature > 1e-6 {
		limit = math.Min(limit, math.Max(80, math.Sqrt(85/curvature)))
	}
	if f.bus.IsArticulated() && math.Abs(sim.NormalizeAngle(f.bus.TrailerHeading-f.bus.Heading)) > .15 {
		limit = math.Min(limit, 100)
	}
	return limit
}

// While a blocked merge is holding a fully aligned physical lane far from
// the junction, follow that lane's leader. A queue in the requested lane must
// not make the bus crawl alongside it before there is any lateral movement.
func (ad *AutopilotDriver) approachTrafficRequest(f autopilotFrame, request *sim.BusLaneMergeRequest) *sim.BusLaneMergeRequest {
	if request == nil || !ad.BerserkMode || ad.approachManeuverSpeed(f, request) <= 160 {
		return request
	}
	if current := ad.heldApproachLane(f, request); current != nil {
		return current
	}
	return request
}

func (ad *AutopilotDriver) heldApproachLane(f autopilotFrame, request *sim.BusLaneMergeRequest) *sim.BusLaneMergeRequest {
	if ad.busTurnHoldDistance <= 0 || f.world == nil || request.Is2Lane ||
		math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) > .06 || math.Abs(f.bus.SteeringAngle) > .05 {
		return nil
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	forward := sim.UnitFromAngle(request.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lanes := f.world.GetRoadLanes(center, ew, f.bus.Pos)
	lane := int(math.Round(f.bus.Pos.Sub(origin).Dot(right)/f.laneWidth - .5))
	if lane < 0 || lane >= lanes/2 {
		return nil
	}
	current := *request
	current.Target = origin.Add(right.Mul((float64(lane) + .5) * f.laneWidth))
	if !wholeBusInMergeLane(f.bus, &current, f.laneWidth) {
		return nil
	}
	return &current
}

// Keep close pursuit throughout the turn, but don't mistake a pending exit
// for a bus already straightening on the outgoing road.
func (ad *AutopilotDriver) exitCruiseSpeed(f autopilotFrame) float64 {
	if !ad.exitCruiseSettling(f) {
		return math.Inf(1)
	}
	if !ad.BerserkMode || f.laneWidth <= 42 {
		return 100
	}
	request := ad.exitCruise
	yaw := math.Abs(sim.NormalizeAngle(f.bus.Heading - request.Heading))
	if request.Joined && yaw < .08 && math.Abs(f.bus.SteeringAngle) < .08 && wholeBusInMergeLane(f.bus, request, f.laneWidth) {
		return math.Inf(1)
	}
	if !request.Joined && yaw > .35 {
		return 220
	}
	// Narrow exits need more time for the trailer and steering to unwind.
	return math.Min(200, 100+(f.laneWidth-42)*4)
}

// Full-speed release needs a settled actuator and trailer, not just a momentary
// lane-center crossing. Otherwise acceleration carries residual yaw over the curb.
func mergePoseSettled(f autopilotFrame, request *sim.BusLaneMergeRequest) bool {
	r := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	tolerance := math.Min(3, math.Max(1, (f.laneWidth-f.bus.Width)/4))
	return math.Abs(request.Target.Sub(f.bus.Pos).Dot(r)) < tolerance &&
		math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) < .02 &&
		math.Abs(f.bus.SteeringAngle) < .02 &&
		(!f.bus.IsArticulated() || math.Abs(sim.NormalizeAngle(f.bus.TrailerHeading-request.Heading)) < .025) &&
		wholeBusInMergeLane(f.bus, request, f.laneWidth)
}
