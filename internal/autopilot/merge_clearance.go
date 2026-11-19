package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func obstacleInParkedOtherStationBay(f autopilotFrame, ob VehicleObstacle) bool {
	if f.world == nil {
		return false
	}
	for _, stop := range f.world.Stops {
		d := ob.Pos.Sub(stop.Pos)
		limit := stop.Length + 100
		if d.Dot(d) > limit*limit {
			continue
		}
		if (f.targetStop != nil && f.targetStop.ID == stop.ID) ||
			(f.fromStop != nil && f.fromStop.ID == stop.ID && f.bus != nil && f.bus.Pos.Sub(stop.Pos).Dot(f.bus.Pos.Sub(stop.Pos)) < limit*limit) ||
			(f.bus != nil && stop.ContainsBus(f.bus)) {
			continue
		}
		u := sim.UnitFromAngle(stop.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		along := math.Abs(d.Dot(u))
		across := math.Abs(d.Dot(r))
		nearBay := ob.Speed < 3.0 && along < stop.Length/2+ob.Length/2+25.0 && across < stop.Width/2+ob.Width/2+15.0
		if obstacleInsideStationBay(stop, ob) || (ob.Bus != nil && stop.ContainsBus(ob.Bus)) || nearBay {
			return true
		}
	}
	return false
}

// Required lane returns use a controlled speed, rather than the extra 200px passing
// runway reserved in isLaneClearToChange. Front gaps remain subject to physical
// headway/braking caps; rear and alongside traffic keep the normal clearance.
func requiredMergeLaneClear(f autopilotFrame, heading, current, target float64) bool {
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	// Once both sections already fit the destination lane, this is a small
	// recenter, not a new merge. A rear follower must not reserve our lane.
	withinLane := true
	targetLine := f.bus.Pos.Add(right.Mul(target - current))
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - heading
		half := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
		if math.Abs(part.Center.Sub(targetLine).Dot(right))+half > f.laneWidth/2-.25 {
			withinLane = false
			break
		}
	}
	if withinLane {
		return true
	}
	var oneOb [1]VehicleObstacle
	cabYaw := f.bus.Heading - heading
	cabHalfWidth := f.bus.Length/2*math.Abs(math.Sin(cabYaw)) + f.bus.Width/2*math.Abs(math.Cos(cabYaw))
	recenteringCab := math.Abs(target-current) <= 4 && math.Abs(target-current)+cabHalfWidth <= f.laneWidth/2-.25
	for _, ob := range f.obstacles {
		// A cab already in its destination lane only needs forward room to
		// straighten its trailer. A stopped, physically separated follower
		// must not reserve a new passing gap behind that tiny recenter.
		if math.Abs(ob.Speed) <= 1 && math.Cos(ob.Heading-heading) > .98 &&
			((math.Abs(target-current) <= 12 && stoppedFollowerClear(f, ob, heading)) ||
				(recenteringCab && stoppedFollowerRecenterClear(f, ob, heading))) {
			continue
		}
		if stoppedOriginFollowerClearsMerge(f, ob, heading, current, target) || stoppedMergingFollowerClears(f, ob, heading, current, target) || stoppedYieldingFollowerClears(f, ob, heading) || stoppedJunctionFollowerClearsMerge(f, ob, heading, current, target) {
			continue
		}
		if laneChangeLeaderClearance(f, ob, f.bus.SteeringAngle, 20, 0.31) {
			continue
		}
		if gap, ok := requiredMergeForwardGap(f, ob, heading, current, target); ok && gap >= 35 && f.speedAbs <= 85 {
			continue
		}
		oneOb[0] = ob
		// Check each crossed lane, including intermediate lanes on wide roads.
		offset := current
		for {
			next := offset + sim.Clamp(target-offset, -f.laneWidth, f.laneWidth)
			if !isLaneClearToChange(f.bus, oneOb[:], forward.X, forward.Y, current, next, f.laneWidth) {
				if obstacleInParkedOtherStationBay(f, ob) {
					break
				}
				return false
			}
			if math.Abs(next-target) < .01 {
				break
			}
			offset = next
		}
	}
	return true
}

// An unfinished junction-lane merge cannot wait for a bent,
// stationary follower that is already wholly behind its actual rear. Keep the
// same complete-body gap used for an acknowledged yielding follower; moving
// traffic and a new cross-lane merge still reserve their normal clearance.
func stoppedJunctionFollowerClearsMerge(f autopilotFrame, ob VehicleObstacle, heading, current, target float64) bool {
	request := f.bus.LaneMergeRequest
	if f.world == nil || (ob.Bus == nil && ob.Car == nil) || math.Abs(ob.Speed) > 1 || request == nil ||
		request.Joined || request.Remaining <= 0 ||
		math.Abs(sim.NormalizeAngle(request.Heading-heading)) > .01 || math.Abs(target-current) > f.laneWidth*.5 ||
		math.Cos(ob.Heading-heading) < .4 || (ob.Bus != nil && (math.Cos(ob.Bus.Heading-heading) < .4 || math.Cos(ob.Bus.TailHeading()-heading) < .4)) ||
		sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 0, 0) {
		return false
	}
	margin := 0.
	if !request.RightTurn && !request.LeftTurn {
		// A straight approach can stop just outside the junction while its
		// cab is finishing a small lane alignment. The follower waits for
		// that same occupied entrance; it cannot reserve a passing gap too.
		margin = f.bus.OverallLength() / 2
		if ob.Car != nil {
			margin = math.Max(margin, 150)
		}
	}
	if ob.Car != nil && (math.Cos(ob.Heading-heading) < .98 || request.RightTurn || request.LeftTurn || math.Abs(target-current) > f.laneWidth*.30) {
		return false
	}
	if !f.world.InsideJunction(f.bus.Pos, f.laneWidth, margin) {
		return false
	}
	return stoppedFollowerClear(f, ob, heading)
}

// A stopped follower in the lane we are leaving need not reserve a passing
// slot in the destination lane. Keep a full body/braking gap behind the tail;
// alongside vehicles and followers occupying the destination still must yield.
func stoppedOriginFollowerClearsMerge(f autopilotFrame, ob VehicleObstacle, heading, current, target float64) bool {
	shift := target - current
	if math.Abs(shift) < .55*f.laneWidth || math.Abs(ob.Speed) > 3 || math.Cos(ob.Heading-heading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := ob.Pos
	if ob.Bus != nil {
		// A bent trailer can shift the envelope's center across the lane
		// tolerance. Use the powered section for lane identity and both
		// sections below for the physical rear gap.
		origin = ob.Bus.Pos
	}
	latToBus := origin.Sub(f.bus.Pos).Dot(r)
	latToTarget := latToBus - shift
	onOriginTransition := math.Abs(latToBus) <= f.laneWidth*.35 && latToTarget*shift < 0 && math.Abs(latToTarget) > f.laneWidth*.45
	if math.Abs(latToBus) > f.laneWidth*.2 && !onOriginTransition {
		return false
	}
	// The leader moves away longitudinally while leaving this lane. Measure
	// the actual rear and nose, including yaw, instead of reserving a passing
	// gap behind a stationary follower.
	return stoppedFollowerClear(f, ob, heading)
}

// Two buses merging toward the same lane can each reserve the other's path
// and both wait forever. The one ahead goes first: a stopped bus that is
// itself still merging, wholly behind our tail, does not reserve a gap. It
// cannot close on us, and it is waiting for this merge anyway. Moving traffic
// and stopped vehicles that are not merging keep the ordinary gap.
func stoppedMergingFollowerClears(f autopilotFrame, ob VehicleObstacle, heading, current, target float64) bool {
	b := ob.Bus
	if b == nil || math.Abs(ob.Speed) > 1 {
		return false
	}
	laneMerging := b.LaneMergeRequest != nil && !b.LaneMergeRequest.Joined && b.LaneMergeRequest.Remaining > 0
	stationDeparting := b.StationMergeRequest != nil && b.StationMergeRequest.Departing && b.StationMergeRequest.Remaining > 0
	if !laneMerging && !stationDeparting {
		return false
	}
	if stoppedFollowerClear(f, ob, heading) {
		return true
	}
	if f.speedAbs > 1 || math.Abs(ob.Speed) > .5 || stoppedFollowerGap(f, ob, heading) <= 1.5 {
		return false
	}
	trial := f
	trial.obstacles = []VehicleObstacle{ob}
	probe := NewAutopilotDriver()
	mergeSteer := sim.Clamp((target-current)/f.laneWidth, -1, 1)
	return probe.controlSweptSpeed(trial, mergeSteer) >= 16 &&
		(!f.bus.IsArticulated() || probe.articulatedSweptSpeed(trial, mergeSteer, true) >= 16)
}

// A stopped follower already yielding to this bus cannot close on our merge
// while yawed behind us; keep checking full rotated nose-to-tail clearance.
func stoppedYieldingFollowerClears(f autopilotFrame, ob VehicleObstacle, heading float64) bool {
	if math.Abs(ob.Speed) > 1 {
		return false
	}
	if ob.YieldingTo != f.bus && (ob.Bus == nil || ob.Bus.YieldingToBus != f.bus) {
		return false
	}
	return stoppedFollowerClear(f, ob, heading)
}

// stoppedFollowerClear reports whether every section of ob, including yaw,
// lies behind every section of this bus with a skin and braking margin.
func stoppedFollowerClear(f autopilotFrame, ob VehicleObstacle, heading float64) bool {
	braking := ob.Speed*ob.Speed/(2*220) + math.Abs(ob.Speed)*.2
	return stoppedFollowerGap(f, ob, heading) > 8+braking
}

func stoppedFollowerRecenterClear(f autopilotFrame, ob VehicleObstacle, heading float64) bool {
	if f.speedAbs > 1 || stoppedFollowerGap(f, ob, heading) <= .25 {
		return false
	}
	trial := f
	trial.obstacles = []VehicleObstacle{ob}
	probe := NewAutopilotDriver()
	steer := sim.Clamp(f.bus.SteeringAngle/f.bus.MaxSteerAngle, -1, 1)
	return probe.controlSweptSpeed(trial, steer) >= 16 && probe.articulatedSweptSpeed(trial, steer, true) >= 16
}

func stoppedFollowerGap(f autopilotFrame, ob VehicleObstacle, heading float64) float64 {
	u := sim.UnitFromAngle(heading)
	rear := math.Inf(1)
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen > 0 {
			yaw := part.Heading - heading
			extent := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
			rear = math.Min(rear, part.Center.Sub(f.bus.Pos).Dot(u)-extent)
		}
	}
	nose := math.Inf(-1)
	for _, part := range predictedObstacleBodies(ob, 0) {
		if part.HalfLen > 0 {
			yaw := part.Heading - heading
			extent := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
			nose = math.Max(nose, part.Center.Sub(f.bus.Pos).Dot(u)+extent)
		}
	}
	if math.IsInf(nose, 0) || math.IsInf(rear, 0) {
		return 0
	}
	return rear - nose
}

func requiredMergeForwardGap(f autopilotFrame, ob VehicleObstacle, heading, current, target float64) (float64, bool) {
	if math.Cos(ob.Heading-heading) < .98 {
		return 0, false
	}
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	delta := ob.Pos.Sub(f.bus.Pos)
	along := delta.Dot(forward)
	if along <= 0 {
		return 0, false
	}
	yaw := f.bus.Heading - heading
	otherYaw := ob.Heading - heading
	busWidth := f.bus.Length/2*math.Abs(math.Sin(yaw)) + f.bus.Width/2*math.Abs(math.Cos(yaw))
	otherWidth := ob.Length/2*math.Abs(math.Sin(otherYaw)) + ob.Width/2*math.Abs(math.Cos(otherYaw))
	lateral := delta.Dot(right)
	shift := target - current
	if lateral < math.Min(0, shift)-busWidth-otherWidth-2 || lateral > math.Max(0, shift)+busWidth+otherWidth+2 {
		return 0, false
	}
	busLength := f.bus.Length/2*math.Abs(math.Cos(yaw)) + f.bus.Width/2*math.Abs(math.Sin(yaw))
	otherLength := ob.Length/2*math.Abs(math.Cos(otherYaw)) + ob.Width/2*math.Abs(math.Sin(otherYaw))
	return along - busLength - otherLength, true
}

func (ad *AutopilotDriver) requiredMergeTrafficSpeed(f autopilotFrame) float64 {
	return mergeTrafficSpeed(f, ad.approachTrafficRequest(f, ad.approachMerge))
}

func mergeTrafficSpeed(f autopilotFrame, request *sim.BusLaneMergeRequest) float64 {
	if request == nil {
		return math.Inf(1)
	}
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	current := 0.0
	target := request.Target.Sub(f.bus.Pos).Dot(right)
	limit := math.Inf(1)
	for _, ob := range f.obstacles {
		gap, ok := requiredMergeForwardGap(f, ob, request.Heading, current, target)
		if !ok || obstacleInParkedOtherStationBay(f, ob) {
			continue
		}
		lead := math.Max(0, ob.Speed*math.Cos(ob.Heading-request.Heading))
		speed := math.Max(0, lead+.7*(gap-35))
		speed = math.Min(speed, math.Sqrt(lead*lead+2*135*math.Max(0, gap-35)))
		stopGap := 43.0
		if !request.RightTurn {
			stopGap = 35
		}
		if gap < 12 || (lead < 1 && gap <= stopGap) {
			speed = 0
		}
		limit = math.Min(limit, speed)
	}
	return limit
}
