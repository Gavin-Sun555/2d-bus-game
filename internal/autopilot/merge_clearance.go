package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

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
	for _, ob := range f.obstacles {
		if stoppedOriginFollowerClearsMerge(f, ob, heading, current, target) {
			continue
		}
		if gap, ok := requiredMergeForwardGap(f, ob, heading, current, target); ok && gap >= 35 && f.speedAbs <= 85 {
			continue
		}
		// Check each crossed lane, including intermediate lanes on wide roads.
		offset := current
		for {
			next := offset + sim.Clamp(target-offset, -f.laneWidth, f.laneWidth)
			if !isLaneClearToChange(f.bus, []VehicleObstacle{ob}, forward.X, forward.Y, current, next, f.laneWidth) {
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
	d := ob.Pos.Sub(f.bus.Pos)
	if math.Abs(d.Dot(r)) > f.laneWidth*.2 {
		return false
	}
	yaw := f.bus.Heading - heading
	body := f.bus.BodyEnvelope()
	extent := body.HalfLen*math.Abs(math.Cos(yaw)) + body.HalfWid*math.Abs(math.Sin(yaw)) - body.Center.Sub(f.bus.Pos).Dot(u)
	braking := ob.Speed*ob.Speed/(2*220) + math.Abs(ob.Speed)*.2
	return d.Dot(u) < -(extent + ob.Length/2 + 25 + braking)
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
	return mergeTrafficSpeed(f, ad.approachMerge)
}

func mergeTrafficSpeed(f autopilotFrame, request *sim.BusLaneMergeRequest) float64 {
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	current := 0.0
	target := request.Target.Sub(f.bus.Pos).Dot(right)
	limit := math.Inf(1)
	for _, ob := range f.obstacles {
		gap, ok := requiredMergeForwardGap(f, ob, request.Heading, current, target)
		if !ok {
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
