package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Locate the next road departure from this ring, even if localization has
// already advanced a little beyond the last circular waypoint.
func (ad *AutopilotDriver) roundaboutExitIndex(rb *sim.Roundabout, s float64) int {
	for i := 1; i < len(ad.drivePath)-1; i++ {
		if ad.cumLen[i] < s-200 {
			continue
		}
		if ad.drivePath[i].Distance(rb.Center) <= rb.Radius+15 && ad.drivePath[i+1].Distance(rb.Center) > rb.Radius+15 {
			return i
		}
	}
	return -1
}

func (ad *AutopilotDriver) retryRoundaboutExit(bus *sim.Bus, rb *sim.Roundabout, obstacles []VehicleObstacle, lw, s, speed float64) bool {
	if bus.Pos.Distance(rb.Center) > rb.Radius+10 {
		return false
	}
	index := ad.roundaboutExitIndex(rb, s)
	if index < 0 {
		return false
	}
	remaining := ad.cumLen[index] - s
	if remaining > math.Max(210, speed*2) {
		return false
	}
	outer := roundaboutOuterLaneRadius(rb)
	radius := bus.Pos.Distance(rb.Center)
	inner := radius < outer-14 || ad.LaneOffset < -lw*.3
	exitAngle := math.Atan2(ad.drivePath[index].Y-rb.Center.Y, ad.drivePath[index].X-rb.Center.X)
	angle := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
	passed := sim.NormalizeAngle(exitAngle - angle)
	outgoing := ad.drivePath[index+1].Sub(ad.drivePath[index]).Normalize()
	leaving := radius >= outer-14 && (sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}).Dot(outgoing) > .85
	missed := !leaving && passed > .12 && passed < 1.0
	blocked := inner && !roundaboutExitMergeClear(bus, rb, obstacles)
	tooLate := inner && remaining < 45
	// Passing the last circular waypoint does not necessarily mean the road
	// mouth is behind us. An outer-lane bus can still make a safe, smooth exit;
	// check that actual curve before committing it to another full lap.
	if missed && !inner && ad.tryDirectRoundaboutExit(bus, rb, obstacles, index, lw) {
		return true
	}
	if !missed && inner && (blocked || tooLate) && ad.tryDirectRoundaboutExit(bus, rb, obstacles, index, lw) {
		return true
	}
	if !blocked && !tooLate && !missed {
		return false
	}
	// Replace the already-passed prefix with one complete counter-clockwise lap.
	// Localization stays monotonic across the duplicate circle and the destination
	// and road departure remain the same; no position or heading is snapped.
	points := make([]sim.Vec2, 0, 60+len(ad.drivePath)-ad.WaypointIdx)
	points = append(points, rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(outer)))
	if !missed {
		for step := 1; step <= 64; step++ {
			theta := angle - 2*math.Pi*float64(step)/64
			points = append(points, rb.Center.Add(sim.Vec2{X: math.Cos(theta), Y: math.Sin(theta)}.Mul(outer)))
		}
	}
	delta := angle - exitAngle
	for delta < 0 {
		delta += 2 * math.Pi
	}
	steps := int(math.Ceil(delta / .10))
	for i := 1; i <= steps; i++ {
		theta := angle - delta*float64(i)/float64(steps)
		points = append(points, rb.Center.Add(sim.Vec2{X: math.Cos(theta), Y: math.Sin(theta)}.Mul(outer)))
	}
	points = append(points, ad.drivePath[index+1:]...)
	ad.drivePath = dedupePath(points)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	ad.WaypointIdx = 0
	ad.LastPathS = 0
	ad.pathSynced = false
	offset := roundaboutLaneOffset(rb, bus, math.Min(ad.LaneOffset, radius-outer))
	ad.LaneOffset, ad.TargetLaneOffset = offset, offset
	ad.LaneOffsetStart = offset
	ad.lastTargetLaneOffset = offset
	ad.LaneOffsetProgress = 1
	ad.OvertakeState = OvertakePassing
	ad.OvertakeTimer = 0
	return true
}

func roundaboutExitMergeClear(bus *sim.Bus, rb *sim.Roundabout, obstacles []VehicleObstacle) bool {
	outer := roundaboutOuterLaneRadius(rb)
	radius := bus.Pos.Distance(rb.Center)
	angle := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
	speed := math.Max(60, math.Min(100, math.Abs(bus.Speed)))
	for t := 0.0; t <= 1.6; t += .08 {
		fraction := t / 1.6
		ease := fraction * fraction * (3 - 2*fraction)
		r := radius + (outer-radius)*ease
		theta := angle - speed*t/math.Max(radius, 1)
		body := sim.OBB{Center: rb.Center.Add(sim.Vec2{X: math.Cos(theta), Y: math.Sin(theta)}.Mul(r)), Heading: theta - math.Pi/2, HalfLen: bus.Length/2 + 4, HalfWid: bus.Width/2 + 4}
		for _, obstacle := range obstacles {
			if hit, _, _ := sim.CheckOBBCollision(body, roundaboutObstacleAt(rb, obstacle, t)); hit {
				return false
			}
		}
	}
	return true
}
