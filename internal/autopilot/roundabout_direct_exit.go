package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Leave on a continuous curve when a separate outward merge no longer fits,
// or the outer-lane bus has passed the planned exit point but can still turn.
// Reserve the entire swept corridor, not only its endpoint.
func (ad *AutopilotDriver) tryDirectRoundaboutExit(bus *sim.Bus, rb *sim.Roundabout, obstacles []VehicleObstacle, index int, lw float64) bool {
	start := bus.Pos
	end := ad.drivePath[index+1]
	outgoing := end.Sub(ad.drivePath[index]).Normalize()
	heading := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
	if end.Sub(start).Dot(heading) < 20 {
		return false
	}
	// Keep the existing short join for inner-lane departures. Near the outer
	// exit, try several steering profiles before deciding the mouth is missed.
	length := end.Distance(ad.drivePath[index])
	var points []sim.Vec2
	joins := []float64{190}
	if start.Distance(rb.Center) >= roundaboutOuterLaneRadius(rb)-14 {
		joins = append(joins, 280, 380)
	}
	for _, join := range joins {
		end = ad.drivePath[index].Add(outgoing.Mul(math.Min(length, join)))
		handles := []float64{math.Min(100, start.Distance(end)*.45)}
		if len(joins) > 1 {
			handles = append(handles, 45, 65, 85, 120, 150, 190)
		}
		for _, first := range handles {
			for _, last := range handles {
				points = ad.roundaboutExitCurve(bus, rb, obstacles, lw, start, end, outgoing, first, last)
				if points != nil {
					break
				}
			}
			if points != nil {
				break
			}
		}
		if points != nil {
			break
		}
	}
	if points == nil {
		return false
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
	ad.LaneOffset, ad.TargetLaneOffset, ad.LaneOffsetStart, ad.lastTargetLaneOffset = 0, 0, 0, 0
	ad.LaneOffsetProgress = 1
	ad.OvertakeState = OvertakeNone
	center := rb.Center
	ad.directRoundaboutExit = &center
	return true
}

func (ad *AutopilotDriver) roundaboutExitBodyOnRoad(body sim.OBB, rb *sim.Roundabout, lw float64) bool {
	forward := sim.Vec2{X: math.Cos(body.Heading), Y: math.Sin(body.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	delta := rb.Center.Sub(body.Center)
	nearest := body.Center.Add(forward.Mul(sim.Clamp(delta.Dot(forward), -body.HalfLen, body.HalfLen))).Add(right.Mul(sim.Clamp(delta.Dot(right), -body.HalfWid, body.HalfWid)))
	if nearest.Distance(rb.Center) < rb.InnerRadius+8 {
		return false
	}
	if ad.pathWorld != nil {
		for _, a := range []float64{-1, 1} {
			for _, b := range []float64{-1, 1} {
				corner := body.Center.Add(forward.Mul(a * body.HalfLen)).Add(right.Mul(b * body.HalfWid))
				if ad.pathWorld.CheckBoundaries(corner, 0, lw).Zone != sim.ZoneRoad {
					return false
				}
			}
		}
	}
	return true
}

// A candidate is accepted only when its full swept body stays on the road,
// clears traffic and fits the bus steering range.
func (ad *AutopilotDriver) roundaboutExitCurve(bus *sim.Bus, rb *sim.Roundabout, obstacles []VehicleObstacle, lw float64, start, end, outgoing sim.Vec2, firstHandle, lastHandle float64) []sim.Vec2 {
	heading := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
	c1 := start.Add(heading.Mul(firstHandle))
	c2 := end.Sub(outgoing.Mul(lastHandle))
	points := make([]sim.Vec2, 0, 34)
	distance := 0.0
	previous, previousHeading := start, bus.Heading
	speed := math.Max(35, math.Min(apRoundaboutSpeed, math.Abs(bus.Speed)))
	for i := 0; i <= 32; i++ {
		u := float64(i) / 32
		v := 1 - u
		point := start.Mul(v * v * v).Add(c1.Mul(3 * v * v * u)).Add(c2.Mul(3 * v * u * u)).Add(end.Mul(u * u * u))
		tangent := c1.Sub(start).Mul(3 * v * v).Add(c2.Sub(c1).Mul(6 * v * u)).Add(end.Sub(c2).Mul(3 * u * u))
		angle := math.Atan2(tangent.Y, tangent.X)
		step := point.Distance(previous)
		if i > 0 && math.Abs(sim.NormalizeAngle(angle-previousHeading)) > step*math.Tan(bus.MaxSteerAngle)/bus.WheelBase*.8 {
			return nil
		}
		distance += step
		body := sim.OBB{Center: point, Heading: angle, HalfLen: bus.Length/2 + 8, HalfWid: bus.Width/2 + 8}
		if !ad.roundaboutExitBodyOnRoad(body, rb, lw) {
			return nil
		}
		for _, obstacle := range obstacles {
			arrival := distance / speed
			for t := math.Max(0, arrival-.5); t <= arrival+.6; t += .1 {
				if hit, _, _ := sim.CheckOBBCollision(body, roundaboutObstacleAt(rb, obstacle, t)); hit {
					return nil
				}
			}
		}
		points = append(points, point)
		previous, previousHeading = point, angle
	}
	return points
}
