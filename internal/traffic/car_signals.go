package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) signalSpeed(f carFrame, targetSpd float64) float64 {
	if f.tls == nil || f.car.InRoundabout || (f.car.JunctionTurn != nil && !f.car.JunctionTurn.LaneChange) {
		return targetSpd
	}
	lookAhead := math.Max(f.laneWidth*1.5+85, f.car.Speed*f.car.Speed/(2*220)+60)
	halfLen := f.car.Length * 0.5
	sinH, cosH := math.Sincos(f.car.Heading)
	bumper := sim.Vec2{X: f.car.Pos.X + cosH*halfLen, Y: f.car.Pos.Y + sinH*halfLen}
	for _, inter := range f.nearbyJunctions() {
		stopLine := inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth)
		delta := inter.Center.Sub(bumper)
		forward := delta.X*cosH + delta.Y*sinH
		lateral := math.Abs(-delta.X*sinH + delta.Y*cosH)
		if forward <= stopLine || forward >= stopLine+lookAhead || lateral >= math.Max(f.laneWidth*3.6, inter.ApproachRoadHalfWidth(f.car.Heading, f.laneWidth)) {
			continue
		}
		wantLeft, wantRight := carIntersectionTurns(f, inter)
		// Apply each junction's own intent so a later right turn cannot exempt
		// a straight movement through an earlier red light. The planned movement
		// is authoritative; a lane-position fallback must not select the left
		// arrow for a path that will actually proceed straight.
		signal := sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
		if stop, distance := signal.CheckStopForVehicleAtSpeed(bumper, f.car.Heading, lookAhead, f.laneWidth, wantLeft, wantRight, false, f.car.Speed, true); stop {
			f.car.IsBraking = true
			if distance <= 14.0 {
				return 0
			}
			return math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, distance-14)))
		}
	}

	// Both green-light movements and right turns on red require a clear box.
	for _, inter := range f.nearbyJunctions() {
		stopLine := inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth)
		delta := inter.Center.Sub(bumper)
		forward := delta.X*cosH + delta.Y*sinH
		lateral := math.Abs(-delta.X*sinH + delta.Y*cosH)
		if !(forward > 0 && forward < stopLine+lookAhead && lateral < f.laneWidth*3.6) {
			continue
		}
		distance := forward - stopLine
		if !(distance > -8.0) {
			continue
		}
		occupied := false
		for _, bus := range f.buses {
			if bus != nil && !carClearsBusInJunction(f, bus, inter) && bus.Pos.Distance(inter.Center) < stopLine+20.0 {
				occupied = true
				break
			}
		}
		if !occupied {
			continue
		}
		f.car.IsBraking = true
		if distance <= 14.0 {
			if distance <= 6.0 && f.car.Speed > 0 {
				f.car.Speed = 0
			}
			return 0
		}
		return math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, distance-14)))
	}
	return targetSpd
}

// A moving bus that has completed its return can lead same-lane straight
// traffic through the box. Its merge priority must not become a second red light.
func canFollowCommittedThroughBus(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection) bool {
	request := bus.LaneMergeRequest
	if request == nil || request.Remaining <= 0 || !request.Joined || request.RightTurn || bus.Speed <= 15 || request.Junction.Distance(inter.Center) > 1 || math.Cos(f.car.Heading-request.Heading) < .98 {
		return false
	}
	forward := sim.UnitFromAngle(request.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	return bus.Pos.Sub(f.car.Pos).Dot(forward) > 0 && math.Abs(f.car.Pos.Sub(request.Target).Dot(right)) < f.laneWidth*.35
}

// intersectionYieldSpeed reserves the box for a lawful crossing/turning bus,
// while allowing a car already inside to clear it if its path is unobstructed.
func (tm *TrafficManager) intersectionYieldSpeed(f carFrame, targetSpd float64) float64 {
	if f.tls == nil || f.car.InRoundabout || (f.car.JunctionTurn != nil && !f.car.JunctionTurn.LaneChange) {
		return targetSpd
	}
	sinH, cosH := math.Sincos(f.car.Heading)
	bumperPos := f.car.Pos.Add(sim.Vec2{X: cosH * f.car.Length * 0.5, Y: sinH * f.car.Length * 0.5})
	for _, inter := range f.nearbyJunctions() {
		if !(f.car.Pos.Distance(inter.Center) < 140.0) {
			continue
		}
		stopLineDist := inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth)
		toInter := inter.Center.Sub(bumperPos)
		forward := toInter.X*cosH + toInter.Y*sinH
		lateral := math.Abs(-toInter.X*sinH + toInter.Y*cosH)
		if !(forward > -15.0 && lateral < f.laneWidth*3.6) {
			continue
		}
		distToLine := forward - stopLineDist
		for _, bus := range f.buses {
			if bus == nil || sim.IsBusFacingWrongWay(bus) || carClearsBusInJunction(f, bus, inter) {
				continue
			}
			if !(inter.Center.Distance(bus.Pos) < stopLineDist+15.0) {
				continue
			}
			headingDiff := math.Cos(bus.Heading - f.car.Heading)
			if !(math.Abs(headingDiff) < 0.707 || math.Abs(bus.Speed) < 15.0) {
				continue
			}
			f.car.IsBraking = true
			if distToLine > 0 {
				if distToLine <= 14.0 {
					targetSpd = 0
					if distToLine <= 6.0 && f.car.Speed > 0 {
						f.car.Speed = 0
					}
				} else {
					targetSpd = math.Min(targetSpd, math.Max(0, (distToLine-10.0)*1.8))
				}
				break
			}
			toBus := bus.Pos.Sub(f.car.Pos)
			fwdBus := toBus.X*cosH + toBus.Y*sinH
			latBus := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
			halfLength, halfWidth := f.car.Length*0.5, f.car.Width*0.5
			if fwdBus > 0 && fwdBus < halfLength+bus.Length*0.5+15.0 && latBus < halfWidth+bus.Width*0.5+4.0 {
				targetSpd = 0
				if fwdBus < halfLength+bus.Length*0.5+6.0 && f.car.Speed > 0 {
					f.car.Speed = 0
				}
			}
			break
		}
	}
	return targetSpd
}
