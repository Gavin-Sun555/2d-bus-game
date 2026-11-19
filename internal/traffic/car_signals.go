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
	sinH, cosH := sim.Sincos(f.car.Heading)
	bumper := sim.Vec2{X: f.car.Pos.X + cosH*halfLen, Y: f.car.Pos.Y + sinH*halfLen}
	if limit := f.world.RailSpeedLimit(f.tls, bumper, f.car.Heading, f.car.Speed, 220); limit < targetSpd {
		f.car.IsBraking = true
		targetSpd = limit
	}
	if limit := f.world.StationCrossingSpeedLimit(f.tls, bumper, f.car.Heading, f.car.Speed, f.laneWidth, 220); limit < targetSpd {
		f.car.IsBraking = true
		targetSpd = limit
	}
	onRailCrossing := f.world.VehicleOnLevelCrossing(bumper, f.car.Heading, f.car.Length)
	for _, inter := range f.nearbyJunctions() {
		stopLine := inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth)
		railPullback := f.world.RailIntersectionPullback(inter, bumper, f.car.Heading, f.laneWidth, f.car.Length)
		delta := inter.Center.Sub(bumper)
		forward := delta.X*cosH + delta.Y*sinH
		lateral := math.Abs(-delta.X*sinH + delta.Y*cosH)
		if forward <= stopLine || forward >= stopLine+railPullback+lookAhead || lateral >= math.Max(f.laneWidth*3.6, inter.ApproachRoadHalfWidth(f.car.Heading, f.laneWidth)) {
			continue
		}
		wantLeft, wantRight := carIntersectionTurns(f, inter)
		// Apply each junction's own intent so a later right turn cannot exempt
		// a straight movement through an earlier red light. The planned movement
		// is authoritative; a lane-position fallback must not select the left
		// arrow for a path that will actually proceed straight.
		signal := sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
		checkSpeed := f.car.Speed
		if onRailCrossing {
			checkSpeed = math.Max(checkSpeed, 140)
		}
		if railPullback > 0 {
			sig := inter.GetSignalForHeading(f.car.Heading, wantLeft)
			greenDur := inter.GreenDuration
			if wantLeft && inter.ApproachHasDedicatedLanesForHeading(f.car.Heading) {
				greenDur = inter.LeftGreenDuration
			}
			distToStopLine := forward - stopLine
			distToHold := distToStopLine - railPullback
			if distToHold > 0 {
				distToClearRail := distToHold + 210.0 + f.car.Length
				timeNeeded := distToClearRail / math.Max(f.car.Speed, 90.0)
				if f.car.Speed < 40.0 {
					timeNeeded = math.Sqrt(2*distToClearRail/70.0) + 0.8
				}
				greenRem := greenDur - inter.Timer
				if sig != sim.SignalGreen || (greenDur > 0 && greenRem < timeNeeded) {
					if distToHold < lookAhead {
						f.car.IsBraking = true
						if distToHold <= 14.0 {
							return 0
						}
						return math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, distToHold-14)))
					}
				}
			}
		}
		if stop, distance := signal.CheckStopForVehicleAtSpeed(bumper, f.car.Heading, lookAhead+railPullback, f.laneWidth, wantLeft, wantRight, onRailCrossing, checkSpeed, true); stop {
			distance, _ = f.world.ClampStopBeforeRailCrossing(bumper, f.car.Heading, distance, f.car.Length, 14)
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
			if bus != nil && (bus.Pos.Distance(inter.Center) < stopLine+20 ||
				curbLeftBusHasEntryPriority(f, bus, inter, bumper, stopLine, distance) ||
				waitingUTurnBlocksCar(f, bus, inter)) && !carClearsBusInJunction(f, bus, inter) && !busExitedJunctionBox(f, bus, inter) {
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

// Two left turns from the same approach converge. Give the curb bus a
// slot when its nose is already at the line, while letting admitted cars clear.
func curbLeftBusHasEntryPriority(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection, bumper sim.Vec2, stopLine, gap float64) bool {
	c := bus.JunctionCorridor
	if c == nil || !c.CurbLeft || c.Remaining <= 0 || c.Center.Distance(inter.Center) > 1 ||
		gap <= 0 || sim.Cos(f.car.Heading-c.EntryHeading) < .98 ||
		inter.GetSignalForHeading(c.EntryHeading, true) != sim.SignalGreen {
		return false
	}
	u := sim.UnitFromAngle(c.EntryHeading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if bus.Pos.Sub(inter.Center).Dot(r) < f.laneWidth {
		return false
	}
	nose := bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Length / 2))
	busGap := inter.Center.Sub(nose).Dot(u) - stopLine
	return busGap >= -bus.Length && busGap <= 25 && nose.Sub(bumper).Dot(u) >= -8
}

// A moving bus that has completed its return can lead same-lane straight
// traffic through the box. Its merge priority must not become a second red light.
func canFollowCommittedThroughBus(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection) bool {
	request := bus.LaneMergeRequest
	if request == nil || request.Remaining <= 0 || !request.Joined || request.RightTurn || bus.Speed <= 15 || request.Junction.Distance(inter.Center) > 1 || sim.Cos(f.car.Heading-request.Heading) < .98 {
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
	sinH, cosH := sim.Sincos(f.car.Heading)
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
			if bus == nil || sim.IsBusFacingWrongWay(bus) || carClearsBusInJunction(f, bus, inter) || busExitedJunctionBox(f, bus, inter) {
				continue
			}
			if !(inter.Center.Distance(bus.Pos) < stopLineDist+15.0) {
				continue
			}
			headingDiff := sim.Cos(bus.Heading - f.car.Heading)
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

func busExitedJunctionBox(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection) bool {
	if bus == nil || inter == nil {
		return false
	}
	u := sim.UnitFromAngle(bus.Heading)
	crossHalf := inter.ApproachRoadHalfWidth(bus.Heading+math.Pi/2, f.laneWidth)
	return bus.Pos.Sub(inter.Center).Dot(u) > crossHalf+bus.OverallLength()*0.5+10
}
