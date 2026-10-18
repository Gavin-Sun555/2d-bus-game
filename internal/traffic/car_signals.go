package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) signalSpeed(f carFrame, targetSpd float64) float64 {
	if f.tls == nil || f.car.InRoundabout {
		return targetSpd
	}
	lookAhead := f.laneWidth*1.5 + 85.0
	halfLen := f.car.Length * 0.5
	cosH, sinH := math.Cos(f.car.Heading), math.Sin(f.car.Heading)
	bumper := sim.Vec2{X: f.car.Pos.X + cosH*halfLen, Y: f.car.Pos.Y + sinH*halfLen}
	roadOffset := math.Abs(f.car.Pos.X - f.car.RoadCenter)
	if f.car.IsEW {
		roadOffset = math.Abs(f.car.Pos.Y - f.car.RoadCenter)
	}
	wantLeft := roadOffset < f.laneWidth*0.95
	if stop, distance := f.tls.CheckStopForVehicle(bumper, f.car.Heading, lookAhead, f.laneWidth, wantLeft); stop {
		f.car.IsBraking = true
		if distance <= 14.0 {
			return 0
		}
		return math.Min(targetSpd, math.Max(0, (distance-10.0)*1.8))
	}

	// A green light still requires an unoccupied intersection box.
	for _, inter := range f.tls.Intersections {
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
			if bus != nil && bus.Pos.Distance(inter.Center) < stopLine+20.0 {
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
		return math.Min(targetSpd, math.Max(0, (distance-10.0)*1.8))
	}
	return targetSpd
}

// intersectionYieldSpeed reserves the box for a lawful crossing/turning bus,
// while allowing a car already inside to clear it if its path is unobstructed.
func (tm *TrafficManager) intersectionYieldSpeed(f carFrame, targetSpd float64) float64 {
	if f.tls == nil || f.car.InRoundabout {
		return targetSpd
	}
	cosH, sinH := math.Cos(f.car.Heading), math.Sin(f.car.Heading)
	bumperPos := f.car.Pos.Add(sim.Vec2{X: cosH * f.car.Length * 0.5, Y: sinH * f.car.Length * 0.5})
	for _, inter := range f.tls.Intersections {
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
			if bus == nil || sim.IsBusFacingWrongWay(bus) {
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
