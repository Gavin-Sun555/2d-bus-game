package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A bay exit can stop with its nose almost touching the median. At a large
// yaw, even maximum forward steering moves that nose farther across the line.
// A short, checked reverse lets the wheels straighten within the carriageway.
func (ad *AutopilotDriver) roadReverseControls(f autopilotFrame) (throttle, brake, steer float64, active bool) {
	if f.world == nil || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 {
		if ad.roadReverseActive {
			f.bus.ShiftTo(sim.GearDrive)
			ad.roadReverseActive = false
		}
		return 0, 0, 0, false
	}
	heading := approachRoadHeading(f.bus.Heading)
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if !ad.roadReverseActive && (!ad.roadSafetyHold || f.speedAbs > .5 || math.Abs(yaw) <= .4 || math.Abs(yaw) > .8) {
		return 0, 0, 0, false
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	clearance := roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew)
	if !ad.roadReverseActive {
		if (!multi && !six) || clearance < 0 || clearance > 2 || !ad.onDepartureRoad(f, center, ew, heading) ||
			f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()) {
			return 0, 0, 0, false
		}
		for _, rail := range f.world.Railways() {
			if math.Abs(f.bus.Pos.Y-rail.Y) < rail.HalfWidth+f.bus.OverallLength()+sim.RailGateMargin {
				return 0, 0, 0, false
			}
		}
		ad.roadReverseActive = true
	}
	if clearance >= 8 && math.Abs(yaw) <= .4 {
		if f.speedAbs > .5 {
			return 0, 1, 0, true
		}
		f.bus.ShiftTo(sim.GearDrive)
		ad.roadReverseActive = false
		return 0, 0, 0, false
	}
	steer = sim.Clamp(3*yaw, -.65, .65)
	if f.bus.Speed > .5 || !roadReverseClear(f, heading, center, ew) {
		return 0, 1, steer, true
	}
	f.bus.ShiftTo(sim.GearReverse)
	throttle, brake = roadReversePedals(f.bus.Speed)
	return throttle, brake, steer, true
}

func roadReversePedals(speed float64) (throttle, brake float64) {
	if speed < -12 {
		return 0, .3
	}
	return sim.Clamp((12+speed)/15, .05, .35), 0
}

func roadReverseClear(f autopilotFrame, heading, center float64, ew bool) bool {
	b := *f.bus
	b.JunctionCorridor, b.RoundaboutEntry, b.LaneMergeRequest, b.StationMergeRequest = nil, nil, nil, nil
	b.TireTracks = nil
	b.ShiftTo(sim.GearReverse)
	for _, ob := range f.obstacles {
		if ob.Bus != f.bus && ob.Speed < 0 {
			reach := b.OverallLength() + ob.Length + math.Abs(ob.Speed)*1.2 + 40
			if ob.Pos.Distance(b.Pos) < reach {
				return false
			}
		}
	}
	normal := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	initial := roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew)
	for tick := 1; tick <= 72; tick++ {
		throttle, brake := roadReversePedals(b.Speed)
		if tick > 60 {
			throttle, brake = 0, 1
		}
		steer := sim.Clamp(3*sim.NormalizeAngle(b.Heading-heading), -.65, .65)
		b.SetNextAccelerationScale(1)
		b.Update(1./60, throttle, brake, steer, false, false)
		if b.HitsHardBarrier(f.world, f.laneWidth) {
			return false
		}
		parts := b.BodyParts()
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if corner.Sub(origin).Dot(normal) < 0 {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			if ob.Bus != f.bus && sweepBodiesCollision(parts, ob, math.Max(0, ob.Speed)*float64(tick)/60, 1, 1) {
				return false
			}
		}
	}
	return roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew) > initial+.1
}
