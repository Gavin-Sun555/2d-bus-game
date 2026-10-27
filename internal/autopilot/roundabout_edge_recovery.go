package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A long vehicle can stop with a safe current pose but no forward turning
// room at the outside curb. Back up only a few pixels after proving both the
// complete reverse sweep and the following forward turn clear all bodies.
type RoundaboutEdgeRecovery struct {
	Start                  sim.Vec2
	Distance, ReverseSteer float64
	AlignForward           bool
}

func edgeRecoveryPose(b sim.Bus, distance, steering float64) sim.Bus {
	h := b.Heading + distance/b.WheelBase*math.Tan(steering*b.MaxSteerAngle)
	return b.AtPose(b.Pos.Add(sim.UnitFromAngle(h).Mul(distance)), h)
}

func edgeRecoveryPoseClear(f autopilotFrame, b *sim.Bus, elapsed float64) bool {
	boundaryProbe := *b
	boundaryProbe.Width += 1
	boundaryProbe.Length += 1
	if math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 || b.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier || boundaryProbe.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
		return false
	}
	for _, ob := range f.obstacles {
		if ob.Pos.Distance(b.Pos) > b.OverallLength()+ob.Length+math.Abs(ob.Speed)*elapsed+40 {
			continue
		}
		for _, body := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
			if body.HalfLen <= 0 {
				continue
			}
			body.HalfLen += .5
			body.HalfWid += .5
			if hit, _, _ := b.CollidesBody(body); hit {
				return false
			}
		}
	}
	return true
}

func (ad *AutopilotDriver) tryRoundaboutEdgeRecovery(f autopilotFrame, steer float64) bool {
	if !f.bus.IsArticulated() || f.speedAbs > .1 || f.world == nil || f.bus.DoorsOpen || math.Abs(steer) < .5 || ad.roundaboutEdgeRetry > 0 {
		return false
	}
	near := false
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+15 {
			near = true
			break
		}
	}
	if !near || f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneShoulder {
		return false
	}
	empty := f
	empty.obstacles = nil
	if ad.articulatedSweptSpeed(empty, steer) > 0 {
		return false
	}
	ad.roundaboutEdgeRetry = 1
	// Never turn a traffic wait into a reverse maneuver. The current full
	// body and every subsequent sample must be physically separated.
	if !edgeRecoveryPoseClear(f, f.bus, 0) {
		return false
	}
	direction := -math.Copysign(1, steer)
	for distance := 6.; distance <= 10; distance += .5 {
		b := *f.bus
		safe := true
		for traveled := .25; traveled <= distance; traveled += .25 {
			next := edgeRecoveryPose(b, -.25, direction)
			if next.Pos == b.Pos || !edgeRecoveryPoseClear(f, &next, 1+traveled/6) {
				safe = false
				break
			}
			b = next
		}
		if !safe {
			continue
		}
		for traveled := .25; traveled <= 60; traveled += .25 {
			next := edgeRecoveryPose(b, .25, -direction)
			if next.Pos == b.Pos || !edgeRecoveryPoseClear(f, &next, 3+traveled/16) {
				safe = false
				break
			}
			b = next
		}
		if safe {
			ad.roundaboutEdgeRecovery = &RoundaboutEdgeRecovery{Start: f.bus.Pos, Distance: distance, ReverseSteer: direction}
			return true
		}
	}
	return false
}

func (ad *AutopilotDriver) roundaboutEdgeControls(f autopilotFrame) (throttle, brake, steer float64) {
	r := ad.roundaboutEdgeRecovery
	if f.bus.DoorsOpen {
		return 0, 1, 0
	}
	f.bus.SetNextAccelerationScale(1)
	if r.AlignForward {
		if math.Abs(f.bus.Speed) > .1 {
			return 0, 1, -r.ReverseSteer
		}
		f.bus.ShiftTo(sim.GearDrive)
		if math.Abs(f.bus.SteeringAngle+r.ReverseSteer*f.bus.MaxSteerAngle) < .02 {
			ad.roundaboutEdgeRecovery = nil
		}
		return 0, 1, -r.ReverseSteer
	}
	remaining := r.Distance - f.bus.Pos.Distance(r.Start)
	if remaining <= .08 {
		r.AlignForward = true
		return 0, 1, -r.ReverseSteer
	}
	if math.Abs(f.bus.SteeringAngle-r.ReverseSteer*f.bus.MaxSteerAngle) > .02 {
		return 0, 1, r.ReverseSteer
	}
	// Recheck a short stopping horizon against actual traffic every tick.
	predicted := *f.bus
	safe := true
	horizon := math.Min(remaining, .8+f.bus.Speed*f.bus.Speed/(2*f.bus.BrakeRate))
	for distance := .1; distance <= horizon; distance += .1 {
		next := edgeRecoveryPose(predicted, -.1, r.ReverseSteer)
		if next.Pos == predicted.Pos || !edgeRecoveryPoseClear(f, &next, distance/6) {
			safe = false
			break
		}
		predicted = next
	}
	if !safe {
		return 0, 1, r.ReverseSteer
	}
	f.bus.ShiftTo(sim.GearReverse)
	speed := math.Abs(f.bus.Speed)
	target := math.Min(6, math.Sqrt(2*60*math.Max(0, remaining-.06)))
	ad.LastTargetSpeed = target
	if speed > target {
		return 0, .2, r.ReverseSteer
	}
	return sim.Clamp((target-speed)/20, .01, .15), 0, r.ReverseSteer
}
