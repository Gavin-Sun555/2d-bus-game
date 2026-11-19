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

// A bus already inside the ring can stall when its nominal steering would
// clip the island. Try steering away from the island at crawl speed, keeping
// the full boundary and traffic sweeps for both sections.
func (ad *AutopilotDriver) roundaboutIslandSteering(f autopilotFrame, desired float64) (float64, bool) {
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) >= rb.Radius {
			continue
		}
		empty := f
		empty.obstacles = nil
		if ad.articulatedSweptSpeed(empty, desired) > 0 {
			return 0, false
		}
		forward := sim.UnitFromAngle(f.bus.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		outward := math.Copysign(1, f.bus.Pos.Sub(rb.Center).Dot(right))
		for _, steer := range []float64{outward * .25, outward * .5, outward, 0} {
			if ad.controlSweptSpeed(f, steer) >= 16 && ad.articulatedSweptSpeed(f, steer, true) >= 16 {
				return steer, true
			}
		}
		break
	}
	return 0, false
}

// roundaboutSweepUnwindSteering softens or adjusts steering when an articulated
// bus inside a roundabout stalls solely on its sharp-turn swept envelope while
// a gentler or slightly adjusted turn clears obstacles and barriers safely.
func (ad *AutopilotDriver) roundaboutSweepUnwindSteering(f autopilotFrame, desired float64) (float64, bool, bool) {
	if f.bus == nil || !f.bus.IsArticulated() || f.world == nil {
		return 0, false, false
	}
	var activeRB *sim.Roundabout
	exitingOutsideRing := false
	for i := range f.world.Roundabouts {
		rb := &f.world.Roundabouts[i]
		dist := f.bus.Pos.Distance(rb.Center)
		if dist < rb.Radius+24 {
			activeRB = rb
			break
		}
		if dist < rb.Radius+f.bus.OverallLength()+60 && f.bus.Pos.Sub(rb.Center).Dot(sim.UnitFromAngle(f.bus.Heading)) > 0 {
			activeRB = rb
			exitingOutsideRing = true
			break
		}
	}
	if activeRB == nil {
		return 0, false, false
	}
	var candidates []float64
	if exitingOutsideRing {
		hinge := sim.NormalizeAngle(f.bus.Heading - f.bus.TailHeading())
		if math.Abs(desired) <= .05 && math.Abs(hinge) <= .15 {
			return 0, false, false
		}
		if math.Abs(desired) > .05 {
			tighter := sim.Clamp(desired+math.Copysign(.30, desired), -1, 1)
			full := math.Copysign(1, desired)
			candidates = append(candidates, desired*.85, desired*.65, tighter, desired*.45, desired*.25, full, desired*.1, 0, -desired*.15, -desired*.35)
		}
		if math.Abs(hinge) > .15 {
			s := math.Copysign(1, hinge)
			candidates = append(candidates, s*.35, s*.65, s*.85, s, -s*.35, -s*.65)
		}
	} else if math.Abs(desired) <= .05 {
		if math.Abs(sim.NormalizeAngle(f.bus.Heading-f.bus.TailHeading())) <= .15 {
			return 0, false, false
		}
		forward := sim.UnitFromAngle(f.bus.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		inward := -math.Copysign(1, f.bus.Pos.Sub(activeRB.Center).Dot(right))
		candidates = []float64{inward * .35, inward * .65, inward * .85, inward, -inward * .35, -inward * .65, -inward * .85, -inward}
	} else {
		tighter := sim.Clamp(desired+math.Copysign(.25, desired), -1, 1)
		full := math.Copysign(1, desired)
		candidates = []float64{desired * .65, tighter, desired * .35, desired * .15, full, 0, -desired * .35, -desired * .65, -desired * .85, -full}
	}
	for _, cand := range candidates {
		if math.Abs(cand-desired) < .01 {
			continue
		}
		if ad.controlSweptSpeed(f, cand) >= 16 && ad.articulatedSweptSpeed(f, cand, true) >= 16 && roundaboutUnwindClearHorizon(f, cand, exitingOutsideRing) {
			return cand, false, true
		}
	}
	if f.speedAbs < .5 {
		for _, cand := range candidates {
			targetWheel := cand * f.bus.MaxSteerAngle
			if math.Abs(f.bus.SteeringAngle-targetWheel) <= .02 {
				continue
			}
			settled := *f.bus
			settled.SteeringAngle = targetWheel
			settledFrame := f
			settledFrame.bus = &settled
			if ad.controlSweptSpeed(settledFrame, cand) >= 16 && ad.articulatedSweptSpeed(settledFrame, cand, true) >= 16 && roundaboutUnwindClearHorizon(settledFrame, cand, exitingOutsideRing) {
				return cand, true, true
			}
		}
	}
	return 0, false, false
}

// Verify that the trailing section also clears stationary nearby vehicles over
// a 48px passing horizon so relaxing an unwind steer early cannot clip a car
// beside the rear half of the trailer beyond articulatedSweptSpeed's 12.8px window.
func roundaboutUnwindClearLong(f autopilotFrame, steer float64) bool {
	return roundaboutUnwindClearHorizon(f, steer, false)
}

func roundaboutUnwindClearHorizon(f autopilotFrame, steer float64, exitingOutsideRing bool) bool {
	if f.bus == nil || !f.bus.IsArticulated() || f.world == nil {
		return true
	}
	var closeObs [32]sim.OBB
	nObs := 0
	reach := f.bus.OverallLength() + 90
	for _, ob := range f.obstacles {
		if math.Abs(ob.Speed) > 1 || ob.Pos.Distance(f.bus.Pos) > reach+ob.Length {
			continue
		}
		for _, body := range predictedObstacleBodies(ob, 0) {
			if body.HalfLen <= 0 {
				continue
			}
			// Keep a positive skin margin if currently separated beyond it.
			skinBody := sim.OBB{Center: body.Center, Heading: body.Heading, HalfLen: body.HalfLen + .15, HalfWid: body.HalfWid + .15}
			if hit, _, _ := f.bus.CollidesBody(skinBody); !hit {
				body = skinBody
			}
			if nObs < len(closeObs) {
				closeObs[nObs] = body
				nObs++
			}
		}
	}
	copy := *f.bus
	speed := math.Max(16, f.speedAbs)
	steps := 60
	if exitingOutsideRing {
		steps = 28
	}
	for step := 1; step <= steps; step++ {
		wheel := copy.SteeringAngle + sim.Clamp(steer*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
		angle := copy.Heading + speed/copy.WheelBase*math.Tan(wheel)*.05
		pos := copy.Pos.Add(sim.UnitFromAngle(angle).Mul(speed * .05))
		copy = copy.AtPose(pos, angle)
		copy.SteeringAngle = wheel
		if step%4 == 0 && copy.HitsHardBarrier(f.world, f.laneWidth) {
			return false
		}
		parts := copy.BodyParts()
		for i := 0; i < nObs; i++ {
			for _, own := range parts {
				if own.HalfLen <= 0 {
					continue
				}
				if touching, _, depth := sim.CheckOBBCollision(own, closeObs[i]); touching && depth > 0 {
					return false
				}
			}
		}
	}
	return true
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
		if edgeRecoveryIgnoresObstacle(f, b, ob) || ob.Pos.Distance(b.Pos) > b.OverallLength()+ob.Length+math.Abs(ob.Speed)*elapsed+40 {
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

// Long-horizon planning inside a roundabout must not project distant highway
// traffic or vehicles still queued outside the yield boundary straight through
// active queues and yield lines into the bus's adjustment slot.
func edgeRecoveryIgnoresObstacle(f autopilotFrame, b *sim.Bus, ob VehicleObstacle) bool {
	if f.bus != nil && ob.Pos.Distance(f.bus.Pos) > b.OverallLength()+ob.Length+220 {
		return true
	}
	if f.world == nil {
		return false
	}
	for _, rb := range f.world.Roundabouts {
		if b.Pos.Distance(rb.Center) < rb.Radius+15 && ob.Pos.Distance(rb.Center) > rb.Radius+ob.Length/2+28 {
			return true
		}
	}
	return false
}

func (ad *AutopilotDriver) tryRoundaboutEdgeRecovery(f autopilotFrame, steer float64) bool {
	if !f.bus.IsArticulated() || f.speedAbs > .1 || f.world == nil || f.bus.DoorsOpen || ad.roundaboutEdgeRetry > 0 {
		return false
	}
	near := false
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+15 {
			near = true
			break
		}
	}
	if !near {
		return false
	}
	zone := f.bus.CheckBoundaries(f.world, f.laneWidth).Zone
	if zone != sim.ZoneShoulder && math.Abs(steer) < .5 {
		return false
	}
	empty := f
	empty.obstacles = nil
	if ad.articulatedSweptSpeed(empty, steer) > 0 {
		return false
	}
	ad.roundaboutEdgeRetry = 1
	primary := -math.Copysign(1, steer)
	if ad.planEdgeRecovery(f, primary) {
		return true
	}
	return ad.planEdgeRecovery(f, -primary)
}

// Check the reverse path and the forward turn before issuing a backing maneuver.
func (ad *AutopilotDriver) planEdgeRecovery(f autopilotFrame, direction float64) bool {
	// Never turn a traffic wait into a reverse maneuver. The current full
	// body and every subsequent sample must be physically separated.
	if !edgeRecoveryPoseClear(f, f.bus, 0) {
		return false
	}
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
