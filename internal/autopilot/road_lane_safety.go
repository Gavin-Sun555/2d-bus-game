package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// The whole bus, rather than only its pursuit point, must remain on its side of
// a multi-lane road. Start unwinding steering early enough to allow for the
// steering response and forward motion at the configured berserk speed.
func (ad *AutopilotDriver) roadLaneSafeSteering(f autopilotFrame, steer float64) float64 {
	ad.roadSafetyHold = false
	ad.roadRecoverySpeed = 0
	if f.world == nil {
		return steer
	}
	straightJunction := !ad.DisallowOncomingOvertake && ad.straightThroughJunctionCorridor(f)
	if f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) && !straightJunction {
		return steer
	}
	// The front section may have left the painted box while the articulated
	// rear is still turning through it. Keep the explicit turn and full-body
	// swept checks authoritative until the entire vehicle has cleared.
	if f.bus.IsArticulated() && (ad.rightTurnJunction != nil || ad.LastLateralErr > 2*f.laneWidth) {
		for _, part := range f.bus.BodyParts() {
			if part.HalfLen > 0 {
				if ad.rightTurnJunction != nil {
					center := *ad.rightTurnJunction
					g := f.world.JunctionAt(center, f.laneWidth)
					box := sim.OBB{Center: center, HalfLen: g.HalfX, HalfWid: g.HalfY}
					if hit, _, _ := sim.CheckOBBCollision(part, box); hit {
						return steer
					}
				} else if f.world.InsideJunction(part.Center, f.laneWidth, part.HalfLen) {
					return steer
				}
			}
		}
	}
	nearRoundabout := false
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			nearRoundabout = true
		}
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+80 {
			return steer
		}
	}
	roadHeadingHint := f.bus.Heading
	if len(ad.drivePath) >= 2 && ad.uTurn == nil && ad.busLaneJunction == nil && ad.rightTurnJunction == nil && ad.fallbackJunction == nil {
		curCenter, curEW, curMulti, curSix := sim.CurrentRoadInfo(f.world, f.bus.Pos, roadHeadingHint, f.laneWidth)
		onCurRoad := false
		if curMulti || curSix {
			if road, ok := f.world.RoadAt(curCenter, curEW, f.bus.Pos); ok {
				lat := math.Abs(f.bus.Pos.X - curCenter)
				if curEW {
					lat = math.Abs(f.bus.Pos.Y - curCenter)
				}
				onCurRoad = lat <= float64(road.LaneCount())*.5*f.laneWidth+48
			}
		}
		if !ad.DisallowOncomingOvertake || !onCurRoad {
			seg := ad.pointAt(f.s + 25).Sub(ad.pointAt(f.s))
			if seg.Length() > 1 {
				th := math.Atan2(seg.Y, seg.X)
				if math.Abs(sim.NormalizeAngle(f.bus.Heading-th)) > .5 && math.Abs(sim.NormalizeAngle(f.bus.Heading-th)) < 1.15 {
					roadHeadingHint = th
				}
			}
		}
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, roadHeadingHint, f.laneWidth)
	heading := 0.0
	if ew {
		if math.Cos(roadHeadingHint) < 0 {
			heading = math.Pi
		}
	} else if math.Sin(roadHeadingHint) > 0 {
		heading = math.Pi / 2
	} else {
		heading = -math.Pi / 2
	}
	narrowDeparture := !multi && !six
	if narrowDeparture {
		// A bay exit can carry inward yaw after a traffic stop.
		// During that departure, protect the two-way centerline just as on
		// an avenue. Deliberate turns and mid-block borrowing are handled
		// by their own lane policies.
		if (f.bus.IsArticulated() && !ad.FleetBus) || ad.dockDepartureRun <= f.s || !ad.onDepartureRoad(f, center, ew, heading) {
			return steer
		}
	}
	// Straight-road lane protection must agree with the route's road axis.
	// A crossing road underneath a corner or a bay taper is not its centerline.
	tangent := ad.pointAt(f.s + 25).Sub(ad.pointAt(f.s)).Normalize()
	onApproach := false
	if ad.busLaneJunction != nil && !ad.busJunctionEntered {
		across := ad.busLaneJunction.Y
		if !ew {
			across = ad.busLaneJunction.X
		}
		onApproach = math.Abs(across-center) < 1 && ad.busLaneJunction.Sub(f.bus.Pos).Dot(sim.UnitFromAngle(heading)) > 0
	}
	if math.Abs(tangent.Dot(sim.UnitFromAngle(heading))) < .97 && !onApproach && !ad.onDepartureRoad(f, center, ew, heading) {
		return steer
	}

	lateralAt := func(pos sim.Vec2) float64 {
		lat := pos.Y - center
		if ew && math.Cos(heading) < 0 {
			lat = -lat
		}
		if !ew {
			lat = pos.X - center
			if math.Sin(heading) > 0 {
				lat = -lat
			}
		}
		return lat
	}
	baseLateral := lateralAt(ad.pointAt(f.s))
	if baseLateral < f.bus.Width/2+2 {
		return steer
	}

	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if math.Abs(yaw) > 1 {
		return steer
	}
	// Curved ring approaches use the ring's radial envelope, rather than
	// extending the straight road's centerline through the entry curve.
	// Predict the steering actuator and both body corners. A target-point clamp
	// alone cannot arrest an ongoing lane change at high speed.
	safe := func(input, margin float64) bool {
		pos, angle, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
		const step = .02
		horizon := .40
		speed := f.speedAbs
		if narrowDeparture {
			speed = math.Max(20, speed)
		}
		if !ad.DisallowOncomingOvertake {
			horizon = math.Max(.40, math.Min(.68, f.speedAbs/480))
		}
		for t := 0.0; t < horizon; t += step {
			target := input * f.bus.MaxSteerAngle
			rate := f.bus.SteerTurnSpeed
			if input == 0 {
				rate = f.bus.SteerReturnSpd
			}
			steering += sim.Clamp(target-steering, -rate*step, rate*step)
			angle += speed / f.bus.WheelBase * math.Tan(steering) * step
			pos = pos.Add(sim.UnitFromAngle(angle).Mul(speed * step))
			if f.world.InsideJunction(pos, f.laneWidth, 0) && !straightJunction {
				return true
			}
			for _, rb := range f.world.Roundabouts {
				if pos.Distance(rb.Center) <= rb.Radius+80 {
					return true
				}
			}
			lateral := lateralAt(pos)
			relative := angle - heading
			clearance := lateral - f.bus.Length/2*math.Abs(math.Sin(relative)) - f.bus.Width/2*math.Abs(math.Cos(relative))
			if clearance < margin {
				return false
			}
		}
		return true
	}
	allowGentleRightUnwind := f.allowBarrierUnwindYaw && f.speedAbs < 16 &&
		((steer >= .20 && safe(steer, 2.0)) || (yaw >= -.55 && lateralAt(f.bus.Pos) >= 2*f.laneWidth && safe(steer, 4.0)))
	if !allowGentleRightUnwind {
		if yaw < -.28 && !onApproach {
			steer = math.Max(steer, sim.Clamp((-yaw-.28)*6+.25, .25, 1))
		} else if yaw < -.20 && steer < 0 && !onApproach {
			steer = 0
		}
	}
	if nearRoundabout {
		return steer
	}
	// Prefer the daylight margin, but an already-small gap must not turn an
	// otherwise safe recovery into full-lock steering across the whole road.
	for _, margin := range []float64{2, .25} {
		if safe(steer, margin) {
			return steer
		}
		for candidate := math.Max(0, steer); candidate < 1; candidate += .05 {
			if safe(candidate, margin) {
				return candidate
			}
		}
	}
	// A lane change that ended a few pixels over the centerline cannot meet
	// either margin from its current pose. Keep driving if the steering only
	// brings the body back toward its side; stopping there gains nothing.
	if current := lateralAt(f.bus.Pos) - f.bus.Length/2*math.Abs(math.Sin(yaw)) - f.bus.Width/2*math.Abs(math.Cos(yaw)); !narrowDeparture && current < .25 && current > -f.bus.Width/4 {
		// The body must never cross more than a pixel further than it is now,
		// and its center must be moving back toward its own side.
		returning := func(input float64) bool {
			pos, angle, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
			const step = .02
			for t := 0.0; t < .40; t += step {
				target := input * f.bus.MaxSteerAngle
				rate := f.bus.SteerTurnSpeed
				if input == 0 {
					rate = f.bus.SteerReturnSpd
				}
				steering += sim.Clamp(target-steering, -rate*step, rate*step)
				angle += f.speedAbs / f.bus.WheelBase * math.Tan(steering) * step
				pos = pos.Add(sim.UnitFromAngle(angle).Mul(f.speedAbs * step))
				relative := angle - heading
				if lateralAt(pos)-f.bus.Length/2*math.Abs(math.Sin(relative))-f.bus.Width/2*math.Abs(math.Cos(relative)) < current-1 {
					return false
				}
			}
			return f.speedAbs < 1 || lateralAt(pos) > lateralAt(f.bus.Pos)
		}
		if returning(steer) {
			return steer
		}
		// A gentle return keeps the far corner from swinging over the line.
		for candidate := .05; candidate <= .5; candidate += .05 {
			if returning(candidate) {
				return candidate
			}
		}
	}
	// No steering input can clear the current swept envelope at this speed.
	// Brake and unwind yaw gently instead of asserting that full right is safe.
	recoverySteer, canRecover := ad.roadLaneRecovery(f, steer, heading, center, ew)
	if canRecover {
		return recoverySteer
	}
	ad.roadSafetyHold = true
	if ad.roadRecoverySpeed > 0 {
		return recoverySteer
	}
	return sim.Clamp(-yaw*2+.08, -.25, .25)
}

// A bay-to-lane taper is deliberately diagonal. Its tangent must not disable
// protection against the centerline of the same road during pull-out.
func (ad *AutopilotDriver) onDepartureRoad(f autopilotFrame, center float64, ew bool, heading float64) bool {
	if ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) {
		return false
	}
	stop := f.world.Stops[ad.pathFromID]
	if math.Cos(stop.Heading-heading) < .99 {
		return false
	}
	roadCenter := stop.RoadCenter.Y
	if !ew {
		roadCenter = stop.RoadCenter.X
	}
	if math.Abs(roadCenter-center) > 1 {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	along := f.bus.Pos.Sub(stop.RoadCenter).Dot(forward)
	return along >= -f.bus.Length && along <= 400
}

func (ad *AutopilotDriver) straightThroughJunctionCorridor(f autopilotFrame) bool {
	if f.world == nil || len(ad.drivePath) < 2 || f.remaining < 400 ||
		ad.uTurn != nil || ad.busLaneJunction != nil || ad.rightTurnJunction != nil || ad.fallbackJunction != nil {
		return false
	}
	base := ad.pointAt(f.s)
	first := ad.pointAt(f.s + 40).Sub(base)
	if first.Length() < 39 {
		return false
	}
	u := first.Normalize()
	if math.Abs(u.X) > .02 && math.Abs(u.Y) > .02 {
		return false
	}
	if u.Dot(sim.UnitFromAngle(f.bus.Heading)) < .42 {
		return false
	}
	for _, d := range []float64{100, 220} {
		v := ad.pointAt(f.s + d).Sub(base)
		if v.Length() < d-1 || v.Normalize().Dot(u) < .9999 {
			return false
		}
	}
	return true
}
