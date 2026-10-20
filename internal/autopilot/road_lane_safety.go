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
	if f.world == nil || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return steer
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
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	if !multi && !six {
		return steer
	}
	heading := 0.0
	if ew {
		if math.Cos(f.bus.Heading) < 0 {
			heading = math.Pi
		}
	} else if math.Sin(f.bus.Heading) > 0 {
		heading = math.Pi / 2
	} else {
		heading = -math.Pi / 2
	}
	// Straight-road lane protection must agree with the route's road axis.
	// A crossing road underneath a corner or a bay taper is not its centerline.
	tangent := ad.pointAt(f.s + 25).Sub(ad.pointAt(f.s)).Normalize()
	if math.Abs(tangent.Dot(sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)})) < .97 && !ad.onDepartureRoad(f, center, ew, heading) {
		return steer
	}

	base := ad.pointAt(f.s)
	baseLateral := base.Y - center
	if ew && math.Cos(heading) < 0 {
		baseLateral = -baseLateral
	}
	if !ew {
		baseLateral = base.X - center
		if math.Sin(heading) > 0 {
			baseLateral = -baseLateral
		}
	}
	if baseLateral < f.bus.Width/2+2 {
		return steer
	}

	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if math.Abs(yaw) > 1 {
		return steer
	}
	// Limit the approach angle as well as the swept envelope: entering a
	// roundabout must not inherit a steep yaw from a last-second lane change.
	if yaw < -.28 {
		steer = math.Max(steer, sim.Clamp((-yaw-.28)*6+.25, .25, 1))
	} else if yaw < -.20 && steer < 0 {
		steer = 0
	}
	// Curved ring approaches use the ring's radial envelope, rather than
	// extending the straight road's centerline through the entry curve.
	if nearRoundabout {
		return steer
	}
	// Predict the steering actuator and both body corners. A target-point clamp
	// alone cannot arrest an ongoing lane change at high speed.
	safe := func(input, margin float64) bool {
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
			pos = pos.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(f.speedAbs * step))
			if f.world.InsideJunction(pos, f.laneWidth, 0) {
				return true
			}
			for _, rb := range f.world.Roundabouts {
				if pos.Distance(rb.Center) <= rb.Radius+80 {
					return true
				}
			}
			lateral := pos.Y - center
			if ew && math.Cos(heading) < 0 {
				lateral = -lateral
			}
			if !ew {
				lateral = pos.X - center
				if math.Sin(heading) > 0 {
					lateral = -lateral
				}
			}
			relative := angle - heading
			clearance := lateral - f.bus.Length/2*math.Abs(math.Sin(relative)) - f.bus.Width/2*math.Abs(math.Cos(relative))
			if clearance < margin {
				return false
			}
		}
		return true
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
	// No steering input can clear the current swept envelope at this speed.
	// Brake and unwind yaw gently instead of asserting that full right is safe.
	ad.roadSafetyHold = true
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
	forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	along := f.bus.Pos.Sub(stop.RoadCenter).Dot(forward)
	return along >= -f.bus.Length && along <= 400
}
