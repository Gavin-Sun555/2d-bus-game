package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A stationary parallel car may touch only the padded sweep of a yawed bus.
// Search a slow, real actuator trajectory; never waive physical body checks or
// use the opposing half of the road to escape that artificial stop.
func (ad *AutopilotDriver) sidecarEscape(f autopilotFrame, desired float64) (float64, bool) {
	if f.world == nil || !f.bus.IsArticulated() || f.speedAbs > 25 || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.roadSafetyHold || ad.State == AutopilotStateDocked || f.bus.RoundaboutAdmissionHold != nil || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) || ad.trafficLightSpeed(f, 30) <= .5 {
		ad.sidecarPresteerValid = false
		ad.sidecarEscapeActive = false
		return 0, false
	}
	if ad.sidecarEscapeRetry > 0 {
		if ad.sidecarPresteerValid {
			return ad.sidecarPresteer, false
		}
		return 0, false
	}
	ad.sidecarPresteerValid = false
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			ad.sidecarEscapeActive, ad.sidecarPresteerValid = false, false
			return 0, false
		}
	}
	heading := approachRoadHeading(f.bus.Heading)
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	if (!multi && !six) || math.Cos(f.bus.Heading-heading) < .9 {
		ad.sidecarEscapeActive, ad.sidecarPresteerValid = false, false
		return 0, false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	away := 0.
	var near [128]VehicleObstacle
	n := 0
	for _, ob := range f.obstacles {
		if stoppedSidecar(f, ob, heading) {
			away = -math.Copysign(1, ob.Pos.Sub(f.bus.Pos).Dot(r))
		}
		d := ob.Pos.Sub(f.bus.Pos)
		reach := f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*2 + 100
		if d.Dot(d) <= reach*reach {
			if n < len(near) {
				near[n] = ob
				n++
			} else {
				n = -1
				break
			}
		}
	}
	if away == 0 {
		ad.sidecarEscapeActive = false
		return 0, false
	}
	if n >= 0 {
		f.obstacles = near[:n]
	}
	ad.sidecarEscapeRetry = .2
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	for _, gain := range []float64{3, 5, 8, 1} {
		steer := sim.Clamp(-yaw*gain, -1, 1)
		if ad.sidecarSweepClear(f, steer, heading, center, ew, gain) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
	}
	choices := []float64{away * .3, away * .1, 0, desired, away * .2, away * .5, away * .75}
	for _, input := range choices {
		steer := sim.Clamp(input, -1, 1)
		if ad.sidecarSweepClear(f, steer, heading, center, ew) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
	}
	for _, gain := range []float64{3, 5, 8, 1} {
		steer := sim.Clamp(-yaw*gain, -1, 1)
		parked := *f.bus
		parked.SteeringAngle = steer * parked.MaxSteerAngle
		aligned := f
		aligned.bus = &parked
		if ad.sidecarSweepClear(aligned, steer, heading, center, ew, gain) {
			ad.sidecarPresteer, ad.sidecarPresteerValid = steer, true
			ad.sidecarEscapeActive = true
			return steer, false
		}
	}

	// Steering at rest changes neither vehicle position nor trailer angle.
	// Prove the subsequent moving trajectory with the settled wheel first.
	for _, input := range choices {
		steer := sim.Clamp(input, -1, 1)
		parked := *f.bus
		parked.SteeringAngle = steer * parked.MaxSteerAngle
		aligned := f
		aligned.bus = &parked
		if ad.sidecarSweepClear(aligned, steer, heading, center, ew) {
			ad.sidecarPresteer, ad.sidecarPresteerValid = steer, true
			ad.sidecarEscapeActive = true
			return steer, false
		}
	}
	return 0, false
}

func stoppedSidecar(f autopilotFrame, ob VehicleObstacle, heading float64) bool {
	if ob.Bus != nil || ob.Length > 65 || math.Abs(ob.Speed) >= 1 || math.Cos(ob.Heading-heading) < .9 {
		return false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	d := ob.Pos.Sub(f.bus.Pos)
	return math.Abs(d.Dot(u)) < f.bus.OverallLength()+ob.Length && math.Abs(d.Dot(r)) > f.bus.Width/2 && math.Abs(d.Dot(r)) < (f.bus.Width+ob.Width)/2+f.laneWidth*.5
}

func (ad *AutopilotDriver) sidecarSweepClear(f autopilotFrame, steer, heading, center float64, ew bool, yawGain ...float64) bool {
	// This narrow recovery predicts forward traffic only. Defer to normal
	// avoidance while a nearby vehicle is reversing.
	for _, ob := range f.obstacles {
		if ob.Speed < 0 {
			return false
		}
	}
	b := *f.bus
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	for tick := 0; tick <= 120; tick++ {
		if tick > 0 {
			if len(yawGain) > 0 {
				steer = sim.Clamp(-sim.NormalizeAngle(b.Heading-heading)*yawGain[0], -1, 1)
			}
			th, br := ad.longitudinalControls(b.Speed, 18)
			b.SetNextAccelerationScale(1)
			b.Update(1./60, th, br, steer, false, false)
		}
		if b.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier || math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
			return false
		}
		parts := b.BodyParts()
		for _, part := range parts {
			if part.HalfLen > 0 {
				yaw := part.Heading - heading
				if part.Center.Sub(origin).Dot(r)-part.HalfLen*math.Abs(math.Sin(yaw))-part.HalfWid*math.Abs(math.Cos(yaw)) < .5 {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			padding := 2.
			if stoppedSidecar(f, ob, heading) {
				padding = .05
			}
			for _, other := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*float64(tick)/60) {
				if other.HalfLen > 0 {
					other.HalfLen += padding
					other.HalfWid += padding
					for _, part := range parts {
						if part.HalfLen <= 0 {
							continue
						}
						// CollidesBody only reports a contact at positive depth.
						if hit, _, depth := sim.CheckOBBCollision(part, other); hit && depth > 0 {
							return false
						}
					}
				}
			}
		}
	}
	return b.Pos.Sub(f.bus.Pos).Dot(u) > 10
}
