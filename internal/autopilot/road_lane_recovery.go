package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func roadMedianClearance(bus *sim.Bus, pos sim.Vec2, angle, heading, center float64, ew bool) float64 {
	normal := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	yaw := angle - heading
	return pos.Sub(origin).Dot(normal) - bus.Length/2*math.Abs(math.Sin(yaw)) - bus.Width/2*math.Abs(math.Cos(yaw))
}

// A stopped bus needs a checked crawl to regain its lane. Small intrusions
// must improve body clearance throughout. A stranded median merge may also
// return from the opposing side while its center moves toward its own lane.
func (ad *AutopilotDriver) roadLaneRecovery(f autopilotFrame, steer, heading, center float64, ew bool) (float64, bool) {
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	clearance := roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew)
	minimumClearance := -math.Min(f.bus.Width/2, f.laneWidth*.25)
	strandedApproach := clearance < minimumClearance && ad.ordinaryBusLeft && ad.busLaneJunction != nil && !ad.busJunctionEntered
	// A restored lane change can straddle the median, including with its cab
	// center just beyond it. While the cab still overlaps its lawful side,
	// allow only a bounded, improving, obstacle-checked crawl below.
	cabLateral := roadMedianClearance(f.bus, f.bus.Pos, heading, heading, center, ew) + f.bus.Width/2
	medianStraddle := clearance < minimumClearance && math.Abs(yaw) <= .4 &&
		(cabLateral > 0 || (cabLateral > -f.bus.Width/2 && math.Abs(yaw) <= .15)) &&
		f.bus.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneRoad
	if f.speedAbs > 25 || math.Abs(yaw) > .85 || (math.Abs(yaw) > .4 && (clearance < minimumClearance || strandedApproach)) || (clearance < minimumClearance && !strandedApproach && !medianStraddle) {
		return steer, false
	}
	if strandedApproach && f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneRoad {
		return steer, false
	}
	const recoverySpeed = 20.
	ad.roadRecoverySpeed = recoverySpeed
	sweepSpeed := math.Max(recoverySpeed, f.speedAbs)
	candidates := []float64{sim.Clamp(steer, -.65, .65), 0, sim.Clamp(-yaw*3, -.65, .65), -.65, .65}
	if strandedApproach {
		candidates[0] = sim.Clamp((.3-yaw)*3, -.65, .65)
	}
	for _, gentle := range []bool{false, true} {
		if gentle && (strandedApproach || (clearance < minimumClearance && !medianStraddle) || minimumClearance >= -1.5) {
			break
		}
		horizon := .8
		passCandidates := candidates
		if gentle {
			horizon = 6.0
			passCandidates = []float64{.05, -.05, .08, -.08, .12, -.12}
		}
		for _, input := range passCandidates {
			pos, angle, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
			previous := clearance
			safe := true
			const dt = .02
			for elapsed := dt; elapsed <= horizon; elapsed += dt {
				rate := f.bus.SteerTurnSpeed
				if input == 0 {
					rate = f.bus.SteerReturnSpd
				}
				steering += sim.Clamp(input*f.bus.MaxSteerAngle-steering, -rate*dt, rate*dt)
				angle += sweepSpeed / f.bus.WheelBase * math.Tan(steering) * dt
				pos = pos.Add(sim.UnitFromAngle(angle).Mul(sweepSpeed * dt))
				next := roadMedianClearance(f.bus, pos, angle, heading, center, ew)
				minimum := math.Min(previous, 2) - .002
				if gentle {
					minimum = math.Max(minimumClearance, math.Min(clearance, 0)-1.0)
					if medianStraddle {
						minimum = clearance - 1
					}
				}
				if strandedApproach {
					// A saved bus may point slightly farther across the median.
					// Check center progress while allowing the tail to swing through
					// the obstacle-tested return, rather than freezing it on that side.
					next = pos.Sub(f.bus.Pos).Dot(sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)})
					minimum = -.25
				}
				if next < minimum {
					safe = false
					break
				}
				previous = next
				body := sim.OBB{Center: pos, Heading: angle, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
				for _, part := range f.bus.BodyPartsAtPose(pos, angle) {
					if part.HalfLen <= 0 {
						continue
					}
					part.HalfLen++
					part.HalfWid++
					for _, corner := range sim.BodyCorners(part) {
						if f.world.CheckBoundaries(corner, 0, f.laneWidth).Zone == sim.ZoneHardBarrier {
							safe = false
						}
					}
				}
				if !safe {
					break
				}
				startBody := sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
				roadFwd := sim.UnitFromAngle(heading)
				for _, ob := range f.obstacles {
					reach := 200 + math.Abs(ob.Speed)*.8
					if gentle {
						reach = math.Max(reach, f.bus.OverallLength()+ob.Length+(sweepSpeed+math.Abs(ob.Speed))*horizon)
					}
					if ob.Pos.Distance(f.bus.Pos) > reach {
						continue
					}
					rearStationary := ob.Pos.Sub(f.bus.Pos).Dot(roadFwd) < -f.bus.Length/2 && math.Abs(ob.Speed) <= 5
					for _, part := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
						if part.HalfLen <= 0 {
							continue
						}
						part.HalfLen++
						part.HalfWid++
						if hit, _, _ := sim.CheckBusAtBody(f.bus, body, part); hit {
							if rearStationary {
								if startParts := predictedObstacleBodies(ob, 0); len(startParts) > 0 {
									alreadyTouching := false
									for _, sp := range startParts {
										if sp.HalfLen <= 0 {
											continue
										}
										sp.HalfLen++
										sp.HalfWid++
										if sHit, _, _ := sim.CheckBusAtBody(f.bus, startBody, sp); sHit {
											alreadyTouching = true
											break
										}
									}
									if alreadyTouching {
										continue
									}
								}
							}
							safe = false
							break
						}
					}
					if !safe {
						break
					}
				}
				if !safe {
					break
				}
			}
			improved := previous > clearance+.1
			if strandedApproach {
				improved = previous > .1
			}
			if safe && improved {
				return input, true
			}
		}
	}
	// Turn the wheels while stationary before retrying the checked crawl.
	return sim.Clamp(-yaw*3, -.65, .65), false
}
