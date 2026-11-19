package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A mid-block lane change can leave a follower beside a yawed leader's
// trailer. Unlike an ordinary queue, moving the leader then sweeps its tail
// into that follower. Retreat only after proving both the full reverse sweep
// and the leader's newly available forward sweep.
func (ad *AutopilotDriver) laneChangeFollowerRetreatDistance(f autopilotFrame) float64 {
	distance, _ := ad.laneChangeFollowerRetreatPlan(f)
	return distance
}

func (ad *AutopilotDriver) laneChangeFollowerRetreatPlan(f autopilotFrame) (float64, float64) {
	if !ad.FleetBus || f.world == nil || f.speedAbs > 1 || f.bus.DoorsOpen ||
		f.bus.PassengerServiceRemaining() > 0 || ad.State == AutopilotStateDocked || f.nearRoundabout ||
		ad.trafficLightSpeed(f, 30) <= .5 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return 0, 0
	}
	heading := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .65 || math.Abs(sim.NormalizeAngle(f.bus.TailHeading()-heading)) > .65 {
		return 0, 0
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	retreatSteers := []float64{0}
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .15 || math.Abs(sim.NormalizeAngle(f.bus.Heading-f.bus.TailHeading())) > .2 {
		// A tilted cab may reach its articulation limit in straight reverse.
		// Try settled-wheel arcs only for that case, and prove each full path.
		retreatSteers = []float64{0, .25, .5, -.25, -.5, 1, -1}
	}
	for _, ob := range f.obstacles {
		leader := ob.Bus
		if leader == nil || !leader.IsArticulated() || leader.DoorsOpen || leader.PassengerServiceRemaining() > 0 || math.Abs(ob.Speed) > 1 {
			continue
		}
		delta := leader.Pos.Sub(f.bus.Pos)
		yaw := math.Abs(sim.NormalizeAngle(leader.Heading - heading))
		turningWheels := math.Abs(leader.SteeringAngle/leader.MaxSteerAngle) >= .35 || math.Abs(sim.NormalizeAngle(leader.Heading-leader.TailHeading())) >= .1
		if delta.Dot(u) < f.bus.Length/2 || delta.Dot(u) > leader.OverallLength()+50 ||
			math.Abs(delta.Dot(r)) > 1.6*f.laneWidth || (yaw < .2 && !turningWheels) || yaw > .65 {
			continue
		}
		probe := NewAutopilotDriver()
		blocked := autopilotFrame{bus: leader, world: f.world, laneWidth: f.laneWidth, obstacles: []VehicleObstacle{ObserveBus(f.bus)}}
		steer := sim.Clamp(leader.SteeringAngle/leader.MaxSteerAngle, -1, 1)
		if probe.articulatedSweptSpeed(blocked, steer, true) > .5 && probe.controlSweptSpeed(blocked, steer) > .5 {
			continue
		}
		emptyLeader := blocked
		emptyLeader.obstacles = nil
		leaderCheckSteers := []float64{steer, 0, -.25, .25, -.65, .65}
		if yaw < .2 {
			leaderCheckSteers = leaderCheckSteers[:1]
		}
		leaderBarrierClear := false
		for _, candidate := range leaderCheckSteers {
			if probe.articulatedSweptSpeed(emptyLeader, candidate, true) >= 16 && probe.controlSweptSpeed(emptyLeader, candidate) >= 16 {
				leaderBarrierClear = true
				break
			}
		}
		if !leaderBarrierClear {
			continue
		}
		for _, askRearCars := range []bool{false, true} {
			retreatFrame := f
			var requests rearReverseYield
			if askRearCars {
				retreatFrame, requests = rearReverseYieldFrame(f)
				if requests.empty() {
					continue
				}
			}
			for _, distance := range []float64{40, 24, 16, 8} {
				for _, retreatSteer := range retreatSteers {
					aligned := retreatFrame
					presteered := *f.bus
					if retreatSteer != 0 {
						presteered.SteeringAngle = retreatSteer * presteered.MaxSteerAngle
						aligned.bus = &presteered
					}
					follower, clear := ad.simulateReversePose(aligned, distance, retreatSteer)
					if !clear {
						continue
					}
					clearsLeader := func(pose *sim.Bus) bool {
						pose.Speed = 0
						trial := blocked
						trial.obstacles = []VehicleObstacle{ObserveBus(pose)}
						for _, other := range retreatFrame.obstacles {
							if other.Bus != leader {
								trial.obstacles = append(trial.obstacles, other)
							}
						}
						steers := []float64{steer, 0, -.25, .25, -.65, .65}
						if yaw < .2 {
							steers = steers[:1]
						}
						for _, candidate := range steers {
							if probe.articulatedSweptSpeed(trial, candidate, true) >= 16 && probe.controlSweptSpeed(trial, candidate) >= 16 {
								return true
							}
						}
						return false
					}
					opens := clearsLeader(follower)
					// A bent trailer may need two short reversing arcs rather
					// than one long arc that exceeds its articulation limit.
					// Prove the continuation too before taking the first step;
					// runtime stops and rechecks all traffic between segments.
					if !opens && retreatSteer != 0 {
						for _, nextSteer := range []float64{.5, 1, .25, 0, -.25, -.5, -1} {
							for _, nextDistance := range []float64{16, 24, 40} {
								post := *follower
								post.SteeringAngle = nextSteer * post.MaxSteerAngle
								nextFrame := retreatFrame
								nextFrame.bus = &post
								nextFrame.speedAbs = 0
								next, ok := ad.simulateReversePose(nextFrame, nextDistance, nextSteer)
								if ok && clearsLeader(next) {
									opens = true
									break
								}
							}
							if opens {
								break
							}
						}
					}
					if opens {
						if requests.empty() {
							return distance, retreatSteer
						}
						requests.send()
						return 0, 0
					}
				}
			}
		}
	}
	return 0, 0
}
