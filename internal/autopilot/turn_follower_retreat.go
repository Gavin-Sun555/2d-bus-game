package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A right-turning leader can be pinned by its own tail sweep into a close
// follower. Back the follower only when its complete reverse is clear AND
// the newly opened space permits a real forward trajectory for the leader.
// The follower may already have nosed into the same turn beside the leader
// (both yawed, follower a lane to the left); the same proof then frees it.
func (ad *AutopilotDriver) turnFollowerRetreatDistance(f autopilotFrame) float64 {
	request := ad.exitCruise
	if request == nil || request.Joined || f.world == nil || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || f.speedAbs > 1 || ad.State == AutopilotStateDocked || f.bus.RoundaboutAdmissionHold != nil {
		return 0
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	turn := sim.NormalizeAngle(request.Heading - heading)
	followerYaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if turn < .7 || turn > 2 || followerYaw < -.15 || followerYaw > .5 || math.Cos(f.bus.TailHeading()-heading) < .98 || f.bus.Pos.Distance(request.Junction) > 700 || f.world.IsAtRoundabout(request.Junction) || ad.trafficLightSpeed(f, 30) <= .5 {
		return 0
	}
	for i, ob := range f.obstacles {
		leader := ob.Bus
		if leader == nil || !leader.IsArticulated() || leader.DoorsOpen || math.Abs(ob.Speed) > 1 {
			continue
		}
		delta := leader.Pos.Sub(f.bus.Pos)
		yaw := sim.NormalizeAngle(leader.Heading - heading)
		// Directly behind: the leader must already be well into its turn.
		// Beside it (a lane to the left), both have only begun to turn.
		lateral := delta.Dot(right)
		alongside := lateral > f.laneWidth*.5 && lateral <= f.laneWidth*1.6 && followerYaw > .15
		minYaw := .4
		if alongside {
			minYaw = .25
		}
		if delta.Dot(u) < 20 || delta.Dot(u) > f.bus.Length+40 || (math.Abs(lateral) > f.laneWidth && !alongside) || yaw < minYaw || yaw > 1.3 || math.Cos(leader.TailHeading()-heading) < .9 {
			continue
		}
		distances := []float64{24, 18, 12, 8}
		if alongside {
			distances = []float64{60, 45, 30, 24}
		}
		for _, distance := range distances {
			if !ad.laneRetreatReverseClear(f, distance, 0) {
				continue
			}
			follower := *f.bus
			for moved := 0.; moved < distance; moved += .25 {
				follower = edgeRecoveryPose(follower, -.25, 0)
			}
			obstacles := make([]VehicleObstacle, 0, len(f.obstacles))
			obstacles = append(obstacles, ObserveBus(&follower))
			for j, other := range f.obstacles {
				if i != j {
					obstacles = append(obstacles, other)
				}
			}
			trial := autopilotFrame{bus: leader, world: f.world, obstacles: obstacles, laneWidth: f.laneWidth}
			probe := NewAutopilotDriver()
			for _, steer := range []float64{-.25, -.5, -1, 0} {
				if probe.stationForwardEscapeClear(trial, steer, 18, leader.Pos.Add(u.Mul(100)), 10) {
					return distance
				}
			}
		}
	}
	return 0
}
