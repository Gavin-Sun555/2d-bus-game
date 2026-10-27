package autopilot

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

// Each cache belongs to one obstacleSpeed invocation, whose bus pose, path,
// station and lane offset remain fixed. Never reuse predicted articulation
// after controls advance or a save is loaded. Separate sampling grids retain
// the existing 5px yielding and 4px following/neighbor checks exactly.
type departureSweepCache struct {
	parallel                      [51]parallelSweepPose
	yielding, following           [61][2]sim.OBB
	yieldingReady, followingReady [61]bool
	neighbor                      sweepPoseCache
}

func departureBodyCollision(f autopilotFrame, kind, index int, front, other sim.OBB) bool {
	if f.departureCache == nil || index < 0 || index >= 61 {
		hit, _, _ := sim.CheckBusAtBody(f.bus, front, other)
		return hit
	}
	c := f.departureCache
	bodies, ready := &c.yielding, &c.yieldingReady
	if kind == 1 {
		bodies, ready = &c.following, &c.followingReady
	}
	if !ready[index] {
		bodies[index] = sweepBodies(f.bus, front)
		ready[index] = true
	}
	for _, part := range bodies[index] {
		if part.HalfLen <= 0 {
			continue
		}
		if hit, _, _ := sim.CheckOBBCollision(part, other); hit {
			return true
		}
	}
	return false
}

type parallelSweepPose struct {
	ready, clear bool
	bodies       [2]sim.OBB
}
