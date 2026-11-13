package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A stuck articulated bus with its powered section turned toward the curb
// can only recover by backing its tail a few pixels. A follower stopped
// against that tail leaves it no room and both wait forever. After a short
// wait, give the leader that room with a proved straight retreat.
func (ad *AutopilotDriver) tryPinnedLeaderRetreat(f autopilotFrame, dt float64) bool {
	if f.world == nil || f.speedAbs > .1 || f.bus.DoorsOpen || ad.headonRecovery != nil || ad.State == AutopilotStateDocked ||
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) {
		ad.pinnedLeaderWait = 0
		return false
	}
	road := approachRoadHeading(f.bus.Heading)
	pinned := false
	if math.Cos(f.bus.Heading-road) > .98 && math.Cos(f.bus.TailHeading()-road) > .98 {
		for _, ob := range f.obstacles {
			if gap, ok := sim.PinnedArticulatedRearGap(ob.Bus, f.bus.Pos, road, f.bus.Length/2, f.bus.Width/2, f.laneWidth*.5); ok && gap < 10 {
				pinned = true
				break
			}
		}
	}
	if !pinned {
		ad.pinnedLeaderWait = 0
		return false
	}
	ad.pinnedLeaderWait += dt
	if ad.pinnedLeaderWait < 3 {
		return false
	}
	for _, distance := range []float64{20, 14} {
		if ad.laneRetreatReverseClear(f, distance, 0) {
			ad.pinnedLeaderWait = 0
			ad.headonRecovery = &HeadonDeadlockRecovery{StartPos: f.bus.Pos, TargetDistance: distance, ParallelRetreat: true}
			return true
		}
	}
	return false
}
