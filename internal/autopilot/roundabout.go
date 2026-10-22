package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// The outer ring is the normal circulation lane. Passing traffic on the inner
// ring adds an unnecessary conflict at the exit, even in berserk mode.
func (ad *AutopilotDriver) updateRoundaboutLane(bus *sim.Bus, rb *sim.Roundabout, obstacles []VehicleObstacle, laneWidth, s, speedAbs, dt float64) {
	if ad.directRoundaboutExit != nil && ad.directRoundaboutExit.Distance(rb.Center) < 1 {
		if bus.Pos.Distance(rb.Center) > rb.Radius+10 {
			ad.directRoundaboutExit = nil
		}
		return
	}
	if ad.retryRoundaboutExit(bus, rb, obstacles, laneWidth, s, speedAbs) {
		return
	}
	// Finish an entry from the left approach lane before attempting to merge.
	if ad.joiningInnerRoundabout(rb, s, laneWidth) {
		ad.TargetLaneOffset = ad.LaneOffset
		ad.OvertakeState = OvertakePassing
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	if math.Abs(ad.LaneOffset) < 1 {
		ad.LaneOffset, ad.TargetLaneOffset = 0, 0
		ad.OvertakeState = OvertakeNone
		ad.OvertakeTimer = 0
		ad.LaneOffsetProgress = 1
		return
	}
	// Outside the circle, retain the chosen approach lane. Once circulating,
	// use the first safe gap rather than waiting until the last exit waypoint.
	if bus.Pos.Distance(rb.Center) > rb.Radius+10 || !roundaboutExitMergeClear(bus, rb, obstacles, laneWidth) {
		ad.TargetLaneOffset = ad.LaneOffset
		ad.LaneOffsetProgress = 1
		ad.OvertakeState = OvertakePassing
		return
	}
	ad.TargetLaneOffset = 0
	ad.OvertakeState = OvertakeReturning
	ad.progressLaneOffset(dt, speedAbs)
}
