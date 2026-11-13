package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A clear curb lane is the default on a straight six-lane road. Only move
// there from the adjacent lane, and use the physical road axis to measure
// gaps so lane-change yaw and planned offsets cannot misclassify traffic.
func (ad *AutopilotDriver) preferClearSixLaneCurb(f autopilotFrame, roadCenter, dt float64) bool {
	if !ad.BerserkMode || f.world == nil || (ad.OvertakeState == OvertakeChangingLane && ad.TargetLaneOffset != 0) {
		return false
	}
	forward := sim.UnitFromAngle(approachRoadHeading(f.bus.Heading))
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	isEW := math.Abs(forward.X) > .5
	road, ok := f.world.RoadAt(roadCenter, isEW, f.bus.Pos)
	if !ok || road.LaneCount() != 6 || road.HasBusLane() || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return false
	}
	curb := road.P1.Dot(right) + 2.5*f.laneWidth
	base := ad.pointAt(f.s)
	if math.Abs(base.Dot(right)-curb) > f.laneWidth*.15 || ad.pointAt(f.s+180).Sub(base).Normalize().Dot(forward) < .998 {
		return false
	}
	actual := f.bus.Pos.Dot(right) - curb
	if actual < -1.5*f.laneWidth || actual > .5*f.laneWidth {
		return false
	}
	_, _, score, _, _, blocked := evaluate6LaneTraffic(f.bus, f.obstacles, forward.X, forward.Y, f.laneWidth, actual, ad.GetBerserkCruiseSpeed())
	if blocked || score > 0 {
		return false
	}
	// The destination check includes a fast follower outside the forward scan.
	if math.Abs(actual) > 4 && !isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, actual, 0, f.laneWidth) {
		return false
	}
	if ad.TargetLaneOffset != 0 {
		ad.TargetLaneOffset = 0
		ad.OvertakeTimer = 0
	}
	ad.OvertakeState = OvertakeReturning
	ad.progressLaneOffset(dt, f.speedAbs)
	if math.Abs(ad.LaneOffset) < 1 {
		ad.LaneOffset = 0
		ad.OvertakeState = OvertakeNone
	}
	return true
}
