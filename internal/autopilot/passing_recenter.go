package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A blocked return can freeze the target at the bus's actual position. If
// that leaves the bus against the right edge of its passing lane, finish the
// pass from the lane center instead of following that fractional offset.
func (ad *AutopilotDriver) recenterInnerPassingLane(f autopilotFrame, roadCenter float64, isEW bool) bool {
	if ad.TargetLaneOffset >= 0 || math.Abs(ad.LaneOffset-ad.TargetLaneOffset) >= 1 {
		return false
	}
	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	origin := sim.Vec2{Y: roadCenter}
	if !isEW {
		origin = sim.Vec2{X: roadCenter}
	}
	base := ad.pointAt(f.s)
	// A bend, taper or junction exit must follow its own path geometry.
	if ad.pointAt(f.s+180).Sub(base).Normalize().Dot(forward) < .998 ||
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) {
		return false
	}
	laneCenter := f.laneWidth / 2
	actual := f.bus.Pos.Sub(origin).Dot(right)
	planned := base.Sub(origin).Dot(right) + ad.TargetLaneOffset
	if actual <= laneCenter+4 || actual >= f.laneWidth || planned <= laneCenter+4 || planned >= f.laneWidth {
		return false
	}
	desired := laneCenter - base.Sub(origin).Dot(right)
	actualOffset := f.bus.Pos.Sub(base).Dot(right)
	if !isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, actualOffset, desired, f.laneWidth) {
		return false
	}
	ad.TargetLaneOffset = desired
	ad.OvertakeState = OvertakeChangingLane
	ad.OvertakeTimer = 0
	return true
}
