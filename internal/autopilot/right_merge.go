package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A return target may survive from an earlier frame whose gap has since closed.
// Keep the actual corridor, rather than reversing into another lane or forcing
// the planned return. The normal lane policy can retry when traffic clears.
func (ad *AutopilotDriver) holdBlockedRightMerge(f autopilotFrame) {
	// Required merges already check every crossed lane and cap approach speed.
	// Reapplying the passing gap here would freeze the same safe queue slot.
	if ad.approachMerge != nil {
		return
	}
	if f.world == nil || len(ad.drivePath) < 2 || len(f.obstacles) == 0 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return
		}
	}
	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	base := ad.pointAt(f.s)
	tangent := ad.pointAt(f.s + 80).Sub(base).Normalize()
	// Junction turns and roundabout arcs have their own swept-path rules.
	if tangent.Dot(forward) < .97 {
		return
	}
	actual := f.bus.Pos.Sub(base).Dot(right)
	desired := ad.TargetLaneOffset
	if desired-actual < f.laneWidth*.35 {
		return
	}
	clear := true
	for offset := actual; offset < desired-.01; {
		offset = math.Min(desired, offset+f.laneWidth)
		if !isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, actual, offset, f.laneWidth) {
			clear = false
			break
		}
	}
	if clear {
		return
	}
	ad.holdLaneOffset(actual)
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	ad.beginMidLaneRecovery(f, heading, center, ew)
}

func (ad *AutopilotDriver) holdLaneOffset(offset float64) {
	ad.LaneOffset, ad.TargetLaneOffset = offset, offset
	ad.LaneOffsetStart, ad.lastTargetLaneOffset = offset, offset
	ad.LaneOffsetProgress = 1
	ad.OvertakeState = OvertakePassing
	ad.OvertakeTimer = 0
}
