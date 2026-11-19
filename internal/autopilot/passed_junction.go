package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Turn tracking can take over before the lane selector updates its latch.
// A vehicle physically in this box must clear it, while the exit/return lane
// policy can retain its existing latch until its own handoff completes.
func (ad *AutopilotDriver) busLaneApproachActive(f autopilotFrame) bool {
	if ad.busLaneJunction == nil || ad.busJunctionEntered {
		return false
	}
	if f.world == nil {
		return true
	}
	geometry := f.world.JunctionAt(*ad.busLaneJunction, f.laneWidth)
	delta := f.bus.Pos.Sub(*ad.busLaneJunction)
	return math.Abs(delta.X) > geometry.HalfX || math.Abs(delta.Y) > geometry.HalfY
}

// A restored/missed junction-entry latch must not reserve an old merge forever.
// Clear only after the junction is well behind the current travel direction
// AND no remaining path segment returns to its box. Live traffic and all normal
// merge/collision rules still decide how the bus proceeds from its actual pose.
func (ad *AutopilotDriver) clearPassedBusLaneJunction(f autopilotFrame) {
	if ad.busLaneJunction == nil || ad.busJunctionEntered || f.world == nil || len(ad.drivePath) < 2 {
		return
	}
	center := *ad.busLaneJunction
	g := f.world.JunctionAt(center, f.laneWidth)
	reach := math.Max(g.HalfX, g.HalfY) + f.bus.OverallLength() + 100
	u := sim.UnitFromAngle(approachRoadHeading(f.bus.Heading))
	// Retain the local exit/merge settling region. Releasing this latch just
	// outside the box changes lane selection while nearby queues still merge.
	clearance := 2 * reach
	if exit := ad.exitCruise; exit != nil && exit.Junction.Distance(center) < 1 && !exit.Joined {
		// The outgoing road is already known, but yaw can keep exit-cruise
		// acquisition pending. Do not apply the old incoming merge there.
		clearance = reach
	}
	if center.Sub(f.bus.Pos).Dot(u) >= -clearance {
		return
	}
	start := max(1, ad.WaypointIdx+1)
	previous := ad.pointAt(f.s)
	for i := start; i < len(ad.drivePath); i++ {
		distance, _ := sim.DistToSegment(center, previous, ad.drivePath[i])
		previous = ad.drivePath[i]
		if distance <= reach {
			return
		}
	}
	ad.busLaneJunction = nil
	ad.busLaneTurnChosen = false
	ad.busJunctionEntered = false
	ad.busTurnHoldDistance = 0
}
