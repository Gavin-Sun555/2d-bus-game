package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

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
	if center.Sub(f.bus.Pos).Dot(u) >= -2*reach {
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
	ad.busJunctionEntered = false
	ad.busTurnHoldDistance = 0
}
