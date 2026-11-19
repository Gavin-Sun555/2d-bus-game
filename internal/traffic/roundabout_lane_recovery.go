package traffic

import (
	"math"
)

// A car separated outward by an entry queue must not immediately spiral back
// through a bus beside it. Hold its actual radius while that inward return is
// blocked, then resume the saved lane target once the car has passed the bus.
// The hold changes no pose and never takes the car farther outside the ring.
func (tm *TrafficManager) holdDisplacedRoundaboutLane(f carFrame) bool {
	c := f.car
	if !c.InRoundabout || c.RoundaboutCurve != nil || c.RoundaboutRadius <= c.RoundaboutTargetR+2 ||
		c.RoundaboutRadius > c.RoundaboutOuterRadius+c.Length/2 || c.RoundaboutOuterRadius <= 0 {
		return false
	}
	held := *c
	held.RoundaboutTargetR = held.RoundaboutRadius
	blocked := false
	for _, b := range f.buses {
		if b == nil || math.Abs(b.Speed) > 1 || b.Pos.Distance(c.Pos) > b.OverallLength()+150 {
			continue
		}
		if !roundaboutBusPathClear(c, b) {
			if !roundaboutBusPathClear(&held, b) {
				return false
			}
			blocked = true
		}
	}
	if blocked {
		c.RoundaboutTargetR = held.RoundaboutTargetR
	}
	return blocked
}
