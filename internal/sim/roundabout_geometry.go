package sim

// Keep waiting noses outside the ring and the swept corners of a long bus.
// The same plane is used by traffic, spawning and the visible yield markings.
const RoundaboutYieldClearance = 40.0

func (rb Roundabout) YieldLineDistance() float64 {
	return rb.Radius + RoundaboutYieldClearance
}
