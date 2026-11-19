package sim

// Keep waiting noses outside the ring and the swept corners of a long bus.
// The same plane is used by traffic, spawning and the visible yield markings.
const RoundaboutYieldClearance = 40.0

func (rb Roundabout) YieldLineDistance() float64 {
	return rb.Radius + RoundaboutYieldClearance
}

// SupportsParallelArticulated reports whether the carriageway ring between
// the inner apron and outer curb is wide enough for two articulated buses to
// circulate abreast without their swept envelopes overlapping.
func (rb Roundabout) SupportsParallelArticulated() bool {
	return rb.Radius-(rb.InnerRadius+16) >= 165
}
