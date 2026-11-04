package sim

import "math"

// MedianBusRightPhase shares the crossing road's protected left-turn phase.
// It never grants right-on-red across the intervening general traffic lanes.
func MedianBusRightPhase(heading float64) LightPhase {
	if math.Abs(math.Cos(heading)) >= math.Abs(math.Sin(heading)) {
		return PhaseNSLeftGreen
	}
	return PhaseEWLeftGreen
}
func (inter *TrafficIntersection) GetMedianBusRightSignal(heading float64) LightSignal {
	if !inter.ApproachMedianBusLane[inter.GetApproachIndexForHeading(heading)] {
		return SignalRed
	}
	phase := MedianBusRightPhase(heading)
	if inter.Phase == phase {
		return SignalGreen
	}
	if inter.Phase == phase+1 {
		return SignalYellow
	}
	return SignalRed
}
func (inter *TrafficIntersection) needsEWLeftPhase() bool {
	return inter.HasEW4Lane() || inter.ApproachMedianBusLane[2] || inter.ApproachMedianBusLane[3]
}
func (inter *TrafficIntersection) needsNSLeftPhase() bool {
	return inter.HasNS4Lane() || inter.ApproachMedianBusLane[0] || inter.ApproachMedianBusLane[1]
}
