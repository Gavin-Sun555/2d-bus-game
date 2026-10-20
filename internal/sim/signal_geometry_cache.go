package sim

// Signals are updated and queried on the simulation thread. Geometry only
// changes when lane width or approach roads change, not when the phase advances.
// Include the complete input key so map edits and direct configuration changes
// cannot reuse stale stop lines.
type approachDimensionsKey struct {
	laneWidth float64
	lanes     [4]int
	roads     [4]bool
}
type approachDimensionsCache struct {
	key    approachDimensionsKey
	values [4]float64
	valid  bool
}
