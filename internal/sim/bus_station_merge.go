package sim

// Controllers refresh this short-lived intent while a bus needs a station gap.
// Expiration prevents cars yielding indefinitely after manual control takes over.
type BusStationMergeRequest struct {
	StopID    int
	Departing bool
	Remaining float64
}
