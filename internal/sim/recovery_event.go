package sim

// TrafficRecoveryEvent is emitted synchronously before a jam recovery changes
// the vehicle. Observers can preserve the original scene without affecting AI.
type TrafficRecoveryEvent struct {
	Kind   string
	ID     int
	Pos    Vec2
	Reason string
}
