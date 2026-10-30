package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Evaluate once per observed bus, not once per station. Unusual geometry keeps
// the exact ContainsBus path, including its existing edge-case semantics.
func busStopRangeReady(b *sim.Bus) bool {
	return b.Length > 0 && b.Length <= 1e6 && b.Width >= 0 && b.Width <= 1e6 &&
		math.Abs(b.Heading) <= 1e6 && math.Abs(b.Pos.X) <= 1e9 && math.Abs(b.Pos.Y) <= 1e9
}

// If ContainsBus can accept the front-section center, both its longitudinal and
// lateral distances fit the platform. Their sum bounds either world-axis delta,
// regardless of road direction or tail bend. The extra margin covers rounding.
// This is only a reject filter; nearby buses still use the original predicate.
func stopOutsideBusRange(s *sim.BusStop, b *sim.Bus, ready bool) bool {
	if !ready || !(s.Length >= 0 && s.Length <= 1e6 && s.Width >= 0 && s.Width <= 1e6 &&
		math.Abs(s.Heading) <= 1e6 && math.Abs(s.Pos.X) <= 1e9 && math.Abs(s.Pos.Y) <= 1e9) {
		return false
	}
	reach := (s.Length+s.Width)*.5 + 8
	return math.Abs(b.Pos.X-s.Pos.X) > reach || math.Abs(b.Pos.Y-s.Pos.Y) > reach
}
