package sim

import (
	"math"
	"math/bits"
)

// Exact memoization of geometry only. Traffic, signals, boundary results and
// controls are never cached. Every input read by the articulation integrator
// is represented bit-for-bit, so moving a bus, loading a save, or changing its
// geometry cannot reuse an incompatible result. A fixed-size table bounds
// memory even for continually moving buses and legacy copies of their poses.
type busPredictionKey struct {
	pose        [8]uint64 // source X/Y, heading, trailer, length; target X/Y, heading
	initialized bool
}
type busPredictionPose struct {
	pos                   Vec2
	heading, trailer      float64
	initialized, rejected bool
}
type busPredictionEntry struct {
	valid bool
	key   busPredictionKey
	pose  busPredictionPose
}
type busPredictionCache struct {
	entries      [256]busPredictionEntry
	hits, misses uint64
}

// Controllers/observers prepare once before taking value copies of the bus.
// Pure standalone AtPose calls on an unprepared bus retain their original path.
func (b *Bus) PreparePosePredictions() {
	if b.IsArticulated() && b.predictionCache == nil {
		b.predictionCache = &busPredictionCache{}
	}
}

// PosePredictionStats supports offline profiling, not saved simulation state.
func (b *Bus) PosePredictionStats() (hits, misses uint64) {
	if b.predictionCache != nil {
		return b.predictionCache.hits, b.predictionCache.misses
	}
	return 0, 0
}

// Return a slot, not a whole Bus, so misses and hits both copy the vehicle
// exactly once in AtPose. A miss is reserved until the integrator completes;
// this helper and the integrator run synchronously on the simulation thread.
func (b *Bus) predictionEntry(pos Vec2, heading float64) (*busPredictionEntry, bool) {
	key := busPredictionKey{pose: [8]uint64{math.Float64bits(b.Pos.X), math.Float64bits(b.Pos.Y), math.Float64bits(b.Heading), math.Float64bits(b.TrailerHeading), math.Float64bits(b.Length), math.Float64bits(pos.X), math.Float64bits(pos.Y), math.Float64bits(heading)}, initialized: b.TrailerInitialized}
	hash := uint64(0x9e3779b97f4a7c15)
	for _, v := range key.pose {
		hash = bits.RotateLeft64(hash^v, 11) * 0x9e3779b97f4a7c15
	}
	if key.initialized {
		hash ^= 0x517cc1b727220a95
	}
	cache := b.predictionCache
	entry := &cache.entries[hash&255]
	if entry.valid && entry.key == key {
		cache.hits++
		return entry, true
	}
	cache.misses++
	entry.valid, entry.key = false, key
	return entry, false
}
