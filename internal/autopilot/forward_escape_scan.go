package autopilot

import (
	"math"
	"math/bits"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// One two-second actuator search, including all steering alternatives. Neither
// the source vehicle nor any observed obstacle moves during these trial copies.
// Keep the original order and per-pose distance test; only discard obstacles
// outside every possible original broad-phase circle. Overflow/abnormal input
// and a trial exceeding the conservative travel bound use the full scan.
const forwardEscapeTicks = 120

type forwardEscapeBoundaryEntry struct {
	pose           [4]uint64
	blocked, valid bool
}

// The static boundary result depends only on the trial pose. The search still
// checks live traffic at each trial time, including after a cache hit.
type forwardEscapeBoundaries struct {
	version       sim.BoundaryQueryVersion
	length, width float64
	articulated   bool
	entries       [2048]forwardEscapeBoundaryEntry
}

func (cache *forwardEscapeBoundaries) reset(f autopilotFrame) {
	version, reusable := f.world.BoundaryQueryVersion(f.laneWidth)
	bus := f.bus
	if !reusable || cache.version != version || cache.length != bus.Length || cache.width != bus.Width || cache.articulated != bus.IsArticulated() {
		clear(cache.entries[:])
	}
	cache.version, cache.length, cache.width, cache.articulated = version, bus.Length, bus.Width, bus.IsArticulated()
}

func (cache *forwardEscapeBoundaries) blocked(f autopilotFrame, bus *sim.Bus) bool {
	if cache == nil {
		return forwardEscapeHitsBoundary(f, bus)
	}
	pose := [4]uint64{math.Float64bits(bus.Pos.X), math.Float64bits(bus.Pos.Y), math.Float64bits(bus.Heading), math.Float64bits(bus.TailHeading())}
	hash := uint64(0)
	for _, value := range pose {
		hash = bits.RotateLeft64(hash^value, 13) * 0x9e3779b97f4a7c15
	}
	entry := &cache.entries[(hash^(hash>>32))&uint64(len(cache.entries)-1)]
	if !entry.valid || entry.pose != pose {
		*entry = forwardEscapeBoundaryEntry{pose: pose, blocked: forwardEscapeHitsBoundary(f, bus), valid: true}
	}
	return entry.blocked
}

func forwardEscapeHitsBoundary(f autopilotFrame, bus *sim.Bus) bool {
	return math.Abs(sim.NormalizeAngle(bus.Heading-bus.TailHeading())) > sim.MaxArticulationAngle-.025 ||
		bus.HitsHardBarrier(f.world, f.laneWidth)
}

type forwardEscapeScan struct {
	origin     sim.Vec2
	travel     float64
	count      int
	valid      bool
	entries    [128]forwardEscapeObstacle
	boundaries *forwardEscapeBoundaries
}

type forwardEscapeObstacle struct {
	index      int
	padding    float64
	stationary bool
	bodies     [2]sim.OBB
}

func (c *forwardEscapeScan) prepare(f autopilotFrame, targetSpeed float64) {
	*c = forwardEscapeScan{}
	if f.bus == nil {
		return
	}
	b := f.bus
	c.origin = b.Pos
	// Throttle is <=1 and stops above target speed. Allow one full manual
	// first-gear acceleration step plus generous roundoff. Runtime validation
	// below keeps even changed/unusual physics on the original full scan.
	c.travel = 2*(math.Max(math.Abs(b.Speed), targetSpeed)+math.Abs(b.AccelRate)*1.3/60) + 8
	if !(c.travel >= 0 && c.travel <= 8192) || !(math.Abs(b.Pos.X) <= 1e9 && math.Abs(b.Pos.Y) <= 1e9) || !(b.OverallLength() > 0 && b.OverallLength() <= 8192) {
		return
	}
	for i, ob := range f.obstacles {
		reach := b.OverallLength() + ob.Length + math.Abs(ob.Speed)*2 + 40 + c.travel
		if !(ob.Length >= 0 && reach >= 0 && reach <= 1e6) || !(math.Abs(ob.Pos.X) <= 1e9 && math.Abs(ob.Pos.Y) <= 1e9) {
			return
		}
		reach += 1e-7 * (1 + reach + math.Abs(b.Pos.X) + math.Abs(b.Pos.Y))
		delta := ob.Pos.Sub(b.Pos)
		if delta.Dot(delta) > reach*reach {
			continue
		}
		if c.count == len(c.entries) {
			return
		}
		e := forwardEscapeObstacle{index: i, padding: 2, stationary: ob.Speed <= 0}
		if ob.Bus != nil && math.Abs(ob.Speed) < 1 && ob.Pos.Sub(b.Pos).Dot(sim.UnitFromAngle(b.Heading)) < -8 {
			e.padding = .05
		}
		if e.stationary {
			e.bodies = predictedObstacleBodies(ob, 0)
		}
		c.entries[c.count] = e
		c.count++
	}
	c.valid = true
}

func (c *forwardEscapeScan) poseClear(f autopilotFrame, b *sim.Bus, elapsed float64) bool {
	delta := b.Pos.Sub(c.origin)
	if !c.valid || !(elapsed >= 0 && elapsed <= 2) || !(delta.Dot(delta) <= c.travel*c.travel) {
		return departureEscapePoseClear(f, b, elapsed)
	}
	if c.boundaries.blocked(f, b) {
		return false
	}
	var own [2]sim.OBB
	ownReady := false
	for i := 0; i < c.count; i++ {
		e := &c.entries[i]
		ob := f.obstacles[e.index]
		if ob.Pos.Distance(b.Pos) > b.OverallLength()+ob.Length+math.Abs(ob.Speed)*elapsed+40 {
			continue
		}
		parts := e.bodies
		if !e.stationary {
			parts = predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed)
		}
		for _, other := range parts {
			if other.HalfLen <= 0 {
				continue
			}
			other.HalfLen += e.padding
			other.HalfWid += e.padding
			if !ownReady {
				own, ownReady = b.BodyParts(), true
			}
			for _, part := range own {
				if part.HalfLen > 0 && obbHits(part, other) {
					return false
				}
			}
		}
	}
	return true
}
