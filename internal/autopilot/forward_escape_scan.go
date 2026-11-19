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

func (cache *forwardEscapeBoundaries) lookup(bus *sim.Bus) (known, blocked bool) {
	if cache == nil {
		return false, false
	}
	pose := [4]uint64{math.Float64bits(bus.Pos.X), math.Float64bits(bus.Pos.Y), math.Float64bits(bus.Heading), math.Float64bits(bus.TailHeading())}
	hash := uint64(0)
	for _, value := range pose {
		hash = bits.RotateLeft64(hash^value, 13) * 0x9e3779b97f4a7c15
	}
	entry := &cache.entries[(hash^(hash>>32))&uint64(len(cache.entries)-1)]
	if entry.valid && entry.pose == pose {
		return true, entry.blocked
	}
	return false, false
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
	origin         sim.Vec2
	travel         float64
	count          int
	valid          bool
	initialChecked bool
	initialClear   bool
	entries        [128]forwardEscapeObstacle
	boundaries     *forwardEscapeBoundaries
	traffic        [][2]sim.OBB
	trafficReady   []bool
}

type forwardEscapeObstacle struct {
	index      int
	padding    float64
	stationary bool
	bodies     [2]sim.OBB
}

func (ad *AutopilotDriver) useForwardEscapeBoundaries(f autopilotFrame, scan *forwardEscapeScan) {
	if f.world == nil || f.bus == nil {
		return
	}
	scratch := ad.geometryScratch()
	if scratch.forwardEscapeBoundaries == nil {
		scratch.forwardEscapeBoundaries = new(forwardEscapeBoundaries)
	}
	scratch.forwardEscapeBoundaries.reset(f)
	scan.boundaries = scratch.forwardEscapeBoundaries
	if scan.valid && scan.count > 0 {
		needed := (forwardEscapeTicks + 1) * scan.count
		if cap(scratch.forwardTraffic) < needed {
			scratch.forwardTraffic = make([][2]sim.OBB, needed)
		}
		scan.traffic = scratch.forwardTraffic[:needed]
		if cap(scratch.forwardTrafficReady) < needed {
			scratch.forwardTrafficReady = make([]bool, needed)
		}
		scan.trafficReady = scratch.forwardTrafficReady[:needed]
		clear(scan.trafficReady)
	}
}

func (c *forwardEscapeScan) prepare(f autopilotFrame, targetSpeed float64) {
	c.origin = sim.Vec2{}
	c.travel = 0
	c.count = 0
	c.valid = false
	c.initialChecked = false
	c.initialClear = false
	c.boundaries = nil
	c.traffic = nil
	c.trafficReady = nil
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

type forwardEscapePose struct {
	pos                  sim.Vec2
	heading, tailHeading float64
	checkBoundary        bool
}

func (c *forwardEscapeScan) poseTrafficClear(f autopilotFrame, b *sim.Bus, elapsed float64) (clear, checkBoundary bool) {
	delta := b.Pos.Sub(c.origin)
	if !c.valid || !(elapsed >= 0 && elapsed <= 2) || !(delta.Dot(delta) <= c.travel*c.travel) {
		return departureEscapePoseClear(f, b, elapsed), false
	}
	if math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
		return false, false
	}
	var own [2]sim.OBB
	ownReady := false
	tick := int(math.Round(elapsed * 60))
	canCache := len(c.trafficReady) == (forwardEscapeTicks+1)*c.count && tick >= 0 && tick <= forwardEscapeTicks && float64(tick)/60 == elapsed
	for i := 0; i < c.count; i++ {
		e := &c.entries[i]
		ob := f.obstacles[e.index]
		d := ob.Pos.Sub(b.Pos)
		limit := b.OverallLength() + ob.Length + math.Abs(ob.Speed)*elapsed + 40
		if d.Dot(d) > limit*limit {
			continue
		}
		parts := e.bodies
		if !e.stationary {
			cacheIdx := tick*c.count + i
			if canCache && c.trafficReady[cacheIdx] {
				parts = c.traffic[cacheIdx]
			} else {
				dist := math.Max(0, ob.Speed) * elapsed
				if ob.Bus != nil && dist > 0 {
					if !ownReady {
						own, ownReady = b.BodyParts(), true
					}
					position, heading := predictedObstacleBusPose(ob, dist)
					if predictedBusSeparated(own, ob.Bus, position, heading, e.padding, e.padding) {
						continue
					}
					parts = ob.Bus.BodyPartsAtPose(position, heading)
				} else {
					parts = predictedObstacleBodies(ob, dist)
				}
				if canCache {
					c.traffic[cacheIdx] = parts
					c.trafficReady[cacheIdx] = true
				}
			}
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
					return false, false
				}
			}
		}
	}
	return true, true
}

func (c *forwardEscapeScan) poseClear(f autopilotFrame, b *sim.Bus, elapsed float64) bool {
	clear, checkBoundary := c.poseTrafficClear(f, b, elapsed)
	if !clear {
		return false
	}
	if checkBoundary && c.boundaries.blocked(f, b) {
		return false
	}
	return true
}
