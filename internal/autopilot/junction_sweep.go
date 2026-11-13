package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A turning bus sweeps across more than its straight-road width. Check this
// before adjacent-lane exemptions can discard a car on the outgoing road.
func (ad *AutopilotDriver) junctionSweptSpeed(f autopilotFrame) float64 {
	if f.world == nil || len(f.obstacles) == 0 || len(ad.drivePath) < 2 || !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return math.Inf(1)
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		u := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		p = p.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(ad.LaneOffset))
		return p, math.Atan2(u.Y, u.X)
	}
	lookahead := math.Max(140, f.speedAbs*f.speedAbs/(2*180)+40)
	scratch := ad.geometryScratch()
	candidates := scratch.junctionCandidates[:0]
	for i, ob := range f.obstacles {
		radius := lookahead + math.Abs(ob.Speed)*1.5 + (f.bus.Length+ob.Length)/2
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > radius*radius {
			continue
		}
		candidates = append(candidates, i)
	}
	scratch.junctionCandidates = candidates
	if len(candidates) == 0 {
		return math.Inf(1)
	}

	base, angle := pointAt(0)
	errorPos := f.bus.Pos.Sub(base)
	errorHeading := sim.NormalizeAngle(f.bus.Heading - angle)
	closingSpeed := math.Max(25, f.speedAbs)
	var actualCache sweepPoseCache
	active := len(candidates)
	// Distance determines the cap. Visit samples in that order so a closer
	// blocker can end the sweep before distant obstacles are predicted.
	for distance := 0.; distance <= lookahead; distance += 6 {
		arrival := distance / closingSpeed
		pos, heading := pointAt(distance)
		sample := sweepSample{front: sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: f.bus.Length/2 + 6, HalfWid: f.bus.Width/2 + 4}}
		for slot, index := range candidates {
			if index < 0 {
				continue
			}
			ob := f.obstacles[index]
			if !sample.collides(f.bus, ob, math.Max(0, ob.Speed)*arrival, 4, 2) {
				continue
			}
			// A queued neighbor may overlap only the extra safety margin.
			// Waive that obstacle only after proving the complete actual sweep.
			if distance <= 30 && (math.Abs(ob.Speed) < 1 || parallelLeftRearCar(f, ob)) {
				if lookahead >= 128*4 && actualCache.overflow == nil {
					actualCache.overflow = junctionPoseStorage(&scratch.junctionActual, lookahead, 4)
				}
				rate := 0.
				if math.Abs(ob.Speed) >= 1 {
					rate = ob.Speed / closingSpeed
				}
				if junctionActualSweepClearAtRate(f.bus, ob, pointAt, errorPos, errorHeading, lookahead, rate, &actualCache) {
					candidates[slot] = -1
					active--
					if active == 0 {
						return math.Inf(1)
					}
					continue
				}
			}
			return math.Sqrt(2 * 180 * math.Max(0, distance-30))
		}
	}
	return math.Inf(1)
}

// A car trailing in the inner lane can touch the padded trailer envelope.
// Keep the time-aligned sweep authoritative when the bus is already admitted.
func parallelLeftRearCar(f autopilotFrame, ob VehicleObstacle) bool {
	c := f.bus.JunctionCorridor
	if c == nil || !c.Entered || len(c.Bodies) < 2 || ob.Bus != nil || ob.Speed <= 0 || ob.Speed > f.speedAbs ||
		math.Cos(ob.Heading-c.EntryHeading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(c.EntryHeading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if math.Abs(ob.Pos.Sub(c.Center).Dot(r)-.5*f.laneWidth) >= .25*f.laneWidth ||
		ob.Pos.Sub(f.bus.Pos).Dot(u) >= -f.bus.Length {
		return false
	}
	if c.CurbLeft {
		return true
	}
	last := c.Bodies[len(c.Bodies)-1]
	out := r.Mul(-1)
	return f.bus.Pos.Sub(c.Center).Dot(r) >= 1.3*f.laneWidth &&
		last.Center.Sub(c.Center).Dot(u) >= 1.3*f.laneWidth &&
		sim.UnitFromAngle(last.Heading).Dot(out) > .98
}

func junctionActualSweepClear(bus *sim.Bus, ob VehicleObstacle, pointAt func(float64) (sim.Vec2, float64), errorPos sim.Vec2, errorHeading, horizon float64, caches ...*sweepPoseCache) bool {
	return junctionActualSweepClearAtRate(bus, ob, pointAt, errorPos, errorHeading, horizon, 0, caches...)
}

func junctionActualSweepClearAtRate(bus *sim.Bus, ob VehicleObstacle, pointAt func(float64) (sim.Vec2, float64), errorPos sim.Vec2, errorHeading, horizon, rate float64, caches ...*sweepPoseCache) bool {
	index := 0
	for distance := 0.; distance <= horizon; distance += 4 {
		var sample *sweepSample
		if len(caches) > 0 {
			sample = caches[0].load(index)
		}
		var uncached sweepSample
		if sample == nil {
			pos, heading := pointAt(distance)
			body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
			uncached = sweepSample{front: body}
			if len(caches) > 0 {
				sample = caches[0].store(index, uncached)
			}
			if sample == nil {
				sample = &uncached
			}
		}
		if sample.collides(bus, ob, distance*rate, 0, 0) {
			return false
		}
		index++
	}
	return true
}

// Per-scan validity; only junction scans may borrow persistent overflow storage.
type sweepPoseCache struct {
	bodies        [128]sweepSample
	ready         [128]bool
	overflow      []sweepSample
	overflowReady int
}

// Only junction scans borrow overflow storage. Reset validity on each scan,
// while retaining bounded capacity in the driver's simulation-thread scratch.
func junctionPoseStorage(storage *[]sweepSample, horizon, step float64) []sweepSample {
	if !(horizon >= 0 && horizon <= 16384) {
		return nil
	}
	count := min(4096, int(horizon/step)+1) - 128
	if count <= 0 {
		return nil
	}
	if cap(*storage) < count {
		// Grow geometrically: acceleration must not allocate a new large buffer
		// every time the speed adds one more prediction sample.
		capacity := max(count, min(4096-128, max(128, cap(*storage)*2)))
		*storage = make([]sweepSample, count, capacity)
	} else {
		*storage = (*storage)[:count]
	}
	return *storage
}
func (c *sweepPoseCache) load(index int) *sweepSample {
	if index < len(c.ready) {
		if c.ready[index] {
			return &c.bodies[index]
		}
		return nil
	}
	i := index - len(c.ready)
	if i < c.overflowReady {
		return &c.overflow[i]
	}
	return nil
}
func (c *sweepPoseCache) store(index int, sample sweepSample) *sweepSample {
	if index < len(c.ready) {
		c.bodies[index], c.ready[index] = sample, true
		return &c.bodies[index]
	}
	i := index - len(c.ready)
	if i < len(c.overflow) {
		c.overflow[i] = sample
		if i == c.overflowReady {
			c.overflowReady++
		}
		return &c.overflow[i]
	}
	return nil
}
