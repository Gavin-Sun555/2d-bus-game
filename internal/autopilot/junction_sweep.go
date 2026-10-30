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
	base, angle := pointAt(0)
	errorPos := f.bus.Pos.Sub(base)
	errorHeading := sim.NormalizeAngle(f.bus.Heading - angle)
	limit := math.Inf(1)
	lookahead := math.Max(140, f.speedAbs*f.speedAbs/(2*180)+40)
	var actualCache, marginCache sweepPoseCache
	if lookahead >= 128*6 {
		marginCache.overflow = junctionPoseStorage(&ad.geometryScratch().junctionMargin, lookahead, 6)
	}
	for _, ob := range f.obstacles {
		radius := lookahead + math.Abs(ob.Speed)*1.5 + (f.bus.Length+ob.Length)/2
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > radius*radius {
			continue
		}
		index := 0
		for distance := 0.; distance <= lookahead; distance += 6 {
			arrival := distance / math.Max(25, f.speedAbs)
			var bodies [2]sim.OBB
			if cached, ok := marginCache.load(index); ok {
				bodies = cached
			} else {
				pos, heading := pointAt(distance)
				body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: f.bus.Length/2 + 6, HalfWid: f.bus.Width/2 + 4}
				bodies = sweepBodies(f.bus, body)
				marginCache.store(index, bodies)
			}
			index++
			if sweepBodiesCollision(bodies, ob, math.Max(0, ob.Speed)*arrival, 4, 2) {
				// A queued neighbor may overlap only our extra safety margin.
				// Let the leading turn clear it when its complete actual sweep
				// is separated, rather than deadlocking both buses at rest.
				if distance <= 30 && math.Abs(ob.Speed) < 1 {
					if lookahead >= 128*4 && actualCache.overflow == nil {
						actualCache.overflow = junctionPoseStorage(&ad.geometryScratch().junctionActual, lookahead, 4)
					}
					if junctionActualSweepClear(f.bus, ob, pointAt, errorPos, errorHeading, lookahead, &actualCache) {
						break
					}
				}
				limit = math.Min(limit, math.Sqrt(2*180*math.Max(0, distance-30)))
				break
			}
		}
	}
	return limit
}

func junctionActualSweepClear(bus *sim.Bus, ob VehicleObstacle, pointAt func(float64) (sim.Vec2, float64), errorPos sim.Vec2, errorHeading, horizon float64, caches ...*sweepPoseCache) bool {
	index := 0
	for distance := 0.; distance <= horizon; distance += 4 {
		var bodies [2]sim.OBB
		cached, ready := [2]sim.OBB{}, false
		if len(caches) > 0 {
			cached, ready = caches[0].load(index)
		}
		if ready {
			bodies = cached
		} else {
			pos, heading := pointAt(distance)
			body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
			bodies = sweepBodies(bus, body)
			if len(caches) > 0 {
				caches[0].store(index, bodies)
			}
		}
		if sweepBodiesCollision(bodies, ob, 0, 0, 0) {
			return false
		}
		index++
	}
	return true
}

// Per-scan validity; only junction scans may borrow persistent overflow storage.
type sweepPoseCache struct {
	bodies        [128][2]sim.OBB
	ready         [128]bool
	overflow      [][2]sim.OBB
	overflowReady int
}

// Only junction scans borrow overflow storage. Reset validity on each scan,
// while retaining bounded capacity in the driver's simulation-thread scratch.
func junctionPoseStorage(storage *[][2]sim.OBB, horizon, step float64) [][2]sim.OBB {
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
		*storage = make([][2]sim.OBB, count, capacity)
	} else {
		*storage = (*storage)[:count]
	}
	return *storage
}
func (c *sweepPoseCache) load(index int) ([2]sim.OBB, bool) {
	if index < len(c.ready) {
		return c.bodies[index], c.ready[index]
	}
	i := index - len(c.ready)
	if i < c.overflowReady {
		return c.overflow[i], true
	}
	return [2]sim.OBB{}, false
}
func (c *sweepPoseCache) store(index int, bodies [2]sim.OBB) {
	if index < len(c.ready) {
		c.bodies[index], c.ready[index] = bodies, true
		return
	}
	i := index - len(c.ready)
	if i < len(c.overflow) {
		c.overflow[i] = bodies
		if i == c.overflowReady {
			c.overflowReady++
		}
	}
}
