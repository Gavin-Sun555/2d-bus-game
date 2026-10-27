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
			if index < len(marginCache.ready) && marginCache.ready[index] {
				bodies = marginCache.bodies[index]
			} else {
				pos, heading := pointAt(distance)
				body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: f.bus.Length/2 + 6, HalfWid: f.bus.Width/2 + 4}
				bodies = sweepBodies(f.bus, body)
				if index < len(marginCache.ready) {
					marginCache.bodies[index], marginCache.ready[index] = bodies, true
				}
			}
			index++
			if sweepBodiesCollision(bodies, ob, math.Max(0, ob.Speed)*arrival, 4, 2) {
				// A queued neighbor may overlap only our extra safety margin.
				// Let the leading turn clear it when its complete actual sweep
				// is separated, rather than deadlocking both buses at rest.
				if distance <= 30 && math.Abs(ob.Speed) < 1 && junctionActualSweepClear(f.bus, ob, pointAt, errorPos, errorHeading, lookahead, &actualCache) {
					break
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
		if len(caches) > 0 && index < len(caches[0].ready) && caches[0].ready[index] {
			bodies = caches[0].bodies[index]
		} else {
			pos, heading := pointAt(distance)
			body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
			bodies = sweepBodies(bus, body)
			if len(caches) > 0 && index < len(caches[0].ready) {
				caches[0].bodies[index], caches[0].ready[index] = bodies, true
			}
		}
		if sweepBodiesCollision(bodies, ob, 0, 0, 0) {
			return false
		}
		index++
	}
	return true
}

// Shared only by the actual-clearance samples of one junction scan.
type sweepPoseCache struct {
	bodies [128][2]sim.OBB
	ready  [128]bool
}
