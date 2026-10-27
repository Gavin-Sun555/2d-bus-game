package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A parked neighbor alongside a departing berth may be passed only when both
// physical sections clear it throughout the actual departure corridor.
func (ad *AutopilotDriver) clearsStationDepartureNeighbor(f autopilotFrame, ob VehicleObstacle) bool {
	if f.world == nil || ad.State == AutopilotStateDocked || ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) || len(ad.drivePath) < 2 || math.Abs(ob.Speed) > 3 || math.Cos(ob.Heading-f.bus.Heading) < .98 || roadSideClearance(f.bus, ob) <= 2 {
		return false
	}
	stop := f.world.Stops[ad.pathFromID]
	u := sim.UnitFromAngle(stop.Heading)
	along := f.bus.Pos.Sub(stop.Pos).Dot(u)
	if along < -80 || along > 400 || math.Abs(ob.Pos.Sub(f.bus.Pos).Dot(u)) > 80 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return false
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		v := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		return p.Add(sim.Vec2{X: -v.Y, Y: v.X}.Mul(ad.LaneOffset)), math.Atan2(v.Y, v.X)
	}
	p, h := pointAt(0)
	if f.departureCache != nil {
		return junctionActualSweepClear(f.bus, ob, pointAt, f.bus.Pos.Sub(p), sim.NormalizeAngle(f.bus.Heading-h), 260, &f.departureCache.neighbor)
	}
	return junctionActualSweepClear(f.bus, ob, pointAt, f.bus.Pos.Sub(p), sim.NormalizeAngle(f.bus.Heading-h), 260)
}
