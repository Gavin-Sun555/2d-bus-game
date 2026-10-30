package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A parked neighbor alongside a departing berth may be passed only when both
// physical sections clear it throughout the actual departure corridor.
func (ad *AutopilotDriver) clearsStationDepartureNeighbor(f autopilotFrame, ob VehicleObstacle) bool {
	if f.world == nil || ad.State == AutopilotStateDocked || ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) || len(ad.drivePath) < 2 || math.Abs(ob.Speed) > 3 {
		return false
	}
	stop := f.world.Stops[ad.pathFromID]
	aligned := math.Cos(ob.Heading-f.bus.Heading) >= .98 || (math.Cos(ob.Heading-stop.Heading) >= .95 && math.Cos(f.bus.Heading-stop.Heading) >= .98)
	if !aligned || math.Max(roadSideClearance(f.bus, ob), stopCorridorSideClearance(f.bus, ob, stop.Heading)) <= 2 {
		return false
	}
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

func stopCorridorSideClearance(bus *sim.Bus, ob VehicleObstacle, stopHeading float64) float64 {
	normal := sim.Vec2{X: -math.Sin(stopHeading), Y: math.Cos(stopHeading)}
	obAngle := ob.Heading - stopHeading
	obHalfWidth := math.Abs(math.Cos(obAngle))*ob.Width/2 + math.Abs(math.Sin(obAngle))*ob.Length/2
	return math.Abs(bus.Pos.Sub(ob.Pos).Dot(normal)) - bus.Width/2 - obHalfWidth
}
