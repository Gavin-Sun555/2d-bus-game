package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Immutable for one synchronous scan; never cached across control ticks/load.
// Preparing the road context once avoids road/trig queries per car or sample.
type departureRearCars struct {
	active                                bool
	position, roadForward, forward, right sim.Vec2
	heading, length, width                float64
}

func (ad *AutopilotDriver) departureRearCars(f autopilotFrame) departureRearCars {
	empty := departureRearCars{}
	if ad == nil || f.world == nil || f.bus == nil || ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) ||
		ad.State == AutopilotStateDocked || f.bus.DoorsOpen || f.bus.Speed < -.1 || ad.approachMerge != nil || ad.midLaneRecovery != nil {
		return empty
	}
	stop := f.world.Stops[ad.pathFromID]
	if stop.RoadLanes < 4 {
		return empty
	}
	u := sim.UnitFromAngle(stop.Heading)
	relative := f.bus.Pos.Sub(stop.RoadCenter)
	along := relative.Dot(u)
	lateral := relative.Dot(sim.Vec2{X: -u.Y, Y: u.X})
	curb := (float64(stop.RoadLanes)/2 - .5) * f.laneWidth
	if lateral < 0 || lateral > curb+stop.Width+80 || along < 0 || along > 800 || math.Cos(f.bus.Heading-stop.Heading) < .5 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return empty
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return empty
		}
	}
	direction := sim.UnitFromAngle(f.bus.Heading)
	return departureRearCars{active: true, position: f.bus.Pos, roadForward: u, forward: direction, right: sim.Vec2{X: -direction.Y, Y: direction.X}, heading: stop.Heading, length: f.bus.Length, width: f.bus.Width}
}

// During a forward departure a stopped small car behind the powered section
// is not a leader. Do not let predicted rear swing veto forward lane entry.
// Physical contacts still resolve normally; forward traffic and signals retain
// priority. A car alongside the powered section needs lateral separation.
func (c departureRearCars) matches(ob VehicleObstacle) bool {
	if !c.active || ob.Bus != nil || ob.Length > 65 || math.Abs(ob.Speed) > 3 {
		return false
	}
	delta := ob.Pos.Sub(c.position)
	if delta.Dot(c.roadForward) >= -5 || math.Cos(ob.Heading-c.heading) < .98 {
		return false
	}
	return delta.Dot(c.forward) < -(c.length+ob.Length)/2-5 || math.Abs(delta.Dot(c.right)) > (c.width+ob.Width)/2+2
}

func (ad *AutopilotDriver) isDepartureRearCar(f autopilotFrame, ob VehicleObstacle) bool {
	return ad.departureRearCars(f).matches(ob)
}
