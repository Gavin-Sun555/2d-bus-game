package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Flexible boarding can label a partly aligned bay arrival as roadside service.
// If the bus is actually beyond the road curb, a road-only departure starts a
// lane away and demands full steering beside the shelter. Reconnect from its
// real pose with the normal pull-forward/taper instead. This also repairs saved
// road-only paths; an already connected departure starts on the actual bay axis.
func (ad *AutopilotDriver) reconnectServedBayDeparture(bus *sim.Bus, stop *sim.BusStop, lw float64) {
	passage := ad.skippedDeparture
	if passage == nil || !passage.served || stop == nil || stop.ID != passage.stop.ID || ad.pathFromID != stop.ID || !bus.IsArticulated() || len(ad.drivePath) < 2 || math.Abs(bus.Speed) > 10 {
		return
	}
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	if math.Cos(bus.Heading-stop.Heading) < .97 || bus.Pos.Distance(passage.position) > bus.OverallLength()+120 || bus.Pos.Sub(stop.RoadCenter).Dot(right) <= float64(stop.RoadLanes)*lw/2 {
		return
	}
	if math.Abs(ad.drivePath[0].Sub(passage.position).Dot(right)) <= lw*.5 {
		return
	}
	ad.connectDockDeparture(bus, stop)
}
