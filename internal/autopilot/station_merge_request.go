package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) requestStationMerge(f autopilotFrame) {
	if f.world == nil || f.bus.DoorsOpen || f.bus.DoorProgress > .05 || ad.State == AutopilotStateDocked {
		return
	}
	if stop, _, _, ok := ad.stationDepartureRoad(f); ok {
		f.bus.StationMergeRequest = &sim.BusStationMergeRequest{StopID: stop.ID, Departing: true, Remaining: .3}
		return
	}
	stop := f.targetStop
	if ad.CanSkipTargetStop || stop == nil || ad.State == AutopilotStateDeparting || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return
	}
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	distance := stop.Pos.Sub(f.bus.Pos).Dot(forward)
	curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
	lateral := f.bus.Pos.Sub(stop.RoadCenter).Dot(right)
	// Reserve only the adjacent curb lane; other traffic remains free to pass.
	if distance > 0 && distance < 650 && lateral >= curb-f.laneWidth-12 && lateral < stop.Pos.Sub(stop.RoadCenter).Dot(right)-3 && math.Cos(f.bus.Heading-stop.Heading) > .9 {
		f.bus.StationMergeRequest = &sim.BusStationMergeRequest{StopID: stop.ID, Remaining: .3}
	}
}
