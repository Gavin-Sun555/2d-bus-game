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
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	distance := stop.Pos.Sub(f.bus.Pos).Dot(forward)
	curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
	lateral := f.bus.Pos.Sub(stop.RoadCenter).Dot(right)
	front := stop.FrontBerthPosition().Sub(stop.Pos).Dot(forward)
	// Reserve only the adjacent curb lane; other traffic remains free to pass.
	// An articulated bus already inside the bay does not reserve the curb lane
	// while waiting behind two buses ahead in the bay; otherwise its request
	// holds the curb-lane car queue and traps those two buses from departing.
	if f.bus.IsArticulated() && stop.HasTwoBerths() && math.Abs(f.bus.Pos.Sub(stop.Pos).Dot(right)) <= stop.Width/2+4 && !ad.stationBoardingCapacity(f.bus, stop, f.obstacles) {
		return
	}
	if stop.CenterBRT {
		dLong := f.bus.Pos.Sub(stop.Pos).Dot(forward)
		islandCutoff := -brtInwardReturnMinClear(f.bus, f.laneWidth, lateral)
		approachWindow := math.Max(1350, f.speedAbs*3.5+f.bus.OverallLength()+320)
		maxLat := float64(max(2, stop.RoadLanes)/2)*f.laneWidth + f.world.BRTStationFlareAt(f.bus.Pos, f.laneWidth) + 12
		if distance > 0 && distance < approachWindow && dLong <= islandCutoff && lateral > f.laneWidth+4 && lateral <= maxLat && math.Cos(f.bus.Heading-stop.Heading) > .9 {
			mergeTarget := stop.RoadCenter.Add(right.Mul(.5 * f.laneWidth)).Add(forward.Mul(f.bus.Pos.Sub(stop.RoadCenter).Dot(forward)))
			f.bus.StationMergeRequest = &sim.BusStationMergeRequest{StopID: stop.ID, Remaining: .3}
			f.bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: mergeTarget, Heading: stop.Heading, Remaining: .3}
		}
		return
	}
	if distance > -front && distance < 650 && lateral >= curb-f.laneWidth-12 && lateral < stop.Pos.Sub(stop.RoadCenter).Dot(right)-3 && math.Cos(f.bus.Heading-stop.Heading) > .9 {
		f.bus.StationMergeRequest = &sim.BusStationMergeRequest{StopID: stop.ID, Remaining: .3}
		// While waiting in the curb lane far upstream of a full bay, do not claim a
		// multi-lane lateral merge slot that freezes buses passing in the adjacent lane.
		if !stop.CenterBRT && !(distance > 360 && math.Abs(lateral-curb) <= 12 && !ad.stationBoardingCapacity(f.bus, stop, f.obstacles)) {
			f.bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: stop.Pos, Heading: stop.Heading, Remaining: .3}
		}
	}
}
