package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type skippedStopDeparture struct {
	stop     *sim.BusStop
	position sim.Vec2
}

func stopRoadPoint(stop *sim.BusStop, lw float64, worlds ...*sim.World) sim.Vec2 {
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	lane := float64(max(2, stop.RoadLanes))/2 - .5
	if stop.RoadLanes == 8 {
		lane = 2.5
	}
	if len(worlds) > 0 && worlds[0] != nil {
		coord := stop.RoadCenter.Y
		ew := math.Abs(forward.X) > .5
		if !ew {
			coord = stop.RoadCenter.X
		}
		return stop.RoadCenter.Add(right.Mul(sim.RoadLaneOffset(coord, ew, lw, worlds[0], stop.RoadCenter)))
	}
	return stop.RoadCenter.Add(right.Mul(lane * lw))
}

// ContinuePastSkippedStop preserves the actual road passage when the game
// advances the route. The next leg must not begin with a fictitious bay exit.
func (ad *AutopilotDriver) ContinuePastSkippedStop(bus *sim.Bus, stop *sim.BusStop, lw float64) {
	world := ad.pathWorld
	ad.Reset()
	ad.LastRemaining = 0
	ad.LastPathS = 0
	ad.skippedDeparture = &skippedStopDeparture{stop: stop, position: bus.Pos}
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	ad.LaneOffset = bus.Pos.Sub(stopRoadPoint(stop, lw, world)).Dot(right)
	ad.TargetLaneOffset = ad.LaneOffset
	ad.LaneOffsetStart = ad.LaneOffset
	ad.lastTargetLaneOffset = ad.LaneOffset
	if math.Abs(ad.LaneOffset) > 1 {
		ad.OvertakeState = OvertakePassing
	}
}

func (ad *AutopilotDriver) prependSkippedDeparture(lw float64, world *sim.World) {
	passage := ad.skippedDeparture
	forward := sim.Vec2{X: math.Cos(passage.stop.Heading), Y: math.Sin(passage.stop.Heading)}
	road := stopRoadPoint(passage.stop, lw, world)
	start := road.Add(forward.Mul(passage.position.Sub(road).Dot(forward)))
	// Retain future turns and trim only the already-passed straight departure.
	points := []sim.Vec2{start}
	for _, point := range ad.drivePath {
		if len(points) == 1 && point.Sub(start).Dot(forward) < 0 {
			continue
		}
		points = append(points, point)
	}
	ad.drivePath = dedupePath(points)
}

func (ad *AutopilotDriver) holdSkippedDepartureLane(bus *sim.Bus, world *sim.World, lw, dt, speed float64) bool {
	passage := ad.skippedDeparture
	if passage == nil || world == nil || len(ad.drivePath) < 2 || ad.State == AutopilotStateDocked || ad.pathFromID != passage.stop.ID {
		return false
	}
	forward := sim.Vec2{X: math.Cos(passage.stop.Heading), Y: math.Sin(passage.stop.Heading)}
	if bus.Pos.Sub(passage.stop.RoadCenter).Dot(forward) > bus.Length/2+80 || math.Cos(bus.Heading-passage.stop.Heading) < .85 || world.InsideJunction(bus.Pos, lw, 0) {
		return false
	}
	ad.TargetLaneOffset = ad.LaneOffset
	ad.progressLaneOffset(dt, speed)
	return true
}

// Once on the final straight approach, continue the existing road lane through
// an eligible stop. This removes both the bay turn and its corner-speed limit.
func (ad *AutopilotDriver) prepareSkippedStopLane(bus *sim.Bus, stop *sim.BusStop, world *sim.World, lw, remaining float64) bool {
	if !ad.CanSkipTargetStop {
		return false
	}
	ad.stationDock = nil
	if ad.bypassStopID == stop.ID || ad.State == AutopilotStateDeparting {
		return false
	}
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	along := stop.RoadCenter.Sub(bus.Pos).Dot(forward)
	lateral := bus.Pos.Sub(stop.RoadCenter).Dot(right)
	if along < 0 || remaining > along+100 || math.Cos(bus.Heading-stop.Heading) < .995 || world.InsideJunction(bus.Pos, lw, 0) {
		return false
	}
	center, ew, _, _ := sim.CurrentRoadInfo(world, bus.Pos, bus.Heading, lw)
	stopCenter := stop.RoadCenter.Y
	if !ew {
		stopCenter = stop.RoadCenter.X
	}
	if ew != (math.Abs(forward.X) > .5) || math.Abs(center-stopCenter) > lw*.2 {
		return false
	}
	if lateral < -lw*.5-2 || math.Abs(lateral) > float64(max(2, stop.RoadLanes))/2*lw-bus.Width/2+5 {
		return false
	}
	// Do not shortcut a real junction, roundabout or return loop that happens
	// to finish on the same road: every remaining waypoint must be straight.
	roadLateral := stopRoadPoint(stop, lw, ad.pathWorld).Sub(stop.RoadCenter).Dot(right)
	for i := ad.WaypointIdx + 1; i < len(ad.drivePath); i++ {
		if math.Abs(ad.drivePath[i].Sub(stop.RoadCenter).Dot(right)-roadLateral) > 1 {
			return false
		}
	}
	ad.drivePath = []sim.Vec2{bus.Pos, stop.RoadCenter.Add(right.Mul(lateral))}
	ad.cumLen = []float64{0, along}
	ad.WaypointIdx = 0
	ad.LastPathS = 0
	ad.pathSynced = false
	ad.LaneOffset, ad.TargetLaneOffset = 0, 0
	ad.LaneOffsetStart = 0
	ad.lastTargetLaneOffset = 0
	ad.LaneOffsetProgress = 1
	ad.OvertakeState = OvertakeNone
	ad.bypassStopID = stop.ID
	return true
}

// A stopped bus wholly inside a skipped bay is alongside the road corridor,
// even where the generic front-obstacle margin would reach into that bay.
func (ad *AutopilotDriver) clearsSkippedStopBay(f autopilotFrame, obstacle VehicleObstacle) bool {
	var stop *sim.BusStop
	if ad.CanSkipTargetStop {
		stop = f.targetStop
	}
	if ad.skippedDeparture != nil && obstacle.Pos.Distance(ad.skippedDeparture.stop.Pos) < 100 {
		stop = ad.skippedDeparture.stop
	}
	if stop == nil || math.Abs(obstacle.Speed) > 5 || obstacle.Pos.Distance(stop.Pos) > stop.Width/2+obstacle.Width/2+5 || math.Cos(obstacle.Heading-stop.Heading) < .98 || math.Cos(f.bus.Heading-stop.Heading) < .98 || roadSideClearance(f.bus, obstacle) <= 2 || len(ad.drivePath) < 2 {
		return false
	}
	projected := ad.basePathPosition(obstacle.Pos, f.world, f.bus.Heading, f.laneWidth)
	return minDistanceToPath(projected, ad.drivePath, ad.WaypointIdx) > f.bus.Width/2+obstacle.Width/2+2
}
