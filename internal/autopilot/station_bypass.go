package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type skippedStopDeparture struct {
	stop     *sim.BusStop
	position sim.Vec2
	served   bool
}

func stopRoadPoint(stop *sim.BusStop, lw float64, worlds ...*sim.World) sim.Vec2 {
	forward := sim.UnitFromAngle(stop.Heading)

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
	// In berserk mode an empty first stop can be skipped while still parked in its
	// bay. It must take the normal pull-out instead of carrying a shoulder
	// offset down the road as though it had already driven past the platform.
	if ad.BerserkMode && math.Abs(bus.Speed) < 10 && stop.ContainsBus(bus) {
		ad.Reset()
		ad.State = AutopilotStateDeparting
		ad.departureFromDock = true
		return
	}
	ad.beginRoadDeparture(bus, stop, lw)
}

// Both a lawful passage and completed roadside boarding leave from the actual
// road lane. Neither should reconnect to the platform's bay-exit taper.
func (ad *AutopilotDriver) beginRoadDeparture(bus *sim.Bus, stop *sim.BusStop, lw float64) {
	world := ad.pathWorld
	ad.Reset()
	ad.LastRemaining = 0
	ad.LastPathS = 0
	ad.skippedDeparture = &skippedStopDeparture{stop: stop, position: bus.Pos}
	forward := sim.UnitFromAngle(stop.Heading)
	// An exhausted, yawed bay arrival needs a proved crawl from its very
	// first moving tick. Waiting for the nominal speed cap to reach zero
	// allows acceleration while the wheels are still pointing at the curb.
	if bus.IsArticulated() && math.Abs(bus.Speed) < .5 && bus.Pos.Sub(stop.FrontBerthPosition()).Dot(forward) > 32 && math.Abs(sim.NormalizeAngle(bus.Heading-stop.Heading)) > .15 {
		ad.departureEscapeActive = true
	}
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
	forward := sim.UnitFromAngle(passage.stop.Heading)
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
	// The actual road can have a dedicated bus lane that was unavailable
	// when the skip was requested before the first leg was prepared.
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	desired := passage.position.Sub(start).Dot(right)
	if math.Abs(ad.LaneOffset-desired) > 1e-6 {
		ad.holdLaneOffset(desired)
	}
}

func (ad *AutopilotDriver) holdSkippedDepartureLane(bus *sim.Bus, world *sim.World, lw, dt, speed float64) bool {
	passage := ad.skippedDeparture
	if passage == nil || passage.served || world == nil || len(ad.drivePath) < 2 || ad.State == AutopilotStateDocked || ad.pathFromID != passage.stop.ID {
		return false
	}
	forward := sim.UnitFromAngle(passage.stop.Heading)
	if bus.Pos.Sub(passage.stop.RoadCenter).Dot(forward) > bus.Length/2+80 || math.Cos(bus.Heading-passage.stop.Heading) < .85 || world.InsideJunction(bus.Pos, lw, 0) {
		return false
	}
	// Reconcile restored road-only legs with their real path origin. A saved
	// fallback lane reference must not push the held passage into the shoulder.
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	desired := passage.position.Sub(ad.drivePath[0]).Dot(right)
	if math.Abs(ad.LaneOffset-desired) > 1e-6 {
		ad.holdLaneOffset(desired)
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
	forward := sim.UnitFromAngle(stop.Heading)
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
	if ad.skippedDeparture != nil {
		stop = ad.skippedDeparture.stop
	}
	if stop == nil || math.Abs(obstacle.Speed) > 5 || !obstacleInsideStationBay(stop, obstacle) || math.Cos(obstacle.Heading-stop.Heading) < .98 || math.Cos(f.bus.Heading-stop.Heading) < .98 || roadSideClearance(f.bus, obstacle) <= 2 || len(ad.drivePath) < 2 {
		return false
	}
	projected := ad.basePathPosition(obstacle.Pos, f.world, f.bus.Heading, f.laneWidth)
	return minDistanceToPath(projected, ad.drivePath, ad.WaypointIdx) > f.bus.Width/2+obstacle.Width/2+2
}

// Front and rear berths are both legitimate bay positions. Test the physical
// sections against the whole platform, not distance from its center; a tail
// protruding into the road must keep its normal obstacle priority.
func obstacleInsideStationBay(stop *sim.BusStop, ob VehicleObstacle) bool {
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	parts := predictedObstacleBodies(ob, 0)
	for _, part := range parts {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - stop.Heading
		along := math.Abs(math.Cos(yaw))*part.HalfLen + math.Abs(math.Sin(yaw))*part.HalfWid
		across := math.Abs(math.Sin(yaw))*part.HalfLen + math.Abs(math.Cos(yaw))*part.HalfWid
		delta := part.Center.Sub(stop.Pos)
		if math.Abs(delta.Dot(u))+along > stop.Length/2+2 || math.Abs(delta.Dot(r))+across > stop.Width/2+2 {
			return false
		}
	}
	return true
}
