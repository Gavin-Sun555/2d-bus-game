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

// throughStationBus identifies a stationary bus serving a stop this driver
// can pass. The route's own docking queue keeps its normal headway.
func (ad *AutopilotDriver) throughStationBus(f autopilotFrame, ob VehicleObstacle) bool {
	if !ad.DisallowOncomingOvertake || f.world == nil || ob.Bus == nil || math.Abs(ob.Speed) >= 3 {
		return false
	}
	canPass := func(stopID int) bool {
		return ad.CanSkipTargetStop || f.targetStop == nil || stopID != f.targetStop.ID
	}
	service := ob.Bus.PassengerService
	if service.Active {
		return canPass(service.StopID)
	}
	if request := ob.Bus.StationMergeRequest; request != nil {
		return canPass(request.StopID)
	}
	if ob.Bus.DoorsOpen {
		for _, stop := range f.world.Stops {
			if stop.ContainsBus(ob.Bus) || stop.ContainsBusInOuterLane(ob.Bus, f.laneWidth) {
				return canPass(stop.ID)
			}
		}
	}
	return false
}

// A parked articulated bus can have a wider bounding box than either body
// section. Test its actual sections against the reserved lane's travel path.
func (ad *AutopilotDriver) stationBusLaneGap(f autopilotFrame, forward, right sim.Vec2, busLane sim.Vec2) float64 {
	runway := math.Max(450, f.speedAbs*2.4+160)
	nose := f.bus.Length / 2
	gap := math.Inf(1)
	corridor := sim.OBB{
		Center:  busLane.Add(forward.Mul(nose + runway/2)),
		Heading: math.Atan2(forward.Y, forward.X),
		HalfLen: runway / 2, HalfWid: f.bus.Width/2 + 2,
	}
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(busLane)
		if math.Abs(delta.Dot(right)) > f.laneWidth*1.5 || delta.Dot(forward) < -ob.Length/2 ||
			delta.Dot(forward) > nose+runway+ob.Length/2 || !ad.throughStationBus(f, ob) {
			continue
		}
		for _, part := range ob.Bus.BodyParts() {
			if part.HalfLen > 0 {
				if hit, _, _ := sim.CheckOBBCollision(corridor, part); hit {
					yaw := part.Heading - corridor.Heading
					rear := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
					gap = math.Min(gap, part.Center.Sub(busLane).Dot(forward)-rear-nose)
				}
			}
		}
	}
	return gap
}

func stopRoadPoint(stop *sim.BusStop, lw float64, worlds ...*sim.World) sim.Vec2 {
	forward := sim.UnitFromAngle(stop.Heading)

	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	lane := float64(max(2, stop.RoadLanes))/2 - .5
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
	// Departure holding belongs only to the stop's own road arm. A later
	// parallel arm can share its heading and still be "behind" the old stop
	// longitudinally, long after the departure maneuver has finished.
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	roadReach := float64(max(2, passage.stop.RoadLanes))*lw/2 + sim.ShoulderWidth + bus.Width
	if math.Abs(bus.Pos.Sub(passage.stop.RoadCenter).Dot(right)) > roadReach {
		return false
	}
	if bus.Pos.Sub(passage.stop.RoadCenter).Dot(forward) > bus.Length/2+80 || math.Cos(bus.Heading-passage.stop.Heading) < .85 || world.InsideJunction(bus.Pos, lw, 0) {
		return false
	}
	// Reconcile restored road-only legs with their real path origin. A saved
	// fallback lane reference must not push the held passage into the shoulder.
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
func (ad *AutopilotDriver) prepareSkippedStopLane(bus *sim.Bus, stop *sim.BusStop, world *sim.World, obstacles []VehicleObstacle, lw, remaining float64) bool {
	if !ad.CanSkipTargetStop {
		return false
	}
	ad.stationDock = nil
	if ad.State == AutopilotStateDeparting || math.Abs(ad.LaneOffset-ad.TargetLaneOffset) > 1 {
		return false
	}
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	repair := ad.bypassStopID == stop.ID
	if repair {
		if len(ad.drivePath) != 2 || math.Abs(ad.LaneOffset) > 1 {
			return false
		}
		// Old saves may already contain a straight bypass along a lane line.
		// Leave valid shortcuts and their lane offsets alone.
		end := ad.drivePath[1].Sub(stop.RoadCenter)
		if math.Abs(end.Dot(forward)) > 1 || math.Abs(ad.drivePath[1].Sub(ad.drivePath[0]).Dot(right)) > 1 {
			return false
		}
		pathLateral := end.Dot(right)
		if math.Abs(pathLateral-(math.Floor(pathLateral/lw)+.5)*lw) < 1 {
			return false
		}
	}
	along := stop.RoadCenter.Sub(bus.Pos).Dot(forward)
	lateral := bus.Pos.Sub(stop.RoadCenter).Dot(right)
	if along <= 3 || remaining > along+100 || math.Cos(bus.Heading-stop.Heading) < .995 || world.InsideJunction(bus.Pos, lw, 0) {
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
	road, ok := world.RoadAt(center, ew, bus.Pos)
	if !ok || lateral < 0 || lateral > float64(road.LaneCount()/2)*lw-bus.Width/2+5 {
		return false
	}
	// Do not shortcut a real junction, roundabout or return loop that happens
	// to finish on the same road: every remaining waypoint must be straight.
	if !repair {
		roadLateral := stopRoadPoint(stop, lw, world).Sub(stop.RoadCenter).Dot(right)
		for i := ad.WaypointIdx + 1; i < len(ad.drivePath); i++ {
			if math.Abs(ad.drivePath[i].Sub(stop.RoadCenter).Dot(right)-roadLateral) > 1 {
				return false
			}
		}
	}
	// The current position may be partway through a merge. Use its lane
	// center as the new path reference only when the bus can safely join it.
	laneCenter := (math.Floor(lateral/lw) + .5) * lw
	f := autopilotFrame{bus: bus, world: world, obstacles: obstacles, targetStop: stop, laneWidth: lw, speedAbs: math.Abs(bus.Speed)}
	if (road.HasMedianBusLane() && brtPlatformBlocksDividerCross(world, bus, stop.Heading, lw, lateral, laneCenter)) ||
		!requiredMergeLaneClear(f, stop.Heading, lateral, laneCenter) {
		return false
	}
	start := bus.Pos.Add(right.Mul(laneCenter - lateral))
	ad.drivePath = []sim.Vec2{start, stop.RoadCenter.Add(right.Mul(laneCenter))}
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
