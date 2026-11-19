package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A later junction maneuver owns its own lane and settling checks. Retaining
// the previous exit target makes the old lane center a permanent speed cap when
// a higher-priority approach returns before updateExitCruise can expire it.
func (ad *AutopilotDriver) exitCruiseSuperseded() bool {
	if ad.exitCruise == nil {
		return false
	}
	center := ad.exitCruise.Junction
	return (ad.approachMerge != nil && ad.approachMerge.Junction.Distance(center) > 1) ||
		(ad.rightTurnJunction != nil && ad.rightTurnJunction.Distance(center) > 1) ||
		(ad.busLaneJunction != nil && ad.busLaneJunction.Distance(center) > 1)
}

func (ad *AutopilotDriver) retireSupersededExitCruise() {
	if ad.exitCruiseSuperseded() {
		ad.exitCruise = nil
	}
}

// Once a turn has straightened, lane position belongs to the outgoing road,
// not to the route's optional return taper. Keep one physical lane until the
// next required maneuver or a stopped obstruction needs the normal lane policy.
func (ad *AutopilotDriver) updateExitCruise(f autopilotFrame, dt float64) bool {
	if f.world == nil || len(ad.drivePath) < 2 {
		return false
	}
	if ad.exitCruise == nil && ad.busLaneJunction != nil {
		// Older saves can already be beyond the junction with an approach latch.
		c := *ad.busLaneJunction
		heading := approachRoadHeading(f.bus.Heading)
		u := sim.UnitFromAngle(heading)
		g := f.world.JunctionAt(c, f.laneWidth)
		half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
		if f.bus.Pos.Sub(c).Dot(u) > half+f.bus.Length/2 {
			ad.exitCruise = &sim.BusLaneMergeRequest{Junction: c, Heading: heading}
		}
	}
	request := ad.exitCruise
	if request == nil {
		return false
	}
	stationRunway := math.Max(560, f.speedAbs*2.4+180)
	if !ad.CanSkipTargetStop && f.remaining < stationRunway {
		ad.exitCruise = nil
		return false
	}
	forward := sim.UnitFromAngle(request.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	g := f.world.JunctionAt(request.Junction, f.laneWidth)
	half := math.Abs(forward.X)*g.HalfX + math.Abs(forward.Y)*g.HalfY
	if !request.Joined {
		if f.bus.Pos.Sub(request.Junction).Dot(forward) < half+f.bus.Length/2 || math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) > .25 {
			return false
		}
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok {
		ad.exitCruise = nil
		return false
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	if !request.Joined {
		lane := max(0, min(road.LaneCount()/2-1, int(math.Round(f.bus.Pos.Sub(origin).Dot(right)/f.laneWidth-.5))))
		request.Target = origin.Add(right.Mul((float64(lane) + .5) * f.laneWidth))
		request.Joined = true
	}
	targetLateral := request.Target.Sub(origin).Dot(right)
	sameRoad := targetLateral >= f.laneWidth*.5-1 && targetLateral <= (float64(road.LaneCount()/2)-.5)*f.laneWidth+1 &&
		math.Abs(sim.NormalizeAngle(approachRoadHeading(f.bus.Heading)-request.Heading)) < .01
	approach := ad.approachingIntersection(f, true, road.LaneCount() >= 6)
	turnRunway := math.Max(650, f.speedAbs*1.6)
	needsTurn := false
	for i := max(1, ad.WaypointIdx+1); i < len(ad.drivePath); i++ {
		if ad.cumLen[i]-f.s > turnRunway {
			break
		}
		if ad.cumLen[i] < f.s {
			continue
		}
		if ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize().Dot(forward) < .9 {
			needsTurn = true
			break
		}
	}
	lane := int(math.Round(request.Target.Sub(origin).Dot(right)/f.laneWidth - .5))
	if !sameRoad || needsTurn || approach.isApproachingIntersection || (!ad.CanSkipTargetStop && f.remaining < stationRunway) {
		ad.exitCruise = nil
		return false
	}
	// The old approach must not cap outgoing speed or reinterpret offsets.
	if ad.busLaneJunction != nil && ad.busLaneJunction.Distance(request.Junction) < 1 {
		ad.busLaneJunction = nil
		ad.busLaneTurnChosen = false
		ad.busJunctionEntered = false
	}
	request.Remaining = math.Max(0, request.Remaining-dt)
	base := ad.pointAt(f.s)
	actual := f.bus.Pos.Sub(base).Dot(right)
	currentLateral := request.Target.Sub(origin).Dot(right)
	physLat := f.bus.Pos.Sub(origin).Dot(right)
	minSectionLat := physLat
	if f.bus.IsArticulated() {
		minSectionLat = math.Min(minSectionLat, f.bus.TrailerBody().Center.Sub(origin).Dot(right))
	}
	islandCenterLat := sim.BRTPlatformOffset(f.laneWidth)
	if road.HasMedianBusLane() && physLat < islandCenterLat && minSectionLat < f.laneWidth+6 && currentLateral > f.laneWidth &&
		brtPlatformBlocksDividerCross(f.world, f.bus, request.Heading, f.laneWidth, minSectionLat, currentLateral) {
		request.Target = origin.Add(right.Mul(.5 * f.laneWidth))
		currentLateral = .5 * f.laneWidth
		lane = 0
	}
	best := ad.busLaneTrafficScore(f, forward, right, lane, true)
	stationGap := ad.stationBusLaneGap(f, forward, right, f.bus.Pos.Add(right.Mul(request.Target.Sub(f.bus.Pos).Dot(right))))
	if request.Remaining == 0 && math.Abs(request.Target.Sub(f.bus.Pos).Dot(right)) < f.laneWidth*.2 &&
		math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) < .1 &&
		((ad.BerserkMode && best > 0) || !math.IsInf(stationGap, 1)) {
		for _, candidate := range []int{lane - 1, lane + 1} {
			if candidate < 0 || candidate >= road.LaneCount()/2 {
				continue
			}
			score := ad.busLaneTrafficScore(f, forward, right, candidate, true)
			lateral := (float64(candidate) + .5) * f.laneWidth
			target := origin.Add(right.Mul(lateral))
			if score+1 < best && requiredMergeLaneClear(f, request.Heading, actual, target.Sub(base).Dot(right)) &&
				!brtPlatformBlocksDividerCross(f.world, f.bus, request.Heading, f.laneWidth, currentLateral, lateral) {
				request.Target = target
				request.Remaining = 6
				break
			}
		}
	}
	target := request.Target
	if currentLateral >= f.laneWidth && f.world.CenterBRT {
		// The outgoing lane is stored in road coordinates, but general lanes
		// flare around BRT islands. An absolute hold must follow that same
		// flare; otherwise it aims straight through the platform shelter.
		ahead := math.Max(120, f.speedAbs*.9+90)
		if f.bus.IsArticulated() {
			ahead = math.Max(ahead, 220)
		}
		flare := f.world.BRTStationFlareAt(f.bus.Pos.Add(forward.Mul(ahead)), f.laneWidth)
		for _, along := range []float64{ahead * .5, 0, -(f.bus.OverallLength() - f.bus.Length/2)} {
			flare = math.Max(flare, f.world.BRTStationFlareAt(f.bus.Pos.Add(forward.Mul(along)), f.laneWidth))
		}
		target = target.Add(right.Mul(flare))
	}
	desired := target.Sub(base).Dot(right)
	// The station flare bends the existing lane; following its center does
	// not require an overtaking slot behind the bus. A genuine change to a
	// different road lane still checks the whole crossed gap, unless the bus
	// is already straddling an imminent Center BRT platform island corridor.
	straddlingBRTIsland := f.world.CenterBRT &&
		physLat > f.laneWidth-f.bus.Width/2-4 &&
		physLat < f.laneWidth+sim.BRTStationMaxFlare+f.bus.Width/2+4 &&
		brtPlatformAlongsideOrImminent(f.world, f.bus, request.Heading, f.laneWidth)
	roadLaneShift := request.Target.Sub(f.bus.Pos).Dot(right)
	if !straddlingBRTIsland && math.Abs(desired-actual) > f.laneWidth*.35 && math.Abs(roadLaneShift) > f.laneWidth*.35 && !requiredMergeLaneClear(f, request.Heading, actual, desired) {
		// Keep moving in the current corridor while the lateral gap is
		// blocked. Stopping here makes a same-lane follower close the very
		// gap we need forever; ordinary following and body sweeps still
		// stop for traffic actually ahead of this held corridor.
		target = f.bus.Pos
		desired = actual
	}
	ad.holdLaneOffset(desired)
	ad.pathReturnHold = &sim.BusLaneMergeRequest{Target: target, Heading: request.Heading}
	return true
}

// A straight path can already be ahead while the steering and trailer are still
// unwinding. Finish that physical turn before accelerating to cruising speed.
func (ad *AutopilotDriver) exitCruiseSettling(f autopilotFrame) bool {
	request := ad.exitCruise
	if request == nil || ad.exitCruiseSuperseded() {
		return false
	}
	// A nearby station already controls its approach, bay taper and speed.
	// A pending exit must not shorten pursuit while the bus is docking.
	dockWindow := math.Max(1200, f.speedAbs*2.4+180)
	if f.targetStop != nil && f.targetStop.CenterBRT {
		dockWindow = 380
	}
	if ad.stationDock != nil || (!ad.CanSkipTargetStop && f.remaining < dockWindow) {
		return false
	}
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) > .03 || math.Abs(f.bus.SteeringAngle) > .05 {
		return true
	}
	if f.bus.IsArticulated() && math.Abs(sim.NormalizeAngle(f.bus.TrailerHeading-request.Heading)) > .04 {
		return true
	}
	if request.Joined {
		right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
		tolerance := math.Min(3, math.Max(1, (f.laneWidth-f.bus.Width)/4))
		return math.Abs(request.Target.Sub(f.bus.Pos).Dot(right)) > tolerance
	}
	return false
}
