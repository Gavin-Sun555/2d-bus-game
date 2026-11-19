package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

const (
	offRouteLanes = 5   // Body distance from the path, in lane widths.
	offRouteDelay = 2.0 // Seconds off route before re-planning.
)

// A fleet bus can leave its route: it misses a turn it could no longer
// make, or an older save restores it past one. Its leg path then lies behind
// or beside it, so every later decision steers toward an unreachable point
// and the bus strands, often across a lane. After a short confirmation,
// re-plan the leg from the road the bus is actually on to its next stop.
func (ad *AutopilotDriver) replanOffRoute(bus *sim.Bus, w *sim.World, to *sim.BusStop, lw, dt float64) bool {
	ad.offRoadNetwork = false
	ad.hardBarrierTimer = math.Max(0, ad.hardBarrierTimer-math.Max(0, dt))
	if !ad.FleetBus || w == nil || to == nil || len(ad.drivePath) < 2 || ad.State == AutopilotStateDocked ||
		bus.DoorsOpen || bus.DoorProgress > .05 || bus.PassengerServiceRemaining() > 0 || bus.TerminalReplacementHold ||
		ad.headonRecovery != nil || ad.roundaboutEdgeRecovery != nil || ad.roadReverseActive || ad.pathOffRouteDistance(bus) <= offRouteLanes*lw {
		ad.offRouteWait = 0
		return false
	}
	for _, rb := range w.Roundabouts {
		if bus.Pos.Distance(rb.Center) < rb.Radius+bus.OverallLength() {
			ad.offRouteWait = 0
			return false
		}
	}
	ad.offRouteWait += math.Max(0, dt)
	if ad.offRouteWait < offRouteDelay {
		return false
	}
	ad.offRouteWait = 0
	heading := approachRoadHeading(bus.Heading)
	road, center, ok := roadAbeam(w, bus.Pos, heading, lw)
	if !ok {
		// Beyond the end of every road, such as past an edge road in an
		// older save: no leg can start here. The fleet resets the bus. A
		// moving bus, or one turned across a road (in a junction or after a
		// collision), is not stranded unless it is stopped pinned against a
		// dead-end barrier.
		_, _, crossRoad := roadAbeam(w, bus.Pos, heading+math.Pi/2, lw)
		nose := bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Length/2 + 8))
		barrierPinned := bus.HitsHardBarrier(w, lw) || w.CheckBoundaries(nose, bus.Width/2, lw).Zone == sim.ZoneHardBarrier || ad.hardBarrierTimer > 0
		ad.offRoadNetwork = (math.Abs(bus.Speed) < 1 && !crossRoad) || barrierPinned
		return false
	}
	from := &sim.BusStop{ID: -1, RoadCenter: center, Heading: heading, RoadLanes: road.LaneCount(), Pos: bus.Pos}
	path := ad.buildLegDrivePath(w, from, to, lw, true, ad.CanSkipTargetStop, sim.RoadPathOptions{OrdinaryBusLaneLeft: false, LongVehicle: ad.articulatedPath})
	if len(path) < 2 {
		return false
	}
	if ad.BeforeRecovery != nil {
		ad.BeforeRecovery()
	}
	ad.resetLegManeuvers()
	ad.offRouteOrigin = from
	// Keep PrepareLeg's cache key, or it would rebuild the stale leg.
	ad.pathOrdinaryBusLeft = ad.ordinaryBusLeft
	ad.pathSkipTarget = ad.CanSkipTargetStop
	ad.drivePath = path
	ad.pathRevision = currentDrivePathRevision
	ad.cumLen = make([]float64, len(path))
	for i := 1; i < len(path); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + path[i].Distance(path[i-1])
	}
	ad.pathSynced = false
	ad.WaypointIdx = 0
	ad.LastPathS = 0
	ad.LastRemaining = ad.cumLen[len(path)-1]
	ad.stationDock = nil
	ad.skippedDeparture = nil
	ad.bypassStopID = -1
	ad.busTurnHoldDistance = 0
	ad.OvertakeState = OvertakeNone
	ad.holdLaneOffset(0)
	bus.LaneMergeRequest = nil
	bus.StationMergeRequest = nil
	return true
}

// pathOffRouteDistance is the distance from the bus's nearest body point to
// its path, ignoring any lane offset: a stale offset is itself the symptom.
func (ad *AutopilotDriver) pathOffRouteDistance(bus *sim.Bus) float64 {
	best := math.Inf(1)
	for idx, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		for i := 1; i < len(ad.drivePath); i++ {
			_, d := projectToSegment(part.Center, ad.drivePath[i-1], ad.drivePath[i])
			best = math.Min(best, d-part.HalfWid)
		}
		if idx == 0 && math.Abs(bus.Speed) < 1 {
			break
		}
	}
	return best
}

// OffRoadNetwork reports, once, a fleet bus that has left its route and is
// not on any road, for example past the end of an edge road in an older save.
// It cannot re-plan from there; the fleet returns it to its line's first stop.
// Reading clears the report: the next controls may return before re-checking.
func (ad *AutopilotDriver) OffRoadNetwork() bool {
	if ad == nil || !ad.offRoadNetwork {
		return false
	}
	ad.offRoadNetwork = false
	return true
}

// roadAbeam finds the road running along heading that the bus lies on or
// beside (a stop bay or shoulder), and the point on its axis abeam of pos.
func roadAbeam(w *sim.World, pos sim.Vec2, heading, lw float64) (sim.RoadSegment, sim.Vec2, bool) {
	u := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	best, bestLateral := sim.RoadSegment{}, math.Inf(1)
	var bestCenter sim.Vec2
	for _, r := range w.Roads {
		axis := r.P2.Sub(r.P1)
		length := axis.Length()
		if length < 1 || math.Abs(axis.Mul(1/length).Dot(u)) < .99 {
			continue
		}
		t := pos.Sub(r.P1).Dot(axis) / (length * length)
		if t < 0 || t > 1 {
			continue
		}
		center := r.P1.Add(axis.Mul(t))
		side := pos.Sub(center).Dot(right)
		half := float64(r.LaneCount()) / 2 * lw
		lateral := math.Abs(side)
		if side >= -half && side <= half+2.5*lw && lateral < bestLateral {
			best, bestLateral, bestCenter = r, lateral, center
		}
	}
	return best, bestCenter, !math.IsInf(bestLateral, 1)
}

// NoteHardBarrierImpact records a hard-barrier pushback so an off-network bus
// oscillating between a dead-end wall and pushback recovery is still detected.
func (ad *AutopilotDriver) NoteHardBarrierImpact() {
	if ad != nil {
		ad.hardBarrierTimer = 3.0
	}
}

// Older saves kept the recovered path but omitted its virtual road origin.
// Recognize a straight road prefix far from the scheduled departure stop so
// later turn-lane choices rebuild from that prefix instead of the old station.
func (ad *AutopilotDriver) inferSavedOffRouteOrigin(w *sim.World) {
	if ad.offRouteOrigin != nil || len(ad.drivePath) < 2 || ad.pathFromID < 0 || ad.pathFromID >= len(w.Stops) || ad.skippedDeparture != nil || ad.dockDepartureRun > 0 || ad.fallbackJunction != nil {
		return
	}
	start := ad.drivePath[0]
	from := w.Stops[ad.pathFromID]
	if start.Distance(from.Pos) < math.Max(600, from.Length+200) {
		return
	}
	d := ad.drivePath[1].Sub(start)
	if d.Length() < 5 {
		return
	}
	h := math.Atan2(d.Y, d.X)
	heading := approachRoadHeading(h)
	if math.Cos(h-heading) < .999 {
		return
	}
	road, center, ok := roadAbeam(w, start, heading, ad.pathLane)
	if !ok {
		return
	}
	ad.offRouteOrigin = &sim.BusStop{ID: -1, Pos: start, RoadCenter: center, Heading: heading, RoadLanes: road.LaneCount()}
}
