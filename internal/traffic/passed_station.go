package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Empty stops use the road-only approach before the bus would brake for the
// bay. A terminal with passengers or a scheduled vehicle action still needs
// its normal stop even when nobody is waiting.
func (ab *AIBus) canSkipEmptyStation(w *sim.World) bool {
	return CanSkipEmptyStation(w, ab.Bus, ab.Route, ab.Driver, ab.Retiring)
}

// CanSkipEmptyStation is the fleet rule for passing the current target stop:
// nobody aboard alights there and nobody waiting can ride this route. The
// player's standard autopilot uses the same rule.
func CanSkipEmptyStation(w *sim.World, bus *sim.Bus, route *sim.Route, driver *autopilot.AutopilotDriver, retiring bool) bool {
	if w == nil || bus == nil || driver == nil || route == nil || len(route.StopIDs) < 2 || driver.State == autopilot.AutopilotStateDocked ||
		bus.DoorsOpen || bus.DoorProgress > .05 || bus.PassengerServiceRemaining() > 0 || bus.TerminalReplacementHold {
		return false
	}
	id := route.GetCurrentTargetStopID()
	if id < 0 || id >= len(w.Stops) {
		return false
	}
	next := (route.CurrentIndex + 1) % len(route.StopIDs)
	if next == 0 && route.StopIDs[0] == id {
		next = 1 // A closed loop resumes at the first distinct stop.
	}
	if route.StopIDs[next] == id {
		return false
	}
	stop := w.Stops[id]
	speed := math.Abs(bus.Speed)
	if stop.RoadCenter.Distance(bus.Pos) > math.Max(650, speed*speed/(2*85)+200) ||
		(!stop.CenterBRT && stop.ContainsBus(bus)) {
		return false
	}
	if route.IsTerminusIndex(route.CurrentIndex) && (bus.PassengerCount > 0 || retiring || bus.PendingModel != nil) {
		return false
	}
	if bus.CountPassengersAlightingAt(id) > 0 {
		return false
	}
	return stop.WaitingPassengers == 0 || !stop.HasMatchingWaiting(route.GetUpcomingCandidateStopIDs(route.CurrentIndex))
}

// StationSkip keeps a skip once decided. A passenger who reaches the
// platform after the bus committed to passing it waits for the next bus:
// braking from cruise speed that late only overshoots the bay. Passengers
// aboard who alight there, a terminus and a stop the bus is already serving
// still end the skip, as does any change of target stop.
type StationSkip struct {
	latched bool
	stop    int
}

func (s *StationSkip) Reset() { s.latched = false }

// Update returns whether the bus skips its current target stop this tick.
func (s *StationSkip) Update(w *sim.World, bus *sim.Bus, route *sim.Route, driver *autopilot.AutopilotDriver, retiring bool) bool {
	if route == nil || driver == nil || bus == nil || w == nil {
		s.latched = false
		return false
	}
	id := route.GetCurrentTargetStopID()
	if CanSkipEmptyStation(w, bus, route, driver, retiring) {
		s.latched, s.stop = true, id
		return true
	}
	if !s.latched || s.stop != id || !driver.CanSkipTargetStop || id < 0 || id >= len(w.Stops) {
		s.latched = false
		return false
	}
	if driver.State == autopilot.AutopilotStateDocked || bus.DoorsOpen || bus.DoorProgress > .05 ||
		bus.PassengerServiceRemaining() > 0 || bus.TerminalReplacementHold || bus.CountPassengersAlightingAt(id) > 0 ||
		(route.IsTerminusIndex(route.CurrentIndex) && (bus.PassengerCount > 0 || retiring || bus.PendingModel != nil)) {
		s.latched = false
		return false
	}
	return true
}

func (ab *AIBus) skipTargetStation(w *sim.World) bool {
	return ab.skip.Update(w, ab.Bus, ab.Route, ab.Driver, ab.Retiring)
}

// Advance only after reaching the end of the road approach, so a nearby
// parallel road or a turn earlier in the leg cannot skip a stop or count a lap.
func (ab *AIBus) advancePastEmptyStation(w *sim.World, laneWidth, dt float64) {
	if stop := AdvancePastEmptyStation(w, ab.Bus, ab.Route, ab.Driver, laneWidth, dt); stop != nil {
		ab.continuePastStation(stop, laneWidth)
	}
}

// AdvancePastEmptyStation advances a bus skipping its target stop once it
// reaches the end of the road approach, returning the stop it passed. The
// caller finishes the passage (see AIBus.continuePastStation).
func AdvancePastEmptyStation(w *sim.World, bus *sim.Bus, route *sim.Route, driver *autopilot.AutopilotDriver, laneWidth, dt float64) *sim.BusStop {
	if !driver.CanSkipTargetStop || driver.State == autopilot.AutopilotStateDocked || bus.Speed < 0 || bus.DoorsOpen ||
		driver.LastRemaining > bus.Speed*dt+8 {
		return nil
	}
	id := route.GetCurrentTargetStopID()
	path := driver.DrivePath()
	if id < 0 || id >= len(w.Stops) || driver.PathTargetStopID() != id || len(path) < 2 {
		return nil
	}
	stop := w.Stops[id]
	forward := sim.UnitFromAngle(stop.Heading)
	delta := bus.Pos.Sub(path[len(path)-1])
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	lateral := bus.Pos.Sub(stop.RoadCenter).Dot(right)
	roadEdge := float64(max(2, stop.RoadLanes)/2) * laneWidth
	if stop.CenterBRT {
		roadEdge += w.BRTStationFlareAt(bus.Pos, laneWidth)
	}
	if math.Cos(bus.Heading-stop.Heading) < .9 || delta.Dot(forward) < -2 ||
		delta.Dot(forward) > bus.Length || lateral < 0 || lateral > roadEdge {
		return nil
	}
	route.AdvanceToNextStop()
	return stop
}

// An exhausted path or stalled late arrival cannot reverse into a missed bay.
// Recover from the actual pose, preserving collision/merge checks on departure.
func (ab *AIBus) recoverPassedStation(w *sim.World, laneWidth, dt float64, heldByController bool) bool {
	if ab == nil {
		return false
	}
	stop := RecoverPassedStation(w, ab.Bus, ab.Route, ab.Driver, ab.Retiring, &ab.lateArrivalWait, laneWidth, dt, heldByController)
	if stop == nil {
		return false
	}
	ab.continuePastStation(stop, laneWidth)
	return true
}

// RecoverPassedStation moves a bus that can no longer reach its target stop
// on to the next one, retargeting passengers bound for it, and returns the
// stop passed. lateWait carries the late-arrival retry timer between ticks.
func RecoverPassedStation(w *sim.World, bus *sim.Bus, route *sim.Route, driver *autopilot.AutopilotDriver, retiring bool, lateWait *float64, laneWidth, dt float64, heldByController bool) *sim.BusStop {
	if w == nil || driver == nil || route == nil || bus == nil || len(route.StopIDs) < 2 || driver.State == autopilot.AutopilotStateDocked || driver.State == autopilot.AutopilotStateDeparting || bus.DoorsOpen || bus.DoorProgress > .05 || bus.PassengerServiceRemaining() > 0 || bus.TerminalReplacementHold {
		*lateWait = 0
		return nil
	}
	id := route.GetCurrentTargetStopID()
	if id < 0 || id >= len(w.Stops) {
		*lateWait = 0
		return nil
	}
	stop := w.Stops[id]
	path := driver.DrivePath()
	if len(path) < 2 || driver.LastTargetID != id || stop.ContainsBus(bus) {
		*lateWait = 0
		return nil
	}
	u := sim.UnitFromAngle(stop.Heading)
	delta := bus.Pos.Sub(stop.FrontBerthPosition())
	// Require a genuinely passed endpoint on the same road, not an approach,
	// rear berth, adjacent street, or a route that still has turns ahead.
	along := delta.Dot(u)
	lateral := math.Abs(delta.X*u.Y - delta.Y*u.X)
	endpointPassed := bus.Pos.Sub(path[len(path)-1]).Dot(u)
	roadsideArrival := !stop.CenterBRT && driver.State == autopilot.AutopilotStateApproaching &&
		heldByController && math.Abs(bus.Speed) < .5 &&
		math.Cos(bus.Heading-stop.Heading) > .95 && math.Cos(bus.TailHeading()-stop.Heading) > .95 &&
		!(route.IsTerminusIndex(route.CurrentIndex) && (retiring || bus.PendingModel != nil)) &&
		!w.InsideJunction(bus.Pos, laneWidth, bus.OverallLength()/2)
	onBoardingRoad := false
	if roadsideArrival {
		ew := math.Abs(u.X) > .5
		coord := stop.RoadCenter.X
		if ew {
			coord = stop.RoadCenter.Y
		}
		if road, ok := w.RoadAt(coord, ew, bus.Pos); ok {
			direction := road.P2.Sub(road.P1).Normalize()
			side := bus.Pos.Sub(stop.RoadCenter).Dot(sim.Vec2{X: -u.Y, Y: u.X})
			halfWidth := bus.Width / 2
			onBoardingRoad = math.Abs(direction.Dot(u)) > .99 && side >= halfWidth+2 && side <= road.GetWidth(laneWidth)/2-halfWidth
		}
	}
	// A saved wide-lane arrival can stop just before the front marker with
	// its bay taper held almost two lanes away. Its nose has already passed
	// the marker and the remaining platform cannot accommodate that merge.
	// Give the dock planner its usual recovery window, then use the existing
	// missed-stop departure while keeping every passenger aboard.
	wideMissedArrival := onBoardingRoad && !driver.BerserkMode && driver.LastRemaining <= bus.OverallLength() &&
		along >= -bus.Length/2-12 && along <= bus.Length/2+12 && endpointPassed >= -bus.Length/2-12 &&
		lateral > stop.Width/2+laneWidth
	// A long bus may exhaust its taper past the last valid front marker
	// while its powered section is still yawed toward the curb. Its straight
	// tail and passed endpoint distinguish this from a turn still in progress.
	// Beyond the powered section, require a sharply yawed nose; an ordinary
	// mild turn still keeps its normal arrival and boarding opportunity.
	withinTaper := along <= bus.Length || (along <= bus.OverallLength() && math.Abs(sim.NormalizeAngle(bus.Heading-stop.Heading)) > .5)
	yawedPastTaper := bus.IsArticulated() && driver.State == autopilot.AutopilotStateApproaching &&
		heldByController && math.Abs(bus.Speed) < .5 && driver.LastRemaining <= 2 &&
		along > 12 && withinTaper && endpointPassed > 12 &&
		lateral < stop.Width+laneWidth &&
		math.Cos(bus.Heading-stop.Heading) > .5 && math.Cos(bus.TailHeading()-stop.Heading) > .95 &&
		bus.Pos.Distance(stop.Pos) < stop.Length && !w.InsideJunction(bus.Pos, laneWidth, bus.OverallLength()/2)
	if yawedPastTaper {
		for _, rb := range w.Roundabouts {
			if bus.Pos.Distance(rb.Center) < rb.Radius+260 {
				yawedPastTaper = false
				break
			}
		}
	}
	// A compact bus can likewise exhaust its arrival curve past the end of
	// the bay with its nose turned into the curb. Nothing in the dock planner
	// backs it out, so its "remaining 0" hold would last forever.
	compactPastBay := !bus.IsArticulated() && !stop.CenterBRT && driver.State == autopilot.AutopilotStateApproaching &&
		heldByController && math.Abs(bus.Speed) < .5 && driver.LastRemaining <= 2 &&
		along > bus.Length/2+12 && lateral < stop.Width+laneWidth &&
		math.Cos(bus.Heading-stop.Heading) > .5 && math.Cos(bus.Heading-stop.Heading) < .98 &&
		bus.Pos.Distance(stop.Pos) < stop.Length && !w.InsideJunction(bus.Pos, laneWidth, bus.Length/2)
	yawedPastTaper = yawedPastTaper || compactPastBay
	// A saved arrival can be well past the berth in a parallel travel lane.
	// The bay-width check below is too narrow for those lanes on wide roads.
	pastRoadsideBerth := onBoardingRoad && driver.LastRemaining <= 2 &&
		along > bus.Length/2+12 && endpointPassed > bus.Length/2+12
	// A slightly yawed arrival can be past the final berth while its path
	// projection still reports distance remaining. On the verified boarding
	// road, use the same three-second retry as an aligned late arrival;
	// the stricter .98 nose gate would otherwise discard its timer forever.
	lateOutside := driver.State == autopilot.AutopilotStateApproaching && math.Abs(bus.Speed) < .5 && heldByController && driver.LastRemaining <= bus.OverallLength() && along > 8 && endpointPassed > 8 && lateral > stop.Width/2+4
	lateOnBoardingRoad := lateOutside && onBoardingRoad
	// An arrival that stopped level with the front berth in the adjacent
	// lane, behind an empty bay, has no runway left for any berth curve and
	// cannot reverse into the queue behind it. After the usual retry window,
	// continue with every passenger aboard rather than block the lane.
	unreachableBeside := onBoardingRoad && heldByController && driver.StationDockUnreachable() &&
		along > -bus.Length/2 && lateral > stop.Width/2+4 && lateral <= stop.Width/2+laneWidth
	lateOnBoardingRoad = lateOnBoardingRoad || unreachableBeside
	if !wideMissedArrival && !pastRoadsideBerth && ((!yawedPastTaper && !lateOnBoardingRoad && math.Cos(bus.Heading-stop.Heading) < .98) || lateral > stop.Width/2+laneWidth) {
		*lateWait = 0
		return nil
	}
	exhausted := pastRoadsideBerth || yawedPastTaper || (driver.LastRemaining <= 2 && along > bus.Length/2+12 && endpointPassed > bus.Length/2+12)
	// A late outer-lane arrival may project onto the final taper with a
	// positive remaining distance although its actual berth is behind it.
	// Give the normal dock planner time to finish first; never skip a moving
	// entry, a waiting rear berth inside the bay, or a stop still ahead.
	if lateOutside || wideMissedArrival || unreachableBeside {
		*lateWait += math.Max(0, dt)
	} else {
		*lateWait = 0
	}
	if !exhausted && *lateWait < 3 {
		return nil
	}
	*lateWait = 0
	route.AdvanceToNextStop()
	next := route.GetCurrentTargetStopID()
	for i := range bus.Passengers {
		if bus.Passengers[i].TargetStopID == id {
			bus.Passengers[i].TargetStopID = next
		}
	}
	return stop
}

func (ab *AIBus) continuePastStation(stop *sim.BusStop, laneWidth float64) {
	ab.Driver.ContinuePastSkippedStop(ab.Bus, stop, laneWidth)
	ab.Driver.CanSkipTargetStop = false
	ab.skip.Reset()
	ab.lateArrivalWait = 0
	ab.LastExchangedStopID = -1
	ab.Bus.StationMergeRequest = nil
	ab.Bus.LaneMergeRequest = nil
}
