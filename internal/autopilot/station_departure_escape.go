package autopilot

import (
	"context"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"runtime/trace"
)

// A yawed articulated departure can be pinned by a stopped rear follower:
// one steering choice swings its tail toward that follower, while a slightly
// different forward steering choice safely draws both sections away.
// Prove a short, slow actuator trajectory against every physical body before
// using it. A missed berth may also use the separately proved short backing
// maneuver when no forward actuator trajectory fits.
func (ad *AutopilotDriver) stationDepartureEscape(f autopilotFrame, desired float64) (float64, bool) {
	ad.departurePresteerValid = false
	active := ad.departureEscapeActive
	ad.departureEscapeActive = false
	inBay := false
	if f.world != nil && ad.pathFromID >= 0 && ad.pathFromID < len(f.world.Stops) {
		stop := f.world.Stops[ad.pathFromID]
		if stop.ContainsBus(f.bus) || stop.ContainsBusInOuterLane(f.bus, f.laneWidth) {
			inBay = true
		} else {
			forward := sim.UnitFromAngle(stop.Heading)
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			relative := f.bus.Pos.Sub(stop.RoadCenter)
			curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
			if relative.Dot(forward) >= -50 && relative.Dot(forward) <= 350 && relative.Dot(right) >= curb+2 {
				inBay = true
			}
		}
	}
	// The state label can already say cruising while the saved departure
	// prefix is still merging out of a rear berth. Its checked crawl remains
	// eligible even behind the bay center, where ContainsBus can reject yaw.
	departing := ad.State == AutopilotStateDeparting || (ad.dockDepartureRun > 0 && f.s < ad.dockDepartureRun)
	if (!f.bus.IsArticulated() && !departing && !inBay) || f.world == nil || f.speedAbs > 25 || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.roadSafetyHold || ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) {
		return 0, false
	}
	if f.bus.RoundaboutAdmissionHold != nil {
		return 0, false
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return 0, false
		}
	}
	// Projected headway can be zero beside an occupied adjacent curb lane.
	// The complete actuator/body proof below, rather than that combined
	// envelope, decides whether a slow forward departure is actually clear.
	from := f.world.Stops[ad.pathFromID]
	// A missed stop can be advanced while the bus is still moving. It may
	// then stop at the taper without having armed beginRoadDeparture's crawl.
	// Reacquire it from the actual stalled pose, including older saves.
	if !active && inBay && !from.CenterBRT && f.bus.IsArticulated() && f.speedAbs <= .1 &&
		ad.skippedDeparture != nil && !ad.skippedDeparture.served && ad.skippedDeparture.stop.ID == from.ID &&
		f.bus.Pos.Sub(from.FrontBerthPosition()).Dot(sim.UnitFromAngle(from.Heading)) > 12 &&
		math.Abs(sim.NormalizeAngle(f.bus.Heading-from.Heading)) > .5 && math.Cos(f.bus.TailHeading()-from.Heading) > .95 {
		empty := f
		empty.obstacles = nil
		active = ad.articulatedSweptSpeed(empty, desired) <= .5
	}
	// A served departure can stall the same way: the commanded steering
	// grazes the shelter with the trailer while a gentler steering, proved by
	// the same full-body scan below, would pass. Only static geometry counts.
	if !active && (departing || inBay) && !from.CenterBRT && f.bus.IsArticulated() && f.speedAbs <= .1 &&
		f.bus.Pos.Distance(from.Pos) <= from.Length+150 {
		empty := f
		empty.obstacles = nil
		active = ad.articulatedSweptSpeed(empty, desired) <= .5
	}
	missedBerth := active && ad.skippedDeparture != nil && !ad.skippedDeparture.served
	minHeadingAlignment := .7
	if missedBerth {
		// A missed berth can leave the powered section angled more sharply
		// than a normal departure. The same full-body sweep still has to pass.
		minHeadingAlignment = .5
	}
	if f.bus.Pos.Distance(from.Pos) > from.Length+150 || math.Cos(f.bus.Heading-from.Heading) < minHeadingAlignment || ad.trafficLightSpeed(f, 30) <= .5 {
		return 0, false
	}
	if ad.followingSpeed(f) <= .5 && !ad.hasParallelDepartureNeighbor(f) {
		return 0, false
	}
	// Keep the proven crawl until the full tail has passed the shelter's
	// longitudinal extent. Otherwise a nominal acceleration can invalidate the
	// low-speed proof on the very next frame while the trailer is still beside it.
	// Clearing the shelter alone is insufficient while the nose still points
	// into its taper: retain the proof until the powered section is aligned.
	if active {
		rear, _ := stationBodyRange(f.bus, sim.UnitFromAngle(from.Heading))
		if rear > from.Pos.Dot(sim.UnitFromAngle(from.Heading))+50 && math.Cos(f.bus.Heading-from.Heading) > .98 {
			return 0, false
		}
	}
	ad.departureEscapeActive = active
	rear := false
	forward := sim.UnitFromAngle(f.bus.Heading)
	for _, ob := range f.obstacles {
		if ob.Bus != nil && math.Abs(ob.Speed) < 1 && ob.Pos.Sub(f.bus.Pos).Dot(forward) < -8 && ob.Pos.Distance(f.bus.Pos) < f.bus.OverallLength()+ob.Length && math.Cos(ob.Heading-f.bus.Heading) > .7 {
			rear = true
			break
		}
	}
	if !rear && !active {
		return 0, false
	}
	var scan forwardEscapeScan
	scan.prepare(f, 18)
	// A rear-berth departure can already be beside an arriving bus with a
	// real gap smaller than the ordinary 2px prediction margin. Retain a
	// positive collision skin, checking every physical section at every tick.
	if departing && !from.CenterBRT {
		u := sim.UnitFromAngle(from.Heading)
		right := sim.Vec2{X: -u.Y, Y: u.X}
		for i := 0; i < scan.count; i++ {
			e := &scan.entries[i]
			ob := f.obstacles[e.index]
			delta := ob.Pos.Sub(f.bus.Pos)
			if ob.Bus != nil && math.Abs(ob.Speed) < 1 && math.Cos(ob.Heading-from.Heading) > .95 &&
				math.Abs(delta.Dot(u)) < (f.bus.Length+ob.Length)/2 && math.Abs(delta.Dot(right)) > (f.bus.Width+ob.Width)*.4 {
				e.padding = .05
			}
		}
	}
	f.forwardEscape = &scan
	turn := math.Copysign(1, f.bus.SteeringAngle)
	choices := []float64{desired, turn, -turn, desired * .5, 0}
	for _, steer := range choices {
		if ad.stationDepartureEscapeClear(f, steer) {
			ad.departureEscapeActive = active || !f.bus.IsArticulated()
			return steer, true
		}
	}
	// A trailer beside a shelter may forbid even the first actuator step with
	// the current wheel angle. Settle the wheels while stopped only after proving
	// the complete moving trajectory from that settled angle.
	if f.speedAbs < .5 {
		for _, steer := range []float64{desired, desired * .5, desired * .25, 0, sim.Clamp(-sim.NormalizeAngle(f.bus.Heading-from.Heading)*3, -1, 1)} {
			parked := *f.bus
			parked.SteeringAngle = steer * parked.MaxSteerAngle
			aligned := f
			aligned.bus = &parked
			if ad.stationDepartureEscapeClear(aligned, steer) {
				ad.departurePresteerValid = true
				ad.departureEscapeActive = true
				return steer, false
			}
		}
	}
	if missedBerth && f.speedAbs <= .1 && ad.roundaboutEdgeRetry <= 0 {
		yaw := sim.NormalizeAngle(f.bus.Heading - from.Heading)
		empty := f
		empty.obstacles, empty.forwardEscape = nil, nil
		ad.roundaboutEdgeRetry = 1
		if math.Abs(yaw) > .5 && !ad.stationDepartureEscapeClear(empty, desired) {
			ad.planEdgeRecovery(f, math.Copysign(1, yaw))
		}
	}

	return 0, false
}
func (ad *AutopilotDriver) stationDepartureEscapeClear(f autopilotFrame, steer float64) bool {
	return ad.stationForwardEscapeClear(f, steer, 18, ad.pointAt(f.s+100), 10)
}

func (ad *AutopilotDriver) stationForwardEscapeClear(f autopilotFrame, steer, speed float64, goal sim.Vec2, minimumProgress float64) bool {
	if trace.IsEnabled() {
		defer trace.StartRegion(context.Background(), "autopilot.stationForwardEscapeClear").End()
	}
	var local forwardEscapeScan
	if f.forwardEscape == nil {
		local.prepare(f, speed)
		f.forwardEscape = &local
	}
	b := *f.bus
	startDistance := b.Pos.Distance(goal)
	if !f.forwardEscape.poseClear(f, &b, 0) {
		return false
	}
	for tick := 1; tick <= forwardEscapeTicks; tick++ {
		throttle, brake := ad.longitudinalControls(b.Speed, speed)
		b.SetNextAccelerationScale(1)
		b.Update(1./60, throttle, brake, steer, false, false)
		if !f.forwardEscape.poseClear(f, &b, float64(tick)/60) {
			return false
		}
	}
	return startDistance-b.Pos.Distance(goal) > minimumProgress
}
func departureEscapePoseClear(f autopilotFrame, b *sim.Bus, elapsed float64) bool {
	if b.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier || math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
		return false
	}
	for _, ob := range f.obstacles {
		if ob.Pos.Distance(b.Pos) > b.OverallLength()+ob.Length+math.Abs(ob.Speed)*elapsed+40 {
			continue
		}
		for _, other := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
			if other.HalfLen <= 0 {
				continue
			}
			padding := 2.
			// Only a stopped rear bus may use the tight recovery skin.
			// All other traffic retains its normal extra clearance.
			if ob.Bus != nil && math.Abs(ob.Speed) < 1 && ob.Pos.Sub(f.bus.Pos).Dot(sim.UnitFromAngle(f.bus.Heading)) < -8 {
				padding = .05
			}
			other.HalfLen += padding
			other.HalfWid += padding
			if hit, _, _ := b.CollidesBody(other); hit {
				return false
			}
		}
	}
	return true
}

// Keep the old cheap stop for ordinary same-lane queues. Only a stopped,
// laterally separated front bus may ask the full actuator proof to resolve a
// conservative envelope headway. This filter grants no collision exemption.
func (ad *AutopilotDriver) hasParallelDepartureNeighbor(f autopilotFrame) bool {
	if ad.State != AutopilotStateDeparting {
		return false
	}
	u := sim.UnitFromAngle(f.bus.Heading)
	parallel := false
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(f.bus.Pos)
		ahead := delta.Dot(u)
		if ahead <= 0 || ahead > (f.bus.Length+ob.Length)/2+80 {
			continue
		}
		lateral := math.Abs(delta.X*u.Y - delta.Y*u.X)
		if lateral < (f.bus.Width+ob.Width)/2+1 {
			return false
		}
		if ob.Bus != nil && math.Abs(ob.Speed) < 1 && math.Cos(ob.Heading-f.bus.Heading) > .98 {
			parallel = true
		}
	}
	return parallel
}
