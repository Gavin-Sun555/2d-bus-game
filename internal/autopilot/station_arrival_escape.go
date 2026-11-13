package autopilot

import (
	"context"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"runtime/trace"
)

// Two arrivals can wait on each other's lateral sweep before a free bay.
// Try a short forward actuator trajectory, rather than reserving the same
// impossible nominal steering forever. Every section/obstacle is still checked.
func (ad *AutopilotDriver) stationArrivalEscape(f autopilotFrame, desired float64) (float64, bool) {
	if trace.IsEnabled() {
		defer trace.StartRegion(context.Background(), "autopilot.stationArrivalEscape").End()
	}
	stop := f.targetStop
	if stop == nil || f.world == nil || ad.arrivalEscapeRetry > 0 || f.speedAbs > 25 || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.roadSafetyHold || ad.CanSkipTargetStop || ad.State == AutopilotStateDocked || ad.State == AutopilotStateDeparting || f.bus.RoundaboutAdmissionHold != nil {
		return 0, false
	}
	if f.bus.Pos.Distance(stop.Pos) > stop.Length+150 || math.Cos(f.bus.Heading-stop.Heading) < .7 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) || ad.trafficLightSpeed(f, 30) <= .5 {
		return 0, false
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return 0, false
		}
	}
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	ownLateral := math.Abs(f.bus.Pos.Sub(stop.Pos).Dot(r))
	frontDistance := stop.FrontBerthPosition().Sub(f.bus.Pos).Dot(u)
	// A berserk bus already in its selected parallel lane does not need a
	// platform slot. Occupied bay padding must not veto its checked crawl.
	parallel := false
	parallelTarget := sim.Vec2{}
	if ad.BerserkMode && !stop.CenterBRT {
		if ad.stationDock != nil && ad.stationDock.rank > 0 && !ad.stationDock.waiting {
			parallelTarget = ad.stationDock.target
			parallel = math.Abs(parallelTarget.Sub(f.bus.Pos).Dot(r)) < f.laneWidth*.25
		} else if ad.stationDock == nil {
			// A padded full-length curve may not fit yet beside occupied bays.
			// Keep the actual road lane for the short, physically checked entry.
			side := f.bus.Pos.Sub(stop.RoadCenter).Dot(r)
			lane := int(math.Round(side/f.laneWidth - .5))
			if lane >= 0 && lane < stop.RoadLanes/2 && math.Abs(side-(float64(lane)+.5)*f.laneWidth) < f.laneWidth*.25 {
				parallel = true
				parallelTarget = stop.FrontBerthPosition().Add(r.Mul((float64(lane)+.5)*f.laneWidth - stop.Pos.Sub(stop.RoadCenter).Dot(r)))
			}
		}
	}
	// A vehicle still crossing a traffic lane keeps its ordinary headway.
	// Only an already aligned bay arrival (or final front-edge correction)
	// may replace the combined-envelope cap with the full actuator proof.
	if !parallel && frontDistance >= 60 && (ownLateral > stop.Width || math.Cos(f.bus.Heading-stop.Heading) < .97) && (ad.followingSpeed(f) <= .5 || ad.obstacleSpeed(f, 30) <= .5) {
		return 0, false
	}
	if stop.FrontBerthPosition().Sub(f.bus.Pos).Dot(u) < -32 {
		return 0, false
	}
	neighbor := false
	for _, ob := range f.obstacles {
		if !parallel && ob.Bus != nil && ob.Bus.DoorsOpen && stop.OccupiesBerth(ob.Bus.BodyEnvelope()) {
			return 0, false // A full platform still needs real entry runway.
		}

		d := ob.Pos.Sub(f.bus.Pos)
		if ob.Bus != nil && math.Abs(ob.Speed) < 1 && math.Cos(ob.Heading-stop.Heading) > .5 && math.Abs(d.Dot(u)) < f.bus.OverallLength()+ob.Length && math.Abs(d.X*u.Y-d.Y*u.X) > (f.bus.Width+ob.Width)*.4 {
			neighbor = true
			break
		}
	}
	// A stale lane offset can leave a free approach aimed into the curb.
	// The normal turning sweep may then veto every restart even after the
	// offset is cleared. Use the same full-actuator crawl proof for this purely
	// geometrical hold; traffic and actual hard barriers remain authoritative.
	geometryHold := false
	if !neighbor && f.bus.IsArticulated() && f.speedAbs <= 25 && math.Cos(f.bus.Heading-stop.Heading) > .97 && f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneHardBarrier {
		empty := f
		empty.obstacles = nil
		geometryHold = ad.articulatedSweptSpeed(empty, desired) <= .5
	}
	if !neighbor && !geometryHold && !(stop.FrontBerthPosition().Sub(f.bus.Pos).Dot(u) < 60 && ownLateral < stop.Width+f.bus.Width) {
		return 0, false
	}
	// Select a conservative neighborhood once for all trial trajectories.
	// At <=25px/s the powered section travels <100px in the two-second proof.
	var nearby [128]VehicleObstacle
	n := 0
	for _, ob := range f.obstacles {
		reach := f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*2 + 100
		d := ob.Pos.Sub(f.bus.Pos)
		if d.Dot(d) > reach*reach {
			continue
		}
		if n == len(nearby) {
			n = -1 // Dense local scenes retain the complete obstacle set.
			break
		}
		nearby[n] = ob
		n++
	}
	if n >= 0 {
		f.obstacles = nearby[:n]
	}
	goal, speed, progress := ad.pointAt(f.s+100), 18., 10.
	if parallel {
		goal = parallelTarget
		delta := goal.Sub(f.bus.Pos)
		alpha := sim.NormalizeAngle(math.Atan2(delta.Y, delta.X) - f.bus.Heading)
		desired = sim.Clamp(math.Atan2(2*f.bus.WheelBase*math.Sin(alpha), math.Max(20, delta.Length()))/f.bus.MaxSteerAngle, -1, 1)
	}
	var boundaries *forwardEscapeBoundaries
	useBoundaries := func(scan *forwardEscapeScan) {
		if boundaries == nil {
			scratch := ad.geometryScratch()
			if scratch.forwardEscapeBoundaries == nil {
				scratch.forwardEscapeBoundaries = new(forwardEscapeBoundaries)
			}
			boundaries = scratch.forwardEscapeBoundaries
			boundaries.reset(f)
		}
		scan.boundaries = boundaries
	}
	// Two closed-door arrivals can be physically separate while their
	// extra margins overlap. First try a short forward move along the
	// bus's own lane or bay, without starting another lateral merge.
	if neighbor && !parallel {
		for _, ob := range f.obstacles {
			if ob.Bus == nil || ob.Bus.DoorsOpen || math.Abs(ob.Speed) >= 1 {
				continue
			}
			request := ob.Bus.StationMergeRequest
			sameArrival := request != nil && request.StopID == stop.ID && !request.Departing
			if !sameArrival && !stop.OccupiesBerth(ob.Bus.BodyEnvelope()) {
				continue
			}
			d := ob.Bus.Pos.Sub(f.bus.Pos)
			// Center offsets only identify the alongside case. The swept
			// bodies below decide whether the angled corners actually clear.
			if math.Abs(d.Dot(u)) >= (f.bus.Length+ob.Bus.Length)/2 || math.Abs(d.Dot(r)) < (f.bus.Width+ob.Bus.Width)*.4 {
				continue
			}
			var scan forwardEscapeScan
			crawl := stationArrivalCrawlSpeed(f)
			scan.prepare(f, crawl)
			useBoundaries(&scan)
			for j := 0; j < scan.count; j++ {
				e := &scan.entries[j]
				if f.obstacles[e.index].Bus == ob.Bus {
					e.padding = 0
				}
			}
			probe := f
			probe.forwardEscape = &scan
			goal := f.bus.Pos.Add(u.Mul(100))
			for _, steer := range []float64{sim.Clamp(-3*sim.NormalizeAngle(f.bus.Heading-stop.Heading), -1, 1), 0, -.15, .15, -.35, .35} {
				if ad.stationForwardEscapeClear(probe, steer, crawl, goal, crawl*.4) {
					return steer, true
				}
			}
		}
	}
	if stationArrivalCrawlSpeed(f) < 18 {
		speed, progress = 8, 3
		goal = stop.FrontBerthPosition().Add(u.Mul(20))
		delta := goal.Sub(f.bus.Pos)
		alpha := sim.NormalizeAngle(math.Atan2(delta.Y, delta.X) - f.bus.Heading)
		desired = sim.Clamp(math.Atan2(2*f.bus.WheelBase*math.Sin(alpha), math.Max(20, delta.Length()))/f.bus.MaxSteerAngle, -1, 1)
	}
	ad.arrivalEscapeRetry = .2
	choices := [...]float64{desired, desired + .1, desired - .1, desired + .2, desired - .2, desired + .3, desired - .3, 0, .05, -.05, .1, -.1, .2, -.2, .3, -.3, .5, -.5, 1, -1}
	var scan forwardEscapeScan
	if f.forwardEscape == nil {
		scan.prepare(f, speed)
		useBoundaries(&scan)
		f.forwardEscape = &scan
	}
	try := func(choice float64) (float64, bool) {
		steer := sim.Clamp(choice, -1, 1)
		return steer, ad.stationForwardEscapeClear(f, steer, speed, goal, progress)
	}
	if geometryHold {
		for _, choice := range [...]float64{sim.Clamp(desired, -.1, .1), sim.Clamp(desired, -.05, .05), 0} {
			if steer, ok := try(choice); ok {
				ad.arrivalEscapeRetry = 0
				return steer, true
			}
		}
	}
	for _, choice := range choices {
		if steer, ok := try(choice); ok {
			ad.arrivalEscapeRetry = 0
			return steer, true
		}
	}
	return 0, false
}

// The leading bus already fits a valid front platform pose. A closed-door
// follower alongside its bent tail must not reserve the leader's boarding work.
// A bus genuinely occupying a berth with open doors still reserves the platform.
func stationFrontArrivalClear(f autopilotFrame) bool {
	u := sim.UnitFromAngle(f.targetStop.Heading)
	_, front := stationBodyRange(f.bus, u)
	for _, ob := range f.obstacles {
		padding := .5
		if ob.Bus == nil && math.Cos(ob.Heading-f.targetStop.Heading) > .9 {
			d := ob.Pos.Sub(f.bus.Pos)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			if d.Dot(u) > 0 && math.Abs(d.Dot(r)) < (f.bus.Width+ob.Width)/2+8 && d.Dot(u)-(f.bus.Length+ob.Length)/2 < 35 {
				return false
			}
		}
		if ob.Bus != nil {
			_, otherFront := stationBodyRange(ob.Bus, u)
			if f.targetStop.OccupiesBerth(ob.Bus.BodyEnvelope()) && (ob.Bus.DoorsOpen || otherFront > front-12) {
				return false
			}
			if stationClosedRearNeighbor(f, ob) {
				padding = .05
			}
		}
		if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, padding, padding) {
			return false
		}
	}
	return f.world == nil || f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneHardBarrier
}

func stationClosedRearNeighbor(f autopilotFrame, ob VehicleObstacle) bool {
	if ob.Bus == nil || ob.Bus.DoorsOpen || math.Abs(ob.Speed) >= 1 || math.Cos(ob.Heading-f.targetStop.Heading) < .9 {
		return false
	}
	u := sim.UnitFromAngle(f.targetStop.Heading)
	_, otherFront := stationBodyRange(ob.Bus, u)
	powered := f.bus.BodyParts()[0]
	yaw := powered.Heading - f.targetStop.Heading
	rear := powered.Center.Dot(u) - powered.HalfLen*math.Abs(math.Cos(yaw)) - powered.HalfWid*math.Abs(math.Sin(yaw))
	return otherFront+12 < rear
}

func stationArrivalCrawlSpeed(f autopilotFrame) float64 {
	if f.targetStop.FrontBerthPosition().Sub(f.bus.Pos).Dot(sim.UnitFromAngle(f.targetStop.Heading)) < 60 {
		return 8
	}
	return 18
}
