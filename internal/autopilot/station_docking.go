package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type stationDockPlan struct {
	stopID  int
	waiting bool
	rank    int // bay, curb lane, then successively farther lanes
	target  sim.Vec2
	points  []sim.Vec2
}

type stationDockInputs struct {
	throttle, brake, steer float64
	toggleDoors, isDocked  bool
}

// Plan in stop coordinates so lane preference is independent of overtaking
// offsets and of the tapered route's changing distance from the road center.
func (ad *AutopilotDriver) stationDockControls(f autopilotFrame) (stationDockInputs, bool) {
	stop := f.targetStop
	if stop == nil {
		return stationDockInputs{}, false
	}
	if (!stop.HasTwoBerths() && !ad.BerserkMode && !ad.ordinaryBusLeft) || ad.CanSkipTargetStop || ad.State == AutopilotStateDeparting || stop == nil {
		return stationDockInputs{}, false
	}
	// Keep high-speed long vehicles on their calibrated arrival taper until
	// they have slowed enough to follow a local berth maneuver safely.
	if f.bus.IsArticulated() && f.speedAbs > 130 {
		return stationDockInputs{}, false
	}
	// Berserk long buses already alongside the boarding platform need not
	// merge into an occupied bay or chase its forward marker. Serve from the
	// actual stationary pose only when both sections fit and remain separated.
	if ad.BerserkMode && f.bus.IsArticulated() && f.speedAbs < 8 && stop.ContainsBusInOuterLane(f.bus, f.laneWidth) {
		clear := f.world == nil || f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneHardBarrier
		for _, ob := range f.obstacles {
			if ob.Pos.Distance(f.bus.Pos) > f.bus.OverallLength()+ob.Length+10 {
				continue
			}
			if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 1, 1) {
				clear = false
				break
			}
		}
		if clear {
			rank := 1
			if stop.ContainsBus(f.bus) {
				rank = 0 // An actual bay arrival still needs its safe bay-exit taper.
			}
			ad.stationDock = &stationDockPlan{stopID: stop.ID, rank: rank, target: f.bus.Pos}
			ad.beginDocking(stop)
			return stationDockInputs{brake: 1, toggleDoors: !f.bus.DoorsOpen && f.bus.DoorProgress <= .05, isDocked: true}, true
		}
	}
	// Complete an already-safe arrival before trying to generate another local
	// curve. A few pixels of residual lateral error can make a tiny cubic turn
	// its tangent sharply enough to fail the trailer constraint while stopped.
	if f.speedAbs < 12 {
		for _, berth := range availableStationBerths(f.bus, stop, f.obstacles) {
			if atSelectedStationBerth(f.bus, berth, stop.Heading) {
				ad.stationDock = &stationDockPlan{stopID: stop.ID, target: berth}
				ad.beginDocking(stop)
				return stationDockInputs{brake: 1, toggleDoors: !f.bus.DoorsOpen && f.bus.DoorProgress <= .05, isDocked: true}, true
			}
		}
	}
	nearbyTraffic := false
	for _, ob := range f.obstacles {
		if ob.Pos.Distance(stop.Pos) < 300 {
			nearbyTraffic = true
			break
		}
	}
	cachedFrontApproach := len(ad.drivePath) > 0 && ad.drivePath[len(ad.drivePath)-1].Distance(stop.FrontBerthPosition()) < 10
	// Long vehicles need the local alignment maneuver even on a free bay:
	// pursuit of the route taper can unwind too late, then accelerate while
	// the front corner is still sweeping toward the shelter.
	if !f.bus.IsArticulated() && ad.stationDock == nil && math.Abs(ad.LaneOffset) < 1 && !nearbyTraffic && !ad.ordinaryBusLeft && cachedFrontApproach {
		return stationDockInputs{}, false
	}
	forward := sim.UnitFromAngle(stop.Heading)
	distance := stop.FrontBerthPosition().Sub(f.bus.Pos).Dot(forward)
	approachDistance := 360.
	if _, occupied := stationQueueTarget(f.bus, stop, f.obstacles); occupied && len(availableStationBerths(f.bus, stop, f.obstacles)) == 0 {
		approachDistance = 650
	}
	if (ad.stationDock == nil || ad.stationDock.stopID != stop.ID) && (f.remaining > math.Max(400, approachDistance) || distance > approachDistance || distance < -20 || math.Cos(f.bus.Heading-stop.Heading) < 0.85) {
		return stationDockInputs{}, false
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	plan := ad.stationDock
	// Preserve the entry curve while stopped; rebuild only as the lead moves
	// or a passenger berth becomes free. Rebuilding a tiny remaining curve
	// midway through a lane change can strand the trailer.
	if plan != nil && plan.waiting {
		target, occupied := stationQueueTarget(f.bus, stop, f.obstacles)
		finishingEntry := f.bus.IsArticulated() && (math.Abs(plan.target.Sub(f.bus.Pos).Dot(right)) > 10 || math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) > .15)
		if !finishingEntry && (!occupied || target.Distance(plan.target) > 8 || len(availableStationBerths(f.bus, stop, f.obstacles)) > 0) {
			plan = nil
		}
	}
	// A rear-berth or queue-staging curve can end before a turning bus has fully
	// straightened. Do not hold a zero remaining distance forever: if the
	// forward berth has opened, use its runway to finish the alignment. The
	// replacement curve still has to clear the complete vehicle and traffic.
	if plan != nil && f.speedAbs < 12 && plan.target.Sub(f.bus.Pos).Dot(forward) < 10 && math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) >= .15 {
		for _, berth := range availableStationBerths(f.bus, stop, f.obstacles) {
			if berth.Sub(plan.target).Dot(forward) > f.bus.Length/2 {
				plan = nil
				break
			}
		}
	}
	// Reconsider nearer lanes while there is enough runway. Once braking beside
	// the stop, hold the chosen lane through the door cycle instead of swerving.
	if plan == nil || plan.stopID != stop.ID || (!plan.waiting && distance > 140 && (!f.bus.IsArticulated() || plan.rank > 0)) || !stationDockCurveClear(f.bus, remainingStationCurve(plan.points, entryPosition(plan.points, f.bus.Pos, forward)), f.obstacles) {
		candidates := availableStationBerths(f.bus, stop, f.obstacles)
		bayCount := len(candidates)
		waiting := false
		if bayCount == 0 {
			if target, ok := stationQueueTarget(f.bus, stop, f.obstacles); ok {
				candidates = append(candidates, target)
				bayCount = 1
				waiting = true
			}
		}
		count := stop.RoadLanes / 2
		if count < 1 {
			count = 1
		}
		outerReady := !waiting || (f.world != nil && distance <= 360 && math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) < .15)
		for lane := 0; ad.BerserkMode && outerReady && lane < count; lane++ {
			front := stop.FrontBerthPosition().Sub(stop.Pos).Dot(forward)
			laneCenter := stop.RoadCenter.Add(right.Mul((float64(count-lane) - 0.5) * f.laneWidth))
			candidates = append(candidates, laneCenter.Add(forward.Mul(front)))
			if f.bus.IsArticulated() {
				// If the front of this lane is occupied, an earlier position
				// alongside the long platform can still serve both sections.
				for _, along := range []float64{front - 48, front - 96} {
					pose := f.bus.AtPose(laneCenter.Add(forward.Mul(along)), stop.Heading)
					if stop.ContainsBusInOuterLane(&pose, f.laneWidth) {
						candidates = append(candidates, pose.Pos)
					}
				}
			}
		}
		if ad.BerserkMode && !waiting && stop.RoadLanes <= 2 {
			candidates = append(candidates, stop.RoadCenter.Add(forward.Mul(stop.FrontBerthPosition().Sub(stop.Pos).Dot(forward))).Sub(right.Mul(f.laneWidth/2)))
		}
		best := (*stationDockPlan)(nil)
		for rank, target := range candidates {
			laneRank := 0
			if rank >= bayCount {
				laneRank = rank - bayCount + 1
			}
			if laneRank > 0 && f.bus.IsArticulated() && !outerBerthExitRoom(f.bus, target, stop.Heading, f.obstacles) {
				continue
			}
			lateral := math.Abs(target.Sub(f.bus.Pos).Dot(right))
			// Keep only the final braking/alignment margin; reserving 60px
			// here rejects every berth after a bus has already begun pulling in.
			if lateral > math.Max(10, (distance-20)*0.65) {
				continue
			}
			// A trailer needs a straight settling run after moving sideways.
			// If already too close, wait for the front bus to create that room.
			if waiting && rank == 0 && f.bus.IsArticulated() && lateral > 10 && target.Sub(f.bus.Pos).Dot(forward) < 230 {
				continue
			}
			points := stationDockCurve(f.bus, target, forward)
			if f.bus.IsArticulated() && waiting {
				// Use all of the available runway for the side-step; the
				// short arrival curve turns too sharply beside the shelter.
				points = gentleDockDeparture(f.bus.Pos, f.bus.Pos, target, forward)
			}
			if !stationDockCurveClear(f.bus, points, f.obstacles) {
				continue
			}
			best = &stationDockPlan{stopID: stop.ID, rank: laneRank, target: target, points: points, waiting: waiting && rank == 0}
			break
		}
		plan = best
		ad.stationDock = plan
	}
	if plan == nil {
		// An unreachable queue curve in the extended early-approach region
		// must not freeze an unfinished road turn. Inside the normal 360px
		// approach, a full platform still holds a late long bus until the lead
		// creates safe entry runway. Do not interrupt a long bus still turning
		// onto this road; its normal physical sweeps remain authoritative.
		turning := f.bus.IsArticulated() && (math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) >= .15 || math.Abs(ad.LaneOffset) > 5 || math.Abs(ad.TargetLaneOffset) > 5)
		if stop.HasTwoBerths() && distance > 50 && (distance > 360 || turning || len(availableStationBerths(f.bus, stop, f.obstacles)) > 0) {
			return stationDockInputs{}, false
		}
		return stationDockInputs{brake: 1}, true
	}
	ad.LaneOffset, ad.TargetLaneOffset = 0, 0
	ad.OvertakeState = OvertakeNone
	ad.LaneOffsetProgress = 1
	ad.lastTargetLaneOffset = 0
	remaining := math.Max(0, plan.target.Sub(f.bus.Pos).Dot(forward))
	lateral := math.Abs(plan.target.Sub(f.bus.Pos).Dot(right))
	aligned := math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) < 0.15
	if !plan.waiting && remaining < 10 && lateral < 10 && aligned && f.speedAbs < 12 {
		ad.beginDocking(stop)
		return stationDockInputs{brake: 1, toggleDoors: !f.bus.DoorsOpen && f.bus.DoorProgress <= 0.05, isDocked: true}, true
	}
	if f.bus.DoorsOpen || f.bus.DoorProgress > 0.05 {
		return stationDockInputs{brake: 1, toggleDoors: f.bus.DoorsOpen && f.speedAbs < 15}, true
	}
	// Look forward beyond the endpoint to finish parallel and never chase the
	// station's bay across several lanes once the selected lane has been reached.
	s := entryPosition(plan.points, f.bus.Pos, forward)
	lookahead := sim.Clamp(f.speedAbs*0.35+30, 35, 65)
	target := entryPointAt(plan.points, forward, s+lookahead)
	if s+lookahead >= stationCurveLength(plan.points) {
		target = plan.target.Add(forward.Mul(lookahead))
	}
	delta := target.Sub(f.bus.Pos)
	alpha := sim.NormalizeAngle(math.Atan2(delta.Y, delta.X) - f.bus.Heading)
	steer := sim.Clamp(math.Atan2(2*f.bus.WheelBase*math.Sin(alpha), math.Max(20, delta.Length()))/f.bus.MaxSteerAngle, -1, 1)
	speed := math.Min(115, math.Sqrt(2*165*math.Max(0, remaining-3)))
	if plan.waiting {
		speed = math.Min(speed, 70)
	}
	if f.bus.IsArticulated() {
		steer = ad.roadLaneSafeSteering(f, steer)
		speed = math.Min(speed, ad.controlSweptSpeed(f, steer))
		speed = math.Min(speed, ad.articulatedSweptSpeed(f, steer, true))
		if ad.roadSafetyHold {
			speed = 0
		}
	}
	speed = ad.trafficLightSpeed(f, speed)
	// Recheck the swept path when traffic changes after selecting a lane.
	if !stationDockCurveClear(f.bus, remainingStationCurve(plan.points, s), f.obstacles) {
		speed = 0
	}
	throttle, brake := ad.longitudinalControls(f.bus.Speed, speed)
	ad.LastRemaining = remaining
	ad.LastTargetSpeed = speed
	ad.LastPursuitPt = target
	return stationDockInputs{throttle: throttle, brake: brake, steer: steer}, true
}

func stationDockCurve(bus *sim.Bus, target, forward sim.Vec2) []sim.Vec2 {
	distance := math.Max(0, target.Sub(bus.Pos).Dot(forward))
	straight := math.Min(200, distance*0.5)
	end := target.Sub(forward.Mul(straight))
	handle := math.Min(100, distance*0.35)
	heading := sim.UnitFromAngle(bus.Heading)
	a, b := bus.Pos.Add(heading.Mul(handle)), end.Sub(forward.Mul(handle))
	points := make([]sim.Vec2, 0, 34)
	for i := 0; i <= 32; i++ {
		t := float64(i) / 32
		u := 1 - t
		points = append(points, bus.Pos.Mul(u*u*u).Add(a.Mul(3*u*u*t)).Add(b.Mul(3*u*t*t)).Add(end.Mul(t*t*t)))
	}
	for along := 5.0; along < straight; along += 5 {
		points = append(points, end.Add(forward.Mul(along)))
	}
	return append(points, target)
}

func stationCurveLength(points []sim.Vec2) float64 {
	length := 0.0
	for i := 1; i < len(points); i++ {
		length += points[i].Distance(points[i-1])
	}
	return length
}

func stationDockCurveClear(bus *sim.Bus, points []sim.Vec2, obstacles []VehicleObstacle) bool {
	travel := 0.0
	predicted := *bus
	for i := 0; i < len(points); i++ {
		delta := sim.UnitFromAngle(bus.Heading)
		if i+1 < len(points) {
			delta = points[i+1].Sub(points[i])
		} else if i > 0 {
			delta = points[i].Sub(points[i-1])
		}
		heading := math.Atan2(delta.Y, delta.X)
		if delta.Length() < 0.01 {
			heading = bus.Heading
		}
		predicted = predicted.AtPose(points[i], heading)
		if predicted.Pos.Distance(points[i]) > .1 {
			return false
		}
		parts := predicted.BodyParts()
		time := travel / math.Max(40, math.Abs(bus.Speed))
		for _, ob := range obstacles {
			reach := bus.OverallLength() + ob.Length/2 + math.Abs(ob.Speed)*time + 16
			offset := ob.Pos.Sub(points[i])
			if offset.Dot(offset) > reach*reach {
				continue
			}
			other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 4, HalfWid: ob.Width/2 + 2}
			// Compare vehicles at the same future instant. Testing every curve
			// point against a moving car's current position freezes its old
			// footprint across the entire approach and creates a false stop.
			other.Center = other.Center.Add(sim.UnitFromAngle(ob.Heading).Mul(ob.Speed * time))
			paddingLen, paddingWidth := 5., 2.
			if stationRearBodyClear(bus, ob) || stationAlongsideBodyClear(bus, ob) {
				paddingLen, paddingWidth = .5, .5
				other.HalfLen = ob.Length/2 + .5
				other.HalfWid = ob.Width/2 + .5
			}
			for _, part := range parts {
				if part.HalfLen <= 0 {
					continue
				}
				part.HalfLen += paddingLen
				part.HalfWid += paddingWidth
				if hit, _, _ := sim.CheckOBBCollision(part, other); hit {
					return false
				}
			}
		}
		travel += delta.Length()
	}
	return true
}

func (ad *AutopilotDriver) beginDocking(stop *sim.BusStop) {
	ad.State = AutopilotStateDocked
	ad.DockedStopID = stop.ID
	ad.DwellTimer = ad.DwellTotal
	if ad.BerserkMode {
		ad.DwellTimer = 1
	}
	ad.DwellFinished = false
}

func remainingStationCurve(points []sim.Vec2, s float64) []sim.Vec2 {
	remaining := []sim.Vec2{}
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += points[i].Distance(points[i-1])
		if total > s+2 {
			remaining = append(remaining, points[i])
		}
	}
	return remaining
}

// Only a stationary, parallel follower whose actual body is currently clear can
// release the normal queue padding. Alongside, moving and real overlaps retain
// the normal docking reservations and must wait for a safe gap.
func stationRearBodyClear(bus *sim.Bus, ob VehicleObstacle) bool {
	if math.Abs(ob.Speed) >= 3 || math.Cos(bus.Heading-ob.Heading) < .9 {
		return false
	}
	forward := sim.UnitFromAngle(ob.Heading)
	if bus.Pos.Sub(ob.Pos).Dot(forward) < (bus.Length+ob.Length)*.4 {
		return false
	}
	a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + .5, HalfWid: bus.Width/2 + .5}
	b := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + .5, HalfWid: ob.Width/2 + .5}
	hit, _, _ := sim.CheckBusAtBody(bus, a, b)
	return !hit
}

// A stationary bus beside the bay must not reserve extra queue padding into
// an already-separated leader's straight arrival. Only the longitudinal
// leader gets this tighter clearance; its full remaining curve and actuator
// sweep still check physical contact. The follower retains normal margins.
func stationAlongsideBodyClear(bus *sim.Bus, ob VehicleObstacle) bool {
	if ob.Bus == nil || math.Abs(ob.Speed) >= 3 || math.Cos(bus.Heading-ob.Heading) < .98 {
		return false
	}
	// Use the same axis and front-section references in both directions, so
	// two angled or articulated buses cannot both consider themselves ahead.
	h := math.Atan2(math.Sin(bus.Heading)+math.Sin(ob.Bus.Heading), math.Cos(bus.Heading)+math.Cos(ob.Bus.Heading))
	u := sim.UnitFromAngle(h)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	d := bus.Pos.Sub(ob.Bus.Pos)
	if d.Dot(u) < 2 || math.Abs(d.Dot(r)) < (bus.Width+ob.Bus.Width)/2+2 {
		return false
	}
	a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + .5, HalfWid: bus.Width/2 + .5}
	return !obstacleSweepCollision(bus, a, ob, 0, .5, .5)
}
