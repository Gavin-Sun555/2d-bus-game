package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// StopDiagnosis is an on-demand snapshot, never a second source of controls.
// Negative limits mean unconstrained (JSON does not support infinity).
type StopDiagnosis struct {
	State                            string
	From, To                         int
	Waypoint                         int
	PathS, Remaining, TargetSpeed    float64
	LaneOffset, TargetLaneOffset     float64
	RoadSafetyHold, RightTurnWaiting bool
	TurnHoldDistance                 float64
	Limits                           map[string]float64
	Blockers                         []StopBlocker
	Path                             []sim.Vec2
}
type StopBlocker struct {
	Position                      sim.Vec2
	Heading, Speed, Length, Width float64
	Limits                        map[string]float64
}

func (ad *AutopilotDriver) DiagnoseStop(bus *sim.Bus, route *sim.Route, world *sim.World, tls *sim.TrafficLightSystem, obstacles []VehicleObstacle, lw float64) StopDiagnosis {
	d := StopDiagnosis{State: ad.State.String(), From: ad.pathFromID, To: ad.pathToID, Waypoint: ad.WaypointIdx, PathS: ad.LastPathS, Remaining: ad.LastRemaining, TargetSpeed: ad.LastTargetSpeed, LaneOffset: ad.LaneOffset, TargetLaneOffset: ad.TargetLaneOffset, RoadSafetyHold: ad.roadSafetyHold, RightTurnWaiting: ad.rightTurnWaiting, TurnHoldDistance: ad.busTurnHoldDistance, Path: append([]sim.Vec2(nil), ad.drivePath...)}
	if bus == nil || route == nil || world == nil || len(ad.drivePath) < 2 {
		return d
	}
	target := route.GetCurrentTargetStopID()
	if target < 0 || target >= len(world.Stops) {
		return d
	}
	// Some geometry helpers cache a selected entry on the driver. Isolate those
	// assignments from the live controller while inspecting its speed constraints.
	copyDriver := *ad
	f := autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: lw, s: ad.LastPathS, speedAbs: math.Abs(bus.Speed), remaining: ad.LastRemaining, targetStop: world.Stops[target], distToStop: bus.Pos.Distance(world.Stops[target].Pos)}
	limits := func(frame autopilotFrame) map[string]float64 {
		probe := copyDriver
		busCopy := *frame.bus
		frame.bus = &busCopy
		steer, _, _, _ := probe.pursuitSteering(frame)
		m := map[string]float64{"signal": probe.trafficLightSpeed(frame, 150), "obstacles": probe.obstacleSpeed(frame, 150), "junction_admission": probe.junctionAdmissionSpeed(frame), "junction_sweep": probe.junctionSweptSpeed(frame)}
		m["roundabout_entry"] = probe.roundaboutEntryObstacleSpeed(frame)
		m["station_departure"] = probe.stationDepartureTrafficSpeed(frame, 150)
		m["following"] = probe.followingSpeed(frame)
		m["control_sweep"] = probe.controlSweptSpeed(frame, steer)
		m["articulated_sweep"] = probe.articulatedSweptSpeed(frame, steer)
		m["roundabout_admission"] = math.Inf(1)
		if hold := frame.bus.RoundaboutAdmissionHold; hold != nil {
			gap := hold.Center.Sub(frame.bus.Pos).Dot(hold.Forward) - hold.StopDistance
			m["roundabout_admission"] = math.Sqrt(2 * 180 * math.Max(0, gap))
		}
		m["required_merge"] = math.Inf(1)
		m["merge_clearance"] = math.Inf(1)
		if probe.approachMerge != nil {
			m["required_merge"] = probe.requiredMergeTrafficSpeed(frame)
			r := sim.Vec2{X: -math.Sin(probe.approachMerge.Heading), Y: math.Cos(probe.approachMerge.Heading)}
			if !requiredMergeLaneClear(frame, probe.approachMerge.Heading, 0, probe.approachMerge.Target.Sub(bus.Pos).Dot(r)) {
				m["merge_clearance"] = 0
			}
		}
		m["turn_merge"] = math.Inf(1)
		m["turn_lane_clearance"] = math.Inf(1)
		if probe.busLaneJunction != nil && !probe.busJunctionEntered {
			heading := approachRoadHeading(frame.bus.Heading)
			request := &sim.BusLaneMergeRequest{Target: probe.pointAt(frame.s), Heading: heading, LeftTurn: true}
			u := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -u.Y, Y: u.X}
			actual := frame.bus.Pos.Sub(request.Target).Dot(right)
			if !requiredMergeLaneClear(frame, heading, actual, 0) {
				m["turn_lane_clearance"] = 0
			}
			for _, ob := range frame.obstacles {
				if probe.ordinaryBusLeft && probe.clearsThroughQueueDuringMerge(frame, ob) {
					continue
				}
				one := frame
				one.obstacles = []VehicleObstacle{ob}
				m["turn_merge"] = math.Min(m["turn_merge"], mergeTrafficSpeed(one, request))
			}
		}
		for k, v := range m {
			if math.IsInf(v, 0) || math.IsNaN(v) {
				m[k] = -1
			}
		}
		return m
	}
	d.Limits = limits(f)
	d.Limits["station"] = copyDriver.stationSpeed(f, 150)
	d.Limits["turn_hold"] = -1
	if ad.busTurnHoldDistance > 0 {
		d.Limits["turn_hold"] = math.Sqrt(2 * 135 * math.Max(0, ad.busTurnHoldDistance-10))
	}
	for _, ob := range obstacles {
		if ob.Pos.Distance(bus.Pos) > 1600 {
			continue
		}
		one := f
		one.obstacles = []VehicleObstacle{ob}
		m := limits(one)
		// The shared signal alone is not evidence against this body.
		blocked := m["control_sweep"] == 0 || m["articulated_sweep"] == 0 || m["obstacles"] == 0 || m["junction_admission"] == 0 || m["junction_sweep"] == 0 || m["required_merge"] == 0 || m["turn_merge"] == 0 || m["turn_lane_clearance"] == 0 || (copyDriver.approachMerge != nil && m["merge_clearance"] == 0)
		if blocked {
			d.Blockers = append(d.Blockers, StopBlocker{Position: ob.Pos, Heading: ob.Heading, Speed: ob.Speed, Length: ob.Length, Width: ob.Width, Limits: m})
		}
	}
	return d
}
