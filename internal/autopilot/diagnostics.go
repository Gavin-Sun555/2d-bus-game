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
		m := map[string]float64{"signal": copyDriver.trafficLightSpeed(frame, 150), "obstacles": copyDriver.obstacleSpeed(frame, 150), "junction_admission": copyDriver.junctionAdmissionSpeed(frame), "junction_sweep": copyDriver.junctionSweptSpeed(frame)}
		for k, v := range m {
			if math.IsInf(v, 0) || math.IsNaN(v) {
				m[k] = -1
			}
		}
		return m
	}
	d.Limits = limits(f)
	d.Limits["station"] = copyDriver.stationSpeed(f, 150)
	for _, ob := range obstacles {
		if ob.Pos.Distance(bus.Pos) > 1600 {
			continue
		}
		one := f
		one.obstacles = []VehicleObstacle{ob}
		m := limits(one)
		// The shared signal alone is not evidence against this body.
		blocked := m["obstacles"] == 0 || m["junction_admission"] == 0 || m["junction_sweep"] == 0
		if blocked {
			d.Blockers = append(d.Blockers, StopBlocker{Position: ob.Pos, Heading: ob.Heading, Speed: ob.Speed, Length: ob.Length, Width: ob.Width, Limits: m})
		}
	}
	return d
}
