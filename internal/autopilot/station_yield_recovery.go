package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Recenter a partly pulled-out bus in its own bay when a stopped road bus
// needs the adjacent lane. This frees a real slot without entering its body.
func (ad *AutopilotDriver) prepareStationYieldRecovery(f autopilotFrame) {
	if ad.dockDepartureRun <= 0 || ad.midLaneRecovery != nil || ad.approachMerge != nil || f.speedAbs > 1 || f.bus.DoorsOpen {
		return
	}
	stop, u, r, active := ad.stationDepartureRoad(f)
	if !active {
		return
	}
	shift := stop.Pos.Sub(f.bus.Pos).Dot(r)
	if shift < 8 || shift > f.laneWidth*.6 {
		return
	}
	for _, ob := range f.obstacles {
		if ob.Bus == nil || math.Abs(ob.Speed) > 1 || math.Cos(ob.Heading-stop.Heading) < .98 {
			continue
		}
		request := ob.Bus.LaneMergeRequest
		if request == nil || request.Remaining <= 0 || request.Joined {
			continue
		}
		along := ob.Bus.Pos.Sub(f.bus.Pos).Dot(u)
		if along <= 0 || along > f.bus.OverallLength()+ob.Bus.OverallLength() {
			continue
		}
		if math.Abs(request.Target.Sub(stop.RoadCenter).Dot(r)-(float64(stop.RoadLanes)/2-.5)*f.laneWidth) > 4 {
			continue
		}
		ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: stop.Pos, Heading: stop.Heading, Remaining: .3}
		return
	}
}

func (ad *AutopilotDriver) stationYieldRecoveryActive(f autopilotFrame) bool {
	if ad.midLaneRecovery == nil || !ad.midLaneSweepClear {
		return false
	}
	stop, _, r, active := ad.stationDepartureRoad(f)
	return active && math.Abs(ad.midLaneRecovery.Target.Sub(stop.Pos).Dot(r)) < 3
}

// Relax a waiting margin only for the stationary bus whose live lane request
// we are making room for. The actuator sweep remains authoritative each tick.
func (ad *AutopilotDriver) clearsStationYieldNeighbor(f autopilotFrame, ob VehicleObstacle) bool {
	if ob.Bus == nil || math.Abs(ob.Speed) >= 1 || !ad.stationYieldRecoveryActive(f) {
		return false
	}
	request := ob.Bus.LaneMergeRequest
	return request != nil && request.Remaining > 0 && !request.Joined
}
