package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A bus a few pixels past the boarding window must not crawl farther past it
// forever. Back into a proven boarding pose, including both articulated bodies,
// only when the actual reverse trajectory is clear. Passenger service and the
// route are unchanged; normal docking opens the doors after the bus stops.
func (ad *AutopilotDriver) tryStationOverrunRetreat(f autopilotFrame, dt float64) bool {
	stop := f.targetStop
	if stop == nil || f.world == nil || ad.headonRecovery != nil || ad.arrivalEscapeRetry > 0 ||
		ad.CanSkipTargetStop || ad.State == AutopilotStateDocked || ad.State == AutopilotStateDeparting ||
		f.speedAbs > .5 || f.bus.DoorsOpen || f.bus.DoorProgress > .05 ||
		f.bus.PassengerServiceRemaining() > 0 || len(ad.drivePath) < 2 || len(ad.cumLen) != len(ad.drivePath) ||
		ad.cumLen[len(ad.cumLen)-1]-f.s > 5 {
		ad.stationOverrunWait = 0
		return false
	}
	u := sim.UnitFromAngle(stop.Heading)
	overshoot := f.bus.Pos.Sub(stop.FrontBerthPosition()).Dot(u)
	yaw := sim.NormalizeAngle(f.bus.Heading - stop.Heading)
	if overshoot < 20 || overshoot > 90 || math.Abs(yaw) > .7 || f.distToStop > stop.Length ||
		ad.drivePath[len(ad.drivePath)-1].Distance(stop.FrontBerthPosition()) > 2 ||
		stop.ContainsBus(f.bus) || (ad.outerDockAllowed && stop.ContainsBusInOuterLane(f.bus, f.laneWidth)) ||
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()) || ad.trafficLightSpeed(f, 20) <= .5 {
		ad.stationOverrunWait = 0
		return false
	}
	ad.stationOverrunWait += math.Max(0, dt)
	if ad.stationOverrunWait < outerDockUnreachableWait {
		return true
	}
	ad.arrivalEscapeRetry = .5
	steer := 0.
	if math.Abs(yaw) > .08 {
		steer = math.Copysign(1, yaw)
	}
	for _, distance := range []float64{15, 25, 35, 45, 60} {
		pose, clear := ad.simulateReversePose(f, distance, steer)
		if !clear || math.Abs(sim.NormalizeAngle(pose.Heading-stop.Heading)) > .3 ||
			(!stop.ContainsBus(pose) && !((ad.outerDockAllowed || ad.FleetBus) && stop.ContainsBusInOuterLane(pose, f.laneWidth))) {
			continue
		}
		ad.stationDock = nil
		ad.sidecarEscapeActive, ad.sidecarPresteerValid = false, false
		ad.outerDockAllowed = ad.outerDockAllowed || !stop.ContainsBus(pose)
		ad.headonRecovery = &HeadonDeadlockRecovery{StartPos: f.bus.Pos, TargetDistance: distance, StationRetreat: true, RetreatSteer: steer, SkipPreSteer: true}
		return true
	}
	return false
}
