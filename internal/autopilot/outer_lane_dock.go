package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

const (
	// A route bus stopped beside the platform boards from the travel lane
	// only once its planner has proved the empty bay unreachable for this
	// long, or once a full bay has kept it waiting there for the second
	// period. Until then it waits for a berth like any other bus.
	outerDockUnreachableWait = 2.0
	outerDockBlockedWait     = 15.0
)

// updateOuterLaneDock decides whether the bus may serve its target stop from
// the travel lane beside the bay this tick. Berserk keeps its outer-lane
// boarding; route buses (and standard autopilot) use it only as a last
// resort.
func (ad *AutopilotDriver) updateOuterLaneDock(f autopilotFrame, dt float64) {
	if !ad.FleetBus {
		ad.outerDockAllowed, ad.outerDockWait = ad.BerserkMode, 0
		return
	}
	stop := f.targetStop
	beside := stop != nil && !stop.CenterBRT && f.speedAbs < 1 && !f.bus.DoorsOpen &&
		stop.ContainsBusInOuterLane(f.bus, f.laneWidth) && !stop.ContainsBus(f.bus)
	if !beside {
		// Keep a granted permission until the bus has actually docked.
		activeOuterPlan := ad.outerDockAllowed && ad.stationDock != nil && stop != nil && ad.stationDock.stopID == stop.ID &&
			ad.stationDock.rank > 0 && !ad.stationDock.waiting && stop.ContainsBusInOuterLane(f.bus, f.laneWidth)
		if ad.State != AutopilotStateDocked && !activeOuterPlan {
			ad.outerDockAllowed = false
		}
		ad.outerDockWait = 0
		return
	}
	ad.outerDockWait += dt
	ad.outerDockAllowed = ad.outerDockAllowed ||
		(ad.stationDockUnreachable && ad.outerDockWait >= outerDockUnreachableWait) ||
		ad.outerDockWait >= outerDockBlockedWait
}

func (ad *AutopilotDriver) waitingAtUsableOuterBoardingPose(f autopilotFrame, inputs stationDockInputs) bool {
	stop := f.targetStop
	if !ad.FleetBus || ad.outerDockAllowed || ad.stationDock != nil || stop == nil || stop.CenterBRT ||
		inputs.throttle != 0 || inputs.brake <= 0 || inputs.toggleDoors || inputs.isDocked || f.speedAbs >= 8 ||
		stop.ContainsBus(f.bus) || !stop.ContainsBusInOuterLane(f.bus, f.laneWidth) ||
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) || !stationBoardingPoseClear(f) {
		return false
	}
	maxYaw := .2
	if !f.bus.IsArticulated() {
		// Match the compact berth crawl's alignment range so a bus that
		// just reached a usable boarding window stays there for permission.
		maxYaw = .25
	}
	return math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) < maxYaw
}

// OuterLaneDockAllowed reports whether the bus may board from the lane
// beside its target bay this tick.
func (ad *AutopilotDriver) OuterLaneDockAllowed() bool {
	return ad != nil && ad.outerDockAllowed
}
