package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Select with enough runway for the configured cruising speed, rather than
// waiting until a stopped queue shrinks the approach lookahead to one lane change.
func (ad *AutopilotDriver) berserkLeftApproach(f autopilotFrame) intersectionApproach {
	planning := f
	planning.speedAbs = math.Max(f.speedAbs, ad.GetBerserkCruiseSpeed())
	return ad.approachingIntersection(planning, true, true)
}

// Choose between the legal left-turn corridors once per junction. The selected
// path is also the required-merge path, so passing and curb policy cannot pull
// the player toward different turn lanes during the same approach.
func (ad *AutopilotDriver) selectBerserkLeftTurn(f autopilotFrame) {
	if !ad.BerserkMode || ad.FleetBus {
		ad.berserkLeftTurnChoice = nil
		return
	}
	if ad.berserkLeftTurnChoice != nil && (ad.busLaneJunction == nil || ad.busLaneJunction.Distance(*ad.berserkLeftTurnChoice) > 1) {
		ad.berserkLeftTurnChoice = nil
	}
	if f.world == nil || f.tls == nil || len(ad.drivePath) < 2 || ad.busJunctionEntered || ad.uTurn != nil || ad.fallbackJunction != nil ||
		ad.departureEscapeActive || ad.departurePresteerValid || ad.sidecarEscapeActive || ad.stationDock != nil ||
		(ad.dockDepartureRun > 0 && f.s < ad.dockDepartureRun) ||
		(ad.State != AutopilotStateCruising && ad.State != AutopilotStateApproaching) {
		return
	}
	if from := ad.fromStop(f.world); from != nil && f.bus.Pos.Distance(from.Pos) < from.Length+150 &&
		(from.ContainsBus(f.bus) || from.ContainsBusInOuterLane(f.bus, f.laneWidth)) {
		// A saved departure can already be labelled cruising. Finish the
		// berth/shoulder trajectory before replacing it with a road-lane merge.
		return
	}
	h := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-h)) > .25 || sidecarBodyInJunction(f) {
		return
	}
	coord, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, h, f.laneWidth)
	road, ok := f.world.RoadAt(coord, ew, f.bus.Pos)
	if !ok || !road.HasBusLane() || road.HasMedianBusLane() || !road.AllowsCurbBusLeft() {
		return
	}
	a := ad.berserkLeftApproach(f)
	if a.approachingInter == nil || !a.isApproachingIntersection || !a.isUpcomingLeft || f.world.IsAtRoundabout(a.approachingInter.Center) {
		return
	}
	junction := a.approachingInter.Center
	if ad.berserkLeftTurnChoice != nil && ad.berserkLeftTurnChoice.Distance(junction) < 1 {
		return
	}
	u := sim.UnitFromAngle(h)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	// Compare moving traffic as well as stopped queues, only on this
	// approach. Traffic beyond the left turn does not occupy its entry lane.
	scoring := f
	scoring.obstacles = nil
	line := a.approachingInter.GetStopLineDistForHeading(h, f.laneWidth)
	for _, ob := range f.obstacles {
		if ob.Pos.Sub(junction).Dot(u) < -line+100 {
			scoring.obstacles = append(scoring.obstacles, ob)
		}
	}
	inner := ad.busLaneTrafficScore(scoring, u, r, 0, false)
	curb := ad.busLaneTrafficScore(scoring, u, r, road.BusLaneIndex(), false)
	ordinary := ad.ordinaryBusLeft
	if !ad.busLaneTurnChosen {
		origin := sim.Vec2{Y: coord}
		if !ew {
			origin = sim.Vec2{X: coord}
		}
		ordinary = f.bus.Pos.Sub(origin).Dot(r) <= 2*f.laneWidth
	}
	if inner+1 < curb {
		ordinary = true
	} else if curb+1 < inner {
		ordinary = false
	}
	before := *ad
	ad.ordinaryBusLeft = ordinary
	from, to := ad.fromStop(f.world), f.targetStop
	if from == nil || to == nil {
		*ad = before
		return
	}
	ad.PrepareLeg(f.world, from, to, f.laneWidth)
	joined := false
	if ordinary {
		joined = ad.joinMedianLeftApproach(f, h, junction)
	} else {
		joined = ad.joinCurbLeftTurn(f, junction, u, (float64(road.BusLaneIndex())+.5)*f.laneWidth, true)
	}
	if !joined {
		*ad = before
		return
	}
	ad.approachMerge, ad.midLaneRecovery = nil, nil
	ad.approachMergeJoined = false
	ad.busLaneJunction = &junction
	ad.busLaneTurnChosen = true
	ad.berserkLeftTurnChoice = &junction
	ad.busTurnHoldDistance = 0
	ad.OvertakeState = OvertakeNone
	f.bus.LaneMergeRequest = nil
	ad.trackPath(f.bus.Pos)
}
