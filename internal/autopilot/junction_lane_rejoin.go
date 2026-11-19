package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A bus displaced beside its own straight lane can have a safe way
// out even when the nominal junction sweep predicts a sharp turn across a
// queued neighbor. Rejoin slowly using the real actuator/body trajectory.
func (ad *AutopilotDriver) junctionLaneRejoinControls(f autopilotFrame) (float64, float64, float64, bool) {
	if f.world == nil || f.bus.IsArticulated() || f.speedAbs > 25 || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || len(ad.drivePath) < 2 ||
		ad.State == AutopilotStateDocked || ad.State == AutopilotStateDeparting || f.nearRoundabout ||
		ad.approachMerge != nil || ad.midLaneRecovery != nil || ad.busTurnHoldDistance > 0 ||
		ad.uTurn != nil || ad.fallbackJunction != nil || mergeYieldHold(f) {
		ad.junctionRejoinActive = false
		return 0, 0, 0, false
	}
	margin := f.bus.Length / 2
	if ad.junctionRejoinActive {
		margin = f.bus.OverallLength()
	}
	if !f.world.InsideJunction(f.bus.Pos, f.laneWidth, margin) {
		ad.junctionRejoinActive = false
		return 0, 0, 0, false
	}
	base := ad.pointAt(f.s)
	delta := ad.pointAt(f.s + 160).Sub(base)
	if delta.Length() < 150 {
		ad.junctionRejoinActive = false
		return 0, 0, 0, false
	}
	heading := math.Atan2(delta.Y, delta.X)
	u := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	optionalMerge := ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.TargetLaneOffset-ad.LaneOffset) < 1 &&
		f.remaining > 600 && ad.busLaneJunction == nil && ad.rightTurnJunction == nil && ad.exitCruise == nil
	offset := 0.
	if optionalMerge {
		// Keep the already selected lane. Returning both side-by-side buses
		// to their route's original lane would create a second merge conflict.
		offset = ad.LaneOffset
	}
	if ad.pointAt(f.s+80).Sub(base).Normalize().Dot(u) < .995 {
		ad.junctionRejoinActive = false
		return 0, 0, 0, false
	}
	lateral := f.bus.Pos.Sub(base.Add(right.Mul(offset))).Dot(right)
	yaw := sim.NormalizeAngle(heading - f.bus.Heading)
	if math.Abs(lateral) < f.laneWidth*.2 && math.Abs(yaw) < .15 {
		ad.junctionRejoinActive = false
		return 0, 0, 0, false
	}
	if !ad.junctionRejoinActive && (f.speedAbs > .5 || math.Abs(yaw) > .9 ||
		(!optionalMerge && ((math.Abs(lateral) <= f.laneWidth && math.Abs(yaw) <= .35) || math.Abs(yaw) < .2))) {
		return 0, 0, 0, false
	}
	if ad.trafficLightSpeed(f, 30) <= .5 || ad.junctionAdmissionSpeed(f) <= .5 {
		return 0, 1, 0, ad.junctionRejoinActive
	}
	goal := ad.pointAt(f.s + math.Max(140, 3*math.Abs(lateral))).Add(right.Mul(offset))
	toGoal := goal.Sub(f.bus.Pos)
	alpha := sim.NormalizeAngle(math.Atan2(toGoal.Y, toGoal.X) - f.bus.Heading)
	desired := sim.Clamp(math.Atan2(2*f.bus.WheelBase*math.Sin(alpha), math.Max(20, toGoal.Length()))/f.bus.MaxSteerAngle, -1, 1)
	choices := []float64{desired, desired * .65, sim.Clamp(desired*1.5, -1, 1)}
	if math.Abs(yaw) > .1 {
		choices = append(choices, math.Copysign(.65, yaw), math.Copysign(1, yaw))
	}
	for _, steer := range choices {
		if ad.junctionRejoinSweepClear(f, steer, goal, heading) {
			ad.junctionRejoinActive = true
			ad.LastPursuitPt = goal
			ad.LastTargetSpeed = 18
			th, br := ad.longitudinalControls(f.bus.Speed, 18)
			return th, br, steer, true
		}
	}
	if f.speedAbs < .5 {
		for _, steer := range choices {
			parked := *f.bus
			parked.SteeringAngle = steer * parked.MaxSteerAngle
			trial := f
			trial.bus = &parked
			if ad.junctionRejoinSweepClear(trial, steer, goal, heading) {
				ad.junctionRejoinActive = true
				ad.LastTargetSpeed = 0
				return 0, 1, steer, true
			}
		}
	}
	return 0, 1, desired, ad.junctionRejoinActive
}

// The junction exit may still have a corner across the painted median.
// Outside the box that intrusion must shrink at every physical update; a
// fully legal pose keeps the normal zero-intrusion boundary.
func (ad *AutopilotDriver) junctionRejoinSweepClear(f autopilotFrame, steer float64, goal sim.Vec2, heading float64) bool {
	if !ad.stationForwardEscapeClear(f, steer, 18, goal, 10) {
		return false
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	b := *f.bus
	initial := roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew)
	previous := initial
	checkedOutside := false
	for tick := 0; tick < 120; tick++ {
		th, br := ad.longitudinalControls(b.Speed, 18)
		b.SetNextAccelerationScale(1)
		b.Update(1./60, th, br, steer, false, false)
		if f.world.InsideJunction(b.Pos, f.laneWidth, 0) {
			continue
		}
		checkedOutside = true
		clearance := roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew)
		if clearance < math.Min(previous, 0)-.01 {
			return false
		}
		previous = clearance
	}
	return !checkedOutside || initial >= 0 || previous > initial+.1
}
