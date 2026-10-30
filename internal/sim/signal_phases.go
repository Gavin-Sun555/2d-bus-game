package sim

import (
	"math"
)

func (tls *TrafficLightSystem) Update(dt float64) {
	for _, inter := range tls.Intersections {
		inter.Timer += dt
		if inter.Cooldown > 0 {
			inter.Cooldown -= dt
		}

		if !tls.BusPriorityEnabled || !inter.advanceBusPriority() {
			inter.advancePhase()
		}

	}

	for _, cam := range tls.RoadCameras {
		if cam.Cooldown > 0 {
			cam.Cooldown -= dt
		}
		if cam.FlashTimer > 0 {
			cam.FlashTimer -= dt
		}
	}
}

// advancePhase advances at most one phase per tick, discarding elapsed
// overshoot exactly as before. Road configuration determines whether a major
// intersection needs a dedicated left-turn phase on each axis.
func (inter *TrafficIntersection) advancePhase() {
	duration, next, valid := inter.phaseTransition()
	if !valid {
		inter.Phase, inter.Timer = PhaseEWGreen, 0
		return
	}
	if inter.Timer >= duration {
		inter.Phase, inter.Timer = next, 0
	}
}

func (inter *TrafficIntersection) phaseTransition() (float64, LightPhase, bool) {
	switch inter.Phase {
	case PhaseEWGreen:
		return inter.GreenDuration, PhaseEWYellow, true
	case PhaseEWYellow:
		next := PhaseNSGreen
		if inter.IsMajor && inter.HasEW4Lane() {
			next = PhaseEWLeftGreen
		}
		return inter.YellowDuration, next, true
	case PhaseNSGreen:
		return inter.GreenDuration, PhaseNSYellow, true
	case PhaseNSYellow:
		next := PhaseEWGreen
		if inter.IsMajor && inter.HasNS4Lane() {
			next = PhaseNSLeftGreen
		}
		return inter.YellowDuration, next, true
	}
	if !inter.IsMajor {
		return 0, PhaseEWGreen, false
	}
	switch inter.Phase {
	case PhaseEWLeftGreen:
		return inter.LeftGreenDuration, PhaseEWLeftYellow, true
	case PhaseEWLeftYellow:
		return inter.YellowDuration, PhaseNSGreen, true
	case PhaseNSLeftGreen:
		return inter.LeftGreenDuration, PhaseNSLeftYellow, true
	case PhaseNSLeftYellow:
		return inter.YellowDuration, PhaseEWGreen, true
	default:
		return 0, PhaseEWGreen, false
	}
}

// GetDetailedSignals returns: (ewThrough, ewLeft, nsThrough, nsLeft)
func (inter *TrafficIntersection) GetDetailedSignals() (ewThrough, ewLeft, nsThrough, nsLeft LightSignal) {
	if !inter.IsMajor {
		// Minor intersection: permissive left turn (shares through light)
		switch inter.Phase {
		case PhaseEWGreen:
			return SignalGreen, SignalGreen, SignalRed, SignalRed
		case PhaseEWYellow:
			return SignalYellow, SignalYellow, SignalRed, SignalRed
		case PhaseNSGreen:
			return SignalRed, SignalRed, SignalGreen, SignalGreen
		case PhaseNSYellow:
			return SignalRed, SignalRed, SignalYellow, SignalYellow
		default:
			return SignalRed, SignalRed, SignalRed, SignalRed
		}
	}

	// Major intersection with dedicated left-turn phases:
	switch inter.Phase {
	case PhaseEWGreen:
		return SignalGreen, SignalRed, SignalRed, SignalRed
	case PhaseEWYellow:
		return SignalYellow, SignalRed, SignalRed, SignalRed
	case PhaseEWLeftGreen:
		return SignalRed, SignalGreen, SignalRed, SignalRed
	case PhaseEWLeftYellow:
		return SignalRed, SignalYellow, SignalRed, SignalRed
	case PhaseNSGreen:
		return SignalRed, SignalRed, SignalGreen, SignalRed
	case PhaseNSYellow:
		return SignalRed, SignalRed, SignalYellow, SignalRed
	case PhaseNSLeftGreen:
		return SignalRed, SignalRed, SignalRed, SignalGreen
	case PhaseNSLeftYellow:
		return SignalRed, SignalRed, SignalRed, SignalYellow
	default:
		return SignalRed, SignalRed, SignalRed, SignalRed
	}
}

// GetSignals returns the primary through signal states for (East-West, North-South)
func (inter *TrafficIntersection) GetSignals() (ew LightSignal, ns LightSignal) {
	ewThru, _, nsThru, _ := inter.GetDetailedSignals()
	return ewThru, nsThru
}

// GetSignalForApproach returns the appropriate through and left-turn signals for vehicle's heading
func (inter *TrafficIntersection) GetSignalForApproach(heading float64) (throughSig LightSignal, leftSig LightSignal) {
	ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
	cosH := math.Abs(math.Cos(heading))
	sinH := math.Abs(math.Sin(heading))
	if cosH > sinH {
		if !inter.ApproachHasDedicatedLanesForHeading(heading) {
			// Single-lane approach: permissive left turn shares through signal!
			return ewThru, ewThru
		}
		return ewThru, ewL
	}
	if !inter.ApproachHasDedicatedLanesForHeading(heading) {
		// Single-lane approach: permissive left turn shares through signal!
		return nsThru, nsThru
	}
	return nsThru, nsL
}

// GetSignalForHeading returns the traffic signal for a vehicle facing heading (optional isLeft flag)
func (inter *TrafficIntersection) GetSignalForHeading(heading float64, isLeft ...bool) LightSignal {
	thru, l := inter.GetSignalForApproach(heading)
	if len(isLeft) > 0 && isLeft[0] {
		return l
	}
	return thru
}
