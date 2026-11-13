package sim

import (
	"math"
)

func (tls *TrafficLightSystem) ClearUTurnOccupants() {
	clear(tls.uTurnOccupants)
}

func (tls *TrafficLightSystem) ObserveUTurnOccupant(center Vec2) {
	if tls.uTurnOccupants == nil {
		tls.uTurnOccupants = make(map[Vec2]bool)
	}
	tls.uTurnOccupants[center] = true
}

// WaitingUTurnHasPriority applies only to pre-entry reservations.
func (inter *TrafficIntersection) WaitingUTurnHasPriority() bool {
	return !inter.UTurnYieldOnGreen
}

func (inter *TrafficIntersection) HoldingUTurnClearance() bool {
	return inter.uTurnClearance
}

func (tls *TrafficLightSystem) Update(dt float64) {
	tls.StationCrossingElapsed = math.Mod(tls.StationCrossingElapsed+dt, stationSignalCycle)
	tls.RailElapsed += dt
	tls.updateStationCrossingSignals(dt)
	for _, inter := range tls.Intersections {
		inter.Timer += dt
		inter.updatePriorityPhaseWait(dt)
		if inter.Cooldown > 0 {
			inter.Cooldown -= dt
		}

		inter.uTurnClearance = false
		phase := inter.Phase
		occupied := tls.uTurnOccupants[inter.Center]
		// A green that served a U-turn gives ordinary conflicting movements
		// first admission on its next occurrence. The yellow belongs to
		// the same movement, including when a turn enters late.
		if occupied && phase >= PhaseEWGreen && phase <= PhaseNSLeftYellow {
			inter.UTurnServedPhases |= 1 << (phase / 2)
		}
		switch inter.Phase {
		case PhaseEWYellow, PhaseEWLeftYellow, PhaseNSYellow, PhaseNSLeftYellow:
			if occupied && inter.Timer >= inter.YellowDuration {
				// Keep the outgoing phase to resume its normal transition, but
				// display all red until the last turning bus clears the box.
				inter.uTurnClearance = true
				continue
			}
		}
		if tls.BusPriorityEnabled && inter.PriorityResumePhase == nil && inter.shouldResumeLeftAfterUTurn() {
			inter.Phase, inter.Timer = phase-1, 0
		} else if !tls.BusPriorityEnabled || !inter.advanceBusPriority() {
			inter.advancePhase()
		}
		if inter.Phase != phase && inter.Phase%2 == 0 {
			mask := uint8(1 << (inter.Phase / 2))
			inter.UTurnYieldOnGreen = inter.UTurnServedPhases&mask != 0
			inter.UTurnServedPhases &^= mask
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

// Give the front of the ordinary left-turn queue one window when a U-turn
// took its previous green. Limit the retry to protect the other directions.
func (inter *TrafficIntersection) shouldResumeLeftAfterUTurn() bool {
	if inter.UTurnYieldOnGreen || inter.Timer < inter.YellowDuration ||
		(inter.Phase != PhaseEWLeftYellow && inter.Phase != PhaseNSLeftYellow) {
		return false
	}
	mask := uint8(1 << (inter.Phase / 2))
	if inter.UTurnServedPhases&inter.waitingRegularLeft&mask == 0 {
		return false
	}
	maxWait := 1.5 * inter.CycleDuration()
	for _, wait := range inter.PriorityPhaseWait {
		if wait >= maxWait {
			return false
		}
	}
	return true
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
		if inter.IsMajor && inter.needsEWLeftPhase() {
			next = PhaseEWLeftGreen
		}
		return inter.YellowDuration, next, true
	case PhaseNSGreen:
		return inter.GreenDuration, PhaseNSYellow, true
	case PhaseNSYellow:
		next := PhaseEWGreen
		if inter.IsMajor && inter.needsNSLeftPhase() {
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
	if inter.uTurnClearance {
		return SignalRed, SignalRed, SignalRed, SignalRed
	}
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

func (tls *TrafficLightSystem) updateStationCrossingSignals(dt float64) {
	for _, st := range tls.StationCrossingStates {
		st.Timer += dt
		if !tls.BusPriorityEnabled || !st.Priority {
			switch st.Phase {
			case StationSignalGreen:
				if st.Timer >= 24.0 {
					st.Phase, st.Timer, st.Extension = StationSignalYellow, 0, 0
				}
			case StationSignalYellow:
				if st.Timer >= 4.0 {
					st.Phase, st.Timer, st.Extension = StationSignalRed, 0, 0
				}
			case StationSignalRed:
				if st.Timer >= 8.0 {
					st.Phase, st.Timer, st.Extension = StationSignalGreen, 0, 0
				}
			}
		} else {
			switch st.Phase {
			case StationSignalGreen:
				if st.Timer >= 24.0 {
					st.Extension += dt
					if st.Extension >= BusPriorityMaxExtension {
						st.Phase, st.Timer, st.Extension = StationSignalYellow, 0, 0
					}
				}
			case StationSignalYellow:
				if st.Timer >= 4.0 {
					st.Phase, st.Timer, st.Extension = StationSignalRed, 0, 0
				}
			case StationSignalRed:
				if st.Timer >= busPriorityMinGreen {
					st.Phase, st.Timer, st.Extension = StationSignalGreen, 0, 0
				}
			}
		}
	}
}
