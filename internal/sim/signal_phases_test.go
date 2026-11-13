package sim

import (
	"math"
	"testing"
)

func TestSignalPhaseTransitionBoundariesAndTopology(t *testing.T) {
	tests := []struct {
		name                            string
		major, ewDedicated, nsDedicated bool
		start, next                     LightPhase
		duration                        float64
	}{
		{"minor EW green", false, false, false, PhaseEWGreen, PhaseEWYellow, 10},
		{"minor EW yellow", false, false, false, PhaseEWYellow, PhaseNSGreen, 3},
		{"minor NS green", false, false, false, PhaseNSGreen, PhaseNSYellow, 10},
		{"minor NS yellow", false, false, false, PhaseNSYellow, PhaseEWGreen, 3},
		{"major EW green", true, true, true, PhaseEWGreen, PhaseEWYellow, 10},
		{"major EW left phase", true, true, false, PhaseEWYellow, PhaseEWLeftGreen, 3},
		{"major EW no left phase", true, false, true, PhaseEWYellow, PhaseNSGreen, 3},
		{"major EW left green", true, true, true, PhaseEWLeftGreen, PhaseEWLeftYellow, 5},
		{"major EW left yellow", true, true, true, PhaseEWLeftYellow, PhaseNSGreen, 3},
		{"major NS green", true, true, true, PhaseNSGreen, PhaseNSYellow, 10},
		{"major NS left phase", true, false, true, PhaseNSYellow, PhaseNSLeftGreen, 3},
		{"major NS no left phase", true, true, false, PhaseNSYellow, PhaseEWGreen, 3},
		{"major NS left green", true, true, true, PhaseNSLeftGreen, PhaseNSLeftYellow, 5},
		{"major NS left yellow", true, true, true, PhaseNSLeftYellow, PhaseEWGreen, 3},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			inter := &TrafficIntersection{Phase: tt.start, Timer: tt.duration - 0.25, IsMajor: tt.major, GreenDuration: 10, YellowDuration: 3, LeftGreenDuration: 5, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachIs4Lane: [4]bool{tt.ewDedicated, tt.ewDedicated, tt.nsDedicated, tt.nsDedicated}}
			lights := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
			lights.Update(0)
			if inter.Phase != tt.start {
				t.Fatal("phase advanced before its duration")
			}
			lights.Update(0.25)
			if inter.Phase != tt.next || inter.Timer != 0 {
				t.Fatalf("at boundary: phase=%v timer=%v, want %v/0", inter.Phase, inter.Timer, tt.next)
			}
			inter.Phase, inter.Timer = tt.start, 0
			lights.Update(1000)
			if inter.Phase != tt.next || inter.Timer != 0 {
				t.Fatal("a large tick must advance exactly one phase and discard overshoot")
			}
		})
	}
}

func TestSignalInvalidPhaseRecovery(t *testing.T) {
	for _, inter := range []*TrafficIntersection{
		{Phase: LightPhase(99), IsMajor: true, Timer: 2},
		{Phase: LightPhase(99), Timer: 2},
		{Phase: PhaseEWLeftGreen, Timer: 2},
	} {
		lights := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
		lights.Update(0)
		if inter.Phase != PhaseEWGreen || inter.Timer != 0 {
			t.Fatal("invalid phase must recover to EW green with a fresh timer")
		}
	}
}

func TestUTurnClearanceBlocksRightOnRedAndPriorityGreen(t *testing.T) {
	const lw = 52
	inter := &TrafficIntersection{
		Phase: PhaseEWLeftYellow, Timer: 2.75, YellowDuration: 3, IsMajor: true,
		ApproachHasRoad:       [4]bool{true, true, true, true},
		ApproachLanes:         [4]int{8, 8, 8, 8},
		ApproachMedianBusLane: [4]bool{true, true, true, true},
		BusPriorityQueue:      []BusSignalRequest{{Phase: PhaseNSGreen}},
	}
	lights := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}, BusPriorityEnabled: true}
	lights.ObserveUTurnOccupant(inter.Center)
	lights.Update(.25)
	ew, ewLeft, ns, nsLeft := inter.GetDetailedSignals()
	if ew != SignalRed || ewLeft != SignalRed || ns != SignalRed || nsLeft != SignalRed {
		t.Fatal("a signal remained open during U-turn clearance")
	}
	if inter.GetMedianBusRightSignal(math.Pi/2) != SignalRed {
		t.Fatal("the median bus right arrow remained open during U-turn clearance")
	}
	line := inter.GetStopLineDistForHeading(0, lw)
	if stop, _ := lights.CheckStopForVehicleAtSpeed(Vec2{X: -line - 20, Y: 3.5 * lw}, 0, 100, lw, false, true, true, 40, true); !stop {
		t.Fatal("curbside right turn was permitted during U-turn clearance")
	}
	lights.ClearUTurnOccupants()
	lights.Update(.25)
	if inter.HoldingUTurnClearance() || inter.Phase != PhaseNSGreen {
		t.Fatalf("priority green did not resume after clearance: phase %v", inter.Phase)
	}
}

func TestUTurnGivesNextMatchingGreenToOrdinaryTraffic(t *testing.T) {
	tls, inter := priorityTestLights()
	inter.Phase = PhaseNSLeftGreen
	tls.ObserveUTurnOccupant(inter.Center)
	tls.Update(.1)
	if !inter.WaitingUTurnHasPriority() {
		t.Fatal("clearing turn lost the current green")
	}
	tls.ClearUTurnOccupants()

	nextGreen := func() {
		t.Helper()
		for range 8 {
			duration, _, _ := inter.phaseTransition()
			tls.Update(duration)
			if inter.Phase == PhaseNSLeftGreen {
				return
			}
		}
		t.Fatal("north-south left green did not recur")
	}
	nextGreen()
	if inter.WaitingUTurnHasPriority() {
		t.Fatal("another U-turn could reserve the next green")
	}
	nextGreen()
	if !inter.WaitingUTurnHasPriority() {
		t.Fatal("U-turn reservation was not restored after yielding a green")
	}
}

func TestLeftGreenResumesOnceAfterUTurnClearance(t *testing.T) {
	const laneWidth = 52
	tls, inter := priorityTestLights()
	inter.Phase, inter.Timer = PhaseNSLeftYellow, inter.YellowDuration
	heading := -math.Pi / 2
	bus := NewBus(3.5*laneWidth, inter.GetStopLineDistForHeading(heading, laneWidth)+58, heading)
	bus.JunctionCorridor = &JunctionCorridor{
		EntryHeading: heading, Remaining: .25,
		Bodies: []OBB{{Heading: heading}, {Heading: math.Pi}},
	}
	observations := []BusSignalObservation{{BusID: 1, Bus: bus, Request: &BusSignalRequest{
		IntersectionID: inter.ID, Heading: heading, Phase: PhaseNSLeftGreen,
	}}}
	tls.ObserveBusPriority(true, observations, laneWidth)
	tls.ObserveUTurnOccupant(inter.Center)
	tls.Update(.1)
	if !inter.HoldingUTurnClearance() {
		t.Fatal("left green reopened before the U-turn cleared")
	}
	tls.ClearUTurnOccupants()
	tls.Update(.1)
	if inter.Phase != PhaseNSLeftGreen || inter.WaitingUTurnHasPriority() {
		t.Fatal("waiting ordinary left did not receive the reopened green")
	}
	inter.Phase, inter.Timer = PhaseNSLeftYellow, inter.YellowDuration
	inter.UTurnServedPhases = 1 << (PhaseNSLeftGreen / 2)
	if inter.shouldResumeLeftAfterUTurn() {
		t.Fatal("already reopened green was eligible for another retry")
	}
}
