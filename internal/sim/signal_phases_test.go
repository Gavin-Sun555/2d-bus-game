package sim

import "testing"

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
