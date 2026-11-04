package sim

import (
	"math"
	"testing"
)

func TestMedianBusRightUsesCrossLeftPhaseOnly(t *testing.T) {
	const lw = 52.
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		it := &TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachIs4Lane: [4]bool{true, true, true, true}, ApproachMedianBusLane: [4]bool{true, true, true, true}, YellowDuration: 3}
		tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{it}}
		u := UnitFromAngle(h)
		r := Vec2{X: -u.Y, Y: u.X}
		line := it.GetStopLineDistForHeading(h, lw)
		for phase := PhaseEWGreen; phase <= PhaseNSLeftYellow; phase++ {
			it.Phase = phase
			it.Timer = 0
			want := SignalRed
			if phase == MedianBusRightPhase(h) {
				want = SignalGreen
			}
			if phase == MedianBusRightPhase(h)+1 {
				want = SignalYellow
			}
			if got := it.GetMedianBusRightSignal(h); got != want {
				t.Fatalf("h%.2f phase%d rightSignal%d want%d", h, phase, got, want)
			}
			pos := u.Mul(-line - 60).Add(r.Mul(.5 * lw))
			stop, _ := tls.CheckStopForVehicleAtSpeed(pos, h, 500, lw, false, true, false, 100, true)
			if stop != (want != SignalGreen) {
				t.Fatalf("wrong protected right stopping h%.2f phase%d stop%v", h, phase, stop)
			}
			// The curb lane retains the ordinary, conflict-checked right-on-red rule.
			pos = u.Mul(-line - 60).Add(r.Mul(3.5 * lw))
			if stop, _ = tls.CheckStopForVehicleAtSpeed(pos, h, 500, lw, false, true, false, 100, true); stop {
				t.Fatal("curb right inherited median arrow")
			}
		}
	}
}

func TestMedianRightGetsPhaseAcrossNarrowRoad(t *testing.T) {
	it := &TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 2, 2}, ApproachIs4Lane: [4]bool{true, true, false, false}, ApproachMedianBusLane: [4]bool{true, true, false, false}, Phase: PhaseNSYellow, YellowDuration: 3, Timer: 3}
	it.advancePhase()
	if it.Phase != PhaseNSLeftGreen {
		t.Fatal("central bus right has no phase across two-lane road")
	}
}

func TestPlayerMedianRightArrowRecordedAtEntry(t *testing.T) {
	const lw = 52.
	for _, green := range []bool{false, true} {
		it := &TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachIs4Lane: [4]bool{true, true, true, true}, ApproachMedianBusLane: [4]bool{true, true, true, true}, Phase: PhaseEWGreen}
		if green {
			it.Phase = PhaseNSLeftGreen
		}
		tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{it}}
		line := it.GetStopLineDistForHeading(0, lw)
		b := NewBus(-line-35, lw*.5, 0)
		b.Speed = 80
		if v, n := tls.CheckPlayerViolation(b, lw); v {
			t.Fatal(n)
		}
		if !it.PlayerEntryMedianBusLane {
			t.Fatal("central entry not captured")
		}
		// A phase change while turning must not revoke lawful entry, nor excuse red entry.
		it.Phase = PhaseEWYellow
		b.Pos = Vec2{X: -100, Y: 80}
		b.Heading = .6
		b.SteeringAngle = .2
		violated, _ := tls.CheckPlayerViolation(b, lw)
		if violated == green {
			t.Fatalf("right turn green%v violation%v", green, violated)
		}
	}
}

func TestMedianRightPriorityRetainsBusInsideCrossPhase(t *testing.T) {
	it := &TrafficIntersection{ID: 1, IsMajor: true, Phase: PhaseNSLeftGreen, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachBusLane: [4]bool{true, true, true, true}, ApproachMedianBusLane: [4]bool{true, true, true, true}, ApproachIs4Lane: [4]bool{true, true, true, true}}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{it}}
	b := NewBus(-500, 26, 0)
	request := &BusSignalRequest{IntersectionID: 1, Heading: 0, Phase: PhaseNSLeftGreen}
	tls.ObserveBusPriority(true, []BusSignalObservation{{BusID: 0, Bus: b, Request: request}}, 52)
	if len(it.BusPriorityQueue) != 1 {
		t.Fatal("right turn request not queued")
	}
	b.Pos = Vec2{X: -100, Y: 80}
	b.Heading = .6
	tls.ObserveBusPriority(true, []BusSignalObservation{{BusID: 0, Bus: b}}, 52)
	if len(it.BusPriorityQueue) != 1 {
		t.Fatal("cross-phase request lost before the bus cleared")
	}
}

func TestMedianRightIgnoresLaterStraightJunction(t *testing.T) {
	it := &TrafficIntersection{IsMajor: true, Phase: PhaseNSLeftGreen, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachIs4Lane: [4]bool{true, true, true, true}, ApproachMedianBusLane: [4]bool{true, true, true, true}}
	next := *it
	next.Center = Vec2{X: 1000}
	next.Phase = PhaseEWLeftGreen
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{it, &next}}
	line := it.GetStopLineDistForHeading(0, 52)
	if stop, _ := tls.CheckStopForVehicleAtSpeed(Vec2{X: -line - 60, Y: 26}, 0, 1800, 52, false, true, false, 100, true); stop {
		t.Fatal("protected right stopped for a junction outside its turn")
	}
}
