package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func priorityTestLights() (*TrafficLightSystem, *TrafficIntersection) {
	i := &TrafficIntersection{ID: 7, IsMajor: true, Phase: PhaseEWGreen, GreenDuration: 8, LeftGreenDuration: 5.5, YellowDuration: 2.5,
		ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachIs4Lane: [4]bool{true, true, true, true}, ApproachBusLane: [4]bool{true, true, true, true}}
	return &TrafficLightSystem{Intersections: []*TrafficIntersection{i}}, i
}

func priorityTestBus(id int, phase LightPhase) BusSignalObservation {
	h := 0.
	if phase == PhaseNSGreen || phase == PhaseNSLeftGreen {
		h = math.Pi / 2
	}
	u := UnitFromAngle(h)
	r := Vec2{X: -u.Y, Y: u.X}
	b := NewBus(0, 0, h)
	b.Pos = u.Mul(-450).Add(r.Mul(150))
	return BusSignalObservation{BusID: id, Bus: b, Request: &BusSignalRequest{IntersectionID: 7, Heading: h, Phase: phase}}
}

func TestBusPriorityExtendsOnlyUntilFullBusClears(t *testing.T) {
	tls, i := priorityTestLights()
	ob := priorityTestBus(1, PhaseEWGreen)
	ob.Bus.SetModel(BusArticulated)
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	i.Timer = 8
	tls.Update(.5)
	if i.Phase != PhaseEWGreen {
		t.Fatal("approaching bus lost its green")
	}
	ob.Request = nil
	ob.Bus.Pos = Vec2{X: 310, Y: 182}
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	if len(i.BusPriorityQueue) != 1 {
		t.Fatal("priority ended while trailer occupied junction")
	}
	tls.Update(.5)
	if i.Phase != PhaseEWGreen {
		t.Fatal("trailer lost green")
	}
	ob.Bus.Pos = Vec2{X: 800, Y: 182}
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	tls.Update(0)
	if len(i.BusPriorityQueue) != 0 || i.Phase != PhaseEWYellow {
		t.Fatal("green remained extended after bus cleared")
	}
}

func TestBusPriorityRedNextGreenFIFOAndYellow(t *testing.T) {
	tls, i := priorityTestLights()
	ns := priorityTestBus(2, PhaseNSLeftGreen)
	ew := priorityTestBus(1, PhaseEWGreen)
	tls.ObserveBusPriority(true, []BusSignalObservation{ns}, 52)
	tls.ObserveBusPriority(true, []BusSignalObservation{ew, ns}, 52)
	if i.BusPriorityQueue[0].BusID != 2 {
		t.Fatal("submission order replaced arrival order")
	}
	i.Timer = 2.9
	tls.Update(.05)
	if i.Phase != PhaseEWGreen {
		t.Fatal("minimum green truncated")
	}
	tls.Update(.1)
	if i.Phase != PhaseEWYellow || i.Timer != 0 {
		t.Fatal("red priority skipped yellow")
	}
	tls.Update(2.49)
	if i.Phase != PhaseEWYellow {
		t.Fatal("yellow shortened")
	}
	tls.Update(.02)
	if i.Phase != PhaseNSLeftGreen {
		t.Fatalf("next green did not match first bus: %v", i.Phase)
	}
	ns.Bus.Pos = Vec2{X: -182, Y: 800}
	ns.Request = nil
	tls.ObserveBusPriority(true, []BusSignalObservation{ew, ns}, 52)
	tls.Update(3)
	if i.Phase != PhaseNSLeftYellow {
		t.Fatal("second bus did not receive next turn")
	}
	tls.Update(2.5)
	if i.Phase != PhaseEWGreen {
		t.Fatal("second direction not served")
	}
}

func TestBusPriorityBoundedAndSaved(t *testing.T) {
	tls, i := priorityTestLights()
	ob := priorityTestBus(1, PhaseEWGreen)
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	i.Timer = 8 + BusPriorityMaxExtension - .1
	tls.Update(.2)
	if i.Phase != PhaseEWYellow || !i.BusPriorityQueue[0].Served {
		t.Fatal("unbounded green extension")
	}
	data, err := json.Marshal(tls)
	if err != nil {
		t.Fatal(err)
	}
	var restored TrafficLightSystem
	if err = json.Unmarshal(data, &restored); err != nil {
		t.Fatal(err)
	}
	restored.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	if !restored.Intersections[0].BusPriorityQueue[0].Served {
		t.Fatal("save/load renewed exhausted request")
	}
	restored.Update(2.5)
	if restored.Intersections[0].Phase != PhaseEWLeftGreen {
		t.Fatal("exhausted request hijacked normal cycle")
	}
	restored.ObserveBusPriority(false, nil, 52)
	if restored.BusPriorityEnabled || len(restored.Intersections[0].BusPriorityQueue) != 0 {
		t.Fatal("disable retained requests")
	}
}

func TestBusPriorityExcludesOrdinaryRoadAndMissingBuses(t *testing.T) {
	tls, i := priorityTestLights()
	i.ApproachBusLane = [4]bool{}
	ob := priorityTestBus(1, PhaseEWGreen)
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	if len(i.BusPriorityQueue) != 0 {
		t.Fatal("ordinary road gained priority")
	}
	i.ApproachBusLane[0] = true
	tls.ObserveBusPriority(true, []BusSignalObservation{ob}, 52)
	tls.ObserveBusPriority(true, nil, 52)
	if len(i.BusPriorityQueue) != 0 {
		t.Fatal("removed bus retained priority")
	}
}

func TestContinuousBusPriorityDoesNotStarveOtherMovements(t *testing.T) {
	tls, i := priorityTestLights()
	tls.BusPriorityEnabled = true
	var elapsed [4]float64
	for tick := 0; tick < 12000; tick++ {
		// A continuing stream replaces each request after its bus has crossed.
		if i.Phase == PhaseNSLeftGreen && i.Timer > 3 {
			i.BusPriorityQueue = nil
		}
		if len(i.BusPriorityQueue) == 0 || i.BusPriorityQueue[0].Served {
			i.BusPriorityQueue = []BusSignalRequest{{BusID: tick + 1, IntersectionID: i.ID, Phase: PhaseNSLeftGreen}}
		}
		tls.Update(.02)
		for index := range elapsed {
			phase := LightPhase(index * 2)
			if i.Phase == phase {
				elapsed[index] = 0
			} else {
				elapsed[index] += .02
			}
			if elapsed[index] > 2*i.CycleDuration() {
				t.Fatalf("movement %d starved for %.1fs by bus priority: phase%d timer%.2f waits%v", phase, elapsed[index], i.Phase, i.Timer, i.PriorityPhaseWait)
			}
		}
	}
}

func TestBusPriorityFairnessSurvivesSaveLoad(t *testing.T) {
	tls, i := priorityTestLights()
	tls.BusPriorityEnabled = true
	i.Phase, i.Timer = PhaseNSLeftYellow, 2.49
	i.PriorityPhaseWait = [4]float64{i.CycleDuration() + 1, 0, 0, 0}
	i.BusPriorityQueue = []BusSignalRequest{{BusID: 99, IntersectionID: i.ID, Phase: PhaseNSLeftGreen}}
	raw, err := json.Marshal(tls)
	if err != nil {
		t.Fatal(err)
	}
	var restored TrafficLightSystem
	if err = json.Unmarshal(raw, &restored); err != nil {
		t.Fatal(err)
	}
	restored.Update(.02)
	if restored.Intersections[0].Phase != PhaseEWGreen {
		t.Fatal("load forgot overdue ordinary movement")
	}
}

func TestOverdueMovementReceivesFullGreen(t *testing.T) {
	tls, i := priorityTestLights()
	tls.BusPriorityEnabled = true
	i.Phase, i.Timer = PhaseNSLeftYellow, i.YellowDuration
	i.PriorityPhaseWait[0] = i.CycleDuration() + 1
	i.BusPriorityQueue = []BusSignalRequest{{BusID: 99, IntersectionID: i.ID, Phase: PhaseNSLeftGreen}}
	tls.Update(.02)
	if i.Phase != PhaseEWGreen {
		t.Fatal("overdue movement was skipped")
	}
	for i.Timer+.02 < i.GreenDuration {
		tls.Update(.02)
		if i.Phase != PhaseEWGreen {
			t.Fatal("bus priority cut short overdue movement")
		}
	}
	tls.Update(.04)
	if i.Phase != PhaseEWYellow {
		t.Fatal("regular green did not resume the normal yellow transition")
	}
}

func TestBusPriorityPreservesWaitingNextMovement(t *testing.T) {
	tls, i := priorityTestLights()
	tls.BusPriorityEnabled = true
	i.Phase, i.Timer = PhaseEWYellow, i.YellowDuration
	i.PriorityPhaseWait[1] = i.CycleDuration() / 2
	i.BusPriorityQueue = []BusSignalRequest{{BusID: 99, IntersectionID: i.ID, Phase: PhaseNSLeftGreen}}
	tls.Update(.02)
	if i.Phase != PhaseEWLeftGreen {
		t.Fatal("bus request skipped the waiting left-turn phase")
	}
	tls.Update(busPriorityMinGreen + .01)
	if i.Phase != PhaseEWLeftGreen {
		t.Fatal("left turn only received the shortened priority green")
	}
}

func TestGreenRemainingUsesActivePriorityWindow(t *testing.T) {
	_, inter := priorityTestLights()
	inter.Phase, inter.Timer = PhaseNSLeftGreen, inter.LeftGreenDuration+2
	inter.BusPriorityQueue = []BusSignalRequest{{Phase: PhaseNSLeftGreen}}
	if got := inter.GreenRemaining(true); got != BusPriorityMaxExtension-2 {
		t.Fatalf("extended green remaining = %g", got)
	}
	if got := inter.GreenRemaining(false); got != 0 {
		t.Fatalf("unextended green remaining = %g", got)
	}
	inter.Timer = 2
	inter.BusPriorityQueue[0].Phase = PhaseEWGreen
	if got := inter.GreenRemaining(true); got != busPriorityMinGreen-2 {
		t.Fatalf("shortened green remaining = %g", got)
	}
	inter.PriorityPhaseWait[3] = inter.priorityServiceThreshold()
	if got := inter.GreenRemaining(true); got != inter.LeftGreenDuration-2 {
		t.Fatalf("protected green remaining = %g", got)
	}
}

func TestWaitingLeftBusProtectsOrdinaryApproachFromPrioritySkip(t *testing.T) {
	const laneWidth = 52
	tls, inter := priorityTestLights()
	inter.ApproachLanes = [4]int{8, 8, 4, 4}
	inter.ApproachBusLane = [4]bool{true, true, false, false}
	inter.Phase, inter.Timer = PhaseNSYellow, inter.YellowDuration
	inter.PriorityPhaseWait[3] = inter.CycleDuration() * .75
	heading := -math.Pi / 2
	bus := NewBus(laneWidth/2, inter.GetStopLineDistForHeading(heading, laneWidth)+62, heading)
	bus.JunctionCorridor = &JunctionCorridor{
		EntryHeading: heading, Remaining: .25,
		Bodies: []OBB{{Heading: heading}, {Heading: math.Pi}},
	}
	priority := priorityTestBus(2, PhaseEWGreen)
	tls.ObserveBusPriority(true, []BusSignalObservation{{BusID: 1, Bus: bus}, priority}, laneWidth)
	if len(inter.BusPriorityQueue) != 1 || inter.BusPriorityQueue[0].BusID != 2 {
		t.Fatal("ordinary approach must not receive an extension request")
	}
	tls.Update(.02)
	if inter.Phase != PhaseNSLeftGreen {
		t.Fatal("priority skipped a waiting left-turn bus")
	}
}

func TestPriorityEndsEmptyGreenAndResumesOriginalSuccessor(t *testing.T) {
	tls, inter := priorityTestLights()
	inter.ApproachBusLane = [4]bool{true, true, false, false}
	inter.Phase, inter.Timer = PhaseNSGreen, .6
	// The next ordinary left movement is empty despite its large service age.
	inter.PriorityPhaseWait[3] = inter.CycleDuration() + 2
	bus := priorityTestBus(1, PhaseEWGreen)
	tls.BeginSignalDemand()
	tls.ObserveBusPriority(true, []BusSignalObservation{bus}, 52)
	tls.Update(.01)
	if inter.Phase != PhaseNSYellow {
		t.Fatal("empty green waited for the ordinary minimum")
	}
	tls.Update(2.49)
	if inter.Phase != PhaseNSYellow {
		t.Fatal("yellow was shortened")
	}
	tls.Update(.02)
	if inter.Phase != PhaseEWGreen || inter.PriorityResumePhase == nil || *inter.PriorityResumePhase != PhaseNSLeftGreen {
		t.Fatal("priority did not skip the empty ordinary phase and remember it")
	}

	raw, err := json.Marshal(tls)
	if err != nil {
		t.Fatal(err)
	}
	var restored TrafficLightSystem
	if err = json.Unmarshal(raw, &restored); err != nil {
		t.Fatal(err)
	}
	tls = &restored
	inter = tls.Intersections[0]
	bus.Bus.Pos.X = 800
	bus.Request = nil
	nextBus := priorityTestBus(2, PhaseEWLeftGreen)
	tls.BeginSignalDemand()
	inter.ObserveSignalDemand(math.Pi/2, true)
	tls.ObserveBusPriority(true, []BusSignalObservation{bus, nextBus}, 52)
	tls.Update(3)
	tls.Update(2.5)
	if inter.Phase != PhaseNSLeftGreen || inter.PriorityResumePhase != nil {
		t.Fatal("next priority request displaced the saved normal successor")
	}
}

func TestWaitingOrdinaryTrafficRetainsGreenProtection(t *testing.T) {
	tls, inter := priorityTestLights()
	inter.ApproachBusLane = [4]bool{true, true, false, false}
	inter.Phase, inter.Timer = PhaseNSGreen, .6
	tls.BeginSignalDemand()
	inter.ObserveSignalDemand(math.Pi/2, false)
	bus := priorityTestBus(1, PhaseEWGreen)
	tls.ObserveBusPriority(true, []BusSignalObservation{bus}, 52)
	tls.Update(.02)
	if inter.Phase != PhaseNSGreen {
		t.Fatal("green ended early with traffic still approaching")
	}

	inter.Phase, inter.Timer = PhaseNSYellow, inter.YellowDuration
	inter.PriorityPhaseWait[3] = inter.CycleDuration()
	tls.BeginSignalDemand()
	inter.ObserveSignalDemand(math.Pi/2, true)
	tls.ObserveBusPriority(true, []BusSignalObservation{bus}, 52)
	tls.Update(.02)
	if inter.Phase != PhaseNSLeftGreen {
		t.Fatal("waiting ordinary left movement lost its green protection")
	}
}

func TestEmptyOrdinaryGreenEndsForDemandWithoutPriorityRequest(t *testing.T) {
	tls, inter := priorityTestLights()
	inter.ApproachBusLane = [4]bool{true, true, false, false}
	inter.Phase, inter.Timer = PhaseNSGreen, 1
	tls.BusPriorityEnabled = true
	tls.BeginSignalDemand()
	inter.ObserveSignalDemand(0, false)
	tls.Update(.02)
	if inter.Phase != PhaseNSYellow {
		t.Fatal("empty ordinary green remained active while other traffic waited")
	}
	tls.Update(inter.YellowDuration)
	if inter.Phase != PhaseNSLeftGreen {
		t.Fatal("natural phase order changed without a priority request")
	}
}

func TestEmptyOverduePriorityPhaseDoesNotRepeatAfterYellow(t *testing.T) {
	tls, inter := priorityTestLights()
	inter.Phase, inter.Timer = PhaseNSYellow, inter.YellowDuration
	inter.PriorityPhaseWait[2] = 125
	inter.PriorityPhaseWait[0] = 106
	bus := priorityTestBus(1, PhaseEWGreen)
	tls.BeginSignalDemand()
	inter.ObserveSignalDemand(0, false)
	tls.ObserveBusPriority(true, []BusSignalObservation{bus}, 52)
	tls.Update(.02)
	if inter.Phase != PhaseEWGreen || inter.PriorityPhaseWait[2] != 0 {
		t.Fatalf("empty north-south phase retained priority: phase %v ages %v", inter.Phase, inter.PriorityPhaseWait)
	}
}
