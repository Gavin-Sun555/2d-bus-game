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
