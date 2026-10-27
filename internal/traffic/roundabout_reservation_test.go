package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func reservationBus(x float64, model sim.BusModel) *sim.Bus {
	b := sim.NewBus(x, 26, 0)
	b.SetModel(model)
	b.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: sim.Vec2{X: 1}, Remaining: .3}
	return b
}

func TestRoundaboutReservationDrainsBeforeAdmission(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	owner := reservationBus(-500, sim.BusArticulated)
	car := &sim.TrafficCar{InRoundabout: true}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	tm.updateRoundaboutReservations(owner, w, 1)
	if owner.RoundaboutAdmissionHold == nil || !tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("new long bus must wait for circulating car")
	}
	car.InRoundabout = false
	tm.updateRoundaboutReservations(owner, w, 1)
	if owner.RoundaboutAdmissionHold != nil {
		t.Fatal("drained ring must release owner")
	}
}

func TestRoundaboutReservationAdmitsOrdinaryQueueLeader(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	long := reservationBus(-580, sim.BusArticulated)
	lead := reservationBus(-420, sim.BusStandard)
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: lead}}}}
	tm.updateRoundaboutReservations(long, w, 0)
	if tm.roundaboutReservations[0].owner != lead || long.RoundaboutAdmissionHold == nil || lead.RoundaboutAdmissionHold != nil {
		t.Fatal("head of queue must clear before long follower")
	}
}

func TestRoundaboutReservationRetainsTailAndReleases(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	b := reservationBus(-500, sim.BusArticulated)
	tm := &TrafficManager{}
	tm.updateRoundaboutReservations(b, w, 0)
	b.Pos = sim.Vec2{X: 180, Y: 26}
	tm.updateRoundaboutReservations(b, w, 0)
	b.Pos.X = 300
	tm.updateRoundaboutReservations(b, w, 0)
	if !tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("tail still occupies ring")
	}
	b.Pos.X = 600
	b.RoundaboutEntry = nil
	tm.updateRoundaboutReservations(b, w, 0)
	if tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("complete vehicle cleared ring")
	}
}

func TestRoundaboutReservationDoesNotChangeShortFleet(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	b := reservationBus(-400, sim.BusStandard)
	tm := &TrafficManager{}
	tm.updateRoundaboutReservations(b, w, 0)
	if tm.roundaboutReserved(sim.Vec2{}) || b.RoundaboutAdmissionHold != nil {
		t.Fatal("ordinary fleet acquired long-vehicle reservation")
	}
}

func TestRoundaboutReservationAcceptsLateLeaderAndCancellation(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	long := reservationBus(-580, sim.BusArticulated)
	lead := reservationBus(-420, sim.BusStandard)
	intent := lead.RoundaboutEntry
	lead.RoundaboutEntry = nil
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: lead}}}}
	tm.updateRoundaboutReservations(long, w, 0)
	if tm.roundaboutReservations[0].owner != long {
		t.Fatal("long arrival should initially reserve ring")
	}
	lead.RoundaboutEntry = intent
	tm.updateRoundaboutReservations(long, w, 0)
	if tm.roundaboutReservations[0].owner != lead {
		t.Fatal("late-published physical leader trapped by follower")
	}
	lead.RoundaboutEntry = nil
	long.RoundaboutEntry = nil
	tm.updateRoundaboutReservations(long, w, 0)
	if tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("cancelled approach retained reservation")
	}
}

func TestReservedRoundaboutCarWaitsThenEnters(t *testing.T) {
	rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -4000}, P2: sim.Vec2{X: 4000}, Lanes: 4}, {P1: sim.Vec2{Y: -4000}, P2: sim.Vec2{Y: 4000}, Lanes: 4}}}
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -500, Y: 78}, Heading: 0, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: 1, IsEW: true}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}, roundaboutReservations: []roundaboutReservation{{owner: reservationBus(-500, sim.BusArticulated)}}}
	for i := 0; i < 600; i++ {
		tm.StepCar(1./60, car, 0, nil, nil, 1, 52, w)
		if car.InRoundabout {
			t.Fatal("new car entered reserved ring")
		}
	}
	if car.Speed > 1 {
		t.Fatalf("waiting car still moving %.1f", car.Speed)
	}
	tm.roundaboutReservations = nil
	for i := 0; i < 600; i++ {
		tm.StepCar(1./60, car, 0, nil, nil, 1, 52, w)
		if car.InRoundabout {
			return
		}
	}
	t.Fatal("car did not enter after reservation released")
}

func TestAbreastArticulatedEntrantsDoNotReserveAgainstInnerLeader(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 240, InnerRadius: 120}}}
	inner := sim.NewBus(-335.21, 26, 0)
	inner.SetModel(sim.BusArticulated)
	outer := sim.NewBus(-335.11, 78, 0)
	outer.SetModel(sim.BusArticulated)
	for _, b := range []*sim.Bus{inner, outer} {
		b.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: sim.Vec2{X: 1}, Remaining: .3}
	}
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{ID: 1, Bus: outer}}}, roundaboutReservations: []roundaboutReservation{{owner: outer}}}
	for tick := 0; tick < 5; tick++ {
		tm.updateRoundaboutReservations(inner, w, 0)
		if tm.roundaboutReservations[0].owner != inner || inner.RoundaboutAdmissionHold != nil || outer.RoundaboutAdmissionHold == nil {
			t.Fatal("outer nose lead blocked the inner entrance")
		}
	}
}
