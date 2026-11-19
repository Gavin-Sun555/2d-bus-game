package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func reservationBus(x float64, model sim.BusModel) *sim.Bus {
	b := sim.NewBus(x, 26, 0)
	b.SetModel(model)
	b.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: sim.Vec2{X: 1}, Remaining: .3}
	return b
}

func approachOwner(tm *TrafficManager, forward sim.Vec2) *sim.Bus {
	for i := range tm.roundaboutReservations {
		for k := range tm.roundaboutReservations[i].approaches {
			app := &tm.roundaboutReservations[i].approaches[k]
			if app.owner != nil && app.forward.Dot(forward) > 0.9 {
				return app.owner
			}
		}
	}
	return nil
}

func TestRoundaboutReservationAllowsEntryWithOtherRingTraffic(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	owner := reservationBus(-350, sim.BusArticulated)
	// Circulating car on the far side of the ring does not force an entrance hold.
	car := &sim.TrafficCar{InRoundabout: true, RoundaboutCenter: sim.Vec2{}, Pos: sim.Vec2{X: 190, Y: 0}}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	tm.updateRoundaboutReservations(owner, w, 1, 52)
	if owner.RoundaboutAdmissionHold != nil {
		t.Fatal("circulating ring traffic must not impose a whole-ring admission hold on the approaching bus")
	}
}

func TestRoundaboutReservationAdmitsOrdinaryQueueLeader(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	long := reservationBus(-580, sim.BusArticulated)
	lead := reservationBus(-420, sim.BusStandard)
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: lead}}}}
	tm.updateRoundaboutReservations(long, w, 0, 52)
	if approachOwner(tm, sim.Vec2{X: 1}) != lead || long.RoundaboutAdmissionHold == nil || lead.RoundaboutAdmissionHold != nil {
		t.Fatal("head of queue must clear before long follower")
	}
}

func TestRoundaboutReservationReleasesOnceTrailerClearsMouth(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	b := reservationBus(-500, sim.BusArticulated)
	tm := &TrafficManager{}
	tm.updateRoundaboutReservations(b, w, 0, 52)
	if !tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("expected approach reservation while entering")
	}
	// Front inside ring, trailer still near West entrance mouth (x=-150 -> tail ~ -294 < -236).
	b.Pos = sim.Vec2{X: -160, Y: 78}
	b.ResetArticulation()
	tm.updateRoundaboutReservations(b, w, 0, 52)
	if !tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("expected approach reservation retained while trailer is still crossing the mouth")
	}
	// Once body has moved past the West mouth into the South/East arc, release immediately
	// without locking the ring until final roundabout exit.
	b.Pos = sim.Vec2{X: 20, Y: 195}
	b.Heading = 0
	b.ResetArticulation()
	tm.updateRoundaboutReservations(b, w, 0, 52)
	if tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("expected approach reservation released once trailer cleared entrance mouth")
	}
}

func TestRoundaboutReservationDoesNotChangeShortFleet(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}}
	b := reservationBus(-400, sim.BusStandard)
	tm := &TrafficManager{}
	tm.updateRoundaboutReservations(b, w, 0, 52)
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
	tm.updateRoundaboutReservations(long, w, 0, 52)
	if approachOwner(tm, sim.Vec2{X: 1}) != long {
		t.Fatal("long arrival should initially hold its own approach")
	}
	lead.RoundaboutEntry = intent
	tm.updateRoundaboutReservations(long, w, 0, 52)
	if approachOwner(tm, sim.Vec2{X: 1}) != lead {
		t.Fatal("late-published physical leader trapped by follower")
	}
	lead.RoundaboutEntry = nil
	long.RoundaboutEntry = nil
	tm.updateRoundaboutReservations(long, w, 0, 52)
	if tm.roundaboutReserved(sim.Vec2{}) {
		t.Fatal("cancelled approach retained reservation")
	}
}

func TestReservedRoundaboutCarWaitsThenEnters(t *testing.T) {
	rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -4000}, P2: sim.Vec2{X: 4000}, Lanes: 4}, {P1: sim.Vec2{Y: -4000}, P2: sim.Vec2{Y: 4000}, Lanes: 4}}}
	sameApproachCar := &sim.TrafficCar{Pos: sim.Vec2{X: -500, Y: 78}, Heading: 0, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: 1, IsEW: true}
	otherApproachCar := &sim.TrafficCar{Pos: sim.Vec2{X: 500, Y: -78}, Heading: math.Pi, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: 1, IsEW: true}
	tm := &TrafficManager{
		Cars: []*sim.TrafficCar{sameApproachCar, otherApproachCar},
		roundaboutReservations: []roundaboutReservation{{
			approaches: [4]roundaboutApproachSlot{{
				forward: sim.Vec2{X: 1},
				owner:   reservationBus(-500, sim.BusArticulated),
			}},
		}},
	}
	if tm.roundaboutReservedForCar(rb, otherApproachCar, sim.Vec2{X: -1}, 52) {
		t.Fatal("west approach owner must not block east approach car")
	}
	for i := 0; i < 600; i++ {
		tm.StepCar(1./60, sameApproachCar, 0, nil, nil, 1, 52, w)
		if sameApproachCar.InRoundabout {
			t.Fatal("same-approach car entered beside reserving articulated bus")
		}
	}
	if sameApproachCar.Speed > 1 {
		t.Fatalf("waiting car still moving %.1f", sameApproachCar.Speed)
	}
	tm.roundaboutReservations = nil
	for i := 0; i < 600; i++ {
		tm.StepCar(1./60, sameApproachCar, 0, nil, nil, 1, 52, w)
		if sameApproachCar.InRoundabout {
			return
		}
	}
	t.Fatal("car did not enter after reservation released")
}

func TestRoundaboutReservationLetsSameLaneCarClearFirst(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	owner := reservationBus(-457, sim.BusArticulated)
	owner.Pos.Y = 78
	lead := &sim.TrafficCar{Pos: sim.Vec2{X: -355, Y: 78}, Length: 46, Width: 22}
	tm := &TrafficManager{
		roundaboutReservations: []roundaboutReservation{{
			approaches: [4]roundaboutApproachSlot{{
				forward: sim.Vec2{X: 1},
				owner:   owner,
			}},
		}},
	}
	forward := sim.Vec2{X: 1}
	if tm.roundaboutReservedForCar(rb, lead, forward, 52) {
		t.Fatal("car ahead of the reserving bus cannot clear the entrance")
	}
	lead.Pos.Y = 26
	if !tm.roundaboutReservedForCar(rb, lead, forward, 52) {
		t.Fatal("adjacent lane car on same approach should yield while owner enters mouth")
	}
	lead.Pos.Y = 78
	tm.roundaboutReservations[0].approaches[0].entered = true
	if !tm.roundaboutReservedForCar(rb, lead, forward, 52) {
		t.Fatal("same-approach car must yield while entered bus still straddles mouth")
	}
}

func TestRoundaboutReservationAllowsConcurrentOppositeApproaches(t *testing.T) {
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 240, InnerRadius: 120}}}
	west := reservationBus(-350, sim.BusArticulated)
	east := reservationBus(350, sim.BusArticulated)
	east.Heading = math.Pi
	east.Pos.Y = -26
	east.ResetArticulation()
	east.RoundaboutEntry.Forward.X = -1
	tm := &TrafficManager{
		AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: east}}},
	}
	tm.updateRoundaboutReservations(west, w, 0, 52)
	if west.RoundaboutAdmissionHold != nil || east.RoundaboutAdmissionHold != nil {
		t.Fatalf("opposite approaches must enter concurrently without mutual admission hold (westHold=%v eastHold=%v)",
			west.RoundaboutAdmissionHold != nil, east.RoundaboutAdmissionHold != nil)
	}
	if approachOwner(tm, sim.Vec2{X: 1}) != west || approachOwner(tm, sim.Vec2{X: -1}) != east {
		t.Fatal("expected each approach to independently own only its own entrance mouth")
	}
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
	tm := &TrafficManager{
		AIBusMgr: &AIBusManager{Buses: []*AIBus{{ID: 1, Bus: outer}}},
		roundaboutReservations: []roundaboutReservation{{
			approaches: [4]roundaboutApproachSlot{{
				forward: sim.Vec2{X: 1},
				owner:   outer,
			}},
		}},
	}
	for tick := 0; tick < 5; tick++ {
		tm.updateRoundaboutReservations(inner, w, 0, 52)
		if approachOwner(tm, sim.Vec2{X: 1}) != inner || inner.RoundaboutAdmissionHold != nil || outer.RoundaboutAdmissionHold == nil {
			t.Fatal("outer nose lead blocked the inner entrance")
		}
	}
}
