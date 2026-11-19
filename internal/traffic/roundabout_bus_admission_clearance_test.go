package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestShortBusCanJoinAwayFromReservedOwner(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{40, 52, 78} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker} {
				t.Run(fmt.Sprintf("h%.2f/lw%.0f/model%d", heading, lw, model), func(t *testing.T) {
					u := sim.UnitFromAngle(heading)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					rb := sim.Roundabout{Center: sim.Vec2{X: 1000, Y: -2000}, Radius: 240, InnerRadius: 120}
					outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
					curve := sim.RoundaboutEntryJoin(rb, u, 1.5*lw, outer, lw)
					start, _ := sim.RoundaboutEntryPoint(curve, 0)
					waiting := sim.NewBus(start.X, start.Y, heading)
					waiting.SetModel(model)
					waiting.RoundaboutEntry = &sim.BusRoundaboutEntry{Center: rb.Center, Forward: u, Remaining: .3}
					// Articulated owner entered from a different approach and already circulates far away.
					otherApproach := sim.Vec2{X: -u.Y, Y: u.X}
					pos := rb.Center.Add(otherApproach.Mul(215)).Add(r.Mul(50))
					owner := sim.NewBus(pos.X, pos.Y, heading+math.Pi/2)
					owner.SetModel(sim.BusArticulated)
					owner.Speed = 90
					tm := &TrafficManager{
						AIBusMgr: &AIBusManager{Buses: []*AIBus{{ID: 14, Bus: waiting}}},
						roundaboutReservations: []roundaboutReservation{{
							center: rb.Center,
							approaches: [4]roundaboutApproachSlot{{
								forward: otherApproach,
								owner:   owner,
								entered: true,
							}},
						}},
					}
					tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, lw)
					if waiting.RoundaboutAdmissionHold != nil {
						t.Fatal("other-approach reserved owner blocked clear entrance")
					}
				})
			}
		}
	}
}

func TestSameApproachTailStillProtectsEnteringBus(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	u := sim.Vec2{X: 1}
	lw := 52.
	outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
	curve := sim.RoundaboutEntryJoin(rb, u, 78, outer, lw)
	p, _ := sim.RoundaboutEntryPoint(curve, 0)
	waiting := sim.NewBus(p.X, p.Y, 0)
	waiting.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: u, Remaining: .3}

	// Articulated owner entered West approach but its trailer still straddles the West mouth.
	owner := sim.NewBus(-170, 78, 0.25)
	owner.SetModel(sim.BusArticulated)
	owner.Speed = 90
	owner.ResetArticulation()
	if !ownerStillAtMouth(owner, rb, u, lw) {
		t.Fatal("fixture owner trailer is not at West entrance mouth")
	}
	tm := &TrafficManager{
		AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: waiting}}},
		roundaboutReservations: []roundaboutReservation{{
			approaches: [4]roundaboutApproachSlot{{
				forward: u,
				owner:   owner,
				entered: true,
			}},
		}},
	}
	tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, lw)
	if waiting.RoundaboutAdmissionHold == nil {
		t.Fatal("same-approach follower lost admission hold while owner trailer still crosses mouth")
	}

	// Once the owner trailer moves past the mouth into the ring, release immediately.
	owner.Pos = sim.Vec2{X: 20, Y: 195}
	owner.Heading = 0
	owner.ResetArticulation()
	tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, lw)
	if waiting.RoundaboutAdmissionHold != nil {
		t.Fatal("same-approach follower still held after owner trailer cleared the mouth")
	}
}

func TestOtherApproachShortBusNeverHeldByWaitingLongBus(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	westOwner := reservationBus(-340, sim.BusArticulated)
	eastWaiting := sim.NewBus(335, -78, math.Pi)
	eastWaiting.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: sim.Vec2{X: -1}, Remaining: .3}
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: eastWaiting}}}}
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
	tm.updateRoundaboutReservations(westOwner, w, 0, 52)
	if eastWaiting.RoundaboutAdmissionHold != nil {
		t.Fatal("east entrance must not be held by west articulated bus")
	}
}

func TestReservedOwnerHandoffRequiresAnActualExit(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	b := sim.NewBus(215, 50, 0)
	b.SetModel(sim.BusArticulated)
	b.Speed = 90
	if !reservedOwnerLeaving(b, rb) {
		t.Fatal("outgoing owner did not hand off")
	}
	b.Heading = math.Pi / 2
	if reservedOwnerLeaving(b, rb) {
		t.Fatal("circulating owner prematurely handed off")
	}
	b.Heading = 0
	b.Speed = 0
	if reservedOwnerLeaving(b, rb) {
		t.Fatal("stationary owner prematurely handed off")
	}
	b.Speed = 90
	b.Pos = sim.Vec2{X: 150}
	if reservedOwnerLeaving(b, rb) {
		t.Fatal("owner still deep in ring prematurely handed off")
	}
}
