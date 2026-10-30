package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
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
					pos := rb.Center.Add(u.Mul(215)).Add(r.Mul(50))
					owner := sim.NewBus(pos.X, pos.Y, heading)
					owner.SetModel(sim.BusArticulated)
					owner.Speed = 90
					owner.TrailerHeading = heading
					owner.TrailerInitialized = true
					tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{ID: 14, Bus: waiting}}}, roundaboutReservations: []roundaboutReservation{{center: rb.Center, owner: owner, entered: true}}}
					tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, lw)
					if waiting.RoundaboutAdmissionHold != nil {
						t.Fatal("remote reserved owner blocked clear entrance")
					}
					if !tm.roundaboutReserved(rb.Center) {
						t.Fatal("car/long bus reservation was removed")
					}
				})
			}
		}
	}
}
func TestReservedEntryRetainsTailAndUnknownGeometry(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	u := sim.Vec2{X: 1}
	lw := 52.
	outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
	curve := sim.RoundaboutEntryJoin(rb, u, 78, outer, lw)
	p, _ := sim.RoundaboutEntryPoint(curve, 0)
	waiting := sim.NewBus(p.X, p.Y, 0)
	waiting.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: u, Remaining: .3}
	// Front section is past the local join; only the trailing section remains
	// beside the incoming curve. A front-only test would incorrectly release.
	owner := sim.NewBus(-90, 305, math.Pi/2)
	owner.Speed = 90
	if !shortBusClearsReservedEntry(waiting, owner, rb, u, lw) {
		t.Fatal("fixture front section has not actually cleared")
	}
	owner.SetModel(sim.BusArticulated)
	owner.TrailerHeading = math.Pi / 2
	owner.TrailerInitialized = true
	if !reservedOwnerLeaving(owner, rb) {
		t.Fatal("fixture owner is not departing")
	}
	if shortBusClearsReservedEntry(waiting, owner, rb, u, lw) {
		t.Fatal("owner's trailing section ignored")
	}
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: waiting}}}, roundaboutReservations: []roundaboutReservation{{owner: owner, entered: true}}}
	tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, lw)
	if waiting.RoundaboutAdmissionHold == nil {
		t.Fatal("owner tail across mouth lost admission hold")
	}
	// A short owner also retains local body protection.
	owner.SetModel(sim.BusStandard)
	p, tangent := sim.RoundaboutEntryPoint(curve, .2)
	owner.Pos = p
	owner.Heading = math.Atan2(tangent.Y, tangent.X)
	if shortBusClearsReservedEntry(waiting, owner, rb, u, lw) {
		t.Fatal("occupied entrance released")
	}
	owner.Pos = sim.Vec2{X: 215, Y: 50}
	waiting.SetModel(sim.BusArticulated)
	if shortBusClearsReservedEntry(waiting, owner, rb, u, lw) {
		t.Fatal("second long entrant received exception")
	}
	waiting.SetModel(sim.BusStandard)
	waiting.Heading = math.Pi / 2
	if shortBusClearsReservedEntry(waiting, owner, rb, u, lw) {
		t.Fatal("cross-lane entrant received exception")
	}
}
func TestShortBusWaitsWhileReservationDrains(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	owner := reservationBus(-500, sim.BusArticulated)
	waiting := sim.NewBus(335, -78, math.Pi)
	waiting.RoundaboutEntry = &sim.BusRoundaboutEntry{Forward: sim.Vec2{X: -1}, Remaining: .3}
	tm := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: waiting}}}, roundaboutReservations: []roundaboutReservation{{owner: owner}}}
	tm.updateRoundaboutReservations(owner, &sim.World{Roundabouts: []sim.Roundabout{rb}}, 0, 52)
	if waiting.RoundaboutAdmissionHold == nil {
		t.Fatal("reservation released before owner admission")
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
