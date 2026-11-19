package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRoundaboutStallTimeoutDefersFrozenOwnerAndFreesSameApproach(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	if len(w.Roundabouts) == 0 {
		t.Fatal("expected at least one roundabout in metropolis world")
	}
	rb := w.Roundabouts[0]

	// Stopped articulated bus at the West approach yield line.
	bus := sim.NewBus(rb.Center.X-rb.YieldLineDistance()-50, rb.Center.Y+1.5*lw, 0)
	bus.SetModel(sim.BusArticulated)
	bus.Speed = 0
	bus.RoundaboutEntry = &sim.BusRoundaboutEntry{
		Center:    rb.Center,
		Forward:   sim.Vec2{X: 1, Y: 0},
		Remaining: 0.3,
	}

	// Car waiting on the SAME West approach (adjacent inner lane) vs OTHER North approach.
	westFwd := sim.Vec2{X: 1, Y: 0}
	westWaitCar := &sim.TrafficCar{
		ID:          101,
		Pos:         sim.Vec2{X: rb.Center.X - rb.YieldLineDistance() - 25, Y: rb.Center.Y + 0.5*lw},
		Heading:     0,
		IsEW:        true,
		LaneIndex:   0,
		Length:      46,
		Width:       22,
		TargetSpeed: 200,
		Speed:       0,
	}
	northFwd := sim.Vec2{X: 0, Y: 1}
	northWaitCar := &sim.TrafficCar{
		ID:          102,
		Pos:         sim.Vec2{X: rb.Center.X - 1.5*lw, Y: rb.Center.Y - rb.YieldLineDistance() - 25},
		Heading:     math.Pi / 2,
		IsEW:        false,
		LaneIndex:   1,
		Length:      46,
		Width:       22,
		TargetSpeed: 200,
		Speed:       0,
	}

	tm := &TrafficManager{Cars: []*sim.TrafficCar{westWaitCar, northWaitCar}}
	tm.updateRoundaboutReservationsAt(1.0/60.0, bus, w, len(tm.Cars), lw)

	if tm.roundaboutReservedForCar(rb, northWaitCar, northFwd, lw) {
		t.Fatal("west bus must never block north approach waiting car")
	}
	if !tm.roundaboutReservedForCar(rb, westWaitCar, westFwd, lw) {
		t.Fatal("expected west bus to initially reserve same west approach against adjacent car")
	}

	// Advance past the 4.5s stall timeout while the reservation owner never moves.
	for step := 0; step < 20; step++ {
		tm.updateRoundaboutReservationsAt(0.25, bus, w, len(tm.Cars), lw)
	}

	deferred := false
	for k := range tm.roundaboutReservations[0].approaches {
		if tm.roundaboutReservations[0].approaches[k].isDeferred(bus) {
			deferred = true
			break
		}
	}
	if !deferred {
		t.Fatal("expected stalled un-entered bus owner to be deferred after 4.5s")
	}
	if tm.roundaboutReservedForCar(rb, westWaitCar, westFwd, lw) {
		t.Fatal("expected waiting same-approach car to be unblocked after owner stall deferral")
	}
}

func TestRoundaboutEntryPrecedesRequiresAbreastWithin12Px(t *testing.T) {
	center := sim.Vec2{}
	intent := func() *sim.BusRoundaboutEntry {
		return &sim.BusRoundaboutEntry{
			Center:    center,
			Forward:   sim.Vec2{X: 1, Y: 0},
			Remaining: 0.3,
		}
	}
	outerLeader := sim.NewBus(-384, 78, 0)
	outerLeader.SetModel(sim.BusArticulated)
	outerLeader.RoundaboutEntry = intent()

	innerTrailer := sim.NewBus(-410.8, 26, 0)
	innerTrailer.SetModel(sim.BusArticulated)
	innerTrailer.RoundaboutEntry = intent()

	// Outer leader 26.8px ahead of trailing inner bus (>12px) keeps precedence;
	// trailing inner bus must not hold or steal priority from it.
	if !roundaboutEntryPrecedes(outerLeader, innerTrailer, center, 384.0, 410.8) {
		t.Fatal("outer leader >12px ahead of trailing inner bus must precede it")
	}
	if roundaboutEntryPrecedes(innerTrailer, outerLeader, center, 410.8, 384.0) {
		t.Fatal("trailing inner bus >12px behind must not precede outer leader")
	}

	// Abreast inner bus (<12px difference along approach) closer to inner lane precedes.
	innerAbreast := sim.NewBus(-388, 26, 0)
	innerAbreast.SetModel(sim.BusArticulated)
	innerAbreast.RoundaboutEntry = intent()
	if !roundaboutEntryPrecedes(innerAbreast, outerLeader, center, 388.0, 384.0) {
		t.Fatal("abreast inner bus within 12px should precede")
	}
}
