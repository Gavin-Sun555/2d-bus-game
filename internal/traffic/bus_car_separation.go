package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// BusCarSeparations is borrowed for one synchronous collision pass. Rebuild it
// after other collision phases/population edits; never reuse the perception index.
type BusCarSeparations struct {
	index      carSpatialIndex
	cars       []*sim.TrafficCar
	candidates []int
	seen       map[*sim.TrafficCar]struct{}
	maxLength  float64
	indexed    bool
}

func (tm *TrafficManager) BeginBusCarSeparations(cars []*sim.TrafficCar) *BusCarSeparations {
	b := &tm.separations
	b.cars = cars
	b.maxLength = 0
	b.indexed = len(cars) > 64 && !tm.disableBusCarIndex
	if !b.indexed {
		return b
	}
	if b.seen == nil {
		b.seen = make(map[*sim.TrafficCar]struct{}, len(cars))
	} else {
		clear(b.seen)
	}
	for _, car := range cars {
		if car == nil || !separationFinite(car.Pos.X, car.Pos.Y, car.Length, car.Width, car.Heading, car.Speed) || math.Abs(car.Pos.X) > 1e9 || math.Abs(car.Pos.Y) > 1e9 || math.Abs(car.Length) > 8192 {
			b.indexed = false
			return b
		}
		if _, duplicate := b.seen[car]; duplicate {
			b.indexed = false
			return b
		}
		b.seen[car] = struct{}{}
		b.maxLength = math.Max(b.maxLength, math.Abs(car.Length))
	}
	b.index.build(cars)
	return b
}

func separationFinite(values ...float64) bool {
	for _, v := range values {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}

// Resolve retains ascending original slots, requerying whenever an earlier
// collision moves the bus. Car moves update this pass's index immediately.
func (b *BusCarSeparations) Resolve(bus *sim.Bus) {
	if bus == nil {
		return
	}
	radius := (math.Abs(bus.OverallLength()) + b.maxLength) * 1.1
	if !b.indexed || !separationFinite(bus.Pos.X, bus.Pos.Y, radius, bus.Heading, bus.Width, bus.Speed) || radius > 20000 || math.Abs(bus.Pos.X) > 1e9 || math.Abs(bus.Pos.Y) > 1e9 {
		for _, car := range b.cars {
			sim.ResolveBusCarSeparation(bus, car)
		}
		// A fallback may produce non-finite positions in malformed public states.
		// Keep the remainder of this pass on the original full scan as well.
		b.indexed = false
		return
	}
	radius += 1e-7 * (1 + radius + math.Abs(bus.Pos.X) + math.Abs(bus.Pos.Y))
	last := -1
	for {
		b.candidates = b.index.query(bus.Pos, radius, b.candidates)
		moved := false
		for _, slot := range b.candidates {
			if slot <= last {
				continue
			}
			last = slot
			before := bus.Pos
			if sim.ResolveBusCarSeparation(bus, b.cars[slot]) {
				b.index.move(slot, b.cars[slot].Pos)
			}
			if bus.Pos != before {
				moved = true
				break
			}
		}
		if !moved {
			return
		}
	}
}
