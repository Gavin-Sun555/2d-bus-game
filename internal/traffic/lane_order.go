package traffic

import (
	"math"
	"slices"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type laneOrderKey struct{ axis, band int }
type laneOrderBucket struct{ ids []int }

// A conservative lane-strip index, not a replacement for lane membership or
// physics. Each car is represented on both road axes, including straddling,
// turning and crossing cars. Immutable start-of-tick coordinates keep each
// longitudinal order valid while the simulation advances sequentially.
// Queries expand by the maximum *observed* displacement of already-updated
// cars, then read live poses. No future-motion estimate or stale leader is used.
type laneOrderIndex struct {
	order                            [2][]int
	buckets                          map[laneOrderKey]*laneOrderBucket
	used                             []*laneOrderBucket
	positions                        []sim.Vec2
	width, halfBound, movedX, movedY float64
	ready                            bool
}

func (s *laneOrderIndex) build(cars []*sim.TrafficCar, width float64) {
	s.ready = false
	for _, b := range s.used {
		b.ids = b.ids[:0]
	}
	s.used = s.used[:0]
	if s.buckets == nil {
		s.buckets = make(map[laneOrderKey]*laneOrderBucket)
	}
	if width <= 0 || math.IsNaN(width) || math.IsInf(width, 0) {
		return
	}
	s.width, s.halfBound, s.movedX, s.movedY = width, 0, 0, 0
	if cap(s.positions) < len(cars) {
		s.positions = make([]sim.Vec2, len(cars))
	} else {
		s.positions = s.positions[:len(cars)]
	}
	for i, c := range cars {
		if !laneOrderSafeCar(c) {
			return
		}
		s.positions[i] = c.Pos
		// L1 bound covers every possible heading, including turns later this tick.
		s.halfBound = math.Max(s.halfBound, (c.Length+c.Width)/2)
	}
	for axis := 0; axis < 2; axis++ {
		if len(s.order[axis]) != len(cars) {
			if cap(s.order[axis]) < len(cars) {
				s.order[axis] = make([]int, len(cars))
			} else {
				s.order[axis] = s.order[axis][:len(cars)]
			}
			for i := range cars {
				s.order[axis][i] = i
			}
		}
		// Reuse a permutation, never old vehicle identity or state. All current
		// slots are still inserted, including after population replacement.
		for _, i := range s.order[axis] {
			_, across := laneCoordinates(s.positions[i], axis)
			key := laneOrderKey{axis, int(math.Floor(across / width))}
			b := s.buckets[key]
			if b == nil {
				b = &laneOrderBucket{}
				s.buckets[key] = b
			}
			if len(b.ids) == 0 {
				s.used = append(s.used, b)
			}
			b.ids = append(b.ids, i)
		}
	}
	s.order[0], s.order[1] = s.order[0][:0], s.order[1][:0]

	for key, b := range s.buckets {
		if len(b.ids) == 0 {
			continue
		}
		slices.SortFunc(b.ids, func(a, b int) int {
			x, _ := laneCoordinates(s.positions[a], key.axis)
			y, _ := laneCoordinates(s.positions[b], key.axis)
			if x < y {
				return -1
			}
			if x > y {
				return 1
			}
			return a - b
		})
		s.order[key.axis] = append(s.order[key.axis], b.ids...)
	}
	s.ready = true
}

func laneOrderSafeCar(c *sim.TrafficCar) bool {
	return c != nil && math.Abs(c.Pos.X) < 1e8 && math.Abs(c.Pos.Y) < 1e8 && c.Length > 0 && c.Width > 0 && c.Length < 1e5 && c.Width < 1e5
}
func laneCoordinates(p sim.Vec2, axis int) (along, across float64) {
	if axis == 0 {
		return p.X, p.Y
	}
	return p.Y, p.X
}
func (s *laneOrderIndex) observe(i int, c *sim.TrafficCar) {
	if !s.ready {
		return
	}
	if !laneOrderSafeCar(c) {
		s.ready = false
		return
	}
	s.movedX = math.Max(s.movedX, math.Abs(c.Pos.X-s.positions[i].X))
	s.movedY = math.Max(s.movedY, math.Abs(c.Pos.Y-s.positions[i].Y))
	s.halfBound = math.Max(s.halfBound, (c.Length+c.Width)/2)
}

// Return only candidates; the original ordered narrow-phase following code
// still decides speed and may apply its original emergency stop side effect.
func (tm *TrafficManager) followingCars(f carFrame) []*sim.TrafficCar {
	s := &tm.laneOrder
	activeCorridor := f.car.JunctionCorridor != nil && !(f.car.JunctionCorridor.Remaining <= 0)
	if tm.disableLaneOrder || !tm.spatialActive || !s.ready || f.candidates != nil || f.localCarRadius <= 0 || f.activeCount != len(s.positions) || f.car.JunctionTurn != nil || activeCorridor || f.car.InRoundabout || !laneOrderSafeCar(f.car) {
		return tm.carsForFrame(f)
	}
	axis, positive := 0, true
	switch f.car.Heading {
	case 0:
	case math.Pi, -math.Pi:
		positive = false
	case math.Pi / 2:
		axis = 1
	case -math.Pi / 2:
		axis = 1
		positive = false
	default:
		return tm.carsForFrame(f)
	}
	along, across := laneCoordinates(f.car.Pos, axis)
	longMove, latMove := s.movedX, s.movedY
	if axis == 1 {
		longMove, latMove = latMove, longMove
	}
	reach := f.car.Width/2 + 5 + s.halfBound + latMove + 1
	first, last := int(math.Floor((across-reach)/s.width)), int(math.Floor((across+reach)/s.width))
	if last-first > 16 || longMove > 4096 {
		return tm.carsForFrame(f)
	}
	lo, hi := cellAt(f.car.Pos.Sub(sim.Vec2{X: f.localCarRadius, Y: f.localCarRadius})), cellAt(f.car.Pos.Add(sim.Vec2{X: f.localCarRadius, Y: f.localCarRadius}))
	lower, upper := float64(lo.x)*512, float64(hi.x+1)*512
	if axis == 1 {
		lower, upper = float64(lo.y)*512, float64(hi.y+1)*512
	}
	if positive {
		lower = math.Max(lower, along+f.car.Length/2-1)
	} else {
		upper = math.Min(upper, along-f.car.Length/2+1)
	}
	lower -= longMove
	upper += longMove
	ids := tm.followingIndices[:0]
	for band := first; band <= last; band++ {
		b := s.buckets[laneOrderKey{axis, band}]
		if b == nil {
			continue
		}
		start := sort.Search(len(b.ids), func(j int) bool { x, _ := laneCoordinates(s.positions[b.ids[j]], axis); return x >= lower })
		for _, id := range b.ids[start:] {
			x, _ := laneCoordinates(s.positions[id], axis)
			if x > upper {
				break
			}
			p := cellAt(tm.Cars[id].Pos)
			if p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y {
				ids = append(ids, id)
			}
		}
	}
	// Preserve the original slot order, not geometric nearest-first priority.
	slices.Sort(ids)
	tm.followingIndices = ids
	tm.followingCandidates = tm.followingCandidates[:0]
	for _, id := range ids {
		tm.followingCandidates = append(tm.followingCandidates, tm.Cars[id])
	}
	return tm.followingCandidates
}
