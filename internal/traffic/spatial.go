package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/bits"
	"slices"
)

type trafficCell struct{ x, y int }
type carBucket struct{ ids []int }

type carSpatialIndex struct {
	buckets map[trafficCell]*carBucket
	cells   []trafficCell
	// The bounded dense directory avoids a map lookup for every empty query
	// cell. Buckets retain their identity when this directory grows; sparse or
	// out-of-directory coordinates still use the exact map fallback.
	grid          []*carBucket
	origin        trafficCell
	width, height int
	touched       []*carBucket
	orderBits     []uint64
}

func cellAt(p sim.Vec2) trafficCell {
	return trafficCell{int(math.Floor(p.X / 512)), int(math.Floor(p.Y / 512))}
}

func (s *carSpatialIndex) lookup(k trafficCell) *carBucket {
	x, y := k.x-s.origin.x, k.y-s.origin.y
	if uint(x) < uint(s.width) && uint(y) < uint(s.height) {
		return s.grid[x*s.height+y]
	}
	return s.buckets[k]
}

func (s *carSpatialIndex) add(k trafficCell, id int) {
	b := s.lookup(k)
	if b == nil {
		b = &carBucket{}
		s.buckets[k] = b
		x, y := k.x-s.origin.x, k.y-s.origin.y
		if uint(x) < uint(s.width) && uint(y) < uint(s.height) {
			s.grid[x*s.height+y] = b
		}
	}
	if len(b.ids) == 0 {
		s.touched = append(s.touched, b)
	}
	b.ids = append(b.ids, id)
}

func (s *carSpatialIndex) build(cars []*sim.TrafficCar) {
	if s.buckets == nil {
		s.buckets = make(map[trafficCell]*carBucket)
	}
	for _, b := range s.touched {
		b.ids = b.ids[:0]
	}
	s.touched = s.touched[:0]
	if cap(s.cells) < len(cars) {
		s.cells = make([]trafficCell, len(cars))
	} else {
		s.cells = s.cells[:len(cars)]
	}
	lo, hi := trafficCell{}, trafficCell{}
	for i, c := range cars {
		k := cellAt(c.Pos)
		s.cells[i] = k
		if i == 0 {
			lo, hi = k, k
		} else {
			lo.x = min(lo.x, k.x)
			lo.y = min(lo.y, k.y)
			hi.x = max(hi.x, k.x)
			hi.y = max(hi.y, k.y)
		}
	}
	if len(cars) > 0 && (s.width == 0 || lo.x < s.origin.x || lo.y < s.origin.y || hi.x >= s.origin.x+s.width || hi.y >= s.origin.y+s.height) {
		// Padding prevents reallocating the directory on normal cell crossings.
		lo.x -= 4
		lo.y -= 4
		hi.x += 4
		hi.y += 4
		if s.width > 0 {
			lo.x = min(lo.x, s.origin.x)
			lo.y = min(lo.y, s.origin.y)
			hi.x = max(hi.x, s.origin.x+s.width-1)
			hi.y = max(hi.y, s.origin.y+s.height-1)
		}
		width, height := hi.x-lo.x+1, hi.y-lo.y+1
		// Custom sparse maps must not allocate a world-sized dense directory.
		if width > 0 && height > 0 && width <= 256 && height <= 256 {
			s.origin, s.width, s.height = lo, width, height
			s.grid = make([]*carBucket, width*height)
			for k, b := range s.buckets {
				x, y := k.x-lo.x, k.y-lo.y
				if uint(x) < uint(width) && uint(y) < uint(height) {
					s.grid[x*height+y] = b
				}
			}
		}
	}
	for i, k := range s.cells {
		s.add(k, i)
	}
}

func (s *carSpatialIndex) move(i int, pos sim.Vec2) {
	next, old := cellAt(pos), s.cells[i]
	if old == next {
		return
	}
	b := s.lookup(old)
	for j, id := range b.ids {
		if id == i {
			b.ids[j] = b.ids[len(b.ids)-1]
			b.ids = b.ids[:len(b.ids)-1]
			break
		}
	}
	s.add(next, i)
	s.cells[i] = next
}
func (s *carSpatialIndex) query(pos sim.Vec2, radius float64, dst []int) []int {
	dst = s.queryUnsorted(pos, radius, dst)
	// Preserve the original sequential traffic priority and braking order.
	if len(dst) < 24 {
		slices.Sort(dst)
		return dst
	}
	words := (len(s.cells) + 63) / 64
	if cap(s.orderBits) < words {
		s.orderBits = make([]uint64, words)
	} else {
		s.orderBits = s.orderBits[:words]
		clear(s.orderBits)
	}
	for _, id := range dst {
		s.orderBits[id/64] |= uint64(1) << uint(id%64)
	}
	dst = dst[:0]
	for word, value := range s.orderBits {
		for value != 0 {
			bit := bits.TrailingZeros64(value)
			dst = append(dst, word*64+bit)
			value &= value - 1
		}
	}
	return dst
}

// Junction queries add committed turns before performing their one stable-ID
// sort. Other perception/collision queries continue to use query above.
func (s *carSpatialIndex) queryUnsorted(pos sim.Vec2, radius float64, dst []int) []int {
	dst = dst[:0]
	lo, hi := cellAt(pos.Sub(sim.Vec2{X: radius, Y: radius})), cellAt(pos.Add(sim.Vec2{X: radius, Y: radius}))
	for x := lo.x; x <= hi.x; x++ {
		for y := lo.y; y <= hi.y; y++ {
			if b := s.lookup(trafficCell{x, y}); b != nil {
				dst = append(dst, b.ids...)
			}
		}
	}
	return dst
}
func localCarQueryRadius(car *sim.TrafficCar) float64 {
	// Cover stopping distance, passing runways, fast rear arrivals and wide
	// junction sweeps. This is only a broad phase; all existing rules still run.
	speed := math.Max(math.Abs(car.Speed), car.TargetSpeed)
	return math.Max(1800, speed*speed/(2*220)+speed*2+300)
}

func (tm *TrafficManager) localCars(car *sim.TrafficCar, radius float64) []*sim.TrafficCar {
	tm.nearbyIndices = tm.spatial.query(car.Pos, radius, tm.nearbyIndices)
	tm.nearbyCars = tm.nearbyCars[:0]
	for _, i := range tm.nearbyIndices {
		tm.nearbyCars = append(tm.nearbyCars, tm.Cars[i])
	}
	return tm.nearbyCars
}

// ResolveCarCollisions visits nearby pairs in the original i/j order. Requery
// after a push so collision chains crossing cell boundaries are not missed.
func (tm *TrafficManager) ResolveCarCollisions(density int) {
	cars := tm.GetActiveCars(density)
	if len(cars) <= 64 || tm.disableSpatial {
		for i := range cars {
			for j := i + 1; j < len(cars); j++ {
				sim.ResolveCarCarSeparation(cars[i], cars[j])
			}
		}
		return
	}
	tm.spatial.build(cars)
	maxLength := 0.
	for _, c := range cars {
		maxLength = math.Max(maxLength, c.Length)
	}
	for i, car := range cars {
		next := i + 1
		for next < len(cars) {
			tm.nearbyIndices = tm.spatial.query(car.Pos, .6*(car.Length+maxLength), tm.nearbyIndices)
			pushed := false
			for _, j := range tm.nearbyIndices {
				if j < next {
					continue
				}
				next = j + 1
				if sim.ResolveCarCarSeparation(car, cars[j]) {
					tm.spatial.move(i, car.Pos)
					tm.spatial.move(j, cars[j].Pos)
					pushed = true
					break
				}
			}
			if !pushed {
				break
			}
		}
	}
}
