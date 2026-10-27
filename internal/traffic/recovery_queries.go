package traffic

import (
	"math"
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Only candidate geometry and position-dependent timeout values are reused.
// Jam state, yielding, timers, and the first despawn slot remain live every tick.
type recoveryQueries struct {
	centers   []sim.Vec2
	cells     map[trafficCell][]sim.Vec2
	signals   []recoverySignal
	timeouts  []recoveryTimeout
	maxLength float64
	ids       []int
}
type recoverySignal struct {
	center sim.Vec2
	cycle  float64
}
type recoveryTimeout struct {
	pos   sim.Vec2
	value float64
	valid bool
}

func recoveryFinitePoint(p sim.Vec2) bool {
	return math.Abs(p.X) < 1e12 && math.Abs(p.Y) < 1e12
}

func (tm *TrafficManager) prepareRecoveryQueries(count int, w *sim.World, tls *sim.TrafficLightSystem, ready bool) bool {
	if tm.disableRecoveryQueries || tm.disableSpatial || count <= 64 {
		return false
	}
	q := &tm.recoveryQueries
	q.maxLength = 0
	for _, c := range tm.Cars[:count] {
		if !recoveryFinitePoint(c.Pos) {
			return false
		}
		q.maxLength = math.Max(q.maxLength, c.Length)
	}
	if math.IsNaN(q.maxLength) || math.IsInf(q.maxLength, 0) || q.maxLength > 8192 {
		return false
	}
	if !ready || len(tm.spatial.cells) != count {
		tm.spatial.build(tm.Cars[:count])
	}
	var centers []sim.Vec2
	if w != nil {
		centers = w.Intersections
	}
	for _, p := range centers {
		if !recoveryFinitePoint(p) {
			return false
		}
	}
	if q.cells == nil || !slices.Equal(q.centers, centers) {
		q.centers = append(q.centers[:0], centers...)
		q.cells = make(map[trafficCell][]sim.Vec2)
		for _, p := range centers {
			if recoveryFinitePoint(p) {
				k := cellAt(p)
				q.cells[k] = append(q.cells[k], p)
			}
		}
	}
	n := 0
	if tls != nil {
		n = len(tls.Intersections)
	}
	changed := len(q.signals) != n
	if cap(q.signals) < n {
		q.signals = make([]recoverySignal, n)
	} else {
		q.signals = q.signals[:n]
	}
	for i := 0; i < n; i++ {
		inter := tls.Intersections[i]
		if inter == nil {
			// Earlier entries may have changed before this fallback. Do not
			// retain memoized timeouts if the public slice is later repaired.
			clear(q.timeouts[:cap(q.timeouts)])
			return false
		}
		key := recoverySignal{inter.Center, inter.CycleDuration()}
		if q.signals[i] != key {
			changed = true
			q.signals[i] = key
		}
	}
	if cap(q.timeouts) < count {
		q.timeouts = make([]recoveryTimeout, count)
	} else {
		q.timeouts = q.timeouts[:count]
	}
	if changed {
		clear(q.timeouts[:cap(q.timeouts)])
	}
	return true
}

func (q *recoveryQueries) nearIntersection(pos sim.Vec2) bool {
	lo, hi := cellAt(pos.Sub(sim.Vec2{X: 130.000001, Y: 130.000001})), cellAt(pos.Add(sim.Vec2{X: 130.000001, Y: 130.000001}))
	for x := lo.x; x <= hi.x; x++ {
		for y := lo.y; y <= hi.y; y++ {
			for _, p := range q.cells[trafficCell{x, y}] {
				if pos.Distance(p) < 130 {
					return true
				}
			}
		}
	}
	return false
}

func (tm *TrafficManager) recoveryCandidates(car *sim.TrafficCar, indexed bool) ([]int, bool) {
	if !indexed {
		return nil, false
	}
	q := &tm.recoveryQueries
	radius := (car.Length+q.maxLength)*.5 + 35
	if radius < 0 || math.IsNaN(radius) || math.IsInf(radius, 0) || radius > 8192 {
		return nil, false
	}
	// Broad phase only; preserve the old Distance comparison at the boundary.
	radius += 1e-7 * (1 + radius)
	q.ids = tm.spatial.query(car.Pos, radius, q.ids)
	return q.ids, true
}

func (q *recoveryQueries) timeoutAt(slot int, pos sim.Vec2) float64 {
	memo := &q.timeouts[slot]
	if memo.valid && memo.pos == pos {
		return memo.value
	}
	cycle, nearest := sim.DefaultSignalCycle, math.Inf(1)
	// A miss retains the original exact nearest search and first-slot tie rule.
	// Only signal-cycle geometry is cached, never permission to start moving.
	for _, inter := range q.signals {
		if d := pos.Distance(inter.center); d < nearest {
			nearest, cycle = d, inter.cycle
		}
	}
	*memo = recoveryTimeout{pos: pos, value: 2 * cycle, valid: true}
	return memo.value
}
