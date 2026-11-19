package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Only immutable curve geometry is memoized. Admission and separation against
// other vehicles are still evaluated from their current bodies every tick.
// Compare every coordinate bit, including signed zero, before reusing a sweep.
type turnExitGeometry struct {
	valid         bool
	length, width uint64
	points        []sim.Vec2
	bodies        []sim.OBB
}

func (m *turnExitGeometry) matches(points []sim.Vec2, length, width float64) bool {
	if !m.valid || m.length != math.Float64bits(length) || m.width != math.Float64bits(width) || len(m.points) != len(points) {
		return false
	}
	for i, p := range points {
		if math.Float64bits(p.X) != math.Float64bits(m.points[i].X) || math.Float64bits(p.Y) != math.Float64bits(m.points[i].Y) {
			return false
		}
	}
	return true
}
func (m *turnExitGeometry) build(points []sim.Vec2, length, width float64) []sim.OBB {
	if !m.matches(points, length, width) {
		m.bodies = sim.AppendPathBodies(m.bodies[:0], points, length, width)
		m.points = append(m.points[:0], points...)
		m.length, m.width, m.valid = math.Float64bits(length), math.Float64bits(width), true
	}
	return m.bodies
}

type turnExitGeometryScratch struct {
	current  turnExitGeometry
	extended [4]turnExitGeometry
	next     int
	points   []sim.Vec2
	outer    []sim.OBB
}

func (s *turnExitGeometryScratch) extendedBodies(curve []sim.Vec2, end sim.Vec2, length, width float64) []sim.OBB {
	s.points = append(s.points[:0], curve...)
	s.points[len(s.points)-1] = end
	for i := range s.extended {
		m := &s.extended[i]
		if m.matches(s.points, length, width) {
			return m.bodies
		}
	}
	m := &s.extended[s.next]
	s.next = (s.next + 1) % len(s.extended)
	return m.build(s.points, length, width)
}
