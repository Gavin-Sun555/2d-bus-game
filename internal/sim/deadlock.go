package sim

import "math"

// DefaultSignalCycle includes both through phases and both protected left turns.
const DefaultSignalCycle = 37.0

// CycleDuration follows the same phase graph as the running signal controller.
func (inter *TrafficIntersection) CycleDuration() float64 {
	copy := *inter
	if copy.GreenDuration <= 0 {
		copy.GreenDuration = 8
	}
	if copy.YellowDuration <= 0 {
		copy.YellowDuration = 2.5
	}
	if copy.LeftGreenDuration <= 0 {
		copy.LeftGreenDuration = 5.5
	}
	copy.Phase = PhaseEWGreen
	total := 0.0
	for i := 0; i < 8; i++ {
		duration, next, valid := copy.phaseTransition()
		if !valid {
			return DefaultSignalCycle
		}
		total += duration
		if next == PhaseEWGreen {
			return total
		}
		copy.Phase = next
	}
	return DefaultSignalCycle
}

// DeadlockTimeoutAt uses the nearest intersection's complete signal cycle.
func (tls *TrafficLightSystem) DeadlockTimeoutAt(pos Vec2) float64 {
	cycle, nearest := DefaultSignalCycle, math.Inf(1)
	if tls != nil {
		for _, inter := range tls.Intersections {
			if d := pos.Distance(inter.Center); d < nearest {
				nearest, cycle = d, inter.CycleDuration()
			}
		}
	}
	return 2 * cycle
}

// StationaryTracker prevents intermittent stops or slow creeping from adding up
// to a continuous stationary jam. A two-pixel tolerance absorbs physics jitter.
type StationaryTracker struct {
	anchor      Vec2
	initialized bool
}

func (s *StationaryTracker) Update(timer, dt float64, pos Vec2, speed float64, jammed bool) float64 {
	moved := s.initialized && pos.Distance(s.anchor) > 2
	if !s.initialized || !jammed || math.Abs(speed) >= 3 || moved {
		s.anchor, s.initialized = pos, true
		if !jammed || math.Abs(speed) >= 3 || moved {
			return 0
		}
	}
	return timer + math.Max(0, dt)
}
