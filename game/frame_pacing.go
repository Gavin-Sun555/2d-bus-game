package game

import "time"

const simulationFrame = time.Second / 60

// presentationClock dates the fixed simulation steps against a monotonic wall
// clock. It never discards a physics tick. Rendering trails by 1.5 steps so both
// endpoints are available even on displays that refresh between updates.
type presentationClock struct {
	origin time.Time
	ticks  int64
}

func (c *presentationClock) advance(now time.Time) time.Duration {
	if c.origin.IsZero() {
		c.origin = now
	}
	c.ticks++
	elapsed := time.Duration(c.ticks) * simulationFrame
	// Gently absorb sustained display-vs-60Hz drift (e.g. 59.82/59.94 Hz VSync)
	// before displayTime hits the edge of the snapshot ring, while leaving
	// normal phase jitter and suspension catch-up batches untouched.
	lag := now.Sub(c.origin) - elapsed
	if lag > simulationFrame/2 && lag < time.Second/2 {
		c.origin = c.origin.Add(min((lag-simulationFrame/2)/16, simulationFrame/4))
	} else if lag < -simulationFrame*2 && lag > -time.Second/2 {
		c.origin = c.origin.Add(max((lag+simulationFrame*2)/16, -simulationFrame/4))
	}
	return elapsed
}
func (c *presentationClock) displayTime(now time.Time) time.Duration {
	if c.origin.IsZero() {
		return 0
	}
	// Re-anchor after a suspension only after the engine has completed its
	// catch-up batch. Resetting during that batch would put rendering ahead
	// of all available snapshots for the rest of the session.
	latest := time.Duration(c.ticks) * simulationFrame
	if lag := now.Sub(c.origin) - latest; lag > time.Second/2 || lag < -time.Second/2 {
		c.origin = now.Add(-latest)
	}
	return max(0, now.Sub(c.origin)-simulationFrame*3/2)
}
