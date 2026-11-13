package sim

import "math"

// Turn signals are presentation state. The blink starts lit on each new turn
// and follows driving updates, independent of the time-of-day rate.
type turnSignal struct {
	direction int
	phase     float64
}

func (s *turnSignal) update(direction int, dt float64) {
	if s.direction != direction {
		*s = turnSignal{direction: direction}
	} else if direction != 0 {
		s.phase = math.Mod(s.phase+dt, .8)
	}
}

func (s turnSignal) litSide() int {
	if s.phase >= .4 {
		return 0
	}
	return s.direction
}

func turnDirection(angle, deadZone float64) int {
	switch {
	case angle > deadZone:
		return 1
	case angle < -deadZone:
		return -1
	default:
		return 0
	}
}

// TurnSignalSide is -1 for the left lamps, 1 for the right lamps, or 0 when dark.
func (b *Bus) TurnSignalSide() int        { return b.turnSignal.litSide() }
func (c *TrafficCar) TurnSignalSide() int { return c.turnSignal.litSide() }

// SetNextLaneChangeSignal supplies the controller's signed lateral intent for
// one physics step. An S-shaped lane change keeps its signal while unwinding.
func (b *Bus) SetNextLaneChangeSignal(lateral float64) {
	b.nextLaneSignal = turnDirection(lateral, 2)
}

func (b *Bus) updateTurnSignal(dt, oldHeading float64) {
	direction := b.nextLaneSignal
	b.nextLaneSignal = 0
	if request := b.LaneMergeRequest; direction == 0 && request != nil && !request.Joined {
		right := UnitFromAngle(request.Heading + math.Pi/2)
		direction = turnDirection(request.Target.Sub(b.Pos).Dot(right), 4)
	}
	if direction == 0 && b.Speed > 1 {
		direction = turnDirection(b.SteeringAngle, .035)
		if previous := b.turnSignal.direction; previous != 0 && direction != previous {
			yaw := NormalizeAngle(b.Heading-b.turnSignalHeading) * float64(previous)
			if yaw > .025 && yaw < .6 {
				direction = previous
			} else if math.Abs(yaw) <= .025 && math.Abs(b.SteeringAngle) < .10 {
				direction = 0
			}
		}
	}
	if direction != b.turnSignal.direction {
		b.turnSignalHeading = oldHeading
	}
	b.turnSignal.update(direction, dt)
}

func (c *TrafficCar) UpdateTurnSignal(dt float64) {
	direction := 0
	if turn := c.JunctionTurn; turn != nil {
		if turn.LaneChange || turn.Straight {
			direction = turnDirection(float64(turn.ExitLane-c.LaneIndex), 0)
		} else {
			direction = turnDirection(NormalizeAngle(turn.ExitHeading-turn.EntryHeading), .1)
		}
	} else if curve := c.RoundaboutCurve; curve != nil && curve.Exiting {
		direction = 1
	}
	c.turnSignal.update(direction, dt)
}
