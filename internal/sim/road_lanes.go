package sim

import "math"

// LaneCount resolves legacy roads that specify only their type.
func (r RoadSegment) LaneCount() int {
	if r.Lanes > 0 {
		return r.Lanes
	}
	switch r.Type {
	case RoadStreet2Lane:
		return 2
	case RoadExpressway6Lane:
		return 6
	case RoadBoulevard8Lane, RoadBusBoulevard8Lane:
		return 8
	default:
		return 4
	}
}

func (r RoadSegment) HasBusLane() bool { return r.Type == RoadBusBoulevard8Lane }

// GeneralTrafficLanes excludes the reserved curb lane from spawning and cruising.
func (r RoadSegment) GeneralTrafficLanes() int {
	n := max(1, r.LaneCount()/2)
	if r.HasBusLane() {
		n--
	}
	return n
}

// RoadAt matches a specific segment, including roads whose type changes at a junction.
func (w *World) RoadAt(coord float64, isEW bool, pos Vec2) (RoadSegment, bool) {
	if w == nil {
		return RoadSegment{}, false
	}
	for _, r := range w.Roads {
		ew := math.Abs(r.P1.Y-r.P2.Y) < 1
		c, along, a, b := r.P1.Y, pos.X, r.P1.X, r.P2.X
		if !ew {
			c, along, a, b = r.P1.X, pos.Y, r.P1.Y, r.P2.Y
		}
		if ew == isEW && math.Abs(c-coord) < 25 && along >= math.Min(a, b)-1 && along <= math.Max(a, b)+1 {
			return r, true
		}
	}
	return RoadSegment{}, false
}
