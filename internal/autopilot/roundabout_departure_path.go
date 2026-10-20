package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A circular tangent can differ almost ninety degrees from the outgoing road.
// Replace that kink before the bus reaches it, with a body-clear continuous curve.
func smoothRoundaboutDepartures(w *sim.World, path []sim.Vec2, lw float64) []sim.Vec2 {
	if w == nil {
		return path
	}
	validator := AutopilotDriver{pathWorld: w}
	for ri := range w.Roundabouts {
		rb := &w.Roundabouts[ri]
		for i := 1; i < len(path)-1; i++ {
			if path[i].Distance(rb.Center) > rb.Radius+15 || path[i+1].Distance(rb.Center) <= rb.Radius+15 {
				continue
			}
			outgoing := path[i+1].Sub(path[i]).Normalize()
			if math.Max(math.Abs(outgoing.X), math.Abs(outgoing.Y)) < .99 {
				continue
			}
			ew := math.Abs(outgoing.X) > .5
			coord := rb.Center.Y
			if !ew {
				coord = rb.Center.X
			}
			if w.GetRoadLanes(coord, ew, rb.Center.Add(outgoing.Mul(rb.Radius+200))) != 2 {
				continue
			}
			chosen, curve := findRoundaboutDeparture(&validator, rb, path, i, outgoing, lw)
			if curve != nil {
				next := make([]sim.Vec2, 0, len(path)+len(curve))
				next = append(next, path[:chosen]...)
				next = append(next, curve...)
				next = append(next, path[i+1:]...)
				path = dedupePath(next)
				i = chosen + len(curve) - 1
			}
		}
	}
	return path
}

// Search progressively earlier tangents and longer joins. All candidates use
// the same curvature and inflated-body checks as runtime exit planning.
func findRoundaboutDeparture(ad *AutopilotDriver, rb *sim.Roundabout, path []sim.Vec2, exit int, outgoing sim.Vec2, lw float64) (int, []sim.Vec2) {
	exitAngle := math.Atan2(path[exit].Y-rb.Center.Y, path[exit].X-rb.Center.X)
	for start := exit - 1; start > 0; start-- {
		if path[start].Distance(rb.Center) > rb.Radius+15 {
			break
		}
		angle := math.Atan2(path[start].Y-rb.Center.Y, path[start].X-rb.Center.X)
		advance := sim.NormalizeAngle(angle - exitAngle)
		if advance < .6 {
			continue
		}
		if advance > 1.4 {
			break
		}
		bus := sim.NewBus(path[start].X, path[start].Y, angle-math.Pi/2)
		for _, join := range []float64{140, 190, 240, 300} {
			end := path[exit].Add(outgoing.Mul(math.Min(join, path[exit].Distance(path[exit+1])*.8)))
			if curve := findDepartureCurve(ad, bus, rb, lw, end, outgoing); curve != nil {
				return start, curve
			}
		}
	}
	return -1, nil
}

func findDepartureCurve(ad *AutopilotDriver, bus *sim.Bus, rb *sim.Roundabout, lw float64, end, outgoing sim.Vec2) []sim.Vec2 {
	for _, first := range []float64{60, 90, 120, 130, 140, 150, 180} {
		for _, last := range []float64{70, 100, 140, 190, 220, 230, 260} {
			curve := ad.roundaboutExitCurve(bus, rb, nil, lw, bus.Pos, end, outgoing, first, last)
			if curve != nil && departureStaysInLane(bus, rb, curve, outgoing) {
				return curve
			}
		}
	}
	return nil
}

// Preserve the outer corridor until departure and leave enough room for the
// entire body on the outgoing side, including the nose while it is still yawed.
func departureStaysInLane(bus *sim.Bus, rb *sim.Roundabout, curve []sim.Vec2, outgoing sim.Vec2) bool {
	right := sim.Vec2{X: -outgoing.Y, Y: outgoing.X}
	for k := 1; k < len(curve); k++ {
		point := curve[k]
		if point.Distance(rb.Center) < roundaboutOuterLaneRadius(rb)-10 {
			return false
		}
		forward := point.Sub(curve[k-1]).Normalize()
		normal := sim.Vec2{X: -forward.Y, Y: forward.X}
		for _, front := range []float64{-1, 1} {
			for _, side := range []float64{-1, 1} {
				corner := point.Add(forward.Mul(front * bus.Length / 2)).Add(normal.Mul(side * bus.Width / 2)).Sub(rb.Center)
				if corner.Dot(outgoing) > rb.Radius && corner.Dot(right) < 2 {
					return false
				}
			}
		}
	}
	return true
}
