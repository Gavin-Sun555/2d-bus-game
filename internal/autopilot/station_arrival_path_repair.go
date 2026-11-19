package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Older curb-left joins could delete the distant start of the station taper.
// Restore just that waypoint, keeping the entire departure/turn prefix and the
// existing service target. This does not move the bus or change its lane offset.
func repairLongStationArrival(path []sim.Vec2, stop *sim.BusStop, lw float64, long bool) ([]sim.Vec2, bool) {
	if stop == nil || len(path) < 3 || lw <= 0 || path[len(path)-1].Distance(stop.FrontBerthPosition()) > 1 {
		return path, false
	}
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	previous, entry, target := path[len(path)-3], path[len(path)-2], path[len(path)-1]
	straight := target.Sub(entry)
	if straight.Dot(u) < 20 || math.Abs(straight.Dot(r)) > .01 {
		return path, false
	}
	gap := entry.Sub(previous)
	shift := previous.Sub(entry).Dot(r)
	// Restrict migration to a long, almost parallel connection from the curb
	// travel lane to the bay; nearby turns and intentional multi-lane joins
	// retain their saved maneuvers.
	if gap.Dot(u) < 800 || math.Abs(shift) < 2 || math.Abs(shift) > lw*1.5 || gap.Normalize().Dot(u) < .99 {
		return path, false
	}
	taper := 170.
	if long {
		taper = 340
	}
	point := entry.Sub(u.Mul(taper)).Add(r.Mul(shift))
	fixed := make([]sim.Vec2, 0, len(path)+1)
	fixed = append(fixed, path[:len(path)-2]...)
	fixed = append(fixed, point, entry, target)
	return fixed, true
}

// Upgrade only an unentered, ordinary long-bus arrival tail. The entry start
// stays in place; the extra straightening room comes from a shorter taper.
func upgradeArticulatedStationRunIn(path []sim.Vec2, stop *sim.BusStop) ([]sim.Vec2, bool) {
	n := len(path)
	if n < 3 || stop == nil || path[n-1].Distance(stop.FrontBerthPosition()) > 1 {
		return path, false
	}
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	straight := path[n-1].Sub(path[n-2])
	if math.Abs(straight.Dot(u)-160) > .01 || math.Abs(straight.Dot(r)) > .01 {
		return path, false
	}
	approach := path[n-1].Sub(path[n-3])
	if approach.Dot(u) < 460 || math.Abs(approach.Dot(r)) < 5 || math.Abs(approach.Dot(r)) > 80 {
		return path, false
	}
	fixed := append([]sim.Vec2(nil), path...)
	fixed[n-2] = path[n-1].Sub(u.Mul(280))
	return fixed, true
}
