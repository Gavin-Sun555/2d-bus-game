package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

type brtPathSignature struct {
	first  *sim.Vec2
	world  *sim.World
	length int
	lane   float64
	long   bool
}

// Finish a return across the median platform divider before the island begins.
// The route may pass a BRT station that is not one of its scheduled stops.
func clearBRTPlatformTransitions(w *sim.World, path []sim.Vec2, lw float64, long bool) ([]sim.Vec2, bool) {
	if w == nil || !w.CenterBRT {
		return path, false
	}
	changed := false
	for _, stop := range w.Stops {
		if stop == nil || !stop.CenterBRT {
			continue
		}
		u := sim.UnitFromAngle(stop.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		for i := 1; i < len(path); i++ {
			a, b := path[i-1].Sub(stop.RoadCenter), path[i].Sub(stop.RoadCenter)
			aLong, bLong := a.Dot(u), b.Dot(u)
			aLat, bLat := a.Dot(r), b.Dot(r)
			if bLong-aLong < 100 || aLat < lw || bLat >= lw || bLat < lw*.3 || aLat > 2*lw {
				continue
			}
			settle, taper := 80., 220.
			if long {
				settle, taper = 160, 360
			}
			endLong := -sim.BRTPlatformHalfLength - settle
			if aLong >= -sim.BRTPlatformHalfLength || bLong <= endLong || aLong > 0 || bLong > sim.BRTPlatformHalfLength {
				continue
			}
			startLong := endLong - math.Max(taper, bLong-aLong)
			first := i - 1
			for first > 0 && path[first].Sub(stop.RoadCenter).Dot(u) > startLong {
				first--
			}
			if path[first].Sub(stop.RoadCenter).Dot(u) > startLong || math.Abs(path[first].Sub(stop.RoadCenter).Dot(r)-aLat) > 1 {
				continue
			}
			// Only rewrite a straight incoming road; a turn or another lane
			// transition still owns its existing geometry.
			straight := true
			for j := first; j < i; j++ {
				if math.Abs(path[j].Sub(stop.RoadCenter).Dot(r)-aLat) > 1 {
					straight = false
				}
			}
			if !straight {
				continue
			}
			points := append([]sim.Vec2(nil), path[:first+1]...)
			points = append(points, stop.RoadCenter.Add(u.Mul(startLong)).Add(r.Mul(aLat)), stop.RoadCenter.Add(u.Mul(endLong)).Add(r.Mul(bLat)))
			path = dedupePath(append(points, path[i:]...))
			changed = true
			break
		}
	}
	return path, changed
}

func (ad *AutopilotDriver) repairUpcomingBRTTransition(pos sim.Vec2) {
	if len(ad.drivePath) < 2 || ad.pathWorld == nil || !ad.pathWorld.CenterBRT {
		return
	}
	key := brtPathSignature{first: &ad.drivePath[0], world: ad.pathWorld, length: len(ad.drivePath), lane: ad.pathLane, long: ad.articulatedPath}
	if key == ad.brtCheckedPath {
		return
	}
	ad.brtCheckedPath = key
	path, changed := clearBRTPlatformTransitions(ad.pathWorld, ad.drivePath, ad.pathLane, ad.articulatedPath)
	if !changed || len(ad.cumLen) != len(ad.drivePath) {
		return
	}
	common, i := 0., 1
	for ; i < len(path) && i < len(ad.drivePath); i++ {
		if path[i].Distance(ad.drivePath[i]) > .01 {
			break
		}
		common += path[i].Distance(path[i-1])
	}
	if i < len(path) && i < len(ad.drivePath) {
		a, b := path[i].Sub(path[i-1]), ad.drivePath[i].Sub(ad.drivePath[i-1])
		if a.Normalize().Dot(b.Normalize()) > .99999 {
			common += math.Min(a.Length(), b.Length())
		}
	}
	nearest, s := math.Inf(1), 0.
	for j := 1; j < len(ad.drivePath); j++ {
		u, d := projectToSegment(pos, ad.drivePath[j-1], ad.drivePath[j])
		if d < nearest {
			nearest = d
			s = ad.cumLen[j-1] + u*(ad.cumLen[j]-ad.cumLen[j-1])
		}
	}
	if s+64 >= common {
		return
	}
	ad.drivePath = path
	ad.brtCheckedPath.first, ad.brtCheckedPath.length = &path[0], len(path)
	ad.cumLen = make([]float64, len(path))
	for j := 1; j < len(path); j++ {
		ad.cumLen[j] = ad.cumLen[j-1] + path[j].Distance(path[j-1])
	}
}
