package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// The moving-conflict predicate is constant for this synchronous observation.
// Keep every circulating vehicle, including fast vehicles on the far side;
// overflow uses the original scan instead of allocating or dropping a vehicle.
func roundaboutMovingCandidates(f autopilotFrame, rb *sim.Roundabout, dst []int, selection ...[]int) ([]int, bool) {
	count := len(f.obstacles)
	if len(selection) > 0 {
		count = len(selection[0])
	}
	dst = dst[:0]
	for i := 0; i < count; i++ {
		index := i
		if len(selection) > 0 {
			index = selection[0][i]
		}
		ob := f.obstacles[index]
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		if !sim.HasRoundaboutPriority(*rb, body) {
			continue
		}
		if len(dst) == cap(dst) {
			return nil, false
		}
		dst = append(dst, index)
	}
	return dst, true
}

type roundaboutEntryBounds struct {
	lo, hi sim.Vec2
	valid  bool
}

// Cover the whole entry polyline (not just the current center), its before-start
// extension, and the yaw-lag envelope used by this consumer. This range is only
// for the static checks. Future moving traffic uses its independent full set.
func roundaboutStaticEntryRange(f autopilotFrame, points []sim.Vec2, forward sim.Vec2, s float64) roundaboutEntryBounds {
	b := f.bus
	if b == nil || len(points) < 2 || len(points) > 4096 || !(math.Abs(s) <= 1e6) ||
		!(b.Length >= 0 && b.Length <= 1e6 && b.Width >= 0 && b.Width <= 1e6 && math.Abs(b.Heading) <= 1e6) ||
		!entryCoordinateSafe(b.Pos) || !(math.Abs(forward.X) <= 2 && math.Abs(forward.Y) <= 2) {
		return roundaboutEntryBounds{}
	}
	lo, hi := points[0], points[0]
	for _, p := range points {
		if !entryCoordinateSafe(p) {
			return roundaboutEntryBounds{}
		}
		lo.X = math.Min(lo.X, p.X)
		lo.Y = math.Min(lo.Y, p.Y)
		hi.X = math.Max(hi.X, p.X)
		hi.Y = math.Max(hi.Y, p.Y)
	}
	start := entryPointAt(points, forward, s)
	if !entryCoordinateSafe(start) {
		return roundaboutEntryBounds{}
	}
	lo.X = math.Min(lo.X, start.X)
	lo.Y = math.Min(lo.Y, start.Y)
	hi.X = math.Max(hi.X, start.X)
	hi.Y = math.Max(hi.Y, start.Y)
	shift := b.Pos.Sub(start)
	// L1 extents cover both OBB headings without trig. Two pixels are an outward
	// roundoff allowance in addition to the unchanged original safety margins.
	// Use the full articulated reach relative to its powered-section reference.
	radius := b.OverallLength() - b.Length/2 + b.Width/2 + 20
	lo.X += math.Min(0, shift.X) - radius
	lo.Y += math.Min(0, shift.Y) - radius
	hi.X += math.Max(0, shift.X) + radius
	hi.Y += math.Max(0, shift.Y) + radius
	return roundaboutEntryBounds{lo: lo, hi: hi, valid: true}
}
func entryCoordinateSafe(p sim.Vec2) bool { return math.Abs(p.X) <= 1e8 && math.Abs(p.Y) <= 1e8 }
func (r roundaboutEntryBounds) excludes(body sim.OBB) bool {
	if !r.valid || !entryCoordinateSafe(body.Center) || !(math.Abs(body.Heading) <= 1e6 && body.HalfLen >= 0 && body.HalfLen <= 1e6 && body.HalfWid >= 0 && body.HalfWid <= 1e6) {
		return false
	}
	radius := body.HalfLen + body.HalfWid + 2
	return body.Center.X+radius < r.lo.X || body.Center.X-radius > r.hi.X || body.Center.Y+radius < r.lo.Y || body.Center.Y-radius > r.hi.Y
}
