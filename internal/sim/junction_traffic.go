package sim

import "math"

// JunctionCorridor is a short-lived statement of an autonomous bus's remaining
// route through one junction. Manual vehicles keep conservative yielding.
type JunctionCorridor struct {
	Center        Vec2
	CachePosition Vec2
	CacheHeading  float64
	CacheTurn     *CarJunctionTurn
	EntryHeading  float64
	Entered       bool
	Bodies        []OBB
	Min, Max      Vec2
	Remaining     float64
}

// AppendPathBodies samples the full vehicle, including its corners, rather than
// treating a turn as a point or a straight extrapolation of the current heading.
func AppendPathBodies(dst []OBB, points []Vec2, length, width float64) []OBB {
	for i := 1; i < len(points); i++ {
		delta := points[i].Sub(points[i-1])
		n := int(math.Ceil(delta.Length() / 16))
		if n < 1 {
			continue
		}
		heading := math.Atan2(delta.Y, delta.X)
		for j := 0; j <= n; j++ {
			p := points[i-1].Add(delta.Mul(float64(j) / float64(n)))
			dst = append(dst, OBB{Center: p, Heading: heading, HalfLen: length/2 + 4, HalfWid: width/2 + 3})
		}
	}
	return dst
}

// CorridorsSeparated is an independent sweep: simultaneous admission is allowed
// only when the bodies cannot touch, regardless of their relative progress.
func CorridorsSeparated(a, b []OBB) bool {
	if len(a) == 0 || len(b) == 0 {
		return false
	}
	amin, amax := corridorBounds(a)
	bmin, bmax := corridorBounds(b)
	if amax.X < bmin.X || bmax.X < amin.X || amax.Y < bmin.Y || bmax.Y < amin.Y {
		return true
	}
	return separatedBodies(a, b)
}

func separatedBodies(a, b []OBB) bool {
	for _, x := range a {
		for _, y := range b {
			r := x.HalfLen + x.HalfWid + y.HalfLen + y.HalfWid
			d := x.Center.Sub(y.Center)
			if d.Dot(d) > r*r {
				continue
			}
			if hit, _, _ := CheckOBBCollision(x, y); hit {
				return false
			}
		}
	}
	return true
}

// ExitQueueBlocksEntry reserves a vehicle-length beyond the box. A stopped
// queue on that outgoing lane must not be joined from inside the junction.
func ExitQueueBlocksEntry(bodies []OBB, center Vec2, g JunctionGeometry, length, width float64, other OBB, speed float64) bool {
	if len(bodies) < 2 || math.Abs(speed) > 5 {
		return false
	}
	exit := bodies[len(bodies)-1]
	if math.Cos(other.Heading-exit.Heading) < .9 {
		return false
	}
	u := Vec2{X: math.Cos(exit.Heading), Y: math.Sin(exit.Heading)}
	right := Vec2{X: -u.Y, Y: u.X}
	half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	along := other.Center.Sub(center).Dot(u)
	if along+other.HalfLen < half || along-other.HalfLen > half+length+25 {
		return false
	}
	return math.Abs(other.Center.Sub(exit.Center).Dot(right)) < width/2+other.HalfWid+4
}

func corridorBounds(bodies []OBB) (Vec2, Vec2) {
	lo := Vec2{X: math.Inf(1), Y: math.Inf(1)}
	hi := Vec2{X: math.Inf(-1), Y: math.Inf(-1)}
	lastHeading, c, s := math.Inf(1), 0., 0.
	for _, b := range bodies {
		if b.Heading != lastHeading {
			lastHeading = b.Heading
			c = math.Abs(math.Cos(b.Heading))
			s = math.Abs(math.Sin(b.Heading))
		}
		x, y := b.HalfLen*c+b.HalfWid*s, b.HalfLen*s+b.HalfWid*c
		if v := b.Center.X - x; v < lo.X {
			lo.X = v
		}
		if v := b.Center.Y - y; v < lo.Y {
			lo.Y = v
		}
		if v := b.Center.X + x; v > hi.X {
			hi.X = v
		}
		if v := b.Center.Y + y; v > hi.Y {
			hi.Y = v
		}
	}
	return lo, hi
}

func (c *JunctionCorridor) RebuildBounds() { c.Min, c.Max = corridorBounds(c.Bodies) }

func SeparatedJunctionCorridors(a, b *JunctionCorridor) bool {
	if a == nil || b == nil || len(a.Bodies) == 0 || len(b.Bodies) == 0 {
		return false
	}
	if a.Max.X < b.Min.X || b.Max.X < a.Min.X || a.Max.Y < b.Min.Y || b.Max.Y < a.Min.Y {
		return true
	}
	return separatedBodies(a.Bodies, b.Bodies)
}

// JunctionLeaderGap distinguishes a vehicle ahead on the same committed lane
// from a crossing movement. A shared entry/exit alone is insufficient: the
// leader's current body must also lie on our remaining curve with matching yaw.
func JunctionLeaderGap(ours, theirs *JunctionCorridor, leader OBB, ownLength, ownWidth float64) (float64, bool) {
	if ours == nil || theirs == nil || ours.Remaining <= 0 || theirs.Remaining <= 0 || ours.Center.Distance(theirs.Center) > 1 || len(ours.Bodies) < 2 || len(theirs.Bodies) < 2 || math.Cos(ours.EntryHeading-theirs.EntryHeading) < .98 {
		return 0, false
	}
	end, otherEnd := ours.Bodies[len(ours.Bodies)-1], theirs.Bodies[len(theirs.Bodies)-1]
	if math.Cos(end.Heading-otherEnd.Heading) < .98 {
		return 0, false
	}
	normal := Vec2{X: -math.Sin(end.Heading), Y: math.Cos(end.Heading)}
	if math.Abs(end.Center.Sub(otherEnd.Center).Dot(normal)) > 12 {
		return 0, false
	}
	best, along, traveled := math.Inf(1), 0., 0.
	for i := 1; i < len(ours.Bodies); i++ {
		a, b := ours.Bodies[i-1].Center, ours.Bodies[i].Center
		segment := b.Sub(a)
		length := segment.Length()
		if length < .01 {
			continue
		}
		u := segment.Mul(1 / length)
		d := Clamp(leader.Center.Sub(a).Dot(u), 0, length)
		point := a.Add(u.Mul(d))
		dist := point.Distance(leader.Center)
		if dist < best && math.Cos(math.Atan2(u.Y, u.X)-leader.Heading) > .94 {
			best, along = dist, traveled+d
		}
		traveled += length
	}
	if best > math.Max(12, ownWidth*.35) || along <= 0 {
		return 0, false
	}
	return along - ownLength/2 - leader.HalfLen, true
}

// Follow a leader's speed continuously; do not require its whole junction
// corridor to empty before admitting the next vehicle on that same curve.
func JunctionFollowingSpeed(gap, leadSpeed, ownSpeed, deceleration float64) float64 {
	lead := math.Max(0, leadSpeed)
	if gap < 8 {
		return 0
	}
	desired := 18 + .3*math.Abs(ownSpeed)
	return math.Min(math.Max(0, lead+.9*(gap-desired)), math.Sqrt(lead*lead+2*deceleration*math.Max(0, gap-18)))
}

// PredictJunctionBody follows the published curve by distance. Extrapolating
// a turning bus along its current yaw falsely puts it across a following lane.
func PredictJunctionBody(c *JunctionCorridor, distance float64) (OBB, bool) {
	if c == nil || c.Remaining <= 0 || len(c.Bodies) < 2 {
		return OBB{}, false
	}
	remaining := math.Max(0, distance)
	for i := 1; i < len(c.Bodies); i++ {
		a, b := c.Bodies[i-1], c.Bodies[i]
		segment := b.Center.Sub(a.Center)
		length := segment.Length()
		if length < .01 {
			continue
		}
		if remaining <= length {
			a.Center = a.Center.Add(segment.Mul(remaining / length))
			a.Heading = math.Atan2(segment.Y, segment.X)
			return a, true
		}
		remaining -= length
	}
	last := c.Bodies[len(c.Bodies)-1]
	last.Center = last.Center.Add(Vec2{X: math.Cos(last.Heading), Y: math.Sin(last.Heading)}.Mul(remaining))
	return last, true
}
