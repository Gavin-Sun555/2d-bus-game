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
	UTurnReady    bool `json:",omitempty"`
	CurbLeft      bool `json:",omitempty"`
	Bodies        []OBB
	Min, Max      Vec2
	Remaining     float64
	segments      []junctionCorridorSegment
	segmentsReady bool
	bodySource    *OBB
	bodyCount     int
	groups        []corridorBodyGroup
	groupsReady   bool
}

type junctionCorridorSegment struct {
	delta        Vec2
	length       float64
	heading      float64
	headingReady bool
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
	if len(a) < 8 || len(b) < 16 || len(a) > 512 || len(b) > 512 {
		return separatedBodiesScan(a, b)
	}
	var storageA, storageB [64]corridorBodyGroup
	groupsA, okA := groupCorridorBodies(a, storageA[:])
	groupsB, okB := groupCorridorBodies(b, storageB[:])
	if !okA || !okB {
		return separatedBodiesScan(a, b)
	}
	return separatedBodyGroups(a, b, groupsA, groupsB)
}

func separatedBodyGroups(a, b []OBB, groupsA, groupsB []corridorBodyGroup) bool {
	for _, ga := range groupsA {
		for _, gb := range groupsB {
			if ga.hi.X < gb.lo.X || gb.hi.X < ga.lo.X || ga.hi.Y < gb.lo.Y || gb.hi.Y < ga.lo.Y {
				continue
			}
			for _, x := range a[ga.begin:ga.end] {
				lo, hi, _ := looseBodyBounds(x)
				if hi.X < gb.lo.X || gb.hi.X < lo.X || hi.Y < gb.lo.Y || gb.hi.Y < lo.Y {
					continue
				}
				for _, y := range b[gb.begin:gb.end] {
					r := x.HalfLen + x.HalfWid + y.HalfLen + y.HalfWid
					d := x.Center.Sub(y.Center)
					if math.Abs(d.X) > r || math.Abs(d.Y) > r || d.Dot(d) > r*r {
						continue
					}
					if hit, _, _ := CheckOBBCollision(x, y); hit {
						return false
					}
				}
			}
		}
	}
	return true
}

func separatedBodiesScan(a, b []OBB) bool {
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

// JunctionExitQueue holds the fixed exit geometry for one admission check.
type JunctionExitQueue struct {
	exit                   OBB
	center, forward, right Vec2
	half, length, width    float64
	valid                  bool
}

func PrepareJunctionExitQueue(bodies []OBB, center Vec2, g JunctionGeometry, length, width float64) JunctionExitQueue {
	if len(bodies) < 2 {
		return JunctionExitQueue{}
	}
	exit := bodies[len(bodies)-1]
	forward := UnitFromAngle(exit.Heading)
	return JunctionExitQueue{
		exit: exit, center: center, forward: forward, right: Vec2{X: -forward.Y, Y: forward.X},
		half:   math.Abs(forward.X)*g.HalfX + math.Abs(forward.Y)*g.HalfY,
		length: length, width: width, valid: true,
	}
}

func (queue JunctionExitQueue) Blocks(other OBB) bool {
	if !queue.valid || math.Cos(other.Heading-queue.exit.Heading) < .9 {
		return false
	}
	along := other.Center.Sub(queue.center).Dot(queue.forward)
	if along+other.HalfLen < queue.half || along-other.HalfLen > queue.half+queue.length+25 {
		return false
	}
	return math.Abs(other.Center.Sub(queue.exit.Center).Dot(queue.right)) < queue.width/2+other.HalfWid+4
}

// ExitQueueBlocksEntry reserves a vehicle-length beyond the box. A stopped
// queue on that outgoing lane must not be joined from inside the junction.
func ExitQueueBlocksEntry(bodies []OBB, center Vec2, g JunctionGeometry, length, width float64, other OBB, speed float64) bool {
	if math.Abs(speed) > 5 {
		return false
	}
	return PrepareJunctionExitQueue(bodies, center, g, length, width).Blocks(other)
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

// RebuildBounds publishes new bodies and invalidates their derived geometry.
func (c *JunctionCorridor) RebuildBounds() {
	c.Min, c.Max = corridorBounds(c.Bodies)
	var source *OBB
	if len(c.Bodies) != 0 {
		source = &c.Bodies[0]
	}
	if c.bodySource != source {
		// A value copy may still refer to the original corridor's scratch.
		c.segments, c.groups = nil, nil
	}
	c.bodySource, c.bodyCount = source, len(c.Bodies)
	c.segmentsReady, c.groupsReady = false, false
}

func (c *JunctionCorridor) geometryPublished() bool {
	return len(c.Bodies) > 0 && c.bodyCount == len(c.Bodies) && c.bodySource == &c.Bodies[0]
}

// Publishing is frequent. Build lengths only if a consumer actually queries
// this path. Unpublished corridors (including old saves) use live bodies.
func (c *JunctionCorridor) pathSegments() []junctionCorridorSegment {
	if !c.geometryPublished() {
		return nil
	}
	if !c.segmentsReady {
		count := len(c.Bodies) - 1
		if cap(c.segments) < count {
			c.segments = make([]junctionCorridorSegment, count)
		} else {
			c.segments = c.segments[:count]
		}
		for i := range c.segments {
			delta := c.Bodies[i+1].Center.Sub(c.Bodies[i].Center)
			c.segments[i] = junctionCorridorSegment{delta: delta, length: delta.Length()}
		}
		c.segmentsReady = true
	}
	return c.segments
}

func (c *JunctionCorridor) bodyGroups() ([]corridorBodyGroup, bool) {
	if !c.groupsReady {
		count := (len(c.Bodies) + 7) / 8
		if cap(c.groups) < count {
			c.groups = make([]corridorBodyGroup, count)
		} else {
			c.groups = c.groups[:count]
		}
		groups, ok := groupCorridorBodies(c.Bodies, c.groups)
		if !ok {
			c.groups = c.groups[:0]
		} else {
			c.groups = groups
		}
		c.groupsReady = true
	}
	return c.groups, len(c.groups) > 0
}

func SeparatedJunctionCorridors(a, b *JunctionCorridor) bool {
	if a == nil || b == nil || len(a.Bodies) == 0 || len(b.Bodies) == 0 {
		return false
	}
	if a.Max.X < b.Min.X || b.Max.X < a.Min.X || a.Max.Y < b.Min.Y || b.Max.Y < a.Min.Y {
		return true
	}
	if len(a.Bodies) >= 8 && len(a.Bodies) <= 512 && len(b.Bodies) >= 16 && len(b.Bodies) <= 512 &&
		a.geometryPublished() && b.geometryPublished() {
		groupsA, okA := a.bodyGroups()
		groupsB, okB := b.bodyGroups()
		if okA && okB {
			return separatedBodyGroups(a.Bodies, b.Bodies, groupsA, groupsB)
		}
		return separatedBodiesScan(a.Bodies, b.Bodies)
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
	distanceLimit := math.Max(12, ownWidth*.35)
	best, along, traveled := math.Inf(1), 0., 0.
	segments := ours.pathSegments()
	for i := 1; i < len(ours.Bodies); i++ {
		a, b := ours.Bodies[i-1].Center, ours.Bodies[i].Center
		var segment Vec2
		var length float64
		if segments != nil {
			segment, length = segments[i-1].delta, segments[i-1].length
		} else {
			segment = b.Sub(a)
			length = segment.Length()
		}
		if length < .01 {
			continue
		}
		u := segment.Mul(1 / length)
		d := Clamp(leader.Center.Sub(a).Dot(u), 0, length)
		point := a.Add(u.Mul(d))
		dx, dy := point.X-leader.Center.X, point.Y-leader.Center.Y
		if math.Abs(dx) > distanceLimit || math.Abs(dy) > distanceLimit {
			traveled += length
			continue
		}
		dist := math.Hypot(dx, dy)
		if dist <= distanceLimit && dist < best && math.Cos(math.Atan2(u.Y, u.X)-leader.Heading) > .94 {
			best, along = dist, traveled+d
		}
		traveled += length
	}
	if best > distanceLimit || along <= 0 {
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
	segments := c.pathSegments()
	for i := 1; i < len(c.Bodies); i++ {
		var delta Vec2
		var length float64
		if segments != nil {
			delta, length = segments[i-1].delta, segments[i-1].length
		} else {
			delta = c.Bodies[i].Center.Sub(c.Bodies[i-1].Center)
			length = delta.Length()
		}
		if length < .01 {
			continue
		}
		if remaining <= length {
			body := c.Bodies[i-1]
			body.Center = body.Center.Add(delta.Mul(remaining / length))
			if segments == nil {
				body.Heading = math.Atan2(delta.Y, delta.X)
			} else {
				segment := &segments[i-1]
				if !segment.headingReady {
					segment.heading = math.Atan2(delta.Y, delta.X)
					segment.headingReady = true
				}
				body.Heading = segment.heading
			}
			return body, true
		}
		remaining -= length
	}
	last := c.Bodies[len(c.Bodies)-1]
	last.Center = last.Center.Add(UnitFromAngle(last.Heading).Mul(remaining))
	return last, true
}

// UTurnPrecedesEntry gives a waiting U-turn one opportunity against incoming
// movements with the same green. Vehicles already in the box keep priority.
func UTurnPrecedesEntry(turn, approach *JunctionCorridor, inter *TrafficIntersection) bool {
	if !inter.WaitingUTurnHasPriority() || turn == nil || !turn.UTurnReady || turn.Entered || turn.Remaining <= 0 || len(turn.Bodies) < 2 ||
		approach == nil || approach.Entered || approach.Center.Distance(turn.Center) > 1 ||
		inter.GetSignalForHeading(turn.EntryHeading, true) != SignalGreen {
		return false
	}
	if approach.UTurnReady {
		return math.Cos(turn.EntryHeading-approach.EntryHeading) < -.98 &&
			UnitFromAngle(turn.EntryHeading).Dot(Vec2{X: 1, Y: 1}) > 0
	}
	return true
}
