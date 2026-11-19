package sim

import "math"

// Pos and Length describe the powered front section, the controller reference.
// The trailing section has the same length; combined straight length is 192px.
const ArticulatedAxleDistance = 60.0
const MaxArticulationAngle = math.Pi / 3

func (b *Bus) IsArticulated() bool { return b.Model == BusArticulated && b.GeometryVersion == 1 }
func (b *Bus) OverallLength() float64 {
	if b.IsArticulated() {
		return 2 * b.Length
	}
	return b.Length
}
func (b *Bus) ResetArticulation() {
	b.TrailerHeading = b.Heading
	b.TrailerInitialized = b.IsArticulated()
}
func (b *Bus) HitchPosition() Vec2 {
	return b.Pos.Sub(UnitFromAngle(b.Heading).Mul(b.Length / 2))
}
func (b *Bus) TailHeading() float64 {
	if !b.TrailerInitialized {
		return b.Heading
	}
	return b.TrailerHeading
}
func (b *Bus) TrailerBody() OBB {
	return trailerBody(b.Pos, b.Heading, b.TailHeading(), b.Length, b.Width)
}
func trailerBody(pos Vec2, heading, tail, length, width float64) OBB {
	hitch := pos.Sub(UnitFromAngle(heading).Mul(length / 2))
	return OBB{Center: hitch.Sub(UnitFromAngle(tail).Mul(length / 2)), Heading: tail, HalfLen: length / 2, HalfWid: width / 2}
}
func (b *Bus) BodyParts() [2]OBB {
	parts := [2]OBB{{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}}
	if b.IsArticulated() {
		parts[1] = b.TrailerBody()
		return parts
	}
	return parts
}
func BodyCorners(b OBB) [4]Vec2 {
	direction := UnitFromAngle(b.Heading)
	u := direction.Mul(b.HalfLen)
	r := Vec2{X: -direction.Y, Y: direction.X}.Mul(b.HalfWid)
	return [4]Vec2{b.Center.Add(u).Add(r), b.Center.Add(u).Sub(r), b.Center.Sub(u).Sub(r), b.Center.Sub(u).Add(r)}
}

// Private pose-keyed cache: every consumer in a traffic tick sees identical
// geometry without recalculating eight corners for each nearby civilian car.
// Pose fields are compared on every call, including after direct edits/load.
type busEnvelopePose struct {
	position                     Vec2
	heading, tail, length, width float64
}
type busEnvelopeCache struct {
	valid bool
	pose  busEnvelopePose
	body  OBB
}

func (b *Bus) BodyEnvelope() OBB {
	if !b.IsArticulated() {
		return OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}
	}
	key := busEnvelopePose{b.Pos, b.Heading, b.TailHeading(), b.Length, b.Width}
	if b.envelopeCache.valid && b.envelopeCache.pose == key {
		return b.envelopeCache.body
	}
	body := b.computeBodyEnvelope()
	b.envelopeCache = busEnvelopeCache{true, key, body}
	return body
}

func (b *Bus) computeBodyEnvelope() OBB {
	if !b.IsArticulated() {
		return OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}
	}
	u := UnitFromAngle(b.Heading)
	r := Vec2{X: -u.Y, Y: u.X}
	minU, maxU, minR, maxR := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, part := range b.BodyParts() {
		for _, p := range BodyCorners(part) {
			d := p.Sub(b.Pos)
			a, c := d.Dot(u), d.Dot(r)
			minU = math.Min(minU, a)
			maxU = math.Max(maxU, a)
			minR = math.Min(minR, c)
			maxR = math.Max(maxR, c)
		}
	}
	return OBB{Center: b.Pos.Add(u.Mul((minU + maxU) / 2)).Add(r.Mul((minR + maxR) / 2)), Heading: b.Heading, HalfLen: (maxU - minU) / 2, HalfWid: (maxR - minR) / 2}
}

// Integrate the no-side-slip trailer axle constraint from hitch displacement.
// Substeps keep the solution independent of display/update cadence. Reverse
// motion naturally increases articulation, as with an unsteered trailer.
func (b *Bus) advanceTrailer(oldPos Vec2, oldHeading float64) (rejected bool) {
	if !b.IsArticulated() {
		return false
	}
	if !b.TrailerInitialized {
		b.ResetArticulation()
		return false
	}
	tail, rejected := integrateTrailer(oldPos, b.Pos, oldHeading, b.Heading, b.TrailerHeading, b.Length)
	b.TrailerHeading = tail
	if rejected {
		b.Pos, b.Heading = oldPos, oldHeading
		b.Speed, b.IsBraking = 0, true
	}
	return rejected
}

func integrateTrailer(oldPos, pos Vec2, oldHeading, heading, tail, length float64) (float64, bool) {
	oldTail := tail
	delta := pos.Sub(oldPos)
	turn := NormalizeAngle(heading - oldHeading)
	steps := max(1, int(math.Ceil(math.Max(delta.Length(), math.Abs(turn)*240))))
	steps = min(steps, 4096)
	oldHitch := oldPos.Sub(UnitFromAngle(oldHeading).Mul(length / 2))
	// A zero-yaw prediction has the same powered-section direction at every
	// substep. Keep the original h arithmetic (including signed zero), and
	// reuse only bit-identical trig results. Keep the length multiplication
	// inside Sub below to preserve the original floating-point fusion.
	var fixedDirection Vec2
	if turn == 0 {
		h := oldHeading + turn*(1/float64(steps))
		fixedDirection = UnitFromAngle(h)
	}
	var tailDirection Vec2
	var tailBits uint64
	tailReady := false
	for i := 1; i <= steps; i++ {
		t := float64(i) / float64(steps)
		h := oldHeading + turn*t
		direction := fixedDirection
		if turn != 0 {
			// Every substep has a new angle; skip the memo.
			hs, hc := math.Sincos(h)
			direction = Vec2{X: hc, Y: hs}
		}
		hitch := oldPos.Add(delta.Mul(t)).Sub(direction.Mul(length / 2))
		d := hitch.Sub(oldHitch)
		currentBits := math.Float64bits(tail)
		if !tailReady || currentBits != tailBits {
			ts, tc := math.Sincos(tail)
			tailDirection = Vec2{X: -ts, Y: tc}
			tailBits, tailReady = currentBits, true
		}
		tail = NormalizeAngle(tail + d.Dot(tailDirection)/ArticulatedAxleDistance)
		angle := NormalizeAngle(h - tail)
		if math.Abs(angle) > MaxArticulationAngle {
			return oldTail, true
		}
		oldHitch = hitch
	}
	return tail, false
}
func (b *Bus) AtPose(pos Vec2, heading float64) Bus {
	copy := *b
	if !b.IsArticulated() {
		if !copy.TrailerInitialized {
			copy.ResetArticulation()
		}
		copy.Pos, copy.Heading = pos, heading
		return copy
	}
	pose := b.articulatedPoseAt(pos, heading)
	copy.Pos, copy.Heading, copy.TrailerHeading, copy.TrailerInitialized = pose.pos, pose.heading, pose.trailer, pose.initialized
	if pose.rejected {
		copy.Speed, copy.IsBraking = 0, true
	}
	return copy
}

func (b *Bus) articulatedPoseAt(pos Vec2, heading float64) busPredictionPose {
	var entry *busPredictionEntry
	if b.predictionCache != nil {
		var found bool
		entry, found = b.predictionEntry(pos, heading)
		if found {
			return entry.pose
		}
	}
	tail, rejected := integrateTrailer(b.Pos, pos, b.Heading, heading, b.TailHeading(), b.Length)
	if rejected {
		pos, heading = b.Pos, b.Heading
	}
	pose := busPredictionPose{pos: pos, heading: heading, trailer: tail, initialized: true, rejected: rejected}
	if entry != nil {
		entry.pose, entry.valid = pose, true
	}
	return pose
}

// TrailerBodyAtPose shares the exact articulation prediction used by AtPose.
func (b *Bus) TrailerBodyAtPose(pos Vec2, heading float64) OBB {
	pose := b.articulatedPoseAt(pos, heading)
	return trailerBody(pose.pos, pose.heading, pose.trailer, b.Length, b.Width)
}

// BodyPartsAtPose returns predicted collision geometry without copying the vehicle state.
func (b *Bus) BodyPartsAtPose(pos Vec2, heading float64) [2]OBB {
	if !b.IsArticulated() {
		return [2]OBB{{Center: pos, Heading: heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}}
	}
	pose := b.articulatedPoseAt(pos, heading)
	return [2]OBB{
		{Center: pose.pos, Heading: pose.heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2},
		trailerBody(pose.pos, pose.heading, pose.trailer, b.Length, b.Width),
	}
}

func (b *Bus) CollidesBody(other OBB) (bool, Vec2, float64) {
	hit := false
	normal := Vec2{}
	depth := 0.
	for _, part := range b.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		if h, n, d := CheckOBBCollision(part, other); h && d > depth {
			hit, normal, depth = true, n, d
		}
	}
	return hit, normal, depth
}
func CheckBusesCollision(a, b *Bus) (bool, Vec2, float64) {
	hit := false
	normal := Vec2{}
	depth := 0.
	for _, part := range b.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		if h, n, d := a.CollidesBody(part); h && d > depth {
			hit, normal, depth = true, n, d
		}
	}
	return hit, normal, depth
}

// CheckBusAtBody includes the predicted trailing section for a front-body pose.
func CheckBusAtBody(bus *Bus, front, other OBB) (bool, Vec2, float64) {
	if !bus.IsArticulated() {
		return CheckOBBCollision(front, other)
	}
	hit, n, d := CheckOBBCollision(front, other)
	tail := bus.TrailerBodyAtPose(front.Center, front.Heading)
	tail.HalfLen += math.Max(0, front.HalfLen-bus.Length/2)
	tail.HalfWid += math.Max(0, front.HalfWid-bus.Width/2)
	if h, n2, d2 := CheckOBBCollision(tail, other); h && (!hit || d2 > d) {
		return h, n2, d2
	}
	return hit, n, d
}

// Extend the berth for both sections, including terminal vehicle exchange.
func EnsureLongBusBays(w *World, route *Route) {
	if w == nil || route == nil {
		return
	}
	for _, id := range route.StopIDs {
		if id >= 0 && id < len(w.Stops) {
			if w.Stops[id].Length < 336 {
				w.Stops[id].Length = 336
				w.boundaryCache = nil
			}
		}
	}
}

func (b *Bus) CheckBoundaries(w *World, lw float64) BoundaryCheckResult {
	result := w.CheckBoundaries(b.Pos, b.Width/2, lw)
	if !b.IsArticulated() {
		return result
	}
	for _, part := range b.BodyParts() {
		for _, corner := range BodyCorners(part) {
			r := w.CheckBoundaries(corner, .5, lw)
			if r.Zone > result.Zone || (r.Zone == result.Zone && r.Penetration > result.Penetration) {
				result = r
			}
		}
	}
	return result
}

// HitsHardBarrier is the boolean version of CheckBoundaries. Callers that only
// need clearance can stop at the first blocked point.
func (b *Bus) HitsHardBarrier(w *World, lw float64) bool {
	if w.CheckBoundaries(b.Pos, b.Width/2, lw).Zone == ZoneHardBarrier {
		return true
	}
	if b.IsArticulated() {
		for _, part := range b.BodyParts() {
			for _, corner := range BodyCorners(part) {
				if w.CheckBoundaries(corner, .5, lw).Zone == ZoneHardBarrier {
					return true
				}
			}
		}
	}
	return false
}
