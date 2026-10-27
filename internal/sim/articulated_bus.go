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
	h := b.TailHeading()
	return OBB{Center: b.HitchPosition().Sub(UnitFromAngle(h).Mul(b.Length / 2)), Heading: h, HalfLen: b.Length / 2, HalfWid: b.Width / 2}
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
	u := UnitFromAngle(b.Heading).Mul(b.HalfLen)
	r := Vec2{X: -math.Sin(b.Heading), Y: math.Cos(b.Heading)}.Mul(b.HalfWid)
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
	oldTail := b.TrailerHeading
	delta := b.Pos.Sub(oldPos)
	turn := NormalizeAngle(b.Heading - oldHeading)
	steps := max(1, int(math.Ceil(math.Max(delta.Length(), math.Abs(turn)*240))))
	steps = min(steps, 4096)
	oldHitch := oldPos.Sub(UnitFromAngle(oldHeading).Mul(b.Length / 2))
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
			direction = UnitFromAngle(h)
		}
		hitch := oldPos.Add(delta.Mul(t)).Sub(direction.Mul(b.Length / 2))
		d := hitch.Sub(oldHitch)
		currentBits := math.Float64bits(b.TrailerHeading)
		if !tailReady || currentBits != tailBits {
			ts, tc := math.Sincos(b.TrailerHeading)
			tailDirection = Vec2{X: -ts, Y: tc}
			tailBits, tailReady = currentBits, true
		}
		b.TrailerHeading = NormalizeAngle(b.TrailerHeading + d.Dot(tailDirection)/ArticulatedAxleDistance)
		angle := NormalizeAngle(h - b.TrailerHeading)
		if math.Abs(angle) > MaxArticulationAngle {
			b.Pos, b.Heading, b.TrailerHeading = oldPos, oldHeading, oldTail
			b.Speed = 0
			b.IsBraking = true
			return true
		}
		oldHitch = hitch
	}
	return false
}
func (b *Bus) AtPose(pos Vec2, heading float64) Bus {
	copy := *b
	var entry *busPredictionEntry
	if b.IsArticulated() && b.predictionCache != nil {
		var found bool
		entry, found = b.predictionEntry(pos, heading)
		if found {
			copy.Pos, copy.Heading, copy.TrailerHeading, copy.TrailerInitialized = entry.pose.pos, entry.pose.heading, entry.pose.trailer, entry.pose.initialized
			if entry.pose.rejected {
				copy.Speed, copy.IsBraking = 0, true
			}
			return copy
		}
	}
	if !copy.TrailerInitialized {
		copy.ResetArticulation()
	}
	copy.Pos, copy.Heading = pos, heading
	rejected := copy.advanceTrailer(b.Pos, b.Heading)
	if entry != nil {
		entry.pose = busPredictionPose{pos: copy.Pos, heading: copy.Heading, trailer: copy.TrailerHeading, initialized: copy.TrailerInitialized, rejected: rejected}
		entry.valid = true
	}
	return copy
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
	copy := bus.AtPose(front.Center, front.Heading)
	hit, n, d := CheckOBBCollision(front, other)
	tail := copy.TrailerBody()
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
