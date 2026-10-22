package sim

import "math"

// RoundaboutJoin offsets a common centerline along its normal. Independent
// cubics for each lane can pinch together while turning, even with separated
// end points; normal offsets preserve the two driving corridors throughout.
type RoundaboutJoin struct {
	Reference              [4]Vec2
	StartOffset, EndOffset float64
	StartShift             Vec2
}

func RoundaboutEntryJoin(rb Roundabout, forward Vec2, lateral, radius, laneWidth float64) RoundaboutJoin {
	outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
	inner := (3*(rb.InnerRadius+16) + rb.Radius) / 4
	outerLateral := math.Min(1.5*laneWidth, outer*.8)
	along := math.Sqrt(outer*outer - outerLateral*outerLateral)
	right := Vec2{X: -forward.Y, Y: forward.X}
	start := rb.Center.Sub(forward.Mul(along + 125)).Add(right.Mul(.5 * laneWidth))
	angle := math.Atan2(forward.Y, forward.X) + math.Pi - math.Asin(outerLateral/outer) - .6
	join := rb.Center.Add(Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(inner))
	tangent := Vec2{X: math.Sin(angle), Y: -math.Cos(angle)}
	return RoundaboutJoin{Reference: [4]Vec2{start, start.Add(forward.Mul(120)), join.Sub(tangent.Mul(60)), join}, StartOffset: lateral - .5*laneWidth, EndOffset: radius - inner}
}

func cubicRoundaboutPoint(curve [4]Vec2, u float64) (Vec2, Vec2) {
	v := 1 - u
	p := curve[0].Mul(v * v * v).Add(curve[1].Mul(3 * v * v * u)).Add(curve[2].Mul(3 * v * u * u)).Add(curve[3].Mul(u * u * u))
	tangent := curve[1].Sub(curve[0]).Mul(3 * v * v).Add(curve[2].Sub(curve[1]).Mul(6 * v * u)).Add(curve[3].Sub(curve[2]).Mul(3 * u * u))
	return p, tangent
}
func (c RoundaboutJoin) position(u float64) Vec2 {
	p, t := cubicRoundaboutPoint(c.Reference, u)
	t = t.Normalize()
	blend := u * u * (3 - 2*u)
	offset := c.StartOffset + (c.EndOffset-c.StartOffset)*blend
	return p.Add(Vec2{X: -t.Y, Y: t.X}.Mul(offset)).Add(c.StartShift.Mul(1 - blend))
}
func RoundaboutEntryPoint(curve RoundaboutJoin, u float64) (Vec2, Vec2) {
	p := curve.position(u)
	const epsilon = 1e-4
	tangent := curve.position(math.Min(1, u+epsilon)).Sub(curve.position(math.Max(0, u-epsilon)))
	return p, tangent
}

// RoundaboutEntryCorridor identifies an actually aligned entrance trajectory.
// A car circulating across the mouth or a vehicle steering across lanes must
// not receive the adjacent-entry exception merely because its center is close.
func RoundaboutEntryCorridor(rb Roundabout, forward Vec2, laneWidth float64, body OBB, bus bool) (int, RoundaboutJoin, bool) {
	outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
	inner := (3*(rb.InnerRadius+16) + rb.Radius) / 4
	if bus {
		inner = math.Max(outer-laneWidth, rb.InnerRadius+body.HalfWid+24)
	}
	for lane, radius := range []float64{inner, outer} {
		curve := RoundaboutEntryJoin(rb, forward, (float64(lane)+.5)*laneWidth, radius, laneWidth)
		best := math.Inf(1)
		var tangent Vec2
		for i := 0; i <= 32; i++ {
			p, t := RoundaboutEntryPoint(curve, float64(i)/32)
			if d := p.Distance(body.Center); d < best {
				best = d
				tangent = t
			}
		}
		before := body.Center.Sub(curve.position(0)).Dot(forward)
		if before < 0 && before > -220 {
			p := curve.position(0).Add(forward.Mul(before))
			if d := p.Distance(body.Center); d < best {
				best = d
				tangent = forward
			}
		}
		if best < 12 && tangent.Normalize().Dot(Vec2{X: math.Cos(body.Heading), Y: math.Sin(body.Heading)}) > .93 {
			return lane, curve, true
		}
	}
	return 0, RoundaboutJoin{}, false
}

// SeparateRoundaboutEntries proves clearance of the whole pair of swept bodies,
// independent of which vehicle starts first or travels faster. Current bodies
// are checked too, including the bus's tracking error during a turn.
func SeparateRoundaboutEntries(rb Roundabout, forward Vec2, laneWidth float64, a OBB, aBus bool, b OBB, bBus bool) bool {
	la, ca, oka := RoundaboutEntryCorridor(rb, forward, laneWidth, a, aBus)
	lb, cb, okb := RoundaboutEntryCorridor(rb, forward, laneWidth, b, bBus)
	if !oka || !okb || la == lb {
		return false
	}
	var paths [2][34]OBB
	for k, c := range []RoundaboutJoin{ca, cb} {
		body := a
		if k == 1 {
			body = b
		}
		body.HalfLen += 3
		body.HalfWid += 3
		for i := 0; i <= 32; i++ {
			p, t := RoundaboutEntryPoint(c, float64(i)/32)
			body.Center = p
			body.Heading = math.Atan2(t.Y, t.X)
			paths[k][i] = body
		}
		body.Center = a.Center
		body.Heading = a.Heading
		if k == 1 {
			body.Center = b.Center
			body.Heading = b.Heading
		}
		paths[k][33] = body
	}
	for _, first := range paths[0] {
		for _, second := range paths[1] {
			reach := first.HalfLen + first.HalfWid + second.HalfLen + second.HalfWid
			d := first.Center.Sub(second.Center)
			if d.Dot(d) > reach*reach {
				continue
			}
			if hit, _, _ := CheckOBBCollision(first, second); hit {
				return false
			}
		}
	}
	return true
}

// A short-lived controller intent distinguishes a tracked parallel join from
// a manual vehicle or a short right exit crossing the other entrance lane.
type BusRoundaboutEntry struct {
	Center, Forward Vec2
	Remaining       float64
}

// Advance a join by driving distance, extending its straight approach and then
// its circular lane. Used for conflict prediction, never to move a live bus.
func RoundaboutEntryAhead(curve RoundaboutJoin, rb Roundabout, pos Vec2, distance float64) (Vec2, float64) {
	var points [49]Vec2
	var lengths [49]float64
	best, bestS := math.Inf(1), 0.
	for i := range points {
		points[i], _ = RoundaboutEntryPoint(curve, float64(i)/48)
		if i > 0 {
			lengths[i] = lengths[i-1] + points[i].Distance(points[i-1])
		}
		if d := points[i].Distance(pos); d < best {
			best = d
			bestS = lengths[i]
		}
	}
	forward := curve.Reference[1].Sub(curve.Reference[0]).Normalize()
	if before := pos.Sub(points[0]).Dot(forward); before < 0 {
		bestS = before
	}
	target := bestS + distance
	if target <= 0 {
		return points[0].Add(forward.Mul(target)), math.Atan2(forward.Y, forward.X)
	}
	for i := 1; i < len(points); i++ {
		if target <= lengths[i] {
			u := (float64(i-1) + (target-lengths[i-1])/math.Max(1e-9, lengths[i]-lengths[i-1])) / 48
			p, t := RoundaboutEntryPoint(curve, u)
			return p, math.Atan2(t.Y, t.X)
		}
	}
	radial := points[48].Sub(rb.Center)
	angle := math.Atan2(radial.Y, radial.X) - (target-lengths[48])/radial.Length()
	return rb.Center.Add(Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radial.Length())), angle - math.Pi/2
}
