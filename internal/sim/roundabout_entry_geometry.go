package sim

import "math"

// RoundaboutEntryGeometry stores pure curve samples, never vehicle state or
// admission decisions. A driver owns its bounded cache on the simulation thread.
// Separate slots prevent its bus geometry from evicting civilian entry geometry.
type RoundaboutEntryGeometry struct {
	samples [2][2]roundaboutEntrySamples
}
type roundaboutEntrySamples struct {
	valid            bool
	key              [12]uint64
	points, tangents [33]Vec2
	start            Vec2
	headings         [33]float64
	headingsReady    bool
}

func roundaboutJoinBits(c RoundaboutJoin) [12]uint64 {
	return [12]uint64{
		math.Float64bits(c.Reference[0].X), math.Float64bits(c.Reference[0].Y),
		math.Float64bits(c.Reference[1].X), math.Float64bits(c.Reference[1].Y),
		math.Float64bits(c.Reference[2].X), math.Float64bits(c.Reference[2].Y),
		math.Float64bits(c.Reference[3].X), math.Float64bits(c.Reference[3].Y),
		math.Float64bits(c.StartOffset), math.Float64bits(c.EndOffset),
		math.Float64bits(c.StartShift.X), math.Float64bits(c.StartShift.Y)}
}
func (g *RoundaboutEntryGeometry) sample(curve RoundaboutJoin, lane int, bus bool) *roundaboutEntrySamples {
	kind := 0
	if bus {
		kind = 1
	}
	entry := &g.samples[kind][lane]
	key := roundaboutJoinBits(curve)
	if !entry.valid || entry.key != key {
		entry.key, entry.valid, entry.headingsReady = key, true, false
		for i := 0; i <= 32; i++ {
			entry.points[i], entry.tangents[i] = RoundaboutEntryPoint(curve, float64(i)/32)
		}
		entry.start = curve.position(0)
	}
	return entry
}

// Mirror RoundaboutEntryCorridor exactly; only curve sample production is
// replaced. Original live distance, tie order and heading tests stay unchanged.
func (g *RoundaboutEntryGeometry) corridor(rb Roundabout, forward Vec2, laneWidth float64, body OBB, bus bool) (int, RoundaboutJoin, bool) {
	outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
	inner := (3*(rb.InnerRadius+16) + rb.Radius) / 4
	if bus {
		inner = math.Max(outer-laneWidth, rb.InnerRadius+body.HalfWid+24)
	}
	for lane, radius := range []float64{inner, outer} {
		curve := RoundaboutEntryJoin(rb, forward, (float64(lane)+.5)*laneWidth, radius, laneWidth)
		sample := g.sample(curve, lane, bus)
		best := math.Inf(1)
		var tangent Vec2
		for i := 0; i <= 32; i++ {
			if d := sample.points[i].Distance(body.Center); d < best {
				best = d
				tangent = sample.tangents[i]
			}
		}
		before := body.Center.Sub(sample.start).Dot(forward)
		if before < 0 && before > -220 {
			p := sample.start.Add(forward.Mul(before))
			if d := p.Distance(body.Center); d < best {
				best = d
				tangent = forward
			}
		}
		if best < 12 && tangent.Normalize().Dot(UnitFromAngle(body.Heading)) > .93 {
			return lane, curve, true
		}
	}
	return 0, RoundaboutJoin{}, false
}
func (g *RoundaboutEntryGeometry) path(curve RoundaboutJoin, lane int, body OBB, bus bool) [34]OBB {
	sample := g.sample(curve, lane, bus)
	if !sample.headingsReady {
		for i, t := range sample.tangents {
			sample.headings[i] = math.Atan2(t.Y, t.X)
		}
		sample.headingsReady = true
	}
	var path [34]OBB
	original := body
	body.HalfLen += 3
	body.HalfWid += 3
	for i := 0; i <= 32; i++ {
		body.Center = sample.points[i]
		body.Heading = sample.headings[i]
		path[i] = body
	}
	body.Center, body.Heading = original.Center, original.Heading
	path[33] = body
	return path
}
