package sim

import "math"

type corridorBodyGroup struct {
	lo, hi     Vec2
	begin, end int
}

// Use an intentionally loose L1 radius, not a new narrow-phase test. Every
// orientation fits inside this square. Outward padding covers floating-point
// roundoff; abnormal dimensions/coordinates retain the original full scan.
func looseBodyBounds(b OBB) (Vec2, Vec2, bool) {
	if !(b.HalfLen >= 0 && b.HalfLen <= 8192 && b.HalfWid >= 0 && b.HalfWid <= 8192 && math.Abs(b.Center.X) <= 1e9 && math.Abs(b.Center.Y) <= 1e9 && math.Abs(b.Heading) <= 1e6) {
		return Vec2{}, Vec2{}, false
	}
	radius := b.HalfLen + b.HalfWid
	pad := 1e-7 * (1 + math.Abs(b.Center.X) + math.Abs(b.Center.Y) + radius)
	radius += pad
	return Vec2{X: b.Center.X - radius, Y: b.Center.Y - radius}, Vec2{X: b.Center.X + radius, Y: b.Center.Y + radius}, true
}
func groupCorridorBodies(b []OBB, dst []corridorBodyGroup) ([]corridorBodyGroup, bool) {
	count := (len(b) + 7) / 8
	if count > len(dst) {
		return nil, false
	}
	for i := 0; i < count; i++ {
		begin, end := i*8, min(len(b), (i+1)*8)
		lo, hi := Vec2{X: math.Inf(1), Y: math.Inf(1)}, Vec2{X: math.Inf(-1), Y: math.Inf(-1)}
		for _, body := range b[begin:end] {
			a, z, ok := looseBodyBounds(body)
			if !ok {
				return nil, false
			}
			lo.X = math.Min(lo.X, a.X)
			lo.Y = math.Min(lo.Y, a.Y)
			hi.X = math.Max(hi.X, z.X)
			hi.Y = math.Max(hi.Y, z.Y)
		}
		dst[i] = corridorBodyGroup{lo: lo, hi: hi, begin: begin, end: end}
	}
	return dst[:count], true
}
