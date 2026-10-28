package sim

import "math"

func referenceSeparatedBodies(a, b []OBB) bool {
	for _, x := range a {
		for _, y := range b {
			r := x.HalfLen + x.HalfWid + y.HalfLen + y.HalfWid
			d := x.Center.Sub(y.Center)
			if d.Dot(d) > r*r {
				continue
			}
			if hit, _, _ := referenceCheckOBBCollision(x, y); hit {
				return false
			}
		}
	}
	return true
}

func referenceCheckOBBCollision(a, b OBB) (bool, Vec2, float64) {
	// Broadphase circle check
	maxRadiusA := math.Hypot(a.HalfLen, a.HalfWid)
	maxRadiusB := math.Hypot(b.HalfLen, b.HalfWid)
	distCenters := a.Center.Distance(b.Center)
	if distCenters > maxRadiusA+maxRadiusB {
		return false, Vec2{}, 0
	}

	// Local axes of A
	uA := UnitFromAngle(a.Heading)
	vA := Vec2{X: -math.Sin(a.Heading), Y: math.Cos(a.Heading)}

	// Local axes of B
	uB := UnitFromAngle(b.Heading)
	vB := Vec2{X: -math.Sin(b.Heading), Y: math.Cos(b.Heading)}

	axes := [4]Vec2{uA, vA, uB, vB}
	d := a.Center.Sub(b.Center)

	minOverlap := math.MaxFloat64
	bestAxis := Vec2{}

	for _, axis := range axes {
		distProj := math.Abs(d.Dot(axis))
		rA := a.HalfLen*math.Abs(uA.Dot(axis)) + a.HalfWid*math.Abs(vA.Dot(axis))
		rB := b.HalfLen*math.Abs(uB.Dot(axis)) + b.HalfWid*math.Abs(vB.Dot(axis))

		overlap := (rA + rB) - distProj
		if overlap <= 0 {
			return false, Vec2{}, 0
		}

		if overlap < minOverlap {
			minOverlap = overlap
			bestAxis = axis
		}
	}

	// Orient normal from B towards A
	if d.Dot(bestAxis) < 0 {
		bestAxis = Vec2{X: -bestAxis.X, Y: -bestAxis.Y}
	}

	return true, bestAxis, minOverlap
}
