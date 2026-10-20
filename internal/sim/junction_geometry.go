package sim

import "math"

// JunctionGeometry is shared by routes, traffic signals, scenery and barriers.
// Road half widths describe the arms; the larger box accommodates a bus's swept
// body and keeps opposing left turns on separate sides of the intersection.
type JunctionGeometry struct {
	HalfX, HalfY float64
	TurnRadius   float64
}

const JunctionStopInset = 31.0

func NewJunctionGeometry(ewHalf, nsHalf, laneWidth float64) JunctionGeometry {
	radius := math.Max(math.Max(190, laneWidth*3.2), math.Max(ewHalf, nsHalf)+60)
	// A curbside bus may turn left across four incoming lanes. Keep the
	// full swept arc inside the box and separate opposing left-turn paths.
	if math.Max(ewHalf, nsHalf) >= 4*laneWidth {
		radius = math.Max(radius, 8*laneWidth)
	}
	minimum := radius - laneWidth/2 + 60
	return JunctionGeometry{HalfX: math.Max(nsHalf+72, minimum), HalfY: math.Max(ewHalf+72, minimum), TurnRadius: radius}
}

func (w *World) JunctionAt(center Vec2, laneWidth float64) JunctionGeometry {
	ew, ns := w.GetIntersectionRoadDimensions(center, laneWidth)
	return NewJunctionGeometry(ew, ns, laneWidth)
}

// InsideJunction excludes roundabouts, whose circulating lanes have their own
// geometry. margin includes the vehicle body when deciding where a turn is legal.
func (w *World) InsideJunction(pos Vec2, laneWidth, margin float64) bool {
	for _, center := range w.Intersections {
		if w.IsAtRoundabout(center) {
			continue
		}
		g := w.JunctionAt(center, laneWidth)
		if math.Abs(pos.X-center.X) <= g.HalfX+margin && math.Abs(pos.Y-center.Y) <= g.HalfY+margin {
			return true
		}
	}
	return false
}

// LeftTurnPath uses a large, tangent-continuous quarter circle. In right-hand
// traffic these opposing arcs pass driver-side to driver-side; a tiny arc at the
// lane crossing cannot be followed by a long-wheelbase vehicle.
func LeftTurnPath(center Vec2, heading, laneWidth, radius float64) []Vec2 {
	in := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	out := Vec2{X: in.Y, Y: -in.X}
	return QuarterTurnPath(center, in, out, laneWidth/2, laneWidth/2, radius)
}

func QuarterTurnPath(center, in, out Vec2, inOffset, outOffset, radius float64) []Vec2 {
	rightIn := Vec2{X: -in.Y, Y: in.X}
	rightOut := Vec2{X: -out.Y, Y: out.X}
	corner := center.Add(rightIn.Mul(inOffset)).Add(rightOut.Mul(outOffset))
	const steps = 24
	points := make([]Vec2, 0, steps+1)
	for i := 0; i <= steps; i++ {
		angle := float64(i) * math.Pi / (2 * steps)
		points = append(points, corner.Add(in.Mul(radius*(math.Sin(angle)-1))).Add(out.Mul(radius*(1-math.Cos(angle)))))
	}
	return points
}
