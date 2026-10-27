package sim

import "math"

// UTurnPath keeps a bus-sized turning radius instead of joining two opposite
// lanes with a perpendicular line. On narrow roads the bus uses the junction's
// turning apron, then eases into the outgoing lane before leaving the box.
func UTurnPath(center Vec2, heading, inOffset, outOffset float64, geometry JunctionGeometry) []Vec2 {
	forward := UnitFromAngle(heading)
	right := Vec2{X: -forward.Y, Y: forward.X}
	radius := math.Max(90, (inOffset+outOffset)/2)
	shift := 2*radius - inOffset - outOffset
	halfAlong := math.Abs(forward.X)*geometry.HalfX + math.Abs(forward.Y)*geometry.HalfY
	startForward := -radius
	if shift > 1e-6 {
		startForward = halfAlong - radius - 30
	}
	points := make([]Vec2, 0, 66)
	points = append(points, center.Sub(forward.Mul(math.Max(80, -startForward+30))).Add(right.Mul(inOffset)))
	circle := center.Add(forward.Mul(startForward)).Add(right.Mul(inOffset - radius))
	for i := 0; i <= 48; i++ {
		theta := math.Pi * float64(i) / 48
		points = append(points, circle.Add(right.Mul(radius*math.Cos(theta))).Add(forward.Mul(radius*math.Sin(theta))))
	}
	if shift > 1e-6 {
		length := halfAlong + startForward - 60
		for i := 1; i <= 12; i++ {
			u := float64(i) / 12
			blend := u * u * (3 - 2*u)
			points = append(points, center.Add(forward.Mul(startForward-length*u)).Add(right.Mul(inOffset-2*radius+shift*blend)))
		}
	}
	return points
}
