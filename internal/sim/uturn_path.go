package sim

import "math"

// UTurnPath keeps a bus-sized turning radius instead of joining two opposite
// lanes with a perpendicular line. On narrow roads the bus uses the junction's
// turning apron, then eases into the outgoing lane before leaving the box.
func UTurnPath(center Vec2, heading, inOffset, outOffset float64, geometry JunctionGeometry) []Vec2 {
	forward := UnitFromAngle(heading)
	right := Vec2{X: -forward.Y, Y: forward.X}
	halfAlong := math.Abs(forward.X)*geometry.HalfX + math.Abs(forward.Y)*geometry.HalfY
	radius := math.Max(90, (inOffset+outOffset)/2)
	const taperRatio = 3.2
	// On a wide road, reverse into the receiving side of the road first,
	// then use the straight exit to reach the destination lane. Keep enough
	// radius for the trailer and enough runway for the lateral taper.
	if radius > 135 {
		taperRadius := (taperRatio*(inOffset+outOffset) - halfAlong + 60) / (2*taperRatio - 1)
		radius = min(radius, max(135, (inOffset+48)/2, taperRadius))
	}
	shift := 2*radius - inOffset - outOffset
	startForward := -radius
	if math.Abs(shift) > 1e-6 {
		// Start as soon as the return taper fits. Small boxes can still
		// use their far-side apron to provide the necessary runway.
		startForward = min(halfAlong-radius-30, max(-radius, taperRatio*math.Abs(shift)+60-halfAlong))
	}
	points := make([]Vec2, 0, 66)
	points = append(points, center.Sub(forward.Mul(math.Max(80, -startForward+30))).Add(right.Mul(inOffset)))
	circle := center.Add(forward.Mul(startForward)).Add(right.Mul(inOffset - radius))
	for i := 0; i <= 48; i++ {
		theta := math.Pi * float64(i) / 48
		points = append(points, circle.Add(right.Mul(radius*math.Cos(theta))).Add(forward.Mul(radius*math.Sin(theta))))
	}
	if math.Abs(shift) > 1e-6 {
		length := halfAlong + startForward - 60
		for i := 1; i <= 12; i++ {
			u := float64(i) / 12
			blend := u * u * (3 - 2*u)
			points = append(points, center.Add(forward.Mul(startForward-length*u)).Add(right.Mul(inOffset-2*radius+shift*blend)))
		}
	}
	return points
}
