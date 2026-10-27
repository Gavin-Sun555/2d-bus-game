package sim

import "math"

// RoundaboutSpeedLimit is the normal operating cap shared by cars and buses.
const RoundaboutSpeedLimit = 90.0

// A stopped circulating vehicle retains priority. Waiting approach vehicles
// must not reserve the ring merely because their noses are close to its curb.
func HasRoundaboutPriority(rb Roundabout, body OBB) bool {
	radial := body.Center.Sub(rb.Center)
	radius := radial.Length()
	if radius < rb.InnerRadius || radius > rb.Radius+body.HalfWid+6 {
		return false
	}
	tangent := Vec2{X: radial.Y, Y: -radial.X}.Normalize()
	return tangent.Dot(UnitFromAngle(body.Heading)) > .7
}

// PredictRoundaboutBody follows circulating traffic along the ring and other
// traffic along its heading. Callers supply their own safety margins.
func PredictRoundaboutBody(rb Roundabout, body OBB, speed, elapsed float64) OBB {
	radial := body.Center.Sub(rb.Center)
	radius := radial.Length()
	forward := UnitFromAngle(body.Heading)
	tangent := Vec2{X: radial.Y, Y: -radial.X}.Normalize()
	alignment := forward.Dot(tangent)
	if radius >= rb.InnerRadius && radius <= rb.Radius+10 && math.Abs(alignment) > .7 {
		turn := -math.Copysign(1, alignment) * speed * elapsed / radius
		s, c := math.Sincos(turn)
		body.Center = rb.Center.Add(Vec2{X: radial.X*c - radial.Y*s, Y: radial.X*s + radial.Y*c})
		body.Heading += turn
	} else {
		body.Center = body.Center.Add(forward.Mul(speed * elapsed))
	}
	return body
}

// RoundaboutEntryArrival includes acceleration from a stopped yield line.
func RoundaboutEntryArrival(distance, speed float64) float64 {
	const acceleration = 100.
	speed = Clamp(speed, 0, RoundaboutSpeedLimit)
	ramp := (RoundaboutSpeedLimit - speed) / acceleration
	rampDistance := (speed + RoundaboutSpeedLimit) * ramp / 2
	if distance < rampDistance {
		return (math.Sqrt(speed*speed+2*acceleration*distance) - speed) / acceleration
	}
	return ramp + (distance-rampDistance)/RoundaboutSpeedLimit
}
