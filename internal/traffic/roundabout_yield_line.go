package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func roundaboutApproachDirection(car *sim.TrafficCar) sim.Vec2 {
	if car.IsEW {
		return sim.Vec2{X: math.Copysign(1, math.Cos(car.Heading))}
	}
	return sim.Vec2{Y: math.Copysign(1, math.Sin(car.Heading))}
}

// Use the approach axis, not radial distance: offset lanes must stop at the
// same transverse line. Include the complete nose even during a steering correction.
func roundaboutYieldGap(car *sim.TrafficCar, rb sim.Roundabout) float64 {
	forward := roundaboutApproachDirection(car)
	angle := car.Heading - math.Atan2(forward.Y, forward.X)
	front := car.Length/2*math.Abs(math.Cos(angle)) + car.Width/2*math.Abs(math.Sin(angle))
	return rb.Center.Sub(car.Pos).Dot(forward) - front - rb.YieldLineDistance()
}
