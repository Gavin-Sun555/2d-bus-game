package sim

import (
	"math"
)

// IsBusFacingWrongWay checks whether the player bus is driving or facing the wrong way (逆行) on a roadway.
// A bus in normal traffic lanes (顺行) or within an intersection is NOT wrong-way.
func IsBusFacingWrongWay(bus *Bus) bool {
	if bus == nil {
		return false
	}

	ys := []float64{-1200, -600, 0, 600, 1200}
	xs := []float64{-2600, -1300, 0, 1300, 2600}

	// 1. If bus is inside or close to an intersection, it is turning or navigating, not wrong-way
	for _, x := range xs {
		for _, y := range ys {
			if math.Hypot(bus.Pos.X-x, bus.Pos.Y-y) < 120.0 {
				return false
			}
		}
	}

	effHeading := bus.Heading
	if bus.Speed < -1.0 {
		effHeading = NormalizeAngle(bus.Heading + math.Pi)
	}

	// Find closest East-West road
	minDistEW := math.MaxFloat64
	closestY := 0.0
	for _, y := range ys {
		d := math.Abs(bus.Pos.Y - y)
		if d < minDistEW {
			minDistEW = d
			closestY = y
		}
	}

	// Find closest North-South road
	minDistNS := math.MaxFloat64
	closestX := 0.0
	for _, x := range xs {
		d := math.Abs(bus.Pos.X - x)
		if d < minDistNS {
			minDistNS = d
			closestX = x
		}
	}

	if minDistEW <= minDistNS && minDistEW < 160.0 {
		// East-West road:
		// South side (Y > closestY + 16): legal flow is East (heading ~ 0, cos > 0)
		// North side (Y < closestY - 16): legal flow is West (heading ~ pi, cos < 0)
		if bus.Pos.Y > closestY+16.0 {
			if math.Cos(effHeading) < -0.35 {
				return true
			}
		} else if bus.Pos.Y < closestY-16.0 {
			if math.Cos(effHeading) > 0.35 {
				return true
			}
		}
	} else if minDistNS < 160.0 {
		// North-South road:
		// West side (X < closestX - 16): legal flow is South (heading ~ pi/2, sin > 0)
		// East side (X > closestX + 16): legal flow is North (heading ~ -pi/2, sin < 0)
		if bus.Pos.X < closestX-16.0 {
			if math.Sin(effHeading) < -0.35 {
				return true
			}
		} else if bus.Pos.X > closestX+16.0 {
			if math.Sin(effHeading) > 0.35 {
				return true
			}
		}
	}

	return false
}
