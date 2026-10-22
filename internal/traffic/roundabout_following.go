package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Proximity alone does not confer priority. A lead car must clear the entry or
// circulating lane instead of waiting for a fully separated follower behind it.
// Use the other vehicle's entire yawed body, rather than only its center.
func roundaboutVehicleFollowing(car *sim.TrafficCar, pos sim.Vec2, heading, length, width float64) bool {
	difference := heading - car.Heading
	if math.Cos(difference) < .85 {
		return false
	}
	forward := sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}
	extent := length/2*math.Abs(math.Cos(difference)) + width/2*math.Abs(math.Sin(difference))
	return pos.Sub(car.Pos).Dot(forward)+extent < -car.Length/2-3
}

func roundaboutBusFollowing(car *sim.TrafficCar, bus *sim.Bus) bool {
	return bus != nil && roundaboutVehicleFollowing(car, bus.Pos, bus.Heading, bus.Length, bus.Width)
}

// Follow a moving leader continuously. A fixed angular stop zone makes every
// following car brake to zero even when the bus ahead is travelling normally.
func roundaboutBusHeadway(car *sim.TrafficCar, bus *sim.Bus) float64 {
	radial := bus.Pos.Sub(car.RoundaboutCenter)
	angle := math.Atan2(radial.Y, radial.X)
	difference := sim.NormalizeAngle(car.RoundaboutAngle - angle)
	tangent := sim.Vec2{X: radial.Y, Y: -radial.X}.Normalize()
	circulating := tangent.Dot(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}) > .85
	if circulating && car.RoundaboutCurve == nil && difference > 0 && difference < 1.3 && math.Abs(car.RoundaboutTargetR-radial.Length()) <= 45 {
		gap := difference*math.Min(car.RoundaboutRadius, radial.Length()) - car.Length/2 - bus.Length/2
		if gap <= 3 {
			return 0
		}
		return math.Max(0, math.Max(0, bus.Speed)+1.6*(gap-24))
	}
	if car.Pos.Distance(bus.Pos) < 85 {
		return 0
	}
	if math.Abs(car.RoundaboutTargetR-radial.Length()) <= 45 {
		if difference > 0 && difference < .7 {
			return 0
		}
		if difference >= .7 && difference < 1.2 {
			return math.Max(0, (difference-.65)*160)
		}
	}
	return math.Inf(1)
}
