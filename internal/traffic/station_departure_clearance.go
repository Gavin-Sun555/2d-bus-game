package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Check the entire rotated bus, not the inflated forward-detection corridor.
// A waiting bus outside this swept lane must leave road traffic free to pass.
func busClearsCarLane(car *sim.TrafficCar, bus *sim.Bus) bool {
	right := sim.Vec2{X: -math.Sin(car.Heading), Y: math.Cos(car.Heading)}
	for _, body := range bus.BodyParts() {
		if body.HalfLen == 0 {
			continue
		}
		yaw := body.Heading - car.Heading
		extent := body.HalfLen*math.Abs(math.Sin(yaw)) + body.HalfWid*math.Abs(math.Cos(yaw))
		if math.Abs(body.Center.Sub(car.Pos).Dot(right))-extent-car.Width/2 <= 2 {
			return false
		}
	}
	return true
}

func canPassWaitingDepartureBus(f carFrame, bus *sim.Bus) bool {
	if f.world == nil || math.Abs(bus.Speed) >= 3 || f.car.JunctionTurn != nil || !busClearsCarLane(f.car, bus) || f.world.InsideJunction(f.car.Pos, f.laneWidth, 0) || f.world.InsideJunction(bus.Pos, f.laneWidth, 0) {
		return false
	}
	for _, stop := range f.world.Stops {
		if math.Cos(f.car.Heading-stop.Heading) < .98 || math.Cos(bus.Heading-stop.Heading) < .8 {
			continue
		}
		forward := sim.UnitFromAngle(stop.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		along := bus.Pos.Sub(stop.RoadCenter).Dot(forward)
		curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
		if along >= -50 && along <= 350 && bus.Pos.Sub(stop.RoadCenter).Dot(right) > curb+3 {
			return true
		}
	}
	return false
}
