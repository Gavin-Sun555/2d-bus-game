package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func moveCar(car *sim.TrafficCar, dt float64) {
	if math.Abs(car.Speed) < 1e-6 {
		return
	}
	// 3. Move along Heading or Circulate in Roundabout
	if car.JunctionTurn != nil {
		moveCarThroughJunction(car, dt)
	} else if car.InRoundabout {
		moveCarRoundabout(car, dt)
	} else if car.Speed > 0 {
		car.Pos.X += car.Speed * math.Cos(car.Heading) * dt
		car.Pos.Y += car.Speed * math.Sin(car.Heading) * dt
	}

}
