package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Nearby traffic in another corridor is not evidence that the bus is wedged.
// In particular, a bay departure can wait lawfully behind its lead bus while
// a diagonally parked small car sits on the other side of a clear lateral gap.
func busCarDeadlockCorridor(bus *sim.Bus, car *sim.TrafficCar) bool {
	body := bus.BodyEnvelope()
	u := sim.UnitFromAngle(bus.Heading)
	d := car.Pos.Sub(body.Center)
	yaw := car.Heading - bus.Heading
	carWidth := car.Length/2*math.Abs(math.Sin(yaw)) + car.Width/2*math.Abs(math.Cos(yaw))
	return math.Abs(d.X*u.Y-d.Y*u.X) <= body.HalfWid+carWidth+8
}
