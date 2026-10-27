package traffic

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

// The ring is a curved movement too. Publish its actual remaining trajectory
// so bus sweeps do not extrapolate a circulating car along a tangent ray and
// accelerate into the lane it is about to occupy.
func publishRoundaboutCorridor(car *sim.TrafficCar) {
	if !car.InRoundabout {
		return
	}
	if car.JunctionCorridor == nil {
		car.JunctionCorridor = &sim.JunctionCorridor{}
	}
	corridor := car.JunctionCorridor
	corridor.Center = car.RoundaboutCenter
	corridor.Remaining = .25
	corridor.Entered = true
	corridor.EntryHeading = car.Heading
	corridor.CachePosition = car.Pos
	corridor.CacheHeading = car.Heading
	corridor.CacheTurn = nil
	corridor.Bodies = corridor.Bodies[:0]
	predicted := *car
	if car.RoundaboutCurve != nil {
		curve := *car.RoundaboutCurve
		predicted.RoundaboutCurve = &curve
	}
	predicted.Speed = sim.RoundaboutSpeedLimit
	for distance := 0.; distance <= 120; distance += 2 {
		corridor.Bodies = append(corridor.Bodies, sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2})
		moveCar(&predicted, 2/sim.RoundaboutSpeedLimit)
	}
	corridor.RebuildBounds()
}
