package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/hajimehoshi/ebiten/v2"
)

// DrawVehicleTurnSignals runs after ambient lighting so amber lamps stay
// visible at night, including on cars rendered from the body sprite cache.
func DrawVehicleTurnSignals(screen *ebiten.Image, cam *sim.Camera, player *sim.Bus, cars *traffic.TrafficManager, fleet *traffic.AIBusManager, density int) {
	drawBusTurnSignals(screen, cam, player)
	if fleet != nil {
		for _, bus := range fleet.Buses {
			drawBusTurnSignals(screen, cam, bus.Bus)
		}
	}
	for _, car := range cars.GetActiveCars(density) {
		side := car.TurnSignalSide()
		if side == 0 || !cam.IsPointVisible(car.Pos, car.Length/2+12) {
			continue
		}
		drawTurnLamp(screen, cam, car.Pos, car.VisualHeading(), car.Length/2-4.5, car.Width, side, 3.5)
		drawTurnLamp(screen, cam, car.Pos, car.VisualHeading(), -car.Length/2+4.5, car.Width, side, 3.5)
	}
}

func drawBusTurnSignals(screen *ebiten.Image, cam *sim.Camera, bus *sim.Bus) {
	side := bus.TurnSignalSide()
	if side == 0 || !cam.IsPointVisible(bus.Pos, bus.OverallLength()+12) {
		return
	}
	drawTurnLamp(screen, cam, bus.Pos, bus.Heading, bus.Length/2-5, bus.Width, side, 4.5)
	rear := bus.BodyParts()[0]
	if bus.IsArticulated() {
		rear = bus.TrailerBody()
	}
	drawTurnLamp(screen, cam, rear.Center, rear.Heading, -rear.HalfLen+5, 2*rear.HalfWid, side, 4.5)
}

func drawTurnLamp(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, heading, along, width float64, side int, size float64) {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	p := center.Add(u.Mul(along)).Add(r.Mul(float64(side) * (width/2 - 2.8)))
	drawGlow(screen, cam, p, size*2.3, size*2.3, 0, 1, .58, .09, .6)
	drawRotatedBox(screen, cam, p, u.X, u.Y, size, size*.8, rgb(255, 182, 52))
}
