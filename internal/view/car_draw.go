package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"math"
)

func DrawTraffic(tm *traffic.TrafficManager, screen *ebiten.Image, cam *sim.Camera, densityLevel int) {
	for _, c := range tm.GetActiveCars(densityLevel) {
		drawSingleCar(screen, cam, c)
	}
}
func drawSingleCar(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar) {
	if !cam.IsPointVisible(c.Pos, math.Hypot(c.Length, c.Width)/2+150) {
		return
	}
	u := sim.UnitFromAngle(c.VisualHeading())
	drawRotatedBox(screen, cam, c.Pos.Add(sim.Vec2{X: 3, Y: 4}), u.X, u.Y, c.Length, c.Width, sim.RGBA(2, 6, 23, 90))
	if cam.Zoom <= .5 && drawCachedCar(screen, cam, c) {
		return
	}
	drawCarDetails(screen, cam, c)
}
func drawCarDetails(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar) {
	drawCarDetailsWithBeams(screen, cam, c, cam.Zoom >= .20)
}
func drawCarDetailsWithBeams(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar, beams bool) {
	u := sim.UnitFromAngle(c.VisualHeading())
	r := sim.Vec2{X: -u.Y, Y: u.X}
	// Four tires, bevelled body and shaped glass distinguish a car from a box.
	for _, axle := range []float64{-.30, .29} {
		for _, side := range []float64{-1, 1} {
			p := c.Pos.Add(u.Mul(c.Length * axle)).Add(r.Mul(side * (c.Width/2 - .3)))
			drawRotatedBox(screen, cam, p, u.X, u.Y, 7, 3, rgb(16, 23, 29))
		}
	}
	cut := math.Min(c.Width*.23, 5.)
	drawVehicleShell(screen, cam, c.Pos, c.VisualHeading(), c.Length, c.Width, c.Color, cut, cut*.7)
	box := func(along, lateral, length, width float64, col [3]uint8) {
		p := c.Pos.Add(u.Mul(along)).Add(r.Mul(lateral))
		drawRotatedBox(screen, cam, p, u.X, u.Y, length, width, rgb(col[0], col[1], col[2]))
	}
	glass := [3]uint8{26, 46, 59}
	box(-c.Length*.06, 0, c.Length*.62, c.Width-5, glass)
	box(c.Length*.18, 0, c.Length*.08, c.Width-8, [3]uint8{76, 109, 127})
	roof := tint(c.Color, rgb(245, 248, 249), .18)
	drawVehicleShell(screen, cam, c.Pos.Sub(u.Mul(c.Length*.06)), c.VisualHeading(), c.Length*.33, c.Width-8, roof, 1, 1)
	// Hood folds, grille and bumpers retain the car's real physical length.
	for _, side := range []float64{-1, 1} {
		box(c.Length*.35, side*(c.Width*.24), c.Length*.17, .7, [3]uint8{170, 180, 183})
		box(0, side*(c.Width/2-2), c.Length*.34, 1.2, [3]uint8{29, 49, 61})
		box(c.Length*.15, side*(c.Width/2+1), 4, 2.5, [3]uint8{42, 52, 62})
	}
	box(c.Length/2-1.6, 0, 1.7, c.Width*.38, [3]uint8{27, 36, 43})
	box(-c.Length/2+1.5, 0, 1.6, c.Width-7, [3]uint8{57, 67, 73})
	if c.Type == sim.CarSUV {
		for _, side := range []float64{-1, 1} {
			box(-2, side*(c.Width/2-4), c.Length*.42, 1.2, [3]uint8{179, 191, 198})
		}
		box(-2, 0, c.Length*.18, c.Width*.38, [3]uint8{36, 57, 70})
	}
	if c.Type == sim.CarTaxi {
		box(-2, 0, 9, 5, [3]uint8{250, 213, 74})
		box(-2, 0, 5, 2, [3]uint8{37, 42, 47})
	}
	lamp := rgb(130, 150, 159)
	if c.HeadlightsOn {
		lamp = rgb(246, 247, 226)
	}
	tail := rgb(94, 25, 33)
	if c.HeadlightsOn {
		tail = rgb(162, 35, 43)
	}
	if c.IsBraking {
		tail = rgb(255, 70, 64)
	}
	for _, side := range []float64{-1, 1} {
		p := c.Pos.Add(u.Mul(c.Length/2 - 1)).Add(r.Mul(side * (c.Width/2 - 3)))
		drawRotatedBox(screen, cam, p, u.X, u.Y, 2.5, 4, lamp)
		if c.HeadlightsOn && beams {
			drawLightCone(screen, cam, p, c.VisualHeading(), 52, 11, .10)
		}
		p = c.Pos.Sub(u.Mul(c.Length/2 - 1)).Add(r.Mul(side * (c.Width/2 - 3)))
		drawRotatedBox(screen, cam, p, u.X, u.Y, 2, 4, tail)
	}
}
