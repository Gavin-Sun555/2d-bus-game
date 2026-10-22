package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// Draw renders all active AI cars
func DrawTraffic(tm *traffic.TrafficManager, screen *ebiten.Image, cam *sim.Camera, densityLevel int) {
	activeCount := len(tm.GetActiveCars(densityLevel))

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]
		drawSingleCar(screen, cam, car)
	}
}

func drawSingleCar(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar) {
	// Keep the full headlight cone visible when the vehicle itself is offscreen.
	if !cam.IsPointVisible(c.Pos, math.Hypot(c.Length, c.Width)/2+150) {
		return
	}
	cosH := math.Cos(c.Heading)
	sinH := math.Sin(c.Heading)

	// Car shadow
	shadowOffset := sim.Vec2{X: 4, Y: 5}
	drawRotatedBox(screen, cam, c.Pos.Add(shadowOffset), cosH, sinH, c.Length, c.Width, sim.RGBA(2, 6, 23, 90))

	if cam.Zoom <= .5 && drawCachedCar(screen, cam, c) {
		return
	}
	drawCarDetails(screen, cam, c)
}

func drawCarDetails(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar) {
	drawCarDetailsWithBeams(screen, cam, c, cam.Zoom >= .20)
}

func drawCarDetailsWithBeams(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar, beams bool) {
	cosH, sinH := math.Cos(c.Heading), math.Sin(c.Heading)
	// Main Car Body
	drawRotatedBox(screen, cam, c.Pos, cosH, sinH, c.Length, c.Width, c.Color)

	// Front & Rear Windows
	wsPos := c.Pos.Add(sim.Vec2{X: cosH * (c.Length * 0.18), Y: sinH * (c.Length * 0.18)})
	drawRotatedBox(screen, cam, wsPos, cosH, sinH, 9, c.Width-6, sim.RGBA(30, 41, 59, 230))

	rwPos := c.Pos.Add(sim.Vec2{X: -cosH * (c.Length * 0.22), Y: -sinH * (c.Length * 0.22)})
	drawRotatedBox(screen, cam, rwPos, cosH, sinH, 7, c.Width-6, sim.RGBA(30, 41, 59, 230))

	roofPos := c.Pos.Add(sim.Vec2{X: -cosH * 2, Y: -sinH * 2})
	drawRotatedBox(screen, cam, roofPos, cosH, sinH, c.Length*0.35, c.Width-6, c.Color)

	if c.Type == sim.CarTaxi {
		drawRotatedBox(screen, cam, roofPos, cosH, sinH, 12, 6, sim.RGBA(254, 240, 138, 255))
	}

	// Headlights (Front Left & Right)
	perpX := -sinH
	perpY := cosH
	hlDist := c.Length / 2
	hlOffset := c.Width/2 - 3
	hl1 := c.Pos.Add(sim.Vec2{X: cosH*hlDist - perpX*hlOffset, Y: sinH*hlDist - perpY*hlOffset})
	hl2 := c.Pos.Add(sim.Vec2{X: cosH*hlDist + perpX*hlOffset, Y: sinH*hlDist + perpY*hlOffset})
	drawRotatedBox(screen, cam, hl1, cosH, sinH, 3, 4, sim.RGBA(254, 240, 138, 255))
	drawRotatedBox(screen, cam, hl2, cosH, sinH, 3, 4, sim.RGBA(254, 240, 138, 255))

	// Soft Headlight beams forward for BOTH left and right headlights
	if beams {
		drawCarHeadlightBeam(screen, cam, hl1, cosH, sinH, perpX, perpY)
		drawCarHeadlightBeam(screen, cam, hl2, cosH, sinH, perpX, perpY)
	}

	// Taillights / Brake lights (Rear)
	tl1 := c.Pos.Add(sim.Vec2{X: -cosH*hlDist - perpX*hlOffset, Y: -sinH*hlDist - perpY*hlOffset})
	tl2 := c.Pos.Add(sim.Vec2{X: -cosH*hlDist + perpX*hlOffset, Y: -sinH*hlDist + perpY*hlOffset})
	tlColor := sim.RGBA(185, 28, 28, 255)
	if c.IsBraking {
		tlColor = sim.RGBA(239, 68, 68, 255)
	}
	drawRotatedBox(screen, cam, tl1, cosH, sinH, 3, 4, tlColor)
	drawRotatedBox(screen, cam, tl2, cosH, sinH, 3, 4, tlColor)
}

func drawCarHeadlightBeam(screen *ebiten.Image, cam *sim.Camera, startPos sim.Vec2, cosH, sinH, perpX, perpY float64) {
	beamLen := 52.0
	beamSpread := 11.0
	p1 := cam.WorldToScreen(startPos)
	endCenter := startPos.Add(sim.Vec2{X: cosH * beamLen, Y: sinH * beamLen})
	p2 := cam.WorldToScreen(endCenter.Add(sim.Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))
	p3 := cam.WorldToScreen(endCenter.Sub(sim.Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))
	pMid := cam.WorldToScreen(endCenter)

	beamColor := sim.RGBA(254, 240, 138, 28)
	coreColor := sim.RGBA(254, 240, 138, 75)
	zoom := float32(cam.Zoom)

	// Small glowing lamp halo at fixture
	filledCircle(screen, float32(p1.X), float32(p1.Y), 3.0*zoom, sim.RGBA(254, 240, 138, 85), true)

	// Center beam ray
	strokeLine(screen, float32(p1.X), float32(p1.Y), float32(pMid.X), float32(pMid.Y), 1.5*zoom, coreColor, true)
	// Outer beam cone edge lines
	strokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y), 1.0*zoom, beamColor, true)
	strokeLine(screen, float32(p1.X), float32(p1.Y), float32(p3.X), float32(p3.Y), 1.0*zoom, beamColor, true)
}
