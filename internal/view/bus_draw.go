package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Draw renders the bus and its components in world space
func DrawBus(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	// 1. Draw tire skid marks
	for _, mark := range b.TireTracks {
		s1 := cam.WorldToScreen(mark.Pos1)
		s2 := cam.WorldToScreen(mark.Pos2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(5*cam.Zoom), sim.RGBA(30, 30, 30, uint8(mark.Alpha*255)), true)
	}

	// 2. Draw vehicle shadow (subtle blur offset)
	shadowOffset := sim.Vec2{X: 6, Y: 8}
	drawBusBody(screen, cam, b.Pos.Add(shadowOffset), b.Heading, b.Length, b.Width,
		sim.RGBA(15, 20, 25, 90), false)

	// 3. Draw Steerable Front Wheels
	drawBusWheels(b, screen, cam)

	// 4. Draw Main Bus Body
	// Modern Transit Livery: Teal / Turquoise Metallic Body (#0ea5e9 / #0284c7)
	bodyColor := sim.RGBA(14, 165, 233, 255)
	drawBusBody(screen, cam, b.Pos, b.Heading, b.Length, b.Width, bodyColor, true)

	// 5. Draw Roof Details, Windows & Air Conditioner Unit
	drawBusRoof(b, screen, cam)

	// 6. Draw Lights
	drawBusLights(b, screen, cam)

	// 7. Draw Doors animation on the right side
	drawBusDoors(b, screen, cam)
}

func drawBusWheels(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.WheelBase / 2
	rearDist := -b.WheelBase / 2
	halfWidth := b.Width/2 - 2

	wheelLen := 18.0
	wheelWidth := 7.0

	// Front steerable wheels
	frontAngle := b.Heading + b.SteeringAngle
	cosF := math.Cos(frontAngle)
	sinF := math.Sin(frontAngle)

	flPos := sim.Vec2{
		X: b.Pos.X + cosH*frontDist - perpX*halfWidth,
		Y: b.Pos.Y + sinH*frontDist - perpY*halfWidth,
	}
	frPos := sim.Vec2{
		X: b.Pos.X + cosH*frontDist + perpX*halfWidth,
		Y: b.Pos.Y + sinH*frontDist + perpY*halfWidth,
	}

	drawRotatedBox(screen, cam, flPos, cosF, sinF, wheelLen, wheelWidth, sim.RGBA(35, 38, 45, 255))
	drawRotatedBox(screen, cam, frPos, cosF, sinF, wheelLen, wheelWidth, sim.RGBA(35, 38, 45, 255))

	// Rear dual wheels (fixed heading)
	rlPos := sim.Vec2{
		X: b.Pos.X + cosH*rearDist - perpX*halfWidth,
		Y: b.Pos.Y + sinH*rearDist - perpY*halfWidth,
	}
	rrPos := sim.Vec2{
		X: b.Pos.X + cosH*rearDist + perpX*halfWidth,
		Y: b.Pos.Y + sinH*rearDist + perpY*halfWidth,
	}
	drawRotatedBox(screen, cam, rlPos, cosH, sinH, wheelLen, wheelWidth, sim.RGBA(30, 32, 38, 255))
	drawRotatedBox(screen, cam, rrPos, cosH, sinH, wheelLen, wheelWidth, sim.RGBA(30, 32, 38, 255))
}

func drawBusRoof(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)

	// Dark glass roof panel
	roofPos := b.Pos.Add(sim.Vec2{X: -cosH * 2, Y: -sinH * 2})
	drawRotatedBox(screen, cam, roofPos, cosH, sinH, b.Length-24, b.Width-10, sim.RGBA(15, 23, 42, 230))

	// Front Windshield (curved gradient glass look)
	windshieldPos := b.Pos.Add(sim.Vec2{X: cosH * (b.Length/2 - 12), Y: sinH * (b.Length/2 - 12)})
	drawRotatedBox(screen, cam, windshieldPos, cosH, sinH, 12, b.Width-8, sim.RGBA(56, 189, 248, 220))

	// AC Unit / Roof Ventilation pods
	acPos := b.Pos.Add(sim.Vec2{X: -cosH * 16, Y: -sinH * 16})
	drawRotatedBox(screen, cam, acPos, cosH, sinH, 24, 18, sim.RGBA(241, 245, 249, 255))
	// AC vents lines
	ventColor := sim.RGBA(148, 163, 184, 255)
	for i := -8; i <= 8; i += 4 {
		p := acPos.Add(sim.Vec2{X: cosH * float64(i), Y: sinH * float64(i)})
		drawRotatedBox(screen, cam, p, cosH, sinH, 2, 14, ventColor)
	}

	// Bus Number/Route display on front forehead
	destPos := b.Pos.Add(sim.Vec2{X: cosH * (b.Length/2 - 5), Y: sinH * (b.Length/2 - 5)})
	drawRotatedBox(screen, cam, destPos, cosH, sinH, 4, 20, sim.RGBA(250, 204, 21, 255)) // Glowing Amber LED banner
}

func drawBusLights(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.Length / 2
	rearDist := -b.Length / 2
	lightOffset := b.Width/2 - 5

	// Headlights (Front Left & Right)
	hlPos := sim.Vec2{X: b.Pos.X + cosH*frontDist - perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist - perpY*lightOffset}
	hrPos := sim.Vec2{X: b.Pos.X + cosH*frontDist + perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist + perpY*lightOffset}

	drawRotatedBox(screen, cam, hlPos, cosH, sinH, 4, 6, sim.RGBA(254, 240, 138, 255))
	drawRotatedBox(screen, cam, hrPos, cosH, sinH, 4, 6, sim.RGBA(254, 240, 138, 255))

	// Headlight Beam cone forward
	drawBusHeadlightBeam(b, screen, cam, hlPos)
	drawBusHeadlightBeam(b, screen, cam, hrPos)

	// Taillights / Brake lights (Rear Left & Right)
	tlPos := sim.Vec2{X: b.Pos.X + cosH*rearDist - perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist - perpY*lightOffset}
	trPos := sim.Vec2{X: b.Pos.X + cosH*rearDist + perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist + perpY*lightOffset}

	tailColor := sim.RGBA(153, 27, 27, 255) // Dim red
	if b.IsBraking || b.IsHandbraking {
		tailColor = sim.RGBA(239, 68, 68, 255) // Bright braking red
	} else if b.IsReversing {
		tailColor = sim.RGBA(255, 255, 255, 255) // White reverse lights
	}

	drawRotatedBox(screen, cam, tlPos, cosH, sinH, 4, 6, tailColor)
	drawRotatedBox(screen, cam, trPos, cosH, sinH, 4, 6, tailColor)
}

func drawBusHeadlightBeam(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera, startPos sim.Vec2) {
	beamLen := 120.0
	beamSpread := 24.0
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	p1 := cam.WorldToScreen(startPos)
	endCenter := startPos.Add(sim.Vec2{X: cosH * beamLen, Y: sinH * beamLen})
	p2 := cam.WorldToScreen(endCenter.Add(sim.Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))
	p3 := cam.WorldToScreen(endCenter.Sub(sim.Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))

	// Soft semi-transparent light cone
	beamColor := sim.RGBA(254, 240, 138, 25)
	vector.DrawFilledCircle(screen, float32(p1.X), float32(p1.Y), float32(6*cam.Zoom), sim.RGBA(254, 240, 138, 80), true)

	// Draw lines for beam cone edges
	vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y), float32(1.5*cam.Zoom), beamColor, true)
	vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p3.X), float32(p3.Y), float32(1.5*cam.Zoom), beamColor, true)
}

func drawBusDoors(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	// Doors are on the passenger side (Right side: +perp)
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	doorOffset := b.Width/2 - 1
	// Front door & Middle door
	fDoorPos := sim.Vec2{X: b.Pos.X + cosH*24 + perpX*doorOffset, Y: b.Pos.Y + sinH*24 + perpY*doorOffset}
	mDoorPos := sim.Vec2{X: b.Pos.X - cosH*10 + perpX*doorOffset, Y: b.Pos.Y - sinH*10 + perpY*doorOffset}

	doorColor := sim.RGBA(15, 23, 42, 255)
	if b.DoorProgress > 0.1 {
		// Open door indicator glow
		doorColor = sim.RGBA(34, 197, 94, uint8(b.DoorProgress*255))
	}

	drawRotatedBox(screen, cam, fDoorPos, cosH, sinH, 14, 3, doorColor)
	drawRotatedBox(screen, cam, mDoorPos, cosH, sinH, 14, 3, doorColor)
}

func drawBusBody(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, heading, length, width float64, c color.RGBA, rounded bool) {
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	drawRotatedBox(screen, cam, center, cosH, sinH, length, width, c)

	if rounded {
		// Front aerodynamic bumper
		frontCenter := center.Add(sim.Vec2{X: cosH * (length/2 - 2), Y: sinH * (length/2 - 2)})
		sc := cam.WorldToScreen(frontCenter)
		radius := float32((width / 2) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(sc.X), float32(sc.Y), radius, c, true)
	}
}

func drawRotatedBox(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, wid float64, c color.RGBA) {
	radius := math.Hypot(len, wid) * 0.5
	if !cam.IsPointVisible(center, radius) {
		return
	}

	perpX := -sinH
	perpY := cosH

	hl := len / 2
	hw := wid / 2

	c1 := center.Add(sim.Vec2{X: cosH*hl - perpX*hw, Y: sinH*hl - perpY*hw})
	c2 := center.Add(sim.Vec2{X: cosH*hl + perpX*hw, Y: sinH*hl + perpY*hw})
	c3 := center.Add(sim.Vec2{X: -cosH*hl + perpX*hw, Y: -sinH*hl + perpY*hw})
	c4 := center.Add(sim.Vec2{X: -cosH*hl - perpX*hw, Y: -sinH*hl - perpY*hw})

	s1 := cam.WorldToScreen(c1)
	s2 := cam.WorldToScreen(c2)
	s3 := cam.WorldToScreen(c3)
	s4 := cam.WorldToScreen(c4)

	cr := float32(c.R) / 255
	cg := float32(c.G) / 255
	cb := float32(c.B) / 255
	ca := float32(c.A) / 255

	// Zero-allocation quad rendering
	sharedQuadVertices[0] = ebiten.Vertex{DstX: float32(s1.X), DstY: float32(s1.Y), ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	sharedQuadVertices[1] = ebiten.Vertex{DstX: float32(s2.X), DstY: float32(s2.Y), ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	sharedQuadVertices[2] = ebiten.Vertex{DstX: float32(s3.X), DstY: float32(s3.Y), ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	sharedQuadVertices[3] = ebiten.Vertex{DstX: float32(s4.X), DstY: float32(s4.Y), ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}

	whiteImg := getWhiteSubImage()
	screen.DrawTriangles(sharedQuadVertices[:], sharedQuadIndices, whiteImg, &sharedDrawTrianglesOpts)
}

var whiteImage *ebiten.Image

func getWhiteSubImage() *ebiten.Image {
	if whiteImage == nil {
		whiteImage = ebiten.NewImage(3, 3)
		whiteImage.Fill(color.White)
	}
	return whiteImage
}

var (
	sharedQuadIndices       = []uint16{0, 1, 2, 0, 2, 3}
	sharedDrawTrianglesOpts = ebiten.DrawTrianglesOptions{}
	sharedQuadVertices      = [4]ebiten.Vertex{}
)
