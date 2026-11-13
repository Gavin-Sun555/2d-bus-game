package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// Draw renders the bus and its components in world space
func DrawBus(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	// 1. Draw tire skid marks
	for _, mark := range b.TireTracks {
		s1 := cam.WorldToScreen(mark.Pos1)
		s2 := cam.WorldToScreen(mark.Pos2)
		strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(5*cam.Zoom), sim.RGBA(30, 30, 30, uint8(mark.Alpha*255)), true)
	}

	// 2. Draw vehicle shadow (subtle blur offset)
	shadowOffset := sim.Vec2{X: 6, Y: 8}
	drawBusBody(screen, cam, b.Pos.Add(shadowOffset), b.Heading, b.Length, b.Width,
		sim.RGBA(15, 20, 25, 90), false)

	bodyColor := sim.RGBA(14, 165, 233, 255)
	if spec, ok := sim.FindVehicleSpec(b.CatalogID); ok {
		bodyColor = sim.RGBA(spec.Color[0], spec.Color[1], spec.Color[2], 255)
	}
	drawBusTrailer(b, screen, cam, bodyColor)

	// 3. Draw Steerable Front Wheels
	drawBusWheels(b, screen, cam)

	// 4. Draw Main Bus Body
	// Modern Transit Livery: Teal / Turquoise Metallic Body (#0ea5e9 / #0284c7)
	drawBusBody(screen, cam, b.Pos, b.Heading, b.Length, b.Width, bodyColor, true)

	// 5. Draw Roof Details, Windows & Air Conditioner Unit
	drawCatalogLivery(b, screen, cam)
	drawBusRoof(b, screen, cam)
	drawBusModelDetails(b, screen, cam)

	// 6. Draw Lights
	drawBusLights(b, screen, cam)

	// 7. Draw Doors animation on the right side
	drawBusDoors(b, screen, cam)
}

func drawBusWheels(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	sinH, cosH := math.Sincos(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.WheelBase / 2
	rearDist := -b.WheelBase / 2
	halfWidth := b.Width/2 - 2

	wheelLen := 18.0
	wheelWidth := 7.0

	// Front steerable wheels
	frontAngle := b.Heading + b.SteeringAngle
	sinF, cosF := math.Sincos(frontAngle)

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
	drawBusRoofPanel(b, screen, cam, true)
}
func drawBusRoofPanel(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera, cab bool) {
	u := sim.UnitFromAngle(b.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	body := sim.RGBA(14, 165, 233, 255)
	if s, ok := sim.FindVehicleSpec(b.CatalogID); ok {
		body = sim.RGBA(s.Color[0], s.Color[1], s.Color[2], 255)
	}
	// Pale roof, inset window bands and narrow pillars define the passenger cabin.
	drawRotatedBox(screen, cam, b.Pos.Sub(u.Mul(4)), u.X, u.Y, b.Length*.70, b.Width*.60, tint(body, rgb(230, 237, 240), .82))
	for _, side := range []float64{-1, 1} {
		for i := 0; i < 5; i++ {
			p := b.Pos.Add(u.Mul(-b.Length*.34 + float64(i)*b.Length*.135)).Add(r.Mul(side * (b.Width/2 - 4)))
			drawRotatedBox(screen, cam, p, u.X, u.Y, b.Length*.115, 4, rgb(25, 49, 65))
		}
	}
	for _, d := range []float64{-.20, .04} {
		p := b.Pos.Add(u.Mul(d * b.Length))
		drawRotatedBox(screen, cam, p, u.X, u.Y, 16, 13, rgb(170, 184, 191))
		drawRotatedBox(screen, cam, p.Add(sim.Vec2{X: -1, Y: -1}), u.X, u.Y, 14, 11, rgb(226, 232, 235))
		for vent := -4.; vent <= 4; vent += 2 {
			drawRotatedBox(screen, cam, p.Add(u.Mul(vent)), u.X, u.Y, .7, 8, rgb(105, 124, 135))
		}
	}
	// Rear engine grille and bumper.
	for i := -2; i <= 2; i++ {
		p := b.Pos.Sub(u.Mul(b.Length * .41)).Add(r.Mul(float64(i) * 4))
		drawRotatedBox(screen, cam, p, u.X, u.Y, 5, 1.1, rgb(58, 74, 84))
	}
	drawRotatedBox(screen, cam, b.Pos.Sub(u.Mul(b.Length/2-2)), u.X, u.Y, 2, b.Width-9, rgb(45, 59, 68))
	if cab {
		windshield := b.Pos.Add(u.Mul(b.Length * .34))
		drawRotatedBox(screen, cam, windshield, u.X, u.Y, 12, b.Width-8, rgb(19, 39, 53))
		drawRotatedBox(screen, cam, windshield.Add(u.Mul(2)).Sub(r.Mul(3)), u.X, u.Y, 3, b.Width-16, sim.RGBA(118, 175, 192, 180))
		// Wipers and front destination display sit within the windshield.
		for _, side := range []float64{-1, 1} {
			p := windshield.Add(r.Mul(side * 6))
			drawRotatedBox(screen, cam, p, u.X, u.Y, 1.1, 8, rgb(10, 20, 29))
			mirror := b.Pos.Add(u.Mul(b.Length * .28)).Add(r.Mul(side * (b.Width/2 + 2)))
			drawRotatedBox(screen, cam, mirror, u.X, u.Y, 7, 3, rgb(38, 48, 57))
		}
		drawRotatedBox(screen, cam, b.Pos.Add(u.Mul(b.Length*.43)), u.X, u.Y, 3, b.Width*.47, rgb(236, 181, 70))
	}
}

func drawBusLights(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	sinH, cosH := math.Sincos(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.Length / 2
	rearDist := -b.Length / 2
	lightOffset := b.Width/2 - 5

	// Headlights (Front Left & Right)
	hlPos := sim.Vec2{X: b.Pos.X + cosH*frontDist - perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist - perpY*lightOffset}
	hrPos := sim.Vec2{X: b.Pos.X + cosH*frontDist + perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist + perpY*lightOffset}

	lamp := rgb(109, 132, 145)
	if b.HeadlightsOn {
		lamp = rgb(246, 247, 225)
	}
	drawRotatedBox(screen, cam, hlPos, cosH, sinH, 3, 5, lamp)
	drawRotatedBox(screen, cam, hrPos, cosH, sinH, 3, 5, lamp)

	// Headlight Beam cone forward
	if b.HeadlightsOn {
		drawBusHeadlightBeam(b, screen, cam, hlPos)
		drawBusHeadlightBeam(b, screen, cam, hrPos)
	}

	if b.IsArticulated() {
		return
	}

	// Taillights / Brake lights (Rear Left & Right)
	tlPos := sim.Vec2{X: b.Pos.X + cosH*rearDist - perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist - perpY*lightOffset}
	trPos := sim.Vec2{X: b.Pos.X + cosH*rearDist + perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist + perpY*lightOffset}

	tailColor := sim.RGBA(82, 22, 28, 255)
	if b.HeadlightsOn {
		tailColor = rgb(153, 27, 27)
	}
	if b.IsBraking || b.IsHandbraking {
		tailColor = sim.RGBA(239, 68, 68, 255) // Bright braking red
	} else if b.IsReversing {
		tailColor = sim.RGBA(255, 255, 255, 255) // White reverse lights
	}

	drawRotatedBox(screen, cam, tlPos, cosH, sinH, 4, 6, tailColor)
	drawRotatedBox(screen, cam, trPos, cosH, sinH, 4, 6, tailColor)
}

func drawBusHeadlightBeam(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera, startPos sim.Vec2) {
	drawLightCone(screen, cam, startPos, b.Heading, 110, 23, .10)
}

// Draw the step well from above. The two glass leaves slide into pockets
// at either end, revealing the entry as the door opens.
func drawBusDoor(screen *ebiten.Image, cam *sim.Camera, b *sim.Bus, along float64) {
	u := sim.UnitFromAngle(b.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	sill := b.Pos.Add(u.Mul(along * b.Length)).Add(r.Mul(b.Width/2 - 3.5))
	progress := sim.Clamp(b.DoorProgress, 0, 1)
	open := progress * progress * (3 - 2*progress)
	const halfOpening = 7.5
	drawRotatedBox(screen, cam, sill, u.X, u.Y, 19, 7, rgb(26, 38, 48))
	drawRotatedBox(screen, cam, sill, u.X, u.Y, 15, 5.5, rgb(75, 88, 97))
	drawRotatedBox(screen, cam, sill.Add(r.Mul(2.5)), u.X, u.Y, 15, 1.2, rgb(242, 188, 69))

	leaf := halfOpening * (1 - open)
	if leaf < .3 {
		return
	}
	for _, side := range []float64{-1, 1} {
		center := sill.Add(u.Mul(side * (halfOpening - leaf/2)))
		drawRotatedBox(screen, cam, center, u.X, u.Y, leaf, 6, rgb(17, 31, 43))
		if leaf > 1.2 {
			drawRotatedBox(screen, cam, center, u.X, u.Y, leaf-1.1, 4, rgb(87, 139, 159))
		}
	}
}

func drawBusDoors(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	drawBusDoor(screen, cam, b, .26)
	drawBusDoor(screen, cam, b, -.13)
}

func drawBusBody(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, heading, length, width float64, c color.RGBA, rounded bool) {
	if !rounded {
		u := sim.UnitFromAngle(heading)
		drawRotatedBox(screen, cam, center, u.X, u.Y, length, width, c)
		return
	}
	drawVehicleShell(screen, cam, center, heading, length, width, c, math.Min(7, width*.22), 3)
}

func drawRotatedBox(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, wid float64, c color.RGBA) {
	halfX := (math.Abs(cosH)*len + math.Abs(sinH)*wid) / 2
	halfY := (math.Abs(sinH)*len + math.Abs(cosH)*wid) / 2
	if !cam.IsBoxVisible(center.X-halfX, center.Y-halfY, center.X+halfX, center.Y+halfY) {
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

// Model details share the existing collision envelope. The articulated variant
// has visible bellows connecting its independently tracked trailer.
func drawBusModelDetails(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	u := sim.UnitFromAngle(b.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	switch b.Model {
	case sim.BusDoubleDecker:
		// Two window bands and a raised upper-deck roof outline.
		for _, side := range []float64{-1, 1} {
			p := b.Pos.Add(r.Mul(side * (b.Width/2 - 4)))
			drawRotatedBox(screen, cam, p, u.X, u.Y, b.Length-30, 3, sim.RGBA(250, 204, 21, 255))
			for i := -2; i <= 2; i++ {
				p := b.Pos.Add(u.Mul(float64(i) * 11)).Add(r.Mul(side * (b.Width/2 - 8)))
				drawRotatedBox(screen, cam, p, u.X, u.Y, 7, 4, sim.RGBA(186, 230, 253, 255))
			}
		}
	case sim.BusArticulated:
		joint := b.HitchPosition()
		u = sim.UnitFromAngle(b.Heading - sim.NormalizeAngle(b.Heading-b.TailHeading())/2)
		drawRotatedBox(screen, cam, joint, u.X, u.Y, 21, b.Width, sim.RGBA(30, 41, 59, 255))
		for i := -3; i <= 3; i++ {
			p := joint.Add(u.Mul(float64(i) * 3))
			drawRotatedBox(screen, cam, p, u.X, u.Y, 1.5, b.Width-2, sim.RGBA(148, 163, 184, 255))
		}
	}
}

func drawBusTrailer(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera, livery color.RGBA) {
	if !b.IsArticulated() {
		return
	}
	tail := b.TrailerBody()
	copy := *b
	copy.Pos, copy.Heading = tail.Center, tail.Heading
	copy.Model = sim.BusStandard
	copy.SteeringAngle = 0
	drawBusBody(screen, cam, tail.Center.Add(sim.Vec2{X: 6, Y: 8}), tail.Heading, b.Length, b.Width, sim.RGBA(15, 20, 25, 90), false)
	u := sim.UnitFromAngle(tail.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	axle := b.HitchPosition().Sub(u.Mul(sim.ArticulatedAxleDistance))
	for _, side := range []float64{-1, 1} {
		drawRotatedBox(screen, cam, axle.Add(r.Mul(side*(b.Width/2-2))), u.X, u.Y, 18, 7, sim.RGBA(30, 32, 38, 255))
	}
	drawBusBody(screen, cam, tail.Center, tail.Heading, b.Length, b.Width, livery, true)
	drawBusRoofPanel(&copy, screen, cam, false)
	drawBusDoor(screen, cam, &copy, -.10)
	brake := rgb(82, 22, 28)
	if b.HeadlightsOn {
		brake = rgb(153, 27, 27)
	}
	if b.IsBraking || b.IsHandbraking {
		brake = sim.RGBA(239, 68, 68, 255)
	}
	for _, side := range []float64{-1, 1} {
		p := tail.Center.Sub(u.Mul(b.Length / 2)).Add(r.Mul(side * (b.Width/2 - 5)))
		drawRotatedBox(screen, cam, p, u.X, u.Y, 4, 6, brake)
	}
}
