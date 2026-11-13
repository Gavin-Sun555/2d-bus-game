package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type metroStyle uint8

const (
	metroShops metroStyle = iota
	metroTower
	metroHomes
	metroCourt
	metroRowhouses
	metroGarden
	metroRail
	metroConventionalRail
	metroTerminal
	metroStadium
	metroHospital
	metroCampus
	metroExhibit
	metroCanal
	metroPlaza

	// Coastal Bay landmarks
	coastalPort
	coastalCivic
	coastalBeach
	coastalResort
	coastalBridgeWest
	coastalDeepwaterPort
	coastalBondedPort
	coastalYachtClub
	coastalFishermanWharf
	coastalSailingMarina
	coastalContainerHub
	coastalLighthouse

	// River Valley landmarks
	valleyRailHub
	valleyHSRHQ
	valleyFinancialTowers
	valleyCultureVista
	valleyNorthForest
	valleyLookout
	valleyEcoNorth
	valleyBridgeheadNorth
	valleyBridgeheadSouth
	valleySmartCampus
	valleyCivicCenter
	valleyMountainLookout

	// Twin Cities landmarks
	twinAirportTerminal
	twinAirportLogistics
	twinRoboticsCampus
	twinGigaFactory
	twinRingCampus
	twinUniversity
	twinAIScienceCity
	twinUnicornHQ
	twinIndustrialWest
	twinInterchangeWest
	twinInterchangeEast
	twinSmartCityCenter
)

type metroLot struct {
	footprint     sim.Building
	heading       float64
	style         metroStyle
	variant       int
	landmarkLabel bool
	titleZh       string
	titleEn       string
}

// Local coordinates span the lot from -.5 to .5. The positive y edge is the
// storefront or entrance, facing the adjacent street.
type metroCanvas struct {
	screen         *ebiten.Image
	cam            *sim.Camera
	lot            metroLot
	forward, front sim.Vec2
	width, depth   float64
	detail         bool
	fine           bool
	litWindows     bool
}

func (a metroCanvas) point(x, y float64) sim.Vec2 {
	return a.lot.footprint.Pos.Add(a.forward.Mul(x * a.width)).Add(a.front.Mul(y * a.depth))
}

func (a metroCanvas) rect(x, y, w, h float64, c color.RGBA) {
	drawRotatedBox(a.screen, a.cam, a.point(x, y), a.forward.X, a.forward.Y, w*a.width, h*a.depth, c)
}

func (a metroCanvas) rectPx(x, y, wPx, hPx float64, c color.RGBA) {
	drawRotatedBox(a.screen, a.cam, a.point(x, y), a.forward.X, a.forward.Y, wPx, hPx, c)
}

func (a metroCanvas) line(x1, y1, x2, y2, width float64, c color.RGBA) {
	worldLine(a.screen, a.cam, a.point(x1, y1), a.point(x2, y2), width, c)
}

func (a metroCanvas) shape(points []sim.Vec2, c color.RGBA) {
	var small [12]sim.Vec2
	world := small[:0]
	for _, p := range points {
		world = append(world, a.point(p.X, p.Y))
	}
	worldPolygon(a.screen, a.cam, world, c)
}

func (a metroCanvas) oval(x, y, w, h float64, c color.RGBA) {
	const segments = 24
	if a.cam.Zoom <= .25 {
		center := a.point(x, y)
		drawScenicEllipse(a.screen, a.cam, center, a.forward, a.front, w*a.width/2, h*a.depth/2, c)
		return
	}
	var points [segments]sim.Vec2
	for i := range points {
		sin, cos := math.Sincos(float64(i) * math.Pi * 2 / float64(len(points)))
		points[i] = a.point(x+cos*w/2, y+sin*h/2)
	}
	worldPolygon(a.screen, a.cam, points[:], c)
}

func (a metroCanvas) circlePx(x, y, rPx float64, c color.RGBA) {
	worldCircle(a.screen, a.cam, a.point(x, y), rPx, c)
}

func metroBox(x, y, w, h float64) []sim.Vec2 {
	return []sim.Vec2{{X: x - w/2, Y: y - h/2}, {X: x + w/2, Y: y - h/2}, {X: x + w/2, Y: y + h/2}, {X: x - w/2, Y: y + h/2}}
}

func metroCorners(x, y, w, h, cutX, cutY float64) []sim.Vec2 {
	left, right, top, bottom := x-w/2, x+w/2, y-h/2, y+h/2
	return []sim.Vec2{{X: left + cutX, Y: top}, {X: right - cutX, Y: top}, {X: right, Y: top + cutY}, {X: right, Y: bottom - cutY}, {X: right - cutX, Y: bottom}, {X: left + cutX, Y: bottom}, {X: left, Y: bottom - cutY}, {X: left, Y: top + cutY}}
}

// shell renders a bevelled architectural volume using the same vertex-shaded
// orthographic language as drawVehicleShell, anchored to world-space sunlight
// (top-left lit, bottom-right shaded) so buildings on both sides of a street
// share a single consistent light source.
func (a metroCanvas) shell(x, y, w, h, cutPx float64, c color.RGBA, shadowPx float64) {
	pos := a.point(x, y)
	W, H := w*a.width, h*a.depth
	if W < 2 || H < 2 {
		return
	}
	cut := math.Min(cutPx, math.Min(W, H)*0.28)
	hw, hh := W/2, H/2
	localPts := [8]sim.Vec2{
		{X: hw - cut, Y: -hh},
		{X: hw, Y: -hh + cut},
		{X: hw, Y: hh - cut},
		{X: hw - cut, Y: hh},
		{X: -hw + cut, Y: hh},
		{X: -hw, Y: hh - cut},
		{X: -hw, Y: -hh + cut},
		{X: -hw + cut, Y: -hh},
	}

	if shadowPx > 0 {
		sOff := sim.Vec2{X: shadowPx * 0.72, Y: shadowPx * 0.95}
		var shadowWorld [8]sim.Vec2
		for i, p := range localPts {
			shadowWorld[i] = pos.Add(a.forward.Mul(p.X)).Add(a.front.Mul(p.Y)).Add(sOff)
		}
		worldPolygon(a.screen, a.cam, shadowWorld[:], sim.RGBA(8, 14, 22, 96))
	}

	var v [9]ebiten.Vertex
	center := a.cam.WorldToScreen(pos)
	v[0] = ebiten.Vertex{
		DstX: float32(center.X), DstY: float32(center.Y),
		SrcX: 1, SrcY: 1,
		ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255,
	}
	rad := math.Max(1, math.Hypot(hw, hh))
	for i, p := range localPts {
		wp := pos.Add(a.forward.Mul(p.X)).Add(a.front.Mul(p.Y))
		q := a.cam.WorldToScreen(wp)
		// Sun shines from top-left (-X, -Y in world space).
		sunDot := ((wp.X-pos.X)*(-0.66) + (wp.Y-pos.Y)*(-0.75)) / rad
		shade := 0.94 + sunDot*0.22
		v[i+1] = ebiten.Vertex{
			DstX: float32(q.X), DstY: float32(q.Y),
			SrcX: 1, SrcY: 1,
			ColorR: float32(math.Min(255, math.Max(0, float64(c.R)*shade))) / 255,
			ColorG: float32(math.Min(255, math.Max(0, float64(c.G)*shade))) / 255,
			ColorB: float32(math.Min(255, math.Max(0, float64(c.B)*shade))) / 255,
			ColorA: float32(c.A) / 255,
		}
	}
	a.screen.DrawTriangles(v[:], vehicleShellIndices, getWhiteSubImage(), nil)
}

// archBlock builds a realistic top-down building mass matching the bus/vehicle
// aesthetic: world-space drop shadow, bevelled parapet frame, inset dark-tinted
// perimeter window bays with sky reflection highlights, and recessed roof deck.
func (a metroCanvas) archBlock(x, y, w, h, cutPx float64, frameCol, deckCol color.RGBA, shadowPx float64, windows bool) {
	a.shell(x, y, w, h, cutPx, frameCol, shadowPx)
	W, H := w*a.width, h*a.depth
	if W < 18 || H < 18 {
		return
	}
	rimW := math.Min(10.0, W*0.14) / a.width
	rimH := math.Min(10.0, H*0.14) / a.depth
	a.shell(x, y, w-rimW*1.5, h-rimH*1.5, math.Max(2, cutPx*0.72), tint(deckCol, rgb(12, 22, 32), 0.35), 0)
	a.shell(x, y, w-rimW*2.2, h-rimH*2.2, math.Max(2, cutPx*0.62), deckCol, 0)

	if !a.detail {
		return
	}
	// Subtle structural roof slab seams.
	if a.fine && W > 48 && H > 38 {
		innerW, innerH := w-rimW*2.8, h-rimH*2.8
		stepsX := int(math.Min(6, math.Floor(W/34)))
		for i := 1; i < stepsX; i++ {
			sx := x - innerW/2 + innerW*float64(i)/float64(stepsX)
			a.line(sx, y-innerH/2, sx, y+innerH/2, 1.0, tint(deckCol, frameCol, 0.32))
		}
	}
	if !windows {
		return
	}
	// Perimeter dark-tinted glass window bays & sky reflection strip (bus window style).
	glass := rgb(21, 44, 60)
	glint := rgb(118, 178, 196)
	winDepthX := math.Min(4.2, H*0.08) / a.depth
	winDepthY := math.Min(4.2, W*0.08) / a.width
	spanW := w - (cutPx*2.2+8)/a.width
	spanH := h - (cutPx*2.2+8)/a.depth
	if spanW > 0.06 {
		nCols := int(math.Max(2, math.Min(9, math.Floor((spanW*a.width)/20))))
		cellW := spanW / float64(nCols)
		fy := y + h/2 - rimH*0.52
		if a.litWindows {
			drawGlow(a.screen, a.cam, a.point(x, fy), spanW*a.width*.60, 16, a.lot.heading, 1, .71, .33, .32)
		}
		for i := 0; i < nCols; i++ {
			wx := x - spanW/2 + cellW*(float64(i)+0.5)
			// Street-facing front window bay + specular reflection.
			lit := a.litWindows && (a.lot.variant+i+int((x+.5)*17))%5 < 2
			if lit {
				a.rect(wx, fy, cellW*.82, winDepthX, rgb(255, 216, 143))
			} else {
				a.rect(wx, fy, cellW*.82, winDepthX, glass)
			}
			if a.fine && !lit {
				a.rect(wx, fy+winDepthX*0.26, cellW*0.72, winDepthX*0.32, glint)
			}
			// Rear window bay.
			by := y - h/2 + rimH*0.52
			a.rect(wx, by, cellW*0.80, winDepthX*0.85, glass)
		}
	}
	if spanH > 0.06 && a.fine {
		nRows := int(math.Max(2, math.Min(7, math.Floor((spanH*a.depth)/22))))
		cellH := spanH / float64(nRows)
		for i := 0; i < nRows; i++ {
			wy := y - spanH/2 + cellH*(float64(i)+0.5)
			a.rect(x-w/2+rimW*0.52, wy, winDepthY, cellH*0.78, glass)
			a.rect(x+w/2-rimW*0.52, wy, winDepthY, cellH*0.78, glass)
		}
	}
}

// hvac renders a bus-style rooftop air-conditioning / chiller unit with louver slats.
func (a metroCanvas) hvac(x, y, wPx, hPx float64) {
	if !a.detail {
		return
	}
	pos := a.point(x, y)
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 2.2, Y: 3.0}), a.forward.X, a.forward.Y, wPx, hPx, sim.RGBA(10, 18, 26, 88))
	drawRotatedBox(a.screen, a.cam, pos, a.forward.X, a.forward.Y, wPx, hPx, rgb(162, 178, 186))
	drawRotatedBox(a.screen, a.cam, pos.Sub(a.forward.Mul(0.8)).Sub(a.front.Mul(0.8)), a.forward.X, a.forward.Y, wPx-3, hPx-2.6, rgb(224, 231, 235))
	if a.fine {
		slats := int(math.Max(3, math.Min(6, math.Floor(wPx/3.6))))
		step := (wPx - 6) / float64(slats)
		for i := 0; i < slats; i++ {
			off := -(wPx-6)/2 + step*(float64(i)+0.5)
			p := pos.Add(a.forward.Mul(off))
			drawRotatedBox(a.screen, a.cam, p, a.forward.X, a.forward.Y, 1.0, hPx-5.2, rgb(96, 116, 128))
		}
	}
}

// chillerBank renders a rooftop mechanical cooling tower bank with circular fan shrouds.
func (a metroCanvas) chillerBank(x, y float64, count int) {
	if !a.detail || count <= 0 {
		return
	}
	unitSpan := 15.0
	wPx := float64(count)*unitSpan + 6
	hPx := 17.0
	pos := a.point(x, y)
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 3, Y: 4}), a.forward.X, a.forward.Y, wPx, hPx, sim.RGBA(10, 18, 26, 92))
	drawRotatedBox(a.screen, a.cam, pos, a.forward.X, a.forward.Y, wPx, hPx, rgb(142, 158, 168))
	drawRotatedBox(a.screen, a.cam, pos, a.forward.X, a.forward.Y, wPx-3, hPx-3, rgb(192, 204, 212))
	for i := 0; i < count; i++ {
		off := -float64(count-1)*unitSpan/2 + float64(i)*unitSpan
		fanPos := pos.Add(a.forward.Mul(off))
		worldCircle(a.screen, a.cam, fanPos, 5.4, rgb(32, 48, 60))
		if a.fine {
			worldLine(a.screen, a.cam, fanPos.Sub(a.forward.Mul(4)), fanPos.Add(a.forward.Mul(4)), 1.1, rgb(132, 150, 162))
			worldLine(a.screen, a.cam, fanPos.Sub(a.front.Mul(4)), fanPos.Add(a.front.Mul(4)), 1.1, rgb(132, 150, 162))
			worldCircle(a.screen, a.cam, fanPos, 1.8, rgb(206, 216, 222))
		}
	}
}

// skylight renders a framed glass atrium roof with mullion bars and sky reflections.
func (a metroCanvas) skylight(x, y, w, h float64, cols, rows int) {
	a.rect(x, y, w, h, rgb(176, 192, 202))
	insetW := math.Min(5.0/a.width, w*0.16)
	insetH := math.Min(5.0/a.depth, h*0.16)
	gw, gh := w-insetW, h-insetH
	a.rect(x, y, gw, gh, rgb(19, 41, 56))
	if !a.detail {
		return
	}
	if a.litWindows {
		// A few lit interior rooms show through the glazed roof.
		for i := 0; i < cols; i++ {
			if (a.lot.variant+i)%4 < 2 {
				cell := gw / float64(cols)
				sx := x - gw/2 + (float64(i)+.5)*cell
				a.rect(sx, y, cell*.88, gh*.86, rgb(255, 199, 117))
			}
		}
	} else {
		a.rect(x-gw*0.14, y-gh*0.12, gw*0.58, gh*0.32, rgb(64, 124, 148))
		a.rect(x-gw*0.18, y+gh*0.16, gw*0.42, gh*0.18, rgb(118, 178, 198))
	}
	if a.fine {
		for i := 1; i < cols; i++ {
			sx := x - gw/2 + gw*float64(i)/float64(cols)
			a.line(sx, y-gh/2, sx, y+gh/2, 1.2, rgb(158, 178, 190))
		}
		for j := 1; j < rows; j++ {
			sy := y - gh/2 + gh*float64(j)/float64(rows)
			a.line(x-gw/2, sy, x+gw/2, sy, 1.2, rgb(158, 178, 190))
		}
	}
}

// helipad renders a rooftop aviation landing pad with a properly proportioned 'H'.
func (a metroCanvas) helipad(x, y, rPx float64) {
	if !a.detail {
		return
	}
	pos := a.point(x, y)
	worldCircle(a.screen, a.cam, pos, rPx, rgb(44, 58, 68))
	worldCircle(a.screen, a.cam, pos, rPx*0.82, rgb(232, 186, 68))
	worldCircle(a.screen, a.cam, pos, rPx*0.70, rgb(52, 68, 80))
	barH := rPx * 0.68
	barW := math.Max(2.2, rPx*0.14)
	sep := rPx * 0.26
	drawRotatedBox(a.screen, a.cam, pos.Sub(a.forward.Mul(sep)), a.forward.X, a.forward.Y, barW, barH, rgb(240, 245, 248))
	drawRotatedBox(a.screen, a.cam, pos.Add(a.forward.Mul(sep)), a.forward.X, a.forward.Y, barW, barH, rgb(240, 245, 248))
	drawRotatedBox(a.screen, a.cam, pos, a.forward.X, a.forward.Y, sep*2+barW, barW, rgb(240, 245, 248))
}

// solarBank renders rooftop photovoltaic solar panels.
func (a metroCanvas) solarBank(x, y, w, h float64, rows, cols int) {
	if !a.detail || rows <= 0 || cols <= 0 {
		return
	}
	cellW := w / float64(cols)
	cellH := h / float64(rows)
	for r := 0; r < rows; r++ {
		cy := y - h/2 + cellH*(float64(r)+0.5)
		for c := 0; c < cols; c++ {
			cx := x - w/2 + cellW*(float64(c)+0.5)
			a.rect(cx, cy, cellW*0.88, cellH*0.78, rgb(156, 174, 186))
			a.rect(cx, cy, cellW*0.80, cellH*0.68, rgb(22, 48, 78))
			if a.fine {
				a.line(cx-cellW*0.36, cy, cx+cellW*0.36, cy, 0.9, rgb(58, 98, 136))
			}
		}
	}
}

// miniCar renders a parked sedan/SUV/taxi matching car_draw.go.
func (a metroCanvas) miniCar(x, y, angleOffset float64, body color.RGBA, taxi bool) {
	if !a.detail {
		return
	}
	pos := a.point(x, y)
	heading := a.lot.heading + angleOffset
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	length, width := 36.0, 17.5
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 2.5, Y: 3.5}), u.X, u.Y, length, width, sim.RGBA(6, 12, 22, 92))
	drawVehicleShell(a.screen, a.cam, pos, heading, length, width, body, 3.8, 2.8)
	drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(length*0.05)), u.X, u.Y, length*0.60, width-4.2, rgb(24, 44, 58))
	if a.fine {
		drawRotatedBox(a.screen, a.cam, pos.Add(u.Mul(length*0.17)), u.X, u.Y, length*0.08, width-6.5, rgb(92, 136, 156))
	}
	roof := tint(body, rgb(244, 248, 250), 0.22)
	drawVehicleShell(a.screen, a.cam, pos.Sub(u.Mul(length*0.06)), heading, length*0.34, width-6.4, roof, 1, 1)
	if taxi && a.fine {
		drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(1.5)), u.X, u.Y, 6.5, 3.6, rgb(250, 212, 72))
	}
	for _, side := range []float64{-1, 1} {
		lp := pos.Add(u.Mul(length/2 - 1)).Add(r.Mul(side * (width/2 - 2.4)))
		drawRotatedBox(a.screen, a.cam, lp, u.X, u.Y, 2.0, 3.0, rgb(210, 224, 230))
	}
}

// miniBus renders a parked city transit bus matching bus_draw.go.
func (a metroCanvas) miniBus(x, y, angleOffset float64, body color.RGBA, articulated bool) {
	pos := a.point(x, y)
	heading := a.lot.heading + angleOffset
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	length, width := 72.0, 22.0
	if articulated {
		length = 94.0
	}
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 4.5, Y: 6.0}), u.X, u.Y, length, width, sim.RGBA(10, 16, 24, 95))
	drawVehicleShell(a.screen, a.cam, pos, heading, length, width, body, 4.5, 3.2)
	// Pale roof panel & perimeter windows.
	drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(3)), u.X, u.Y, length*0.72, width*0.62, tint(body, rgb(232, 239, 242), 0.82))
	if a.detail {
		windshield := pos.Add(u.Mul(length * 0.36))
		drawRotatedBox(a.screen, a.cam, windshield, u.X, u.Y, 9.0, width-6.0, rgb(19, 39, 53))
		drawRotatedBox(a.screen, a.cam, windshield.Add(u.Mul(1.5)).Sub(r.Mul(2)), u.X, u.Y, 2.4, width-11, rgb(118, 175, 192))
		drawRotatedBox(a.screen, a.cam, pos.Add(u.Mul(length*0.44)), u.X, u.Y, 2.4, width*0.46, rgb(236, 181, 70))
		for _, side := range []float64{-1, 1} {
			for i := 0; i < 4; i++ {
				wp := pos.Add(u.Mul(-length*0.32 + float64(i)*length*0.16)).Add(r.Mul(side * (width/2 - 2.8)))
				drawRotatedBox(a.screen, a.cam, wp, u.X, u.Y, length*0.12, 3.0, rgb(25, 49, 65))
			}
		}
		for _, d := range []float64{-0.20, 0.06} {
			ac := pos.Add(u.Mul(d * length))
			drawRotatedBox(a.screen, a.cam, ac, u.X, u.Y, 12, 9.5, rgb(170, 184, 191))
			drawRotatedBox(a.screen, a.cam, ac, u.X, u.Y, 10, 7.8, rgb(226, 232, 235))
		}
		if articulated {
			bellows := pos.Sub(u.Mul(length * 0.05))
			drawRotatedBox(a.screen, a.cam, bellows, u.X, u.Y, 9.0, width-2.0, rgb(64, 76, 86))
		}
	}
}

// miniAmbulance renders a white-and-red emergency ambulance at medical centers.
func (a metroCanvas) miniAmbulance(x, y, angleOffset float64) {
	if !a.detail {
		return
	}
	pos := a.point(x, y)
	heading := a.lot.heading + angleOffset
	u := sim.UnitFromAngle(heading)
	length, width := 42.0, 19.0
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 3, Y: 4}), u.X, u.Y, length, width, sim.RGBA(8, 14, 22, 92))
	drawVehicleShell(a.screen, a.cam, pos, heading, length, width, rgb(238, 243, 246), 4.0, 2.5)
	drawRotatedBox(a.screen, a.cam, pos.Add(u.Mul(length*0.28)), u.X, u.Y, 7.5, width-5.0, rgb(22, 44, 58))
	// Emergency red stripe, lightbar and roof red cross.
	drawRotatedBox(a.screen, a.cam, pos.Add(u.Mul(length*0.16)), u.X, u.Y, 3.2, width-6.0, rgb(220, 52, 52))
	drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(length*0.10)), u.X, u.Y, 9.0, 3.2, rgb(214, 48, 48))
	drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(length*0.10)), u.X, u.Y, 3.2, 9.0, rgb(214, 48, 48))
}

func (a metroCanvas) treeCluster(x, y, rPx float64) {
	if !a.detail {
		return
	}
	pos := a.point(x, y)
	worldCircle(a.screen, a.cam, pos, rPx*1.32, rgb(44, 82, 58))
	worldCircle(a.screen, a.cam, pos.Add(sim.Vec2{X: 3, Y: 4}), rPx, sim.RGBA(8, 15, 22, 82))
	worldCircle(a.screen, a.cam, pos, rPx, rgb(30, 82, 54))
	worldCircle(a.screen, a.cam, pos.Add(sim.Vec2{X: -rPx * 0.22, Y: -rPx * 0.22}), rPx*0.72, rgb(62, 128, 80))
	if a.fine {
		worldCircle(a.screen, a.cam, pos.Add(sim.Vec2{X: -rPx * 0.34, Y: -rPx * 0.32}), rPx*0.38, rgb(104, 166, 106))
	}
}

func (a metroCanvas) flowerBed(x, y, w, h float64, bloom color.RGBA) {
	if !a.detail {
		return
	}
	a.oval(x, y, w, h, rgb(38, 78, 52))
	a.oval(x, y, w*0.78, h*0.74, rgb(52, 108, 68))
	a.oval(x-w*0.16, y, w*0.26, h*0.34, bloom)
	a.oval(x+w*0.16, y, w*0.26, h*0.34, bloom)
}

func metropolisVariant(pos sim.Vec2) int {
	// Anchor the pattern in world space. Camera movement never changes it.
	x, y := uint32(int32(pos.X)), uint32(int32(pos.Y))
	h := x*0x9e3779b1 ^ y*0x85ebca77
	h ^= h >> 16
	return int(h % 997)
}

func (a metroCanvas) softPlaza(x, y, w, h, cutX, cutY float64, c color.RGBA) {
	if !a.detail {
		a.rect(x, y, w, h, c)
		return
	}
	bg := rgb(42, 60, 72)
	a.shape(metroCorners(x, y, w+0.07, h+0.07, cutX+0.03, cutY+0.03), tint(bg, c, 0.38))
	a.shape(metroCorners(x, y, w+0.025, h+0.025, cutX+0.015, cutY+0.015), tint(bg, c, 0.72))
	a.shape(metroCorners(x, y, w, h, cutX, cutY), c)
}

func (a metroCanvas) softLawn(x, y, w, h float64) {
	if !a.detail {
		a.rect(x, y, w, h, rgb(64, 112, 76))
		return
	}
	bg := rgb(56, 94, 68)
	turf := rgb(64, 112, 76)
	a.oval(x, y, w+0.10, h+0.12, tint(bg, turf, 0.38))
	a.oval(x, y, w+0.04, h+0.05, tint(bg, turf, 0.72))
	a.oval(x, y, w, h, turf)
}

func (a metroCanvas) lotFoundation() {
	// Buildings, courtyards, and parks sit directly on the continuous city-block
	// ground rendered by drawTerrain / drawScenicFoundations so no lot ever has
	// an artificial rectangular border around it.
}

func (a metroCanvas) drawOverviewLot(plan, night bool) {
	var wall, deck color.RGBA
	switch a.lot.style {
	case metroTower, valleyFinancialTowers, valleyHSRHQ, twinSmartCityCenter, twinUnicornHQ:
		wall = rgb(68, 92, 114)
		deck = rgb(148, 168, 180)
	case metroShops, coastalResort, coastalFishermanWharf, coastalYachtClub:
		wall = rgb(82, 102, 118)
		deck = rgb(154, 166, 174)
	case metroHomes, metroRowhouses:
		wall = rgb(136, 122, 110)
		deck = rgb(172, 158, 144)
	case metroCourt, coastalCivic, valleyCivicCenter, valleyCultureVista:
		wall = rgb(96, 106, 116)
		deck = rgb(156, 162, 166)
	case metroGarden, valleyNorthForest, valleyEcoNorth, coastalBeach:
		wall = rgb(48, 92, 60)
		deck = rgb(64, 112, 74)
	case metroTerminal, metroRail, metroConventionalRail, valleyRailHub, twinAirportTerminal, twinGigaFactory, twinAirportLogistics, coastalPort, coastalDeepwaterPort, coastalContainerHub:
		wall = rgb(104, 116, 126)
		deck = rgb(140, 152, 162)
	case metroStadium, metroHospital, metroCampus, twinUniversity, twinRoboticsCampus, twinAIScienceCity:
		wall = rgb(118, 128, 136)
		deck = rgb(160, 168, 174)
	default:
		wall = rgb(92, 104, 116)
		deck = rgb(146, 156, 164)
	}

	if night {
		wall = tint(wall, rgb(16, 22, 34), 0.55)
		deck = tint(deck, rgb(20, 26, 38), 0.55)
	}

	// Subtle building shadow
	if !plan && !night && a.cam.Zoom >= 0.07 {
		sOff := sim.Vec2{X: 6.0 * a.cam.Zoom, Y: 8.0 * a.cam.Zoom}
		drawRotatedBox(a.screen, a.cam, a.lot.footprint.Pos.Add(sOff), a.forward.X, a.forward.Y, a.width, a.depth, sim.RGBA(8, 14, 22, 80))
	}

	// Architectural mass and roof deck
	a.shell(0, 0, 0.94, 0.92, 5, wall, 0)
	a.shell(0, 0, 0.78, 0.76, 3, deck, 0)
}

func drawMetropolisLot(screen *ebiten.Image, cam *sim.Camera, lot metroLot, plan, night bool) {
	b := lot.footprint
	if !cam.IsBoxVisible(b.Pos.X-b.W/2-35, b.Pos.Y-b.H/2-35, b.Pos.X+b.W/2+35, b.Pos.Y+b.H/2+35) {
		return
	}
	u := sim.UnitFromAngle(lot.heading)
	w, h := b.W, b.H
	if math.Abs(u.Y) > .5 {
		w, h = h, w
	}
	a := metroCanvas{
		screen:  screen,
		cam:     cam,
		lot:     lot,
		forward: u,
		front:   sim.Vec2{X: -u.Y, Y: u.X},
		width:   w,
		depth:   h,
		detail:  !plan && cam.Zoom >= .18,
		fine:    !plan && cam.Zoom >= .32,
	}
	if night && cam.Zoom >= .24 {
		// Fixed by address so the same rooms stay lit while the camera moves.
		switch lot.style {
		case metroRail, metroConventionalRail, metroTerminal, metroHospital, valleyRailHub, twinAirportTerminal, coastalPort, coastalDeepwaterPort:
			a.litWindows = true
		case metroHomes, metroRowhouses:
			a.litWindows = metropolisVariant(b.Pos)%5 < 3
		case metroShops, metroTower, coastalResort, coastalYachtClub, coastalFishermanWharf, coastalSailingMarina,
			valleyFinancialTowers, valleyHSRHQ, twinSmartCityCenter, twinUnicornHQ:
			a.litWindows = metropolisVariant(b.Pos)%5 < 4
		default:
			a.litWindows = metropolisVariant(b.Pos)%5 < 2
		}
	}
	a.lotFoundation()
	if !lot.landmarkLabel && cam.Zoom < .05 {
		return
	}
	// At extreme zoom-out or schematic plan mode, buildings are smaller than 12-16px on screen.
	// Render an optimized architectural footprint massing instead of dozens of subpixel elements.
	if cam.Zoom < 0.11 || (plan && cam.Zoom < 0.18) {
		a.drawOverviewLot(plan, night)
		return
	}
	switch lot.style {
	case metroShops:
		a.shops()
	case metroTower:
		a.tower()
		a.drawLotTitle(-0.12, 0.23, "国际金融中心", "IFC TOWER")
	case metroHomes:
		a.homes()
	case metroCourt:
		a.court()
		a.drawLotTitle(0, -0.22, "文创艺术街区", "CREATIVE DISTRICT")
	case metroRowhouses:
		a.rowhouses()
	case metroGarden:
		a.garden()
		a.drawLotTitle(0, 0.24, "运河生态湿地", "ECO WETLANDS")
	default:
		a.landmark()
	}
}

var metroAwningColors = [...]color.RGBA{
	rgb(196, 74, 62),
	rgb(38, 128, 122),
	rgb(214, 152, 54),
	rgb(58, 108, 164),
	rgb(162, 78, 108),
}

var metroCarPalette = [...]color.RGBA{
	rgb(212, 68, 62),
	rgb(64, 128, 192),
	rgb(226, 232, 238),
	rgb(54, 152, 134),
	rgb(82, 96, 112),
}

func (a metroCanvas) shops() {
	sub := a.lot.variant % 3
	if sub == 1 {
		// Modern Shopping Plaza / Department Store with sculpted stone forecourt & side greenbeds.
		a.softPlaza(-0.08, 0.06, 0.76, 0.72, 0.08, 0.08, rgb(126, 136, 142))
		a.archBlock(-0.10, -0.04, 0.70, 0.60, 12, rgb(74, 96, 112), rgb(138, 154, 164), 14, true)
		a.archBlock(-0.12, -0.05, 0.52, 0.42, 9, rgb(52, 82, 102), rgb(168, 182, 190), 8, false)
		a.skylight(-0.12, -0.05, 0.32, 0.22, 5, 3)
		a.chillerBank(-0.34, -0.18, 3)
		a.hvac(0.12, -0.18, 22, 15)
		// Entrance glass canopy & colorful retail fascia.
		a.rect(-0.10, 0.275, 0.36, 0.085, rgb(32, 78, 102))
		a.rect(-0.10, 0.30, 0.32, 0.032, rgb(124, 186, 206))
		if a.litWindows {
			a.rect(-.10, .315, .29, .028, rgb(255, 220, 157))
		}
		for i := 0; i < 4; i++ {
			c := metroAwningColors[(a.lot.variant+i)%len(metroAwningColors)]
			a.rect(-0.36+float64(i)*0.18, 0.255, 0.13, 0.035, c)
		}
		// Side surface parking bay with parked cars & flanking trees.
		a.rect(0.36, 0.0, 0.18, 0.56, rgb(54, 66, 76))
		for i := 0; i < 3; i++ {
			py := -0.18 + float64(i)*0.18
			a.line(0.30, py-0.07, 0.42, py-0.07, 1.2, rgb(146, 160, 170))
			if (a.lot.variant+i)%2 == 0 {
				a.miniCar(0.36, py, 0, metroCarPalette[(a.lot.variant+i)%len(metroCarPalette)], i == 2)
			}
		}
		a.flowerBed(-0.32, 0.38, 0.18, 0.08, rgb(228, 146, 72))
		a.treeCluster(-0.46, 0.32, 11.5)
		a.treeCluster(0.46, 0.32, 10.5)
		a.treeCluster(-0.45, -0.26, 11.0)
		return
	}
	// Urban Retail Street Row: staggered commercial buildings along a sculpted pedestrian arcade.
	a.softPlaza(0, 0.16, 0.92, 0.58, 0.07, 0.08, rgb(128, 136, 140))
	n := 3 + a.lot.variant%2
	cell := 0.94 / float64(n)
	framePalette := [...]color.RGBA{
		rgb(82, 102, 116),
		rgb(112, 98, 88),
		rgb(74, 94, 104),
		rgb(104, 112, 118),
	}
	deckPalette := [...]color.RGBA{
		rgb(144, 158, 168),
		rgb(168, 156, 144),
		rgb(132, 148, 156),
		rgb(156, 164, 168),
	}
	for i := 0; i < n; i++ {
		x := -0.47 + cell*(float64(i)+0.5)
		c := metroAwningColors[(a.lot.variant+i)%len(metroAwningColors)]
		depth := 0.52 + float64((a.lot.variant+i*2)%3)*0.08
		cy := 0.28 - depth/2
		idx := (a.lot.variant + i) % len(framePalette)
		a.archBlock(x, cy, cell*0.92, depth, 8, framePalette[idx], deckPalette[idx], 11, true)
		a.hvac(x-cell*0.18, cy-depth*0.16, 18, 13)
		if (i+a.lot.variant)%2 == 0 {
			a.skylight(x+cell*0.14, cy+depth*0.04, cell*0.34, depth*0.28, 3, 2)
		} else {
			a.hvac(x+cell*0.16, cy+depth*0.06, 16, 12)
		}
		a.rect(x, 0.285, cell*0.86, 0.075, c)
		if a.litWindows && (a.lot.variant+i)%4 != 0 {
			a.rect(x, .329, cell*.63, .038, rgb(255, 219, 154))
		}
		if a.detail {
			for k := -2; k <= 2; k++ {
				if k%2 == 0 {
					a.rect(x+float64(k)*cell*0.15, 0.285, cell*0.11, 0.075, tint(c, rgb(246, 240, 224), 0.72))
				}
			}
			if i%2 == 0 {
				a.circlePx(x-cell*0.20, 0.37, 5.8, c)
				a.circlePx(x-cell*0.20, 0.37, 2.0, rgb(244, 240, 230))
			}
		}
	}
	a.treeCluster(-0.46, 0.35, 11.0)
	a.treeCluster(0.46, 0.35, 11.0)
	a.treeCluster(-0.34, -0.32, 11.5)
	a.treeCluster(0.34, -0.32, 11.5)
}

func (a metroCanvas) tower() {
	if !a.lot.landmarkLabel {
		switch a.lot.variant % 3 {
		case 0:
			// Stepped Glass Curtain-Wall Office Tower + Retail Atrium Podium.
			a.softPlaza(-0.02, 0.10, 0.84, 0.68, 0.08, 0.08, rgb(124, 134, 140))
			a.archBlock(-0.15, 0.0, 0.56, 0.64, 14, rgb(54, 92, 118), rgb(104, 144, 166), 17, true)
			a.archBlock(-0.16, -0.03, 0.38, 0.44, 10, rgb(40, 76, 100), rgb(132, 170, 188), 9, false)
			a.skylight(-0.16, -0.03, 0.20, 0.22, 3, 3)
			a.chillerBank(-0.16, -0.18, 2)
			a.archBlock(0.28, 0.05, 0.26, 0.52, 8, rgb(138, 134, 122), rgb(182, 176, 162), 12, true)
			a.solarBank(0.28, -0.04, 0.15, 0.24, 2, 2)
			a.hvac(0.28, 0.16, 18, 13)
			a.rect(-0.15, 0.33, 0.26, 0.065, rgb(36, 84, 108))
			a.miniCar(0.24, 0.35, 0, metroCarPalette[a.lot.variant%len(metroCarPalette)], false)
			a.flowerBed(0.06, 0.36, 0.14, 0.07, rgb(228, 146, 72))
			a.treeCluster(-0.46, 0.32, 11.5)
			a.treeCluster(0.45, -0.22, 11.0)
			a.treeCluster(-0.44, -0.28, 11.5)
			return
		case 1:
			// Twin Asymmetric Modern Office Complex with skybridge & forecourt plaza.
			a.softPlaza(0, 0.08, 0.86, 0.68, 0.08, 0.08, rgb(124, 134, 140))
			a.rect(-0.02, -0.02, 0.32, 0.18, rgb(38, 78, 102))
			a.skylight(-0.02, -0.02, 0.24, 0.12, 4, 2)
			a.archBlock(-0.24, -0.02, 0.38, 0.64, 12, rgb(48, 86, 112), rgb(96, 138, 162), 18, true)
			a.archBlock(-0.24, -0.04, 0.26, 0.44, 8, rgb(36, 70, 94), rgb(124, 166, 186), 8, false)
			a.chillerBank(-0.24, -0.08, 2)
			a.archBlock(0.24, 0.02, 0.36, 0.56, 10, rgb(76, 102, 118), rgb(144, 164, 174), 14, true)
			a.skylight(0.24, 0.02, 0.18, 0.26, 3, 3)
			a.hvac(0.24, -0.16, 20, 14)
			a.rect(-0.24, 0.31, 0.22, 0.065, rgb(42, 94, 118))
			a.miniCar(0.18, 0.36, 0, rgb(235, 186, 46), true)
			a.flowerBed(0.0, 0.24, 0.16, 0.10, rgb(214, 92, 84))
			a.treeCluster(-0.46, 0.32, 11.5)
			a.treeCluster(0.46, 0.32, 11.5)
			a.treeCluster(0.0, -0.28, 11.5)
			return
		default:
			// L-Shaped Executive Corporate Headquarters with solar roof & parking court.
			a.softPlaza(0, 0.06, 0.84, 0.68, 0.08, 0.08, rgb(124, 134, 140))
			a.rect(0.20, 0.14, 0.42, 0.38, rgb(54, 66, 76))
			a.archBlock(-0.20, 0.0, 0.44, 0.66, 11, rgb(62, 96, 116), rgb(122, 152, 168), 16, true)
			a.archBlock(0.14, -0.16, 0.56, 0.34, 10, rgb(62, 96, 116), rgb(122, 152, 168), 16, true)
			a.skylight(-0.20, 0.04, 0.22, 0.32, 3, 4)
			a.solarBank(0.18, -0.16, 0.34, 0.18, 2, 4)
			a.chillerBank(-0.20, -0.20, 2)
			a.miniCar(0.14, 0.22, 0, metroCarPalette[(a.lot.variant+1)%len(metroCarPalette)], false)
			a.miniCar(0.32, 0.22, 0, rgb(235, 186, 46), true)
			a.treeCluster(0.02, 0.32, 11.5)
			a.treeCluster(-0.45, -0.24, 11.0)
			a.treeCluster(0.45, 0.32, 11.0)
			return
		}
	}
	// Landmark IFC Skyscraper Complex (#2 国际金融中心):
	a.softPlaza(0, 0.08, 0.88, 0.72, 0.09, 0.09, rgb(126, 136, 142))
	a.archBlock(-0.12, 0.02, 0.62, 0.68, 16, rgb(52, 92, 118), rgb(94, 136, 158), 18, true)
	a.archBlock(-0.13, -0.02, 0.46, 0.50, 13, rgb(40, 78, 104), rgb(114, 162, 182), 12, true)
	a.shell(-0.14, -0.05, 0.34, 0.36, 10, rgb(30, 64, 88), 8)
	a.helipad(-0.14, -0.05, 19)
	a.archBlock(0.30, 0.06, 0.24, 0.52, 8, rgb(148, 142, 126), rgb(188, 182, 164), 12, true)
	a.skylight(0.30, 0.02, 0.13, 0.24, 2, 3)
	a.hvac(0.30, -0.14, 18, 13)
	a.rect(-0.12, 0.34, 0.28, 0.075, rgb(36, 84, 108))
	a.rect(-0.12, 0.34, 0.24, 0.035, rgb(126, 188, 208))
	a.miniCar(0.22, 0.36, 0, rgb(235, 186, 46), true)
	a.flowerBed(0.08, 0.36, 0.14, 0.07, rgb(228, 146, 72))
	a.treeCluster(-0.44, 0.34, 11.5)
	a.treeCluster(0.44, 0.34, 11.5)
}

func (a metroCanvas) homes() {
	// Organic landscaped residential courtyard (soft oval lawns instead of a hard green rectangle).
	a.softLawn(0.14, 0.08, 0.56, 0.52)
	a.softLawn(-0.16, -0.10, 0.44, 0.42)
	// Internal stone garden paths & leisure pool.
	a.rect(0.08, 0.10, 0.42, 0.05, rgb(156, 152, 138))
	a.rect(0.06, 0.16, 0.045, 0.52, rgb(156, 152, 138))
	a.oval(0.24, 0.12, 0.18, 0.22, rgb(186, 180, 164))
	a.oval(0.24, 0.12, 0.14, 0.17, rgb(52, 148, 162))
	a.oval(0.23, 0.11, 0.09, 0.11, rgb(98, 192, 202))
	// West residential slab building.
	a.archBlock(-0.26, -0.02, 0.34, 0.68, 8, rgb(146, 108, 92), rgb(192, 164, 144), 14, true)
	a.hvac(-0.26, -0.22, 18, 13)
	a.solarBank(-0.26, 0.04, 0.20, 0.24, 2, 2)
	// North-east residential slab building.
	a.archBlock(0.16, -0.20, 0.46, 0.32, 8, rgb(156, 116, 98), rgb(202, 174, 152), 14, true)
	a.hvac(0.04, -0.20, 18, 13)
	a.solarBank(0.24, -0.20, 0.18, 0.16, 1, 3)
	if a.detail {
		for i := 0; i < 4; i++ {
			bx := -0.37 + float64(i)*0.074
			a.rect(bx, 0.325, 0.052, 0.032, rgb(124, 168, 184))
		}
		for i := 0; i < 5; i++ {
			bx := -0.02 + float64(i)*0.088
			a.rect(bx, -0.035, 0.062, 0.032, rgb(124, 168, 184))
		}
	}
	a.archBlock(0.02, 0.33, 0.12, 0.075, 3, rgb(92, 108, 118), rgb(154, 166, 172), 6, false)
	a.miniCar(0.36, 0.28, math.Pi/2, rgb(224, 230, 236), false)
	a.treeCluster(0.02, 0.18, 10.5)
	a.treeCluster(0.40, 0.04, 11.5)
	a.treeCluster(-0.04, -0.18, 10.5)
	a.treeCluster(-0.44, 0.30, 10.5)
}

func (a metroCanvas) court() {
	isPost := a.lot.titleZh != "" && len(a.lot.titleZh) >= 6 && (a.lot.titleZh[len(a.lot.titleZh)-6:] == "邮局")
	if !a.lot.landmarkLabel && !isPost && a.lot.variant%2 == 1 {
		// Modern Cultural / Bookstore / Community Atrium Complex.
		a.archBlock(-0.16, -0.04, 0.54, 0.58, 11, rgb(92, 106, 118), rgb(158, 170, 178), 14, true)
		a.archBlock(0.24, 0.04, 0.34, 0.52, 9, rgb(134, 98, 82), rgb(184, 156, 138), 12, true)
		a.circlePx(-0.08, -0.02, 22, rgb(176, 190, 198))
		a.circlePx(-0.08, -0.02, 17, rgb(24, 58, 78))
		a.circlePx(-0.08, -0.02, 9, rgb(104, 172, 194))
		a.skylight(0.24, 0.02, 0.16, 0.26, 2, 3)
		a.hvac(-0.28, -0.18, 18, 13)
		a.miniCar(-0.22, 0.34, 0, metroCarPalette[a.lot.variant%len(metroCarPalette)], false)
		a.treeCluster(0.04, 0.31, 10.5)
		a.treeCluster(0.44, 0.32, 11)
		a.treeCluster(-0.46, 0.28, 10.5)
		return
	}
	frameCol := rgb(138, 84, 70)
	deckCol := rgb(178, 146, 128)
	accentCol := rgb(196, 84, 68)
	if isPost {
		frameCol = rgb(42, 108, 78)
		deckCol = rgb(158, 182, 168)
		accentCol = rgb(34, 136, 88)
	}
	// Sculpted inner courtyard plaza & reflecting pool inside the U-wings.
	a.softPlaza(0, 0.06, 0.52, 0.56, 0.08, 0.10, rgb(142, 138, 126))
	a.oval(0, 0.12, 0.20, 0.26, rgb(52, 128, 144))
	a.oval(0, 0.12, 0.13, 0.17, rgb(96, 174, 186))
	a.circlePx(0, 0.12, 5.5, accentCol)
	a.archBlock(-0.32, 0.0, 0.24, 0.68, 7, frameCol, deckCol, 12, true)
	a.archBlock(0.32, 0.0, 0.24, 0.68, 7, frameCol, deckCol, 12, true)
	a.archBlock(0, -0.22, 0.46, 0.24, 7, frameCol, deckCol, 12, true)
	a.skylight(-0.32, -0.02, 0.12, 0.36, 2, 4)
	a.skylight(0.32, -0.02, 0.12, 0.36, 2, 4)
	a.hvac(0, -0.22, 22, 14)
	a.rect(0, 0.31, 0.38, 0.055, accentCol)
	if isPost {
		a.miniCar(-0.16, 0.36, 0, rgb(34, 132, 84), false)
		a.miniCar(0.16, 0.36, 0, rgb(34, 132, 84), false)
	}
	a.treeCluster(-0.15, 0.28, 10)
	a.treeCluster(0.15, 0.28, 10)
	a.treeCluster(-0.46, 0.32, 10.5)
	a.treeCluster(0.46, 0.32, 10.5)
}

func (a metroCanvas) rowhouses() {
	if a.lot.variant%2 == 1 {
		// Staggered Modern Mid-Rise Apartment Terraces with organic garden lawn.
		a.softLawn(0, 0.08, 0.62, 0.52)
		a.softPlaza(0, 0.14, 0.32, 0.44, 0.06, 0.08, rgb(138, 142, 138))
		for i, x := range []float64{-0.27, 0.27} {
			frame := rgb(72, 98, 114)
			deck := rgb(146, 164, 174)
			if i == 1 {
				frame = rgb(138, 96, 82)
				deck = rgb(188, 162, 146)
			}
			a.archBlock(x, -0.02, 0.32, 0.62, 9, frame, deck, 13, true)
			a.skylight(x, -0.08, 0.15, 0.22, 2, 3)
			a.hvac(x, 0.14, 17, 12)
		}
		a.archBlock(0, -0.20, 0.32, 0.24, 7, rgb(88, 108, 118), rgb(158, 172, 178), 10, true)
		a.miniCar(-0.08, 0.28, 0, metroCarPalette[a.lot.variant%len(metroCarPalette)], false)
		a.treeCluster(0.08, 0.14, 10.5)
		a.treeCluster(-0.44, 0.30, 10)
		a.treeCluster(0.44, 0.30, 10)
		return
	}
	roofPalette := [...]color.RGBA{
		rgb(76, 104, 124),
		rgb(154, 92, 76),
		rgb(68, 114, 106),
		rgb(122, 106, 92),
	}
	for i := 0; i < 3; i++ {
		x := -0.30 + float64(i)*0.30
		y := -0.04 + float64((i+a.lot.variant)%3)*0.045
		frame := roofPalette[(i+a.lot.variant)%len(roofPalette)]
		deck := tint(frame, rgb(216, 208, 194), 0.46)
		// Soft rounded front garden & stone driveway extending to the street.
		a.softLawn(x-0.03, y+0.30, 0.20, 0.16)
		a.rect(x+0.06, y+0.36, 0.08, 0.18, rgb(122, 132, 138))
		a.archBlock(x, y, 0.25, 0.56, 7, frame, deck, 11, true)
		a.skylight(x-0.03, y-0.06, 0.10, 0.18, 2, 2)
		a.hvac(x+0.05, y+0.10, 14, 11)
		a.treeCluster(x-0.06, y+0.31, 8.5)
		if i == a.lot.variant%3 {
			a.miniCar(x+0.06, y+0.35, math.Pi/2, metroCarPalette[(i+a.lot.variant)%len(metroCarPalette)], false)
		}
	}
}

func (a metroCanvas) garden() {
	if !a.lot.landmarkLabel && a.lot.variant%2 == 1 {
		// Community Sports & Fitness Park on an organic grass & stone terrace (no rectangular box).
		a.softLawn(-0.18, 0.0, 0.54, 0.70)
		a.softLawn(0.18, 0.0, 0.54, 0.70)
		// West Tennis Court (blue inner court, green apron, white lines & net).
		a.rect(-0.19, -0.02, 0.32, 0.52, rgb(46, 116, 84))
		a.rect(-0.19, -0.02, 0.22, 0.40, rgb(48, 106, 152))
		if a.detail {
			a.line(-0.30, -0.02, -0.08, -0.02, 1.6, rgb(238, 242, 244))
			a.line(-0.30, -0.22, -0.08, -0.22, 1.3, rgb(238, 242, 244))
			a.line(-0.30, 0.18, -0.08, 0.18, 1.3, rgb(238, 242, 244))
			a.line(-0.30, -0.22, -0.30, 0.18, 1.3, rgb(238, 242, 244))
			a.line(-0.08, -0.22, -0.08, 0.18, 1.3, rgb(238, 242, 244))
		}
		// East Basketball Court (terracotta court, green apron & center circle).
		a.rect(0.19, -0.08, 0.32, 0.42, rgb(46, 116, 84))
		a.rect(0.19, -0.08, 0.24, 0.32, rgb(184, 84, 66))
		if a.detail {
			a.line(0.07, -0.08, 0.31, -0.08, 1.3, rgb(238, 242, 238))
			a.oval(0.19, -0.08, 0.08, 0.10, rgb(238, 242, 238))
		}
		a.archBlock(0.19, 0.21, 0.24, 0.11, 5, rgb(82, 106, 118), rgb(156, 174, 182), 8, false)
		for _, p := range []sim.Vec2{{-0.40, -0.24}, {-0.40, 0.24}, {0.40, -0.24}, {0.40, 0.24}, {0, 0.32}} {
			a.treeCluster(p.X, p.Y, 11.5)
		}
		return
	}
	// Contoured ecological wetland park with overlapping oval meadows (no rectangular frame).
	a.softLawn(-0.10, -0.02, 0.72, 0.68)
	a.softLawn(0.14, 0.02, 0.66, 0.64)
	// Permeable red-terracotta park jogging trail.
	a.oval(0, -0.01, 0.70, 0.56, rgb(158, 104, 84))
	a.oval(0, -0.01, 0.62, 0.46, rgb(54, 106, 72))
	// Multi-layered wetland pond.
	a.oval(-0.14, -0.04, 0.44, 0.46, rgb(164, 158, 138))
	a.oval(-0.14, -0.04, 0.40, 0.42, rgb(54, 136, 148))
	a.oval(-0.15, -0.05, 0.28, 0.30, rgb(32, 98, 116))
	// Timber boardwalk bridge & waterside octagonal pavilion.
	a.rect(-0.14, 0.08, 0.52, 0.055, rgb(166, 134, 96))
	a.rect(0.14, -0.04, 0.055, 0.44, rgb(166, 134, 96))
	a.shell(0.14, 0.08, 0.14, 0.18, 8, rgb(148, 76, 62), 8)
	a.shell(0.14, 0.08, 0.09, 0.12, 5, rgb(196, 146, 108), 0)
	for _, p := range []sim.Vec2{
		{X: 0.32, Y: -0.22},
		{X: 0.06, Y: -0.24},
		{X: 0.34, Y: 0.10},
		{X: -0.36, Y: 0.18},
		{X: -0.34, Y: -0.22},
		{X: 0.26, Y: 0.26},
	} {
		a.treeCluster(p.X, p.Y, 12.5)
	}
}
