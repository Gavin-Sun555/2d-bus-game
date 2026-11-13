package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

const westCanalWidth = 380.0

func (s *streetScenery) setupMetropolisCorridors(w *sim.World) {
	for _, lot := range s.metroLots {
		if !lot.landmarkLabel {
			continue
		}
		u := sim.UnitFromAngle(lot.heading)
		front := sim.Vec2{X: -u.Y, Y: u.X}
		hLot := lot.footprint.H
		if math.Abs(u.Y) > .5 {
			hLot = lot.footprint.W
		}
		switch lot.style {
		case metroCanal:
			quayPt := lot.footprint.Pos.Add(front.Mul(0.005 * hLot))
			s.hasCanalCorridor = true
			s.canalQuayX = quayPt.X
			s.canalCenterY = lot.footprint.Pos.Y
			s.canalJoinY = -300.0
			if len(w.Rivers) > 0 {
				s.canalJoinY = w.Rivers[0].Y
			}
			s.canalLot = lot

		case twinAirportTerminal, twinAirportLogistics:
			runwayCenter := lot.footprint.Pos.Add(front.Mul(-0.40 * hLot))
			s.hasAirportCorridor = true
			s.airportX = runwayCenter.X
			s.airportHalfW = 85.0
		}
	}
	if w.Theme == sim.MapThemeCoastalBay {
		s.hasCoastalPromenade = true
	}
}

func (s *streetScenery) corridorsClear(b sim.Building) bool {
	if s.hasCanalCorridor {
		canalLeft := s.canalQuayX - westCanalWidth - 24
		canalRight := s.canalQuayX + 24
		if b.Pos.X+b.W/2 > canalLeft && b.Pos.X-b.W/2 < canalRight && b.Pos.Y-b.H/2 < s.canalJoinY+45 {
			return false
		}
	}
	for _, rail := range s.railways {
		if math.Abs(b.Pos.Y-rail.Y) < b.H/2+rail.HalfWidth+16 {
			return false
		}
	}
	if s.hasAirportCorridor {
		if math.Abs(b.Pos.X-s.airportX) < b.W/2+s.airportHalfW+35 {
			return false
		}
	}
	if s.hasCoastalPromenade {
		if math.Abs(b.Pos.X) < b.W/2+38 {
			return false
		}
	}
	return true
}

// drawMetropolisCanalCorridor renders the continuous North-South West Grand Canal
// connecting the West Canal Port (西运河客运港) south into the Central Canal (Y = -300)
// and north all the way out past the map boundary (Y = -24000).
func drawMetropolisCanalCorridor(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	if w.Theme != sim.MapThemeMetropolis {
		return
	}
	scene := sceneryFor(w, laneWidth)
	if !scene.hasCanalCorridor {
		return
	}
	p := sceneColors(w.Theme)
	quayX := scene.canalQuayX
	joinY := scene.canalJoinY

	canalW := westCanalWidth
	canalCenterX := quayX - canalW*0.5
	canalTopY := -24000.0
	canalBotY := joinY + 12.0
	canalLen := canalBotY - canalTopY
	canalMidY := (canalTopY + canalBotY) * 0.5
	if !cam.IsBoxVisible(quayX-canalW-40, canalTopY, quayX+45, canalBotY+40) {
		return
	}

	// Western & Eastern stone canal banks (stopping just north of the Central Canal so the confluence is seamless).
	bankLen := (joinY - 30.0) - canalTopY
	bankMidY := (canalTopY + joinY - 30.0) * 0.5
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX - canalW - 10, Y: bankMidY}, 0, 1, bankLen, 22, p.paving)
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX - canalW - 2, Y: bankMidY}, 0, 1, bankLen, 7, p.curb)
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX + 10, Y: bankMidY}, 0, 1, bankLen, 22, rgb(156, 152, 138))
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX + 2, Y: bankMidY}, 0, 1, bankLen, 6, rgb(206, 200, 184))

	// Continuous deep canal water & turquoise shallow shelves along both banks.
	drawRotatedBox(screen, cam, sim.Vec2{X: canalCenterX, Y: canalMidY}, 0, 1, canalLen, canalW, p.water)
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX - 24, Y: bankMidY}, 0, 1, bankLen, 48, p.shallows)
	drawRotatedBox(screen, cam, sim.Vec2{X: quayX - canalW + 24, Y: bankMidY}, 0, 1, bankLen, 48, p.shallows)

	if cam.Zoom > 0.18 {
		_, minY, _, maxY := visibleWorldBounds(cam)
		startY := math.Max(canalTopY+80, math.Floor(minY/160)*160)
		endY := math.Min(canalBotY-60, maxY)
		for y := startY; y < endY; y += 160 {
			shift := float64(int(math.Abs(y))%5*20 - 40)
			a := sim.Vec2{X: canalCenterX + shift, Y: y}
			worldLine(screen, cam, a, a.Add(sim.Vec2{X: 4, Y: 46}), 1.5, tint(p.shallows, p.foam, 0.55))
		}
	}

	// Cruising passenger water-buses / canal ferries along the West Grand Canal.
	if cam.Zoom >= 0.14 {
		detail := cam.Zoom >= 0.22
		drawCanalCruisingVessel(screen, cam, sim.Vec2{X: quayX - 210, Y: scene.canalCenterY - 150}, -math.Pi/2, rgb(38, 136, 158), detail)
		drawCanalCruisingVessel(screen, cam, sim.Vec2{X: quayX - 255, Y: scene.canalCenterY + 180}, math.Pi/2, rgb(214, 138, 62), detail)
		drawCanalCruisingVessel(screen, cam, sim.Vec2{X: quayX - 195, Y: scene.canalJoinY - 260}, -math.Pi/2, rgb(52, 148, 124), detail)
	}
}

func drawCanalCruisingVessel(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading float64, trimCol color.RGBA, detail bool) {
	if !cam.IsPointVisible(pos, 90) {
		return
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	length, width := 68.0, 22.0
	if detail {
		stern := pos.Sub(u.Mul(length * 0.48))
		worldLine(screen, cam, stern, stern.Sub(u.Mul(34)).Add(r.Mul(14)), 1.8, sim.RGBA(168, 218, 226, 160))
		worldLine(screen, cam, stern, stern.Sub(u.Mul(34)).Sub(r.Mul(14)), 1.8, sim.RGBA(168, 218, 226, 160))
	}
	drawRotatedBox(screen, cam, pos.Add(sim.Vec2{X: 3, Y: 4}), u.X, u.Y, length, width, sim.RGBA(12, 38, 52, 95))
	drawVehicleShell(screen, cam, pos, heading, length, width, rgb(236, 242, 246), 9.0, 4.5)
	drawVehicleShell(screen, cam, pos.Sub(u.Mul(3)), heading, length*0.68, width-5.0, rgb(22, 46, 62), 5.0, 3.0)
	drawVehicleShell(screen, cam, pos.Sub(u.Mul(4)), heading, length*0.56, width-8.0, rgb(224, 232, 238), 4.0, 2.5)
	drawRotatedBox(screen, cam, pos.Sub(u.Mul(4)), u.X, u.Y, length*0.36, width-11.0, trimCol)
}

func drawWorldBulletTrain(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading, length float64, stripe color.RGBA, detail bool, dark float64) {
	if !cam.IsPointVisible(pos, length*0.6) {
		return
	}
	width := 23.0
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	drawRotatedBox(screen, cam, pos.Add(sim.Vec2{X: 3.5, Y: 5.0}), u.X, u.Y, length, width, sim.RGBA(8, 14, 22, 96))
	drawVehicleShell(screen, cam, pos, heading, length, width, railShade(rgb(236, 241, 245), dark), 11.0, 11.0)
	drawRotatedBox(screen, cam, pos, u.X, u.Y, length-18, width-4.5, railShade(stripe, dark))
	drawRotatedBox(screen, cam, pos, u.X, u.Y, length-22, width-8.5, railShade(rgb(224, 232, 238), dark))
	if !detail {
		return
	}
	for _, dir := range []float64{-1, 1} {
		cab := pos.Add(u.Mul(dir * (length/2 - 12)))
		drawRotatedBox(screen, cam, cab, u.X, u.Y, 9.0, width-7.0, railShade(rgb(19, 39, 53), dark))
		drawRotatedBox(screen, cam, cab.Add(u.Mul(dir*1.8)), u.X, u.Y, 2.5, width-11.0, railShade(rgb(118, 178, 198), dark))
	}
	cars := 3
	carLen := (length - 28) / float64(cars)
	for i := 0; i < cars; i++ {
		cx := -((length - 28) / 2) + carLen*(float64(i)+0.5)
		cp := pos.Add(u.Mul(cx))
		drawRotatedBox(screen, cam, cp, u.X, u.Y, 16, 10, railShade(rgb(168, 182, 190), dark))
		drawRotatedBox(screen, cam, cp, u.X, u.Y, 13, 8, railShade(rgb(214, 224, 230), dark))
		if i > 0 {
			joint := pos.Add(u.Mul(-((length - 28) / 2) + carLen*float64(i)))
			drawRotatedBox(screen, cam, joint, u.X, u.Y, 4.5, width-2.5, railShade(rgb(54, 68, 78), dark))
		}
		for _, side := range []float64{-1, 1} {
			wp := cp.Add(r.Mul(side * (width/2 - 1.8)))
			drawRotatedBox(screen, cam, wp, u.X, u.Y, carLen*0.76, 2.2, railShade(rgb(20, 42, 58), dark))
		}
	}
}

// Draw bridge parapets where east-west streets cross the West Grand Canal.
func drawMetropolisCanalOverpasses(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	if w.Theme != sim.MapThemeMetropolis || planMode {
		return
	}
	scene := sceneryFor(w, laneWidth)
	canalSpan := westCanalWidth + 36.0
	canalMidX := scene.canalQuayX - westCanalWidth*0.5

	for _, r := range w.Roads {
		dir := r.P2.Sub(r.P1)
		length := dir.Length()
		if length < 1 {
			continue
		}
		u := dir.Mul(1.0 / length)
		right := sim.Vec2{X: -u.Y, Y: u.X}
		roadHalf := r.GetWidth(laneWidth)/2 + sim.ShoulderWidth

		if scene.hasCanalCorridor && math.Abs(u.X) > 0.5 {
			// East-West street bridging over the West Grand Canal.
			ry := r.P1.Y
			if ry < scene.canalJoinY-40 {
				minX, maxX := math.Min(r.P1.X, r.P2.X), math.Max(r.P1.X, r.P2.X)
				if canalMidX >= minX+40 && canalMidX <= maxX-40 {
					center := sim.Vec2{X: canalMidX, Y: ry}
					if cam.IsPointVisible(center, canalSpan) {
						for _, side := range []float64{-1, 1} {
							wall := center.Add(right.Mul(side * (roadHalf + 4)))
							drawRotatedBox(screen, cam, wall, u.X, u.Y, canalSpan, 9.0, rgb(164, 158, 142))
							drawRotatedBox(screen, cam, wall, u.X, u.Y, canalSpan-6, 4.0, rgb(214, 208, 192))
						}
					}
				}
			}
		}
	}
}

// drawTwinCitiesAirportCorridor renders the continuous North-South airport runway
// and taxiway system along the west boundary of Twin Cities (-24000 .. +24000).
func drawTwinCitiesAirportCorridor(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	if w.Theme != sim.MapThemeTwinCities || planMode {
		return
	}
	scene := sceneryFor(w, laneWidth)
	if !scene.hasAirportCorridor {
		return
	}
	rX := scene.airportX
	rW := scene.airportHalfW * 2
	if !cam.IsBoxVisible(rX-rW*0.6, -24000, rX+rW*0.6, 24000) {
		return
	}
	// 1. Runway grass safety strip & paved asphalt runway
	drawRotatedBox(screen, cam, sim.Vec2{X: rX, Y: 0}, 0, 1, 48000, rW+40, rgb(42, 74, 58))
	drawRotatedBox(screen, cam, sim.Vec2{X: rX, Y: 0}, 0, 1, 48000, rW, rgb(38, 44, 50))
	drawRotatedBox(screen, cam, sim.Vec2{X: rX, Y: 0}, 0, 1, 48000, rW-8, rgb(46, 52, 58))

	// 2. Continuous white runway edge lines
	edgeOffset := rW*0.5 - 6.0
	worldLine(screen, cam, sim.Vec2{X: rX - edgeOffset, Y: -24000}, sim.Vec2{X: rX - edgeOffset, Y: 24000}, 2.5, rgb(245, 248, 252))
	worldLine(screen, cam, sim.Vec2{X: rX + edgeOffset, Y: -24000}, sim.Vec2{X: rX + edgeOffset, Y: 24000}, 2.5, rgb(245, 248, 252))

	// 3. Centerline dashes and touchdown zones
	_, minY, _, maxY := visibleWorldBounds(cam)
	startY := math.Max(-23950.0, math.Floor((minY-100)/120.0)*120.0)
	endY := math.Min(23950.0, maxY+100)
	for y := startY; y <= endY; y += 120.0 {
		worldLine(screen, cam, sim.Vec2{X: rX, Y: y - 25}, sim.Vec2{X: rX, Y: y + 25}, 3.0, rgb(245, 248, 252))
		if int(math.Abs(y))%360 == 0 {
			for _, side := range []float64{-1, 1} {
				bx := rX + side*28
				worldLine(screen, cam, sim.Vec2{X: bx - 4, Y: y - 20}, sim.Vec2{X: bx - 4, Y: y + 20}, 2.2, rgb(240, 244, 250))
				worldLine(screen, cam, sim.Vec2{X: bx + 4, Y: y - 20}, sim.Vec2{X: bx + 4, Y: y + 20}, 2.2, rgb(240, 244, 250))
			}
		}
	}
	// 4. Threshold piano keys at intervals
	for _, threshY := range []float64{-23800, -12000, 0, 12000, 23800} {
		if math.Abs(threshY-cam.Pos.Y) < 1500 {
			for i := -6; i <= 6; i++ {
				if i == 0 {
					continue
				}
				kx := rX + float64(i)*6.5
				worldLine(screen, cam, sim.Vec2{X: kx, Y: threshY - 18}, sim.Vec2{X: kx, Y: threshY + 18}, 2.8, rgb(250, 252, 255))
			}
		}
	}
}

// drawCoastalPromenadeCorridor renders the continuous seaside stone seawall,
// pedestrian promenade boardwalk, and royal palm avenue along X = 0 (-24000 .. +24000).
func drawCoastalPromenadeCorridor(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	if w.Theme != sim.MapThemeCoastalBay || planMode {
		return
	}
	if !cam.IsBoxVisible(-60, -24000, 60, 24000) {
		return
	}
	p := sceneColors(w.Theme)
	// 1. Wide stone & timber promenade boardwalk
	drawRotatedBox(screen, cam, sim.Vec2{X: -16, Y: 0}, 0, 1, 48000, 36, rgb(206, 194, 168))
	drawRotatedBox(screen, cam, sim.Vec2{X: -16, Y: 0}, 0, 1, 48000, 28, rgb(224, 214, 190))
	// 2. Heavy granite seawall coping along the sand line
	drawRotatedBox(screen, cam, sim.Vec2{X: 1, Y: 0}, 0, 1, 48000, 8, rgb(164, 156, 140))
	drawRotatedBox(screen, cam, sim.Vec2{X: 4, Y: 0}, 0, 1, 48000, 3, rgb(218, 212, 198))

	if cam.Zoom >= 0.16 {
		_, minY, _, maxY := visibleWorldBounds(cam)
		startY := math.Max(-23980.0, math.Floor((minY-80)/95.0)*95.0)
		endY := math.Min(23980.0, maxY+80)
		for y := startY; y <= endY; y += 95.0 {
			worldCircle(screen, cam, sim.Vec2{X: -32, Y: y}, 14.5, p.foliage)
			worldCircle(screen, cam, sim.Vec2{X: -32, Y: y}, 6.5, p.leafLight)
			worldCircle(screen, cam, sim.Vec2{X: -2, Y: y + 45}, 2.5, rgb(68, 64, 58))
			worldCircle(screen, cam, sim.Vec2{X: -2, Y: y + 45}, 1.2, rgb(248, 224, 142))
		}
	}
}

// drawRiverValleyEmbankments renders stone embankment retaining walls and riverside
// promenades along both the North and South riverbanks (-24000 .. +24000).
func drawRiverValleyEmbankments(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	if w.Theme != sim.MapThemeRiverValley || planMode || len(w.Rivers) == 0 {
		return
	}
	rY := w.Rivers[0].Y
	rHalfH := w.Rivers[0].Height * 0.5
	p := sceneColors(w.Theme)

	for _, bankY := range []float64{rY - rHalfH, rY + rHalfH} {
		if !cam.IsBoxVisible(-24000, bankY-30, 24000, bankY+30) {
			continue
		}
		drawRotatedBox(screen, cam, sim.Vec2{X: 0, Y: bankY}, 1, 0, 48000, 12, rgb(148, 154, 158))
		drawRotatedBox(screen, cam, sim.Vec2{X: 0, Y: bankY}, 1, 0, 48000, 4, rgb(206, 212, 216))
		promenadeY := bankY - 18
		if bankY > rY {
			promenadeY = bankY + 18
		}
		drawRotatedBox(screen, cam, sim.Vec2{X: 0, Y: promenadeY}, 1, 0, 48000, 22, p.paving)

		if cam.Zoom >= 0.16 {
			minX, _, maxX, _ := visibleWorldBounds(cam)
			startX := math.Max(-23980.0, math.Floor((minX-80)/110.0)*110.0)
			endX := math.Min(23980.0, maxX+80)
			for x := startX; x <= endX; x += 110.0 {
				worldCircle(screen, cam, sim.Vec2{X: x, Y: promenadeY}, 12.0, p.foliage)
				worldCircle(screen, cam, sim.Vec2{X: x, Y: promenadeY}, 5.5, p.leafLight)
			}
		}
	}
}
