package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawRailGround(r *sim.Railway, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	if cam.IsPointVisible(r.StationApproach, 550) {
		access := r.StationCenter.Sub(r.StationApproach)
		center := r.StationApproach.Add(access.Mul(.5))
		drawRotatedBox(screen, cam, center, 0, 1, access.Length(), 100, rgb(132, 146, 151))
		drawRotatedBox(screen, cam, center, 0, 1, access.Length(), 73, rgb(164, 171, 169))
	}
	if !cam.IsBoxVisible(r.StartX, r.Y-r.HalfWidth-15, r.EndX, r.Y+r.HalfWidth+15) {
		return
	}
	minX, _, maxX, _ := visibleWorldBounds(cam)
	start, end := math.Max(r.StartX, minX-90), math.Min(r.EndX, maxX+90)
	center := sim.Vec2{X: (start + end) / 2, Y: r.Y}
	drawRotatedBox(screen, cam, center, 1, 0, end-start, r.HalfWidth*2+23, rgb(46, 79, 62))
	drawRotatedBox(screen, cam, center, 1, 0, end-start, r.HalfWidth*2, rgb(55, 63, 69))
	drawRailTracks(screen, cam, r, start, end, !plan && cam.Zoom >= .18, 0)
}

func drawRailTracks(screen *ebiten.Image, cam *sim.Camera, r *sim.Railway, start, end float64, detail bool, dark float64) {
	for track := range sim.RailTrackCount {
		y := r.TrackY(track)
		if detail {
			for x := math.Ceil(start/18) * 18; x < end; x += 18 {
				drawRotatedBox(screen, cam, sim.Vec2{X: x, Y: y}, 1, 0, 4, 23, railShade(rgb(106, 94, 79), dark))
			}
		}
		for _, side := range [...]float64{-1, 1} {
			railY := y + side*6.2
			worldLine(screen, cam, sim.Vec2{X: start, Y: railY}, sim.Vec2{X: end, Y: railY}, 2.2, railShade(rgb(191, 205, 213), dark))
		}
	}
}

func drawRailLevelCrossings(r *sim.Railway, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	for _, c := range r.Crossings {
		if !c.Level || !cam.IsPointVisible(c.Center, c.RoadHalfWidth+r.HalfWidth+60) {
			continue
		}
		width := c.RoadHalfWidth + sim.ShoulderWidth
		drawRotatedBox(screen, cam, c.Center, 1, 0, 2*width, 2*r.HalfWidth+4, rgb(92, 96, 101))
		drawRailTracks(screen, cam, r, c.Center.X-width, c.Center.X+width, false, 0)
		if plan || cam.Zoom < .19 {
			continue
		}
		for _, side := range [...]float64{-1, 1} {
			line := c.Center.Add(sim.Vec2{X: side * c.RoadHalfWidth / 2, Y: side * (r.HalfWidth + sim.RailGateMargin + 7)})
			drawRotatedBox(screen, cam, line, 1, 0, c.RoadHalfWidth, 4, rgb(239, 245, 248))
		}
	}
}

func railShade(c color.RGBA, dark float64) color.RGBA {
	if dark == 0 {
		return c
	}
	return tint(c, rgb(5, 12, 28), dark)
}

func railViaductVisible(cam *sim.Camera, c sim.RailCrossing, r *sim.Railway) bool {
	return !c.Level && cam.IsBoxVisible(c.Center.X-c.ViaductHalfSpan-30, c.Center.Y-r.HalfWidth-22,
		c.Center.X+c.ViaductHalfSpan+48, c.Center.Y+r.HalfWidth+95)
}

// Bridge shadows and the pier footings are on the road level, below cars.
func drawRailViaductGround(r *sim.Railway, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	if plan || cam.Zoom < .16 {
		return
	}
	for _, c := range r.Crossings {
		if !railViaductVisible(cam, c, r) {
			continue
		}
		half := c.ViaductHalfSpan
		center := c.Center
		drawRotatedBox(screen, cam, center.Add(sim.Vec2{X: 18, Y: 40}), 1, 0, 2*half+24, 2*r.HalfWidth+36, sim.RGBA(7, 13, 25, 85))
		drawRotatedBox(screen, cam, center.Add(sim.Vec2{X: 12, Y: 30}), 1, 0, 2*half+8, 2*r.HalfWidth+20, sim.RGBA(8, 14, 24, 92))
		for _, side := range [...]float64{-1, 1} {
			pier := center.Add(sim.Vec2{X: side * (half - 47), Y: r.HalfWidth + 23})
			drawRotatedBox(screen, cam, pier.Add(sim.Vec2{X: 8, Y: 21}), 1, 0, 43, 25, sim.RGBA(8, 15, 24, 145))
			drawRotatedBox(screen, cam, pier.Add(sim.Vec2{Y: 14}), 1, 0, 43, 25, rgb(126, 144, 151))
			drawRotatedBox(screen, cam, pier, 1, 0, 26, 43, rgb(146, 161, 166))
			drawRotatedBox(screen, cam, pier.Add(sim.Vec2{X: 8}), 1, 0, 10, 43, rgb(88, 111, 123))
		}
	}
}

func drawRailViaductDeck(screen *ebiten.Image, cam *sim.Camera, r *sim.Railway, c sim.RailCrossing, dark float64, detail bool) {
	x, y := c.Center.X, r.Y
	half, bed := c.ViaductHalfSpan, r.HalfWidth
	west, east := x-half, x+half
	shade := func(col color.RGBA) color.RGBA { return railShade(col, dark) }

	// A deep front girder projects below the deck. Its tapered ends join the
	// embankment, while the two rails of each track run straight through.
	worldQuad(screen, cam, sim.Vec2{X: west, Y: y + bed}, sim.Vec2{X: east, Y: y + bed},
		sim.Vec2{X: east - 24, Y: y + bed + 26}, sim.Vec2{X: west + 24, Y: y + bed + 26}, shade(rgb(83, 110, 127)))
	drawRotatedBox(screen, cam, c.Center, 1, 0, 2*half, 2*bed+15, shade(rgb(151, 175, 185)))
	drawRotatedBox(screen, cam, c.Center, 1, 0, 2*half, 2*bed-2, shade(rgb(61, 73, 80)))
	drawRailTracks(screen, cam, r, west, east, detail, dark)
	for _, side := range [...]float64{-1, 1} {
		wall := c.Center.Add(sim.Vec2{Y: side * (bed + 4)})
		drawRotatedBox(screen, cam, wall, 1, 0, 2*half, 7, shade(rgb(213, 228, 233)))
	}
	if !detail {
		return
	}
	frontY := y + bed + 10
	drawRotatedBox(screen, cam, sim.Vec2{X: x, Y: frontY + 12}, 1, 0, 2*half-45, 3.5, shade(rgb(160, 185, 196)))
	for seam := west + 42; seam < east-28; seam += 58 {
		drawRotatedBox(screen, cam, sim.Vec2{X: seam, Y: frontY + 3}, 1, 0, 2, 15, shade(rgb(54, 82, 101)))
	}
	for _, end := range [...]float64{west + 13, east - 13} {
		drawRotatedBox(screen, cam, sim.Vec2{X: end, Y: y - bed - 4}, 1, 0, 7, 15, shade(rgb(223, 232, 229)))
		drawRotatedBox(screen, cam, sim.Vec2{X: end, Y: y + bed + 4}, 1, 0, 7, 15, shade(rgb(223, 232, 229)))
	}
}

// DrawRailTraffic composites the deck and trains over road fixtures and
// vehicle lights. Apply the same ambient shade as the ground layer.
func DrawRailTraffic(w *sim.World, tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	dark := w.Clock.Darkness()
	if plan {
		dark = math.Min(dark, .28)
	}
	detail := !plan && cam.Zoom >= .22
	elapsed := 0.
	if tls != nil {
		elapsed = tls.RailElapsed
	}
	for _, r := range w.Railways() {
		for _, c := range r.Crossings {
			if railViaductVisible(cam, c, r) {
				drawRailViaductDeck(screen, cam, r, c, dark, detail)
			}
		}
		r.ForEachTrain(elapsed, func(train sim.RailTrain) {
			if train.Service != sim.RailHighSpeed {
				drawConventionalTrain(screen, cam, train, detail, dark)
				return
			}
			stripe := rgb(30, 116, 198)
			if train.Heading != 0 {
				stripe = rgb(201, 56, 53)
			}
			drawWorldBulletTrain(screen, cam, train.Pos, train.Heading, train.Length(), stripe, detail, dark)
		})
	}
}

// DrawRailGates keeps the signal aspects and barriers readable at night.
func DrawRailGates(w *sim.World, tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	if cam.Zoom < .12 {
		return
	}
	elapsed := 0.
	if tls != nil {
		elapsed = tls.RailElapsed
	}
	dark := w.Clock.Darkness()
	if plan {
		dark = math.Min(dark, .28)
	}
	for _, r := range w.Railways() {
		drawRailLineGates(r, screen, cam, elapsed, dark)
	}
}

func drawRailLineGates(r *sim.Railway, screen *ebiten.Image, cam *sim.Camera, elapsed, dark float64) {
	shade := func(c color.RGBA) color.RGBA { return tint(c, rgb(5, 12, 28), dark) }
	for _, c := range r.Crossings {
		if !c.Level || !cam.IsPointVisible(c.Center, c.RoadHalfWidth+r.HalfWidth+75) {
			continue
		}
		gate := c.Gate(elapsed)
		for _, side := range [...]float64{-1, 1} {
			base := c.Center.Add(sim.Vec2{X: side * (c.RoadHalfWidth + sim.ShoulderWidth*.6), Y: side * (r.HalfWidth + sim.RailGateMargin)})
			drawRotatedBox(screen, cam, base, 1, 0, 14, 13, shade(rgb(234, 182, 76)))
			arm := math.Max(7, (c.RoadHalfWidth+sim.ShoulderWidth*.6)*gate.Lowered)
			segments := max(1, int(math.Ceil(arm/13)))
			for i := 0; i < segments; i++ {
				length := math.Min(13, arm-float64(i)*13)
				pos := base.Add(sim.Vec2{X: -side * (float64(i)*13 + length/2)})
				col := rgb(240, 243, 236)
				if i%2 == 0 {
					col = rgb(215, 47, 54)
				}
				drawRotatedBox(screen, cam, pos, 1, 0, length+.2, 6, shade(col))
			}
			head := base.Add(sim.Vec2{X: side * 13, Y: side * 15})
			drawRotatedBox(screen, cam, head, 1, 0, 22, 10, shade(rgb(26, 37, 50)))
			if gate.Red {
				for i, offset := range [...]float64{-5.5, 5.5} {
					col := shade(rgb(103, 37, 43))
					if int(elapsed*3)%2 == i {
						col = rgb(255, 50, 63)
					}
					worldCircle(screen, cam, head.Add(sim.Vec2{X: offset}), 3.4, col)
				}
			} else {
				worldCircle(screen, cam, head, 3.5, rgb(65, 228, 124))
			}
		}
	}
}

// Conventional trains use a locomotive and separately coupled coaches or wagons.
func drawConventionalTrain(screen *ebiten.Image, cam *sim.Camera, train sim.RailTrain, detail bool, dark float64) {
	length := train.Length()
	if !cam.IsPointVisible(train.Pos, length*.6) {
		return
	}
	shade := func(c color.RGBA) color.RGBA { return railShade(c, dark) }
	forward := sim.UnitFromAngle(train.Heading)
	at := func(x float64) sim.Vec2 { return train.Pos.Add(forward.Mul(x)) }
	const locoLength = 62.
	cars := 5
	livery := rgb(74, 126, 101)
	if train.Service == sim.RailFreight {
		cars = 7
		livery = rgb(186, 86, 54)
	}
	drawRotatedBox(screen, cam, train.Pos, forward.X, forward.Y, length, 7, shade(rgb(42, 47, 50)))
	loco := at((length - locoLength) / 2)
	drawVehicleShell(screen, cam, loco, train.Heading, locoLength, 23, shade(livery), 6, 3)
	drawRotatedBox(screen, cam, at(length/2-10), forward.X, forward.Y, 7, 16, shade(rgb(32, 53, 61)))
	drawRotatedBox(screen, cam, loco, forward.X, forward.Y, 22, 13, shade(rgb(101, 108, 112)))
	if dark > .3 {
		drawRotatedBox(screen, cam, at(length/2-3), forward.X, forward.Y, 3, 10, rgb(246, 235, 193))
	}

	wagonLength := (length - locoLength) / float64(cars)
	containers := [...]color.RGBA{rgb(173, 84, 62), rgb(60, 112, 130), rgb(181, 141, 68), rgb(103, 130, 91)}
	for i := range cars {
		pos := at(length/2 - locoLength - (float64(i)+.5)*wagonLength)
		if train.Service == sim.RailFreight {
			drawRotatedBox(screen, cam, pos, forward.X, forward.Y, wagonLength-4, 23, shade(rgb(72, 68, 64)))
			drawRotatedBox(screen, cam, pos, forward.X, forward.Y, wagonLength-11, 19, shade(containers[i%len(containers)]))
			if detail {
				drawRotatedBox(screen, cam, pos, forward.X, forward.Y, 2, 19, shade(rgb(190, 185, 165)))
			}
		} else {
			drawVehicleShell(screen, cam, pos, train.Heading, wagonLength-4, 23, shade(livery), 3, 3)
			drawRotatedBox(screen, cam, pos, forward.X, forward.Y, wagonLength-8, 14, shade(rgb(208, 215, 207)))
			if detail {
				right := sim.Vec2{X: -forward.Y, Y: forward.X}
				for _, side := range [...]float64{-1, 1} {
					drawRotatedBox(screen, cam, pos.Add(right.Mul(side*8.5)), forward.X, forward.Y, wagonLength-15, 3, shade(rgb(30, 57, 61)))
				}
			}
		}
	}
}
