package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// Draw renders the bus stop, waiting shelter, and passenger dots
func DrawStop(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, isTarget bool, isHovered bool, isPlanMode ...bool) {
	DrawStopGeometry(s, screen, cam, isTarget, isHovered, isPlanMode...)
	drawBRTOverpass(s, screen, cam, 0)
	DrawStopLabel(s, screen, cam, isTarget, isHovered, isPlanMode...)
}

func DrawStopGeometry(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, isTarget bool, isHovered bool, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	cullRadius := s.Length/2 + 40
	if s.CenterBRT {
		cullRadius = math.Max(cullRadius, s.ShelterPos.Distance(s.RoadCenter)*4.8+60)
	}
	// Frustum culling: skip drawing if stop and shelter are completely off-screen
	if !cam.IsPointVisible(s.Pos, cullRadius) && !cam.IsPointVisible(s.ShelterPos, 160) {
		return
	}

	sinH, cosH := math.Sincos(s.Heading)

	// 1. Bus Stop Bay (Marked stopping area on the road)
	bayBorderColor := sim.RGBA(234, 179, 8, 220) // Vibrant yellow
	if isTarget {
		bayBorderColor = sim.RGBA(59, 130, 246, 255) // Neon blue for next stop target
	}

	if s.CenterBRT {
		// Central BRT: Bus stops directly within the red median bus lane.
		// There is NO platform in the road center (s.RoadCenter); opposing bus lanes are separated
		// only by the road centerline. The station has two distinct side platforms on the outer right
		// side of each bus lane.
		right := sim.Vec2{X: -sinH, Y: cosH}
		lw := math.Abs(s.Pos.Sub(s.RoadCenter).Dot(right)) * 2.0
		if lw < 30 {
			lw = 52.0
		}
		// Clean yellow berth stop box on the red bus lane surface
		drawRotatedBox(screen, cam, s.Pos, cosH, sinH, s.Length, lw-4, sim.RGBA(127, 29, 29, 120))
		drawRotatedBoxOutline(screen, cam, s.Pos, cosH, sinH, s.Length, lw-4, 2.5, bayBorderColor)
		// White BRT bus stop word marking or stop line
		if !planMode && cam.Zoom >= 0.40 {
			stopLinePos := s.Pos.Add(sim.Vec2{X: cosH * (s.Length*0.5 - 8), Y: sinH * (s.Length*0.5 - 8)})
			drawRotatedBox(screen, cam, stopLinePos, cosH, sinH, 4.0, lw-8, sim.RGBA(255, 255, 255, 230))
		}
	} else {
		// Ordinary Roadside Stop: Indented cutout bay outside the curb
		drawRotatedBox(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, sim.RGBA(24, 24, 27, 240))
		drawRotatedBoxOutline(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, 3, bayBorderColor)

		// Diagonal yellow hazard stripes inside the indented bay
		if !planMode && cam.Zoom >= 0.40 {
			numStripes := 6
			step := s.Length / float64(numStripes)
			for i := -numStripes / 2; i <= numStripes/2; i++ {
				offset := float64(i) * step
				p1 := s.Pos.Add(sim.Vec2{X: cosH*offset - sinH*(s.Width/2-4), Y: sinH*offset + cosH*(s.Width/2-4)})
				p2 := s.Pos.Add(sim.Vec2{X: cosH*(offset+20) + sinH*(s.Width/2-4), Y: sinH*(offset+20) - cosH*(s.Width/2-4)})
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
					float32(2.5*cam.Zoom), sim.RGBA(234, 179, 8, 140), true)
			}
		}
	}

	if !planMode && cam.Zoom >= .4 && s.HasTwoBerths() {
		front := s.FrontBerthPosition()
		right := sim.Vec2{X: -sinH, Y: cosH}
		for _, p := range []sim.Vec2{front, s.RearBerthPosition()} {
			a := cam.WorldToScreen(p.Add(sim.Vec2{X: cosH, Y: sinH}.Mul(48)).Add(right.Mul(-s.Width/2 + 3)))
			b := cam.WorldToScreen(p.Add(sim.Vec2{X: cosH, Y: sinH}.Mul(48)).Add(right.Mul(s.Width/2 - 3)))
			strokeLine(screen, float32(a.X), float32(a.Y), float32(b.X), float32(b.Y), float32(2*cam.Zoom), bayBorderColor, true)
		}
	}

	// 2. Bus Shelter & BRT Narrow Divider Platform + Pedestrian Overpass / Underpass
	if s.CenterBRT {
		drawCenterBRTPlatformAndCrossing(s, screen, cam, cosH, sinH, planMode)
	} else {
		shelterLen := 68.0
		shelterWid := 18.0
		drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen, shelterWid, sim.RGBA(30, 41, 59, 240))
		// Shelter roof accent bar
		drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen-4, 4, s.AccentColor)
	}

	// 3. Waiting Passengers visualization (dots lined up near shelter; skip when zoomed out)
	if !planMode && cam.Zoom >= 0.45 {
		perpX := -sinH
		perpY := cosH
		for i := 0; i < s.WaitingPassengers && i < 24; i++ {
			row := float64(i / 12)
			col := float64(i % 12)
			latOffset := row*6 + 12
			longOffset := col*6 - 32
			if s.CenterBRT {
				latOffset = (row - 0.5) * 5.0
				longOffset = col*7.5 - 41
			}
			pWorld := s.ShelterPos.Add(sim.Vec2{
				X: cosH*longOffset + perpX*latOffset,
				Y: sinH*longOffset + perpY*latOffset,
			})
			ps := cam.WorldToScreen(pWorld)
			filledCircle(screen, float32(ps.X), float32(ps.Y), float32(2.8*cam.Zoom),
				sim.RGBA(251, 191, 36, 230), true)
		}
	}
}

func DrawStopLabel(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, isTarget bool, isHovered bool, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	cullRadius := s.Length/2 + 40
	if s.CenterBRT {
		cullRadius = math.Max(cullRadius, s.ShelterPos.Distance(s.RoadCenter)*4.8+60)
	}
	if !cam.IsPointVisible(s.Pos, cullRadius) && !cam.IsPointVisible(s.ShelterPos, 160) {
		return
	}

	shelterScreen := cam.WorldToScreen(s.ShelterPos)

	// 4. Station Marker Pole / Flag
	badgeRadius := float32(14 * cam.Zoom)
	minRadius := float32(8)
	if planMode && cam.Zoom < .1 && !isTarget && !isHovered {
		minRadius = 2
	}
	if badgeRadius < minRadius {
		badgeRadius = minRadius
	}

	placeBelow := false
	if s.CenterBRT {
		rcScreen := cam.WorldToScreen(s.RoadCenter)
		dy := shelterScreen.Y - rcScreen.Y
		dx := shelterScreen.X - rcScreen.X
		placeBelow = dy > 2.0 || (math.Abs(dy) <= 2.0 && dx > 0)
	}

	badgeY := shelterScreen.Y - float64(badgeRadius*2.5)
	if placeBelow {
		badgeY = shelterScreen.Y + float64(badgeRadius*2.5)
	}

	// Glowing circle badge
	badgeColor := s.AccentColor
	if isTarget {
		badgeColor = sim.RGBA(59, 130, 246, 255)
	}
	filledCircle(screen, float32(shelterScreen.X), float32(badgeY),
		badgeRadius, badgeColor, true)
	strokeCircle(screen, float32(shelterScreen.X), float32(badgeY),
		badgeRadius, float32(math.Max(1.5, 2*cam.Zoom)), sim.RGBA(255, 255, 255, 255), true)

	if planMode && cam.Zoom < .1 && !isTarget && !isHovered {
		return
	}

	// Station info label
	infoText := s.FormatInfoText(isTarget, isHovered, planMode, cam.Zoom)
	if s.CenterBRT {
		if (!planMode || isTarget || isHovered) && cam.Zoom >= 0.35 {
			if isBRTUnderpass(s.RoadCenter) {
				infoText = "BRT(地道) · " + infoText
			} else {
				infoText = "BRT(天桥) · " + infoText
			}
		} else {
			infoText = "BRT · " + infoText
		}
	}
	labelW, labelH := textui.MeasureText(infoText, 12)
	labelX := shelterScreen.X - 60
	labelY := shelterScreen.Y - float64(badgeRadius*4)
	if s.CenterBRT {
		labelX = shelterScreen.X - float64(labelW)*0.5
		if placeBelow {
			labelY = badgeY + float64(badgeRadius) + 6
		} else {
			labelY = badgeY - float64(badgeRadius) - float64(labelH) - 6
		}
	}
	filledRect(screen, float32(labelX-5), float32(labelY-3), float32(labelW+10), float32(labelH+6), sim.RGBA(15, 25, 35, 218), true)
	textui.DrawText(screen, infoText, labelX, labelY, 12, color.White)
}

func isBRTUnderpass(roadCenter sim.Vec2) bool {
	gx := int(math.Round(roadCenter.X / 50.0))
	gy := int(math.Round(roadCenter.Y / 50.0))
	h := gx*37 + gy*101
	if h < 0 {
		h = -h
	}
	return h%2 == 1
}

// drawCenterBRTPlatformAndCrossing renders the widened right-side BRT island platform on the
// dividing line between the median bus lane and general traffic lanes, plus a symmetric
// pedestrian overpass (天桥) or underpass (地道) connecting the platform to both sides of the road.
// Crucially, NO stairs, elevators, or platforms exist in the road center between oncoming bus lanes.
func drawCenterBRTPlatformAndCrossing(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, cosH, sinH float64, planMode bool) {
	forward := sim.Vec2{X: cosH, Y: sinH}
	right := sim.Vec2{X: -sinH, Y: cosH}
	laneWidth := math.Abs(s.Pos.Sub(s.RoadCenter).Dot(right)) * 2.0
	if laneWidth < 30 {
		laneWidth = 52.0
	}
	// Widened platform geometry (24px wide island platform on the right side of the bus lane):
	platOffset := sim.BRTPlatformOffset(laneWidth)
	platCenter := s.RoadCenter.Add(right.Mul(platOffset))
	if math.Abs(s.ShelterPos.Sub(platCenter).Length()) < 8.0 {
		platCenter = s.ShelterPos
	}

	sidewalkLat := brtSidewalkOffset(s, laneWidth)
	underpass := isBRTUnderpass(s.RoadCenter)
	crossOrigin := brtCrossingOrigin(s)
	crossPlatform := crossOrigin.Add(right.Mul(platOffset))
	sidewalkPos := crossOrigin.Add(right.Mul(sidewalkLat))

	// 1. If Underpass (地道), draw subterranean corridor beneath the lanes first so platform sits on top
	if underpass && cam.Zoom >= 0.22 {
		tunnelSpan := sidewalkLat - platOffset + 10.0
		tunnelCenter := crossPlatform.Add(right.Mul(tunnelSpan * 0.5))
		drawRotatedBox(screen, cam, tunnelCenter, -sinH, cosH, tunnelSpan, 15.0, sim.RGBA(15, 23, 42, 70))
		if cam.Zoom >= 0.35 {
			for _, longOff := range []float64{-7.0, 7.0} {
				for lat := 6.0; lat < tunnelSpan-8.0; lat += 18.0 {
					p1 := crossPlatform.Add(forward.Mul(longOff)).Add(right.Mul(lat))
					p2 := crossPlatform.Add(forward.Mul(longOff)).Add(right.Mul(math.Min(tunnelSpan-4.0, lat+10.0)))
					s1 := cam.WorldToScreen(p1)
					s2 := cam.WorldToScreen(p2)
					strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
						float32(math.Max(1.0, 1.5*cam.Zoom)), sim.RGBA(125, 211, 252, 115), true)
				}
			}
		}
	}

	// 2. Widened Right-Side BRT Island Platform (Width 24px)
	platLen := math.Min(s.Length-36.0, 224.0)
	if platLen < 120.0 {
		platLen = 120.0
	}
	platWid := sim.BRTPlatformWidth
	// Barrier curb base & stone deck
	drawRotatedBox(screen, cam, platCenter, cosH, sinH, platLen+12.0, platWid+3.0, sim.RGBA(15, 23, 42, 250))
	drawRotatedBox(screen, cam, platCenter, cosH, sinH, platLen, platWid, sim.RGBA(148, 163, 184, 245))
	// Yellow tactile boarding strip on the left edge of the platform (facing right-hand bus doors)
	tactilePos := platCenter.Add(right.Mul(-platWid*0.5 + 2.0))
	drawRotatedBox(screen, cam, tactilePos, cosH, sinH, platLen-6.0, 2.4, sim.RGBA(250, 204, 21, 240))
	// Steel guardrail on the right edge of the platform (separating flared general car lanes)
	guardPos := platCenter.Add(right.Mul(platWid*0.5 - 1.5))
	drawRotatedBox(screen, cam, guardPos, cosH, sinH, platLen-4.0, 2.2, sim.RGBA(226, 232, 240, 235))

	// Crash-attenuator nose cones & reflective bollards at both ends of the platform barrier
	if cam.Zoom >= 0.30 {
		for _, sign := range []float64{-1, 1} {
			nosePos := platCenter.Add(forward.Mul(sign * (platLen*0.5 + 4.0)))
			drawRotatedBox(screen, cam, nosePos, cosH, sinH, 7.0, platWid-2.0, sim.RGBA(245, 158, 11, 245))
			ns := cam.WorldToScreen(nosePos)
			filledCircle(screen, float32(ns.X), float32(ns.Y), float32(math.Max(2.0, 3.2*cam.Zoom)), sim.RGBA(254, 240, 138, 255), true)
		}
	}

	// Boarding screen gates (屏蔽门) facing the median bus lane
	if !planMode && cam.Zoom >= 0.40 {
		for _, gateLong := range []float64{-68, -32, 32, 68, 98} {
			if math.Abs(gateLong) > platLen*0.5-12 {
				continue
			}
			gatePos := platCenter.Add(forward.Mul(gateLong)).Add(right.Mul(-platWid*0.5 + 1.0))
			drawRotatedBox(screen, cam, gatePos, cosH, sinH, 12.0, 2.2, sim.RGBA(248, 250, 252, 250))
		}
	}

	// Sleek wide glass shelter canopy over the platform
	canopyLen := 140.0
	canopyWid := platWid - 4.0
	drawRotatedBox(screen, cam, platCenter, cosH, sinH, canopyLen, canopyWid, sim.RGBA(30, 41, 59, 238))
	drawRotatedBox(screen, cam, platCenter, cosH, sinH, canopyLen-8.0, 4.0, s.AccentColor)

	if cam.Zoom < 0.22 {
		return
	}

	// 3. Sidewalk Landing Pavilion & Stairs/Elevator
	if underpass {
		// Underpass portal on the widened BRT platform (descending stairwell with glass hood)
		platPortal := crossPlatform.Add(forward.Mul(-16.0))
		drawRotatedBox(screen, cam, platPortal, cosH, sinH, 26.0, 12.0, sim.RGBA(15, 23, 42, 248))
		drawRotatedBoxOutline(screen, cam, platPortal, cosH, sinH, 26.0, 12.0, 1.4, sim.RGBA(56, 189, 248, 220))
		if cam.Zoom >= 0.42 {
			for stepOff := -10.0; stepOff <= 6.0; stepOff += 3.8 {
				tread := platPortal.Add(forward.Mul(stepOff))
				drawRotatedBox(screen, cam, tread, cosH, sinH, 1.8, 9.6, sim.RGBA(148, 163, 184, 215))
			}
		}
		drawRotatedBox(screen, cam, platPortal.Add(forward.Mul(8.0)), cosH, sinH, 12.0, 13.0, sim.RGBA(14, 116, 144, 210))

		// Outer sidewalk underpass entrance pavilion (地道出入口)
		drawRotatedBox(screen, cam, sidewalkPos, cosH, sinH, 44.0, 16.0, sim.RGBA(30, 41, 59, 245))
		drawRotatedBoxOutline(screen, cam, sidewalkPos, cosH, sinH, 44.0, 16.0, 1.5, sim.RGBA(125, 211, 252, 220))
		drawRotatedBox(screen, cam, sidewalkPos.Add(forward.Mul(-6.0)), cosH, sinH, 24.0, 11.0, sim.RGBA(15, 23, 42, 250))
		if cam.Zoom >= 0.40 {
			for stepOff := -15.0; stepOff <= 1.0; stepOff += 4.0 {
				tread := sidewalkPos.Add(forward.Mul(stepOff))
				drawRotatedBox(screen, cam, tread, cosH, sinH, 1.8, 9.5, sim.RGBA(148, 163, 184, 220))
			}
		}
		canopySide := sidewalkPos.Add(forward.Mul(9.0))
		drawRotatedBox(screen, cam, canopySide, cosH, sinH, 18.0, 14.0, sim.RGBA(14, 116, 144, 225))
		drawRotatedBox(screen, cam, canopySide, cosH, sinH, 14.0, 3.5, s.AccentColor)
		return
	}

	// The bridge shadow belongs to the ground pass so it stays below vehicles.
	bridgeSpan := sidewalkLat + 8
	bridgeCenter := crossOrigin.Add(right.Mul(bridgeSpan * .5))
	shadowCenter := bridgeCenter.Add(forward.Mul(5))
	drawRotatedBox(screen, cam, shadowCenter, -sinH, cosH, bridgeSpan, 15, sim.RGBA(9, 13, 22, 95))
}

func brtSidewalkOffset(s *sim.BusStop, laneWidth float64) float64 {
	return math.Max(4, float64(s.RoadLanes)/2)*laneWidth + sim.BRTStationMaxFlare + 38
}

func brtCrossingOrigin(s *sim.BusStop) sim.Vec2 {
	if math.Abs(s.Heading) > 1 && math.Abs(math.Abs(s.Heading)-math.Pi) > .5 && math.Abs(s.RoadCenter.Y+900) < 95 {
		// At the canal bridge, put the pedestrian crossing on the solid bank.
		return s.RoadCenter.Add(sim.Vec2{Y: 108})
	}
	return s.RoadCenter
}

// DrawBRTOverpasses draws elevated decks and landings above vehicles.
func DrawBRTOverpasses(w *sim.World, screen *ebiten.Image, cam *sim.Camera, plan bool) {
	if !w.CenterBRT || cam.Zoom < .22 {
		return
	}
	darkness := w.Clock.Darkness()
	if plan {
		darkness = math.Min(darkness, .28)
	}
	for _, s := range w.Stops {
		drawBRTOverpass(s, screen, cam, darkness)
	}
}

func drawBRTOverpass(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, darkness float64) {
	if !s.CenterBRT || cam.Zoom < .22 || isBRTUnderpass(s.RoadCenter) {
		return
	}
	crossOrigin := brtCrossingOrigin(s)
	sinH, cosH := math.Sincos(s.Heading)
	forward := sim.Vec2{X: cosH, Y: sinH}
	right := sim.Vec2{X: -sinH, Y: cosH}
	laneWidth := math.Abs(s.Pos.Sub(s.RoadCenter).Dot(right)) * 2
	if laneWidth < 30 {
		laneWidth = 52
	}
	sidewalkLat := brtSidewalkOffset(s, laneWidth)
	if !cam.IsPointVisible(crossOrigin, sidewalkLat+55) {
		return
	}
	crossPlatform := crossOrigin.Add(right.Mul(sim.BRTPlatformOffset(laneWidth)))
	sidewalkPos := crossOrigin.Add(right.Mul(sidewalkLat))
	bridgeSpan := sidewalkLat + 8
	bridgeCenter := crossOrigin.Add(right.Mul(bridgeSpan * .5))
	shade := func(c color.RGBA) color.RGBA {
		shaded := tint(c, rgb(5, 12, 28), darkness)
		shaded.A = c.A
		return shaded
	}

	// Platform stair descending from overhead deck strictly onto the widened BRT platform
	platStair := crossPlatform.Add(forward.Mul(-18.0))
	drawRotatedBox(screen, cam, platStair, cosH, sinH, 28.0, 13.0, shade(sim.RGBA(51, 65, 85, 248)))
	if cam.Zoom >= 0.40 {
		for stepOff := -10.0; stepOff <= 10.0; stepOff += 4.0 {
			tread := platStair.Add(forward.Mul(stepOff))
			drawRotatedBox(screen, cam, tread, cosH, sinH, 1.8, 11.0, shade(sim.RGBA(203, 213, 225, 225)))
		}
	}

	// Outer sidewalk stair & elevator landing pavilion (天桥两侧人行道梯道)
	drawRotatedBox(screen, cam, sidewalkPos, cosH, sinH, 48.0, 16.0, shade(sim.RGBA(51, 65, 85, 246)))
	drawRotatedBoxOutline(screen, cam, sidewalkPos, cosH, sinH, 48.0, 16.0, 1.4, shade(sim.RGBA(148, 163, 184, 230)))
	if cam.Zoom >= 0.38 {
		for _, stairSign := range []float64{-1, 1} {
			for k := 1; k <= 4; k++ {
				tread := sidewalkPos.Add(forward.Mul(stairSign * (6.0 + float64(k)*4.0)))
				drawRotatedBox(screen, cam, tread, cosH, sinH, 1.8, 11.0, shade(sim.RGBA(226, 232, 240, 225)))
			}
		}
	}

	// Main cross-road overpass bridge deck (spanning from crossOrigin high overhead to outer sidewalk)
	// Sleek continuous modern tubular glass-enclosed pedestrian walkway.
	drawRotatedBox(screen, cam, bridgeCenter, -sinH, cosH, bridgeSpan, 14.0, shade(sim.RGBA(71, 85, 105, 246)))
	drawRotatedBox(screen, cam, bridgeCenter, -sinH, cosH, bridgeSpan-2.0, 10.0, shade(sim.RGBA(203, 213, 225, 240)))
	drawRotatedBox(screen, cam, bridgeCenter, -sinH, cosH, bridgeSpan-4.0, 6.5, shade(sim.RGBA(56, 189, 248, 165)))
	drawRotatedBox(screen, cam, bridgeCenter, -sinH, cosH, bridgeSpan-6.0, 2.5, shade(s.AccentColor))

	// Glass elevator towers strictly on the BRT platform (crossPlatform) and the outer sidewalk (sidewalkPos).
	// Crucially, NO elevator tower or ground structure exists at crossOrigin (road center).
	for _, towerPos := range []sim.Vec2{crossPlatform.Add(forward.Mul(14.0)), sidewalkPos} {
		drawRotatedBox(screen, cam, towerPos, cosH, sinH, 12.0, 12.0, shade(sim.RGBA(30, 58, 138, 245)))
		drawRotatedBoxOutline(screen, cam, towerPos, cosH, sinH, 12.0, 12.0, 1.4, shade(sim.RGBA(186, 230, 253, 245)))
	}
}

func drawRotatedBoxOutline(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, wid, stroke float64, c color.RGBA) {
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

	sw := float32(math.Max(1.5, stroke*cam.Zoom))
	strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), sw, c, true)
	strokeLine(screen, float32(s2.X), float32(s2.Y), float32(s3.X), float32(s3.Y), sw, c, true)
	strokeLine(screen, float32(s3.X), float32(s3.Y), float32(s4.X), float32(s4.Y), sw, c, true)
	strokeLine(screen, float32(s4.X), float32(s4.Y), float32(s1.X), float32(s1.Y), sw, c, true)
}
