package view

import (
	"fmt"
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// Draw renders the bus stop, waiting shelter, and passenger dots
func DrawStop(s *sim.BusStop, screen *ebiten.Image, cam *sim.Camera, isTarget bool, isHovered bool, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	// Frustum culling: skip drawing if stop and shelter are completely off-screen
	if !cam.IsPointVisible(s.Pos, s.Length/2+40) && !cam.IsPointVisible(s.ShelterPos, 160) {
		return
	}

	sinH, cosH := math.Sincos(s.Heading)

	// 1. Bus Stop Bay (Yellow bordered bay marked on the road)
	bayBorderColor := sim.RGBA(234, 179, 8, 220) // Vibrant yellow
	if isTarget {
		bayBorderColor = sim.RGBA(59, 130, 246, 255) // Neon blue for next stop target
	}
	drawRotatedBox(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, sim.RGBA(24, 24, 27, 240))
	drawRotatedBoxOutline(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, 3, bayBorderColor)

	// Diagonal yellow hazard stripes inside the bay (skip in plan mode or when zoomed out to reduce vector calls)
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

	if !planMode && cam.Zoom >= .4 && s.HasTwoBerths() {
		front := s.FrontBerthPosition()
		right := sim.Vec2{X: -sinH, Y: cosH}
		for _, p := range []sim.Vec2{front, s.RearBerthPosition()} {
			a := cam.WorldToScreen(p.Add(sim.Vec2{X: cosH, Y: sinH}.Mul(48)).Add(right.Mul(-s.Width/2 + 3)))
			b := cam.WorldToScreen(p.Add(sim.Vec2{X: cosH, Y: sinH}.Mul(48)).Add(right.Mul(s.Width/2 - 3)))
			strokeLine(screen, float32(a.X), float32(a.Y), float32(b.X), float32(b.Y), float32(2*cam.Zoom), bayBorderColor, true)
		}
	}

	// 2. Glass Bus Shelter on Sidewalk
	shelterLen := 68.0
	shelterWid := 18.0
	drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen, shelterWid, sim.RGBA(30, 41, 59, 240))
	// Shelter roof accent bar
	drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen-4, 4, s.AccentColor)

	// 3. Waiting Passengers visualization (dots lined up near shelter; skip when zoomed out)
	shelterScreen := cam.WorldToScreen(s.ShelterPos)
	if !planMode && cam.Zoom >= 0.45 {
		perpX := -sinH
		perpY := cosH
		for i := 0; i < s.WaitingPassengers && i < 24; i++ {
			row := float64(i / 12)
			col := float64(i % 12)
			pWorld := s.ShelterPos.Add(sim.Vec2{
				X: cosH*(col*6-32) + perpX*(row*6+12),
				Y: sinH*(col*6-32) + perpY*(row*6+12),
			})
			ps := cam.WorldToScreen(pWorld)
			filledCircle(screen, float32(ps.X), float32(ps.Y), float32(2.8*cam.Zoom),
				sim.RGBA(251, 191, 36, 230), true)
		}
	}

	// 4. Station Marker Pole / Flag
	badgeRadius := float32(14 * cam.Zoom)
	minRadius := float32(8)
	if planMode && cam.Zoom < .1 && !isTarget && !isHovered {
		minRadius = 2
	}
	if badgeRadius < minRadius {
		badgeRadius = minRadius
	}

	// Glowing circle badge
	badgeColor := s.AccentColor
	if isTarget {
		badgeColor = sim.RGBA(59, 130, 246, 255)
	}
	filledCircle(screen, float32(shelterScreen.X), float32(shelterScreen.Y-float64(badgeRadius*2.5)),
		badgeRadius, badgeColor, true)
	strokeCircle(screen, float32(shelterScreen.X), float32(shelterScreen.Y-float64(badgeRadius*2.5)),
		badgeRadius, float32(math.Max(1.5, 2*cam.Zoom)), sim.RGBA(255, 255, 255, 255), true)

	if planMode && cam.Zoom < .1 && !isTarget && !isHovered {
		return
	}

	// Station info label
	infoText := fmt.Sprintf("#%d %s (%d人)", s.ID+1, s.LocalizedName(), s.WaitingPassengers)
	if isTarget {
		infoText = "★ [目标站] " + infoText
		if s.WaitingPassengers == 0 {
			infoText += " [空站无候车]"
		}
	}
	if planMode && cam.Zoom < 0.2 && !isTarget && !isHovered {
		infoText = fmt.Sprintf("#%d %s", s.ID+1, s.CleanName())
	}
	labelX, labelY := shelterScreen.X-60, shelterScreen.Y-float64(badgeRadius*4)
	labelW, labelH := textui.MeasureText(infoText, 12)
	filledRect(screen, float32(labelX-5), float32(labelY-3), float32(labelW+10), float32(labelH+6), sim.RGBA(15, 25, 35, 218), true)
	textui.DrawText(screen, infoText, shelterScreen.X-60, shelterScreen.Y-float64(badgeRadius*4), 12, color.White)
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
