package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func drawRoundabouts(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, rb := range w.Roundabouts {
		if !cam.IsPointVisible(rb.Center, rb.Radius+80) {
			continue
		}

		cScreen := cam.WorldToScreen(rb.Center)
		rOuter := float32(rb.Radius * cam.Zoom)
		rInner := float32(rb.InnerRadius * cam.Zoom)
		if rInner < 3 {
			rInner = 3
		}

		// 1. Outer Slate Shoulder / Sidewalk Buffer Ring
		rShoulder := float32((rb.Radius + 18.0) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rShoulder, sceneColors(w.Theme).paving, true)

		// 2. Circular Asphalt Rotary Carriageway
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rOuter, sceneColors(w.Theme).asphalt, true)
		vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rOuter, float32(2.5*cam.Zoom), sim.RGBA(148, 163, 184, 200), true)

		// 3. Mountable Truck Apron Ring (环岛卡车围裙) around the central island
		rApron := float32((rb.InnerRadius + 16.0) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rApron, sim.RGBA(71, 85, 105, 240), true)
		vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rApron, float32(1.8*cam.Zoom), sim.RGBA(100, 116, 139, 180), true)

		// 4. Circulating Lane Dividing Dashed Circle (in middle of carriageway)
		if cam.Zoom >= 0.25 {
			midR := (rb.InnerRadius + 16.0 + rb.Radius) * 0.5
			numDashes := 24
			dashStep := (2 * math.Pi) / float64(numDashes)
			dashLen := dashStep * 0.55
			dashW := float32(2.2 * cam.Zoom)
			dashCol := sim.RGBA(248, 250, 252, 215)
			for i := 0; i < numDashes; i++ {
				th1 := float64(i) * dashStep
				th2 := th1 + dashLen
				p1 := rb.Center.Add(sim.Vec2{X: math.Cos(th1) * midR, Y: math.Sin(th1) * midR})
				p2 := rb.Center.Add(sim.Vec2{X: math.Cos(th2) * midR, Y: math.Sin(th2) * midR})
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), dashW, dashCol, true)
			}
		}

		// 5. Entry Yield Markings (让行虚线 & 让行倒三角) on incoming approaches
		if cam.Zoom >= 0.32 {
			drawRoundaboutYieldEntrances(screen, cam, w, rb, laneWidth)
		}

		// 6. Central Island & Themed Monuments
		switch rb.Type {
		case sim.RoundaboutCivic:
			// Concentric plaza paving
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, sceneColors(w.Theme).asphalt, true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.0*cam.Zoom), sim.RGBA(203, 213, 225, 230), true)
			if rInner > 15 {
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner*0.75, float32(1.5*cam.Zoom), sim.RGBA(71, 85, 105, 180), true)
				// Central water fountain
				fountainR := rInner * 0.45
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR, sim.RGBA(14, 116, 144, 240), true)
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR, float32(2*cam.Zoom), sim.RGBA(56, 189, 248, 220), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR*0.35, sim.RGBA(255, 255, 255, 255), true)
			}

		case sim.RoundaboutLighthouse:
			// Coastal green lawn island
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, sim.RGBA(22, 101, 52, 255), true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.0*cam.Zoom), sim.RGBA(203, 213, 225, 230), true)
			if rInner > 15 {
				// Historic Nautical Lighthouse tower
				towerR := rInner * 0.4
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR, sim.RGBA(248, 250, 252, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.7, sim.RGBA(239, 68, 68, 255), true)
				// Beacon lamp glow
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.35, sim.RGBA(250, 204, 21, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.8, sim.RGBA(254, 240, 138, 90), true)
			}

		case sim.RoundaboutIsland:
			// Emerald water moat buffer & green island
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, sim.RGBA(22, 101, 52, 255), true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.5*cam.Zoom), sim.RGBA(4, 120, 87, 230), true)
			if rInner > 15 {
				// Classical Chinese octagonal gazebo/pavilion roof
				pavR := rInner * 0.42
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR, sim.RGBA(217, 119, 6, 255), true)
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR, float32(2*cam.Zoom), sim.RGBA(180, 83, 9, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR*0.28, sim.RGBA(250, 204, 21, 255), true)
			}
		}
	}
}

func drawRoundaboutYieldEntrances(screen *ebiten.Image, cam *sim.Camera, w *sim.World, rb sim.Roundabout, laneWidth float64) {
	entries := []struct {
		pos, dir sim.Vec2
		width    float64
	}{}
	for _, dir := range []sim.Vec2{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}} {
		probe := rb.Center.Sub(dir.Mul(rb.YieldLineDistance() + 60))
		ew := math.Abs(dir.X) > .5
		coord := rb.Center.X
		if ew {
			coord = rb.Center.Y
		}
		road, ok := w.RoadAt(coord, ew, probe)
		if !ok {
			continue
		}
		width := float64(road.LaneCount()/2) * laneWidth
		right := sim.Vec2{X: -dir.Y, Y: dir.X}
		entries = append(entries, struct {
			pos, dir sim.Vec2
			width    float64
		}{rb.Center.Sub(dir.Mul(rb.YieldLineDistance())).Add(right.Mul(width / 2)), dir, width - 12})
	}

	for _, e := range entries {
		perp := sim.Vec2{X: -e.dir.Y, Y: e.dir.X}
		halfW := e.width * 0.5
		pA := e.pos.Add(perp.Mul(halfW))
		pB := e.pos.Sub(perp.Mul(halfW))

		// Draw dash segments across entry
		for i := 0; i < 4; i++ {
			t1 := float64(i) / 4.0
			t2 := t1 + 0.16
			pt1 := pA.Add(pB.Sub(pA).Mul(t1))
			pt2 := pA.Add(pB.Sub(pA).Mul(t2))
			s1 := cam.WorldToScreen(pt1)
			s2 := cam.WorldToScreen(pt2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
				float32(3.0*cam.Zoom), sim.RGBA(248, 250, 252, 220), true)
		}

		// Yield Triangle (▽) pointing towards the roundabout
		triTip := e.pos.Sub(e.dir.Mul(14.0))
		triBaseCenter := e.pos.Sub(e.dir.Mul(34.0))
		triL := triBaseCenter.Add(perp.Mul(9.0))
		triR := triBaseCenter.Sub(perp.Mul(9.0))

		sTip := cam.WorldToScreen(triTip)
		sL := cam.WorldToScreen(triL)
		sR := cam.WorldToScreen(triR)
		vector.StrokeLine(screen, float32(sTip.X), float32(sTip.Y), float32(sL.X), float32(sL.Y), float32(2.0*cam.Zoom), sim.RGBA(248, 250, 252, 210), true)
		vector.StrokeLine(screen, float32(sTip.X), float32(sTip.Y), float32(sR.X), float32(sR.Y), float32(2.0*cam.Zoom), sim.RGBA(248, 250, 252, 210), true)
		vector.StrokeLine(screen, float32(sL.X), float32(sL.Y), float32(sR.X), float32(sR.Y), float32(2.0*cam.Zoom), sim.RGBA(248, 250, 252, 210), true)
	}
}
