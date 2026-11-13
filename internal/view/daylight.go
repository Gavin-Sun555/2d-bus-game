package view

import (
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/hajimehoshi/ebiten/v2"
)

type streetLamp struct {
	base, head, pool sim.Vec2
	heading          float64
}
type lampScene struct {
	world *sim.World
	first *sim.RoadSegment
	count int
	width float64
	lamps []streetLamp
}

var lampScenes = map[sim.MapTheme]lampScene{}

func streetLamps(w *sim.World, lw float64) []streetLamp {
	if len(w.Roads) == 0 {
		return nil
	}
	first := &w.Roads[0]
	if c, ok := lampScenes[w.Theme]; ok && c.world == w && c.first == first && c.count == len(w.Roads) && c.width == lw {
		return c.lamps
	}
	c := lampScene{world: w, first: first, count: len(w.Roads), width: lw}
	for index, road := range w.Roads {
		length := road.P2.Sub(road.P1).Length()
		if length < 300 {
			continue
		}
		u := road.P2.Sub(road.P1).Normalize()
		r := sim.Vec2{X: -u.Y, Y: u.X}
		heading := math.Atan2(u.Y, u.X)
		var blocked [][2]float64
		for _, center := range w.Intersections {
			d := center.Sub(road.P1)
			along := d.Dot(u)
			if along < 0 || along > length || math.Abs(d.Dot(r)) > 1 {
				continue
			}
			g := w.JunctionAt(center, lw)
			half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY + 70
			blocked = append(blocked, [2]float64{along - half, along + half})
		}
		sort.Slice(blocked, func(i, j int) bool { return blocked[i][0] < blocked[j][0] })
		block := 0
		spacing := math.Max(340, lw*6)
		for d := 160 + float64(index%4)*27; d < length-100; d += spacing {
			for block < len(blocked) && blocked[block][1] < d {
				block++
			}
			if block < len(blocked) && d >= blocked[block][0] {
				continue
			}
			center := road.P1.Add(u.Mul(d))
			half := road.GetWidth(lw)/2 + sim.ShoulderWidth + 15
			for _, side := range []float64{-1, 1} {
				base := center.Add(r.Mul(side * half))
				head := base.Sub(r.Mul(side * 18))
				c.lamps = append(c.lamps, streetLamp{base: base, head: head, pool: head.Sub(r.Mul(side * 48)), heading: heading})
			}
			if road.LaneCount() >= 6 {
				c.lamps = append(c.lamps, streetLamp{base: center, head: center.Add(r.Mul(10)), pool: center, heading: heading})
			}
		}
	}
	lampScenes[w.Theme] = c
	return c.lamps
}
func drawStreetLampFixtures(screen *ebiten.Image, cam *sim.Camera, w *sim.World, lw float64) {
	if cam.Zoom < .16 {
		return
	}
	for _, lamp := range streetLamps(w, lw) {
		if !cam.IsPointVisible(lamp.base, 25) {
			continue
		}
		worldLine(screen, cam, lamp.base.Add(sim.Vec2{X: 3, Y: 4}), lamp.head.Add(sim.Vec2{X: 3, Y: 4}), 2, sim.RGBA(0, 0, 0, 100))
		worldCircle(screen, cam, lamp.base, 3, rgb(78, 94, 108))
		worldLine(screen, cam, lamp.base, lamp.head, 2.2, rgb(143, 161, 172))
		u := sim.UnitFromAngle(lamp.heading)
		drawRotatedBox(screen, cam, lamp.head, u.X, u.Y, 9, 4, rgb(136, 156, 165))
	}
}

var lightPoolTexture *ebiten.Image

func glowTexture() *ebiten.Image {
	if lightPoolTexture != nil {
		return lightPoolTexture
	}
	const size = 96
	pixels := make([]byte, size*size*4)
	for y := 0; y < size; y++ {
		for x := 0; x < size; x++ {
			dx, dy := (float64(x)+.5-size/2)/(size/2), (float64(y)+.5-size/2)/(size/2)
			v := math.Max(0, 1-dx*dx-dy*dy)
			a := uint8(v * v * 255)
			i := (y*size + x) * 4
			pixels[i], pixels[i+1], pixels[i+2], pixels[i+3] = a, a, a, a
		}
	}
	lightPoolTexture = ebiten.NewImage(size, size)
	lightPoolTexture.WritePixels(pixels)
	return lightPoolTexture
}
func drawGlow(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, rx, ry, heading float64, r, g, b, strength float32) {
	if !cam.IsPointVisible(pos, math.Max(rx, ry)) {
		return
	}
	texture := glowTexture()
	size := float64(texture.Bounds().Dx())
	p := cam.WorldToScreen(pos)
	var op ebiten.DrawImageOptions
	op.GeoM.Translate(-size/2, -size/2)
	op.GeoM.Scale(2*rx*cam.Zoom/size, 2*ry*cam.Zoom/size)
	op.GeoM.Rotate(heading - cam.Rotation)
	op.GeoM.Translate(p.X, p.Y)
	op.ColorScale.Scale(r*strength, g*strength, b*strength, strength)
	op.Blend = ebiten.BlendLighter
	op.Filter = ebiten.FilterLinear
	screen.DrawImage(texture, &op)
}

// A filled, feathering cone replaces the former three visible headlight rays.
func drawLightCone(screen *ebiten.Image, cam *sim.Camera, start sim.Vec2, heading, length, spread, strength float64) {
	if !cam.IsPointVisible(start, length+spread) {
		return
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	var verts [9]ebiten.Vertex
	for i, d := range []float64{0, length * .28, length} {
		half := []float64{2, spread * .45, spread}[i]
		alpha := []float64{strength * .7, strength, 0}[i]
		for col := 0; col < 3; col++ {
			side := float64(col - 1)
			p := cam.WorldToScreen(start.Add(u.Mul(d)).Add(r.Mul(side * half)))
			fade := 0.
			if col == 1 {
				fade = 1
			}
			verts[i*3+col] = ebiten.Vertex{DstX: float32(p.X), DstY: float32(p.Y), SrcX: 1, SrcY: 1, ColorR: 1, ColorG: .97, ColorB: .80, ColorA: float32(alpha * fade)}
		}
	}
	screen.DrawTriangles(verts[:], []uint16{0, 1, 3, 1, 4, 3, 1, 2, 4, 2, 5, 4, 3, 4, 6, 4, 7, 6, 4, 5, 7, 5, 8, 7}, getWhiteSubImage(), &ebiten.DrawTrianglesOptions{Blend: ebiten.BlendLighter})
}
func vehicleLightPositions(pos sim.Vec2, heading, length, width, inset float64) (sim.Vec2, sim.Vec2) {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	front := pos.Add(u.Mul(length / 2))
	return front.Sub(r.Mul(width/2 - inset)), front.Add(r.Mul(width/2 - inset))
}
func DrawDayNightLighting(screen *ebiten.Image, cam *sim.Camera, w *sim.World, lw float64, player *sim.Bus, tm *traffic.TrafficManager, fleet *traffic.AIBusManager, density int, plan bool) {
	darkness := w.Clock.Darkness()
	if plan {
		darkness = math.Min(darkness, .28)
	}
	if darkness > 0 {
		filledRect(screen, 0, 0, float32(screen.Bounds().Dx()), float32(screen.Bounds().Dy()), sim.RGBA(5, 12, 28, uint8(darkness*255)), false)
	}
	if plan {
		return
	}
	lamps := streetLamps(w, lw)
	// Light pools are batched in one additive texture pass, independent of bodies.
	if w.Clock.IsNight() && cam.Zoom >= .12 {
		for _, lamp := range lamps {
			drawGlow(screen, cam, lamp.pool, 140, 100, lamp.heading, 1, .85, .61, .20)
			drawGlow(screen, cam, lamp.head, 12, 12, 0, 1, .86, .63, .45)
		}
		drawNightStationLights(screen, cam, w)
	}
	buses := []*sim.Bus{player}
	if fleet != nil {
		for _, a := range fleet.Buses {
			buses = append(buses, a.Bus)
		}
	}
	if darkness > 0 {
		for _, b := range buses {
			if b != nil && b.HeadlightsOn && cam.IsPointVisible(b.Pos, 260) {
				a, z := vehicleLightPositions(b.Pos, b.Heading, b.Length, b.Width, 5)
				for _, p := range []sim.Vec2{a, z} {
					drawGlow(screen, cam, p, 10, 10, 0, 1, .97, .8, .34)
				}
			}
		}
		if tm != nil {
			for _, c := range tm.GetActiveCars(density) {
				if c.HeadlightsOn && cam.IsPointVisible(c.Pos, 190) {
					a, z := vehicleLightPositions(c.Pos, c.VisualHeading(), c.Length, c.Width, 3)
					drawGlow(screen, cam, a, 6, 6, 0, 1, .97, .8, .3)
					drawGlow(screen, cam, z, 6, 6, 0, 1, .97, .8, .3)
				}
			}
		}
	}
	// Beams illuminate the asphalt after the ambient pass; HUD is drawn later.
	if darkness > 0 {
		for _, b := range buses {
			if b == nil || !b.HeadlightsOn || !cam.IsPointVisible(b.Pos, 260) {
				continue
			}
			a, z := vehicleLightPositions(b.Pos, b.Heading, b.Length, b.Width, 5)
			drawLightCone(screen, cam, a, b.Heading, 210, 40, .26)
			drawLightCone(screen, cam, z, b.Heading, 210, 40, .26)
			u := sim.UnitFromAngle(b.Heading)
			drawRotatedBox(screen, cam, a, u.X, u.Y, 3, 5, rgb(250, 249, 231))
			drawRotatedBox(screen, cam, z, u.X, u.Y, 3, 5, rgb(250, 249, 231))
		}
		if tm != nil {
			for _, c := range tm.GetActiveCars(density) {
				if !c.HeadlightsOn || !cam.IsPointVisible(c.Pos, 190) {
					continue
				}
				a, z := vehicleLightPositions(c.Pos, c.VisualHeading(), c.Length, c.Width, 3)
				drawLightCone(screen, cam, a, c.VisualHeading(), 130, 26, .22)
				drawLightCone(screen, cam, z, c.VisualHeading(), 130, 26, .22)
				u := sim.UnitFromAngle(c.VisualHeading())
				drawRotatedBox(screen, cam, a, u.X, u.Y, 2, 3, rgb(250, 249, 231))
				drawRotatedBox(screen, cam, z, u.X, u.Y, 2, 3, rgb(250, 249, 231))
				tail := rgb(167, 34, 42)
				if c.IsBraking {
					tail = rgb(255, 68, 60)
				}
				right := sim.Vec2{X: -u.Y, Y: u.X}
				for _, side := range []float64{-1, 1} {
					p := c.Pos.Sub(u.Mul(c.Length/2 - 1)).Add(right.Mul(side * (c.Width/2 - 3)))
					drawRotatedBox(screen, cam, p, u.X, u.Y, 2, 4, tail)
				}
			}
		}
	}
	if darkness > 0 {
		for _, b := range buses {
			if b == nil || !cam.IsPointVisible(b.Pos, b.OverallLength()+20) || (!b.HeadlightsOn && !b.IsBraking && !b.IsHandbraking && !b.IsReversing) {
				continue
			}
			pos, h := b.Pos, b.Heading
			if b.IsArticulated() {
				tail := b.TrailerBody()
				pos, h = tail.Center, tail.Heading
			}
			u := sim.UnitFromAngle(h)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			col := rgb(175, 35, 43)
			if b.IsBraking || b.IsHandbraking {
				col = rgb(255, 68, 60)
			} else if b.IsReversing {
				col = rgb(245, 245, 236)
			}
			for _, side := range []float64{-1, 1} {
				p := pos.Sub(u.Mul(b.Length / 2)).Add(r.Mul(side * (b.Width/2 - 5)))
				drawRotatedBox(screen, cam, p, u.X, u.Y, 3, 5, col)
			}
		}
	}
	if w.Clock.IsNight() && cam.Zoom >= .16 {
		for _, lamp := range lamps {
			if cam.IsPointVisible(lamp.head, 12) {
				u := sim.UnitFromAngle(lamp.heading)
				drawRotatedBox(screen, cam, lamp.head, u.X, u.Y, 8, 3, rgb(255, 232, 179))
			}
		}
	}
}

func drawNightStationLights(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	if cam.Zoom < .16 {
		return
	}
	warm := rgb(255, 237, 191)
	for _, stop := range w.Stops {
		if !cam.IsPointVisible(stop.ShelterPos, 115) {
			continue
		}
		forward := sim.UnitFromAngle(stop.Heading)
		span, glow := 44., 52.
		if stop.CenterBRT {
			span, glow = 104, 89
		}
		drawGlow(screen, cam, stop.ShelterPos, glow, 30, stop.Heading, 1, .86, .64, .25)
		drawRotatedBox(screen, cam, stop.ShelterPos, forward.X, forward.Y, span, 4, warm)
	}
	for _, rail := range w.Railways() {
		if !cam.IsPointVisible(rail.StationCenter, 390) {
			continue
		}
		// Island platforms and the entrance stay lit between trains.
		for _, side := range [...]float64{-1, 1} {
			for _, x := range [...]float64{-210, -70, 70, 210} {
				pos := sim.Vec2{X: rail.StationCenter.X + x, Y: rail.Y + side*rail.TrackSpacing*.24}
				drawGlow(screen, cam, pos, 76, 22, 0, 1, .88, .69, .29)
				drawRotatedBox(screen, cam, pos, 1, 0, 36, 3, warm)
			}
		}
		entrance := rail.StationCenter.Add(rail.StationApproach.Sub(rail.StationCenter).Normalize().Mul(.34 * rail.StationDepth))
		drawGlow(screen, cam, entrance, 70, 40, 0, 1, .82, .56, .35)
		drawRotatedBox(screen, cam, entrance, 1, 0, 52, 4, warm)
	}
}
