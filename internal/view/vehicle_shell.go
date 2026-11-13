package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
	"image/color"
	"math"
)

var vehicleShellIndices = []uint16{0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 7, 0, 7, 8, 0, 8, 1}

// The outline stays inside the physical footprint; bevels, edge shading and
// separate bumpers replace the old rectangular body with an oversized nose.
func drawVehicleShell(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading, length, width float64, c color.RGBA, front, rear float64) {
	if !cam.IsPointVisible(pos, math.Hypot(length, width)/2+10) {
		return
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	hl, hw := length/2, width/2
	points := [8]sim.Vec2{{X: hl - front, Y: -hw}, {X: hl, Y: -hw + front}, {X: hl, Y: hw - front}, {X: hl - front, Y: hw}, {X: -hl + rear, Y: hw}, {X: -hl, Y: hw - rear}, {X: -hl, Y: -hw + rear}, {X: -hl + rear, Y: -hw}}
	var v [9]ebiten.Vertex
	center := cam.WorldToScreen(pos)
	v[0] = ebiten.Vertex{DstX: float32(center.X), DstY: float32(center.Y), SrcX: 1, SrcY: 1, ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255}
	for i, p := range points {
		q := cam.WorldToScreen(pos.Add(u.Mul(p.X)).Add(r.Mul(p.Y)))
		shade := .72
		if p.Y < 0 {
			shade = 1.12
		}
		v[i+1] = ebiten.Vertex{DstX: float32(q.X), DstY: float32(q.Y), SrcX: 1, SrcY: 1, ColorR: float32(math.Min(255, float64(c.R)*shade)) / 255, ColorG: float32(math.Min(255, float64(c.G)*shade)) / 255, ColorB: float32(math.Min(255, float64(c.B)*shade)) / 255, ColorA: float32(c.A) / 255}
	}
	screen.DrawTriangles(v[:], vehicleShellIndices, getWhiteSubImage(), nil)
}
