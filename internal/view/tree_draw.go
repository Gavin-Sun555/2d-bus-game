package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawTree(screen *ebiten.Image, cam *sim.Camera, t sim.Tree, plan bool, theme sim.MapTheme) {
	if !cam.IsPointVisible(t.Pos, t.Radius*2+16) {
		return
	}
	p := sceneColors(theme)
	if theme == sim.MapThemeCustom {
		p.foliage = t.Color
		p.leafLight = tint(t.Color, rgb(180, 210, 145), 0.4)
	}
	radius := t.Radius * 1.2
	if plan || cam.Zoom < 0.3 {
		worldCircle(screen, cam, t.Pos, radius, p.foliage)
		return
	}
	worldCircle(screen, cam, t.Pos.Add(sim.Vec2{X: 9, Y: 11}), radius, sim.RGBA(8, 25, 25, 65))
	switch theme {
	case sim.MapThemeCoastalBay:
		worldCircle(screen, cam, t.Pos, 3, rgb(144, 103, 65))
		for i := 0; i < 7; i++ {
			angle := float64(i)*2*math.Pi/7 + t.Pos.X*0.01
			f := sim.UnitFromAngle(angle)
			side := sim.Vec2{X: -f.Y, Y: f.X}
			tip := t.Pos.Add(f.Mul(radius * 1.65))
			middle := t.Pos.Add(f.Mul(radius * .62))
			worldPolygon(screen, cam, []sim.Vec2{t.Pos, middle.Add(side.Mul(radius * .26)), tip, middle.Sub(side.Mul(radius * .18))}, p.foliage)
			worldLine(screen, cam, t.Pos, tip, 0.8, p.leafLight)
		}
	case sim.MapThemeRiverValley:
		for i := 2; i >= 0; i-- {
			y := float64(i)*radius*.36 - radius*.65
			size := radius * (.66 + float64(i)*.18)
			center := t.Pos.Add(sim.Vec2{Y: y})
			worldPolygon(screen, cam, []sim.Vec2{center.Add(sim.Vec2{Y: -size}), center.Add(sim.Vec2{X: size * .8, Y: size * .5}), center.Add(sim.Vec2{X: -size * .8, Y: size * .5})}, tint(p.foliage, p.leafLight, float64(2-i)*.12))
		}
	case sim.MapThemeEmeraldLake:
		worldCircle(screen, cam, t.Pos, radius, p.foliage)
		for i := 0; i < 6; i++ {
			angle := float64(i) * math.Pi / 3
			center := t.Pos.Add(sim.Vec2{X: math.Cos(angle) * radius * .55, Y: math.Sin(angle) * radius * .55})
			worldCircle(screen, cam, center, radius*.52, tint(p.foliage, p.leafLight, float64(i%3)*.13))
			worldLine(screen, cam, center, center.Add(sim.Vec2{Y: radius * .9}), 1.6, p.leafLight)
		}
	default:
		for i := 0; i < 4; i++ {
			angle := float64(i) * math.Pi / 2
			center := t.Pos.Add(sim.Vec2{X: math.Cos(angle) * radius * .28, Y: math.Sin(angle) * radius * .28})
			worldCircle(screen, cam, center, radius*.74, tint(p.foliage, p.leafLight, float64(i)*.08))
		}
		worldCircle(screen, cam, t.Pos.Add(sim.Vec2{X: -radius * .25, Y: -radius * .3}), radius*.43, tint(p.foliage, p.leafLight, .65))
	}
}
