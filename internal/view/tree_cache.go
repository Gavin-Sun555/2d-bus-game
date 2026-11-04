package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type treeSpriteKey struct {
	theme  sim.MapTheme
	radius int
	color  color.RGBA
}

type treeSprite struct {
	image  *ebiten.Image
	origin sim.Vec2
}

var treeSprites = map[treeSpriteKey]treeSprite{}

// drawCachedTree renders a static pre-baked tree sprite if available,
// collapsing up to 16 individual DrawTriangles calls per tree into a single batched DrawImage.
func drawCachedTree(screen *ebiten.Image, cam *sim.Camera, t sim.Tree, theme sim.MapTheme) bool {
	if cam.Zoom < 0.28 {
		return false
	}
	radius := t.Radius * 1.2
	rInt := int(math.Round(radius))
	if rInt <= 0 {
		return false
	}
	key := treeSpriteKey{theme: theme, radius: rInt, color: t.Color}
	sprite, ok := treeSprites[key]
	if !ok {
		if len(treeSprites) >= 64 {
			return false
		}
		// Size with room for shadow and spread
		size := int(math.Ceil(radius*3.6 + 50))
		img := ebiten.NewImage(size, size)
		origin := sim.Vec2{X: float64(size) / 2, Y: float64(size) / 2}

		localCam := sim.NewCamera(size, size)
		localCam.Pos = sim.Vec2{}
		localCam.Zoom = 1.0

		// Render the tree centered at (0, 0)
		renderTreeGeometry(img, localCam, sim.Vec2{}, radius, t.Color, theme)

		sprite = treeSprite{image: img, origin: origin}
		treeSprites[key] = sprite
	}

	p := cam.WorldToScreen(t.Pos)
	var op ebiten.DrawImageOptions
	op.GeoM.Translate(-sprite.origin.X, -sprite.origin.Y)
	op.GeoM.Scale(cam.Zoom, cam.Zoom)
	op.GeoM.Translate(p.X, p.Y)
	op.Filter = ebiten.FilterLinear
	screen.DrawImage(sprite.image, &op)
	return true
}

func renderTreeGeometry(target *ebiten.Image, cam *sim.Camera, center sim.Vec2, radius float64, customColor color.RGBA, theme sim.MapTheme) {
	p := sceneColors(theme)
	if theme == sim.MapThemeCustom {
		p.foliage = customColor
		p.leafLight = tint(customColor, rgb(180, 210, 145), 0.4)
	}

	// Ambient shadow
	worldCircle(target, cam, center.Add(sim.Vec2{X: 9, Y: 11}), radius, sim.RGBA(8, 25, 25, 65))

	switch theme {
	case sim.MapThemeCoastalBay:
		worldCircle(target, cam, center, 3, rgb(144, 103, 65))
		for i := 0; i < 7; i++ {
			angle := float64(i)*2*math.Pi/7 + 0.01
			f := sim.UnitFromAngle(angle)
			side := sim.Vec2{X: -f.Y, Y: f.X}
			tip := center.Add(f.Mul(radius * 1.65))
			middle := center.Add(f.Mul(radius * .62))
			worldQuad(target, cam, center, middle.Add(side.Mul(radius*.26)), tip, middle.Sub(side.Mul(radius*.18)), p.foliage)
			worldLine(target, cam, center, tip, 0.8, p.leafLight)
		}
	case sim.MapThemeRiverValley:
		for i := 2; i >= 0; i-- {
			y := float64(i)*radius*.36 - radius*.65
			size := radius * (.66 + float64(i)*.18)
			c := center.Add(sim.Vec2{Y: y})
			worldTriangle(target, cam, c.Add(sim.Vec2{Y: -size}), c.Add(sim.Vec2{X: size * .8, Y: size * .5}), c.Add(sim.Vec2{X: -size * .8, Y: size * .5}), tint(p.foliage, p.leafLight, float64(2-i)*.12))
		}
	case sim.MapThemeEmeraldLake:
		worldCircle(target, cam, center, radius, p.foliage)
		for i := 0; i < 6; i++ {
			angle := float64(i) * math.Pi / 3
			c := center.Add(sim.Vec2{X: math.Cos(angle) * radius * .55, Y: math.Sin(angle) * radius * .55})
			worldCircle(target, cam, c, radius*.52, tint(p.foliage, p.leafLight, float64(i%3)*.13))
			worldLine(target, cam, c, c.Add(sim.Vec2{Y: radius * .9}), 1.6, p.leafLight)
		}
	default:
		for i := 0; i < 4; i++ {
			angle := float64(i) * math.Pi / 2
			c := center.Add(sim.Vec2{X: math.Cos(angle) * radius * .28, Y: math.Sin(angle) * radius * .28})
			worldCircle(target, cam, c, radius*.68, tint(p.foliage, p.leafLight, float64(i)*.12))
		}
		worldCircle(target, cam, center, radius*.78, p.foliage)
		worldCircle(target, cam, center.Add(sim.Vec2{X: -radius * .16, Y: -radius * .16}), radius*.38, p.leafLight)
	}
}

