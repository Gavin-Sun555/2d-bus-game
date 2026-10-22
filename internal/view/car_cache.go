package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
	"image/color"
	"math"
)

type carSpriteKey struct {
	Color          color.RGBA
	Length, Width  float64
	Type           sim.CarType
	Braking, Beams bool
}
type carSprite struct {
	image  *ebiten.Image
	origin sim.Vec2
}

var carSprites = map[carSpriteKey]carSprite{}

// Rasterize each stable livery once at 2x resolution, then batch its body,
// lamps and beams as one image. Shadows stay in world space. Braking and zoom
// beam visibility have independent keys, so the cache never hides live state.
func drawCachedCar(screen *ebiten.Image, cam *sim.Camera, c *sim.TrafficCar) bool {
	key := carSpriteKey{c.Color, c.Length, c.Width, c.Type, c.IsBraking, cam.Zoom >= .20}
	sprite, ok := carSprites[key]
	if !ok {
		if len(carSprites) >= 128 {
			return false
		}
		width, height := int(math.Ceil((c.Length+65)*2)), int(math.Ceil((c.Width+30)*2))
		sprite = carSprite{image: ebiten.NewImage(width, height), origin: sim.Vec2{X: (c.Length/2 + 4) * 2, Y: float64(height) / 2}}
		local := sim.NewCamera(width, height)
		local.Zoom = 2
		local.Pos = sim.Vec2{X: (float64(width)/2 - sprite.origin.X) / 2}
		copyCar := *c
		copyCar.Pos = sim.Vec2{}
		copyCar.Heading = 0
		drawCarDetailsWithBeams(sprite.image, local, &copyCar, key.Beams)
		carSprites[key] = sprite
	}
	var op ebiten.DrawImageOptions
	op.GeoM.Translate(-sprite.origin.X, -sprite.origin.Y)
	op.GeoM.Scale(cam.Zoom/2, cam.Zoom/2)
	op.GeoM.Rotate(c.Heading - cam.Rotation)
	p := cam.WorldToScreen(c.Pos)
	op.GeoM.Translate(p.X, p.Y)
	op.Filter = ebiten.FilterLinear
	screen.DrawImage(sprite.image, &op)
	return true
}
