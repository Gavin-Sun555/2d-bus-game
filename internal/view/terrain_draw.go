package view

import (
	"image"
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

// Small cached material tiles are world-anchored: camera motion never makes the
// grain crawl. Their deterministic hash deliberately does not use simulation RNG.
var terrainTiles = map[sim.MapTheme]*ebiten.Image{}

const terrainTileSize = 512.0

func terrainTile(theme sim.MapTheme) *ebiten.Image {
	if tile := terrainTiles[theme]; tile != nil {
		return tile
	}
	p := sceneColors(theme)
	pixels := image.NewRGBA(image.Rect(0, 0, 256, 256))
	for y := 0; y < 256; y++ {
		for x := 0; x < 256; x++ {
			hash := uint32(x*374761393) ^ uint32(y*668265263) ^ uint32(theme+1)*2246822519
			hash = (hash ^ (hash >> 13)) * 1274126177
			amount := float64(hash%100) / 100 * 0.22
			c := tint(p.ground, p.grain, amount)
			switch theme {
			case sim.MapThemeMetropolis, sim.MapThemeTwinCities:
				if x%64 == 0 || y%64 == 0 {
					c = tint(p.ground, p.grain, 0.65)
				}
				if x%64 == 1 || y%64 == 1 {
					c = tint(p.ground, rgb(0, 0, 0), 0.14)
				}
			case sim.MapThemeCoastalBay:
				if (y+int(3*math.Sin(float64(x)*0.04)))%29 == 0 {
					c = tint(c, p.grain, 0.4)
				}
			case sim.MapThemeRiverValley:
				if (x/5+y/3)%47 == 0 {
					c = tint(c, p.grain, 0.35)
				}
			case sim.MapThemeEmeraldLake:
				if hash%61 < 2 {
					c = tint(c, p.leafLight, 0.25)
				}
			}
			pixels.SetRGBA(x, y, c)
		}
	}
	tile := ebiten.NewImageFromImage(pixels)
	terrainTiles[theme] = tile
	return tile
}

func visibleWorldBounds(cam *sim.Camera) (float64, float64, float64, float64) {
	minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
	for _, q := range []sim.Vec2{{}, {X: float64(cam.ScreenWidth)}, {Y: float64(cam.ScreenHeight)}, {X: float64(cam.ScreenWidth), Y: float64(cam.ScreenHeight)}} {
		p := cam.ScreenToWorld(q)
		minX = math.Min(minX, p.X)
		maxX = math.Max(maxX, p.X)
		minY = math.Min(minY, p.Y)
		maxY = math.Max(maxY, p.Y)
	}
	return minX, minY, maxX, maxY
}

func drawTerrain(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	p := sceneColors(w.Theme)
	screen.Fill(p.ground)
	if cam.Zoom < 0.16 {
		return
	}
	tile := terrainTile(w.Theme)
	minX, minY, maxX, maxY := visibleWorldBounds(cam)
	for y := math.Floor(minY/terrainTileSize) * terrainTileSize; y <= maxY; y += terrainTileSize {
		for x := math.Floor(minX/terrainTileSize) * terrainTileSize; x <= maxX; x += terrainTileSize {
			origin := cam.WorldToScreen(sim.Vec2{X: x, Y: y})
			op := ebiten.DrawImageOptions{}
			op.GeoM.Scale(2*cam.Zoom, 2*cam.Zoom)
			op.GeoM.Rotate(-cam.Rotation)
			op.GeoM.Translate(origin.X, origin.Y)
			op.Filter = ebiten.FilterLinear
			screen.DrawImage(tile, &op)
		}
	}
}

// These are scenic foundations, rendered before roads. They never add collision
// objects or alter a route, bay, bridge or road's physical dimensions.
func drawScenicFoundations(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	p := sceneColors(w.Theme)
	for _, b := range w.Buildings {
		if !cam.IsPointVisible(b.Pos, math.Hypot(b.W, b.H)) {
			continue
		}
		border := 26.0
		ground := tint(p.ground, p.paving, 0.35)
		switch w.Theme {
		case sim.MapThemeCoastalBay:
			border = 65
			ground = rgb(192, 175, 129)
		case sim.MapThemeRiverValley:
			for layer := 3; layer > 0; layer-- {
				padding := float64(layer) * 34
				drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W+padding, b.H+padding, tint(p.ground, p.grain, float64(layer)*0.15))
			}
		case sim.MapThemeEmeraldLake:
			border = 45
			ground = rgb(184, 193, 159)
		case sim.MapThemeTwinCities:
			if b.Pos.X < 0 {
				ground = rgb(83, 93, 101)
			} else {
				ground = rgb(58, 84, 87)
			}
		}
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W+border*2, b.H+border*2, ground)
		if cam.Zoom > 0.22 {
			for x := -b.W/2 - border; x < b.W/2+border; x += 55 {
				a := b.Pos.Add(sim.Vec2{X: x, Y: -b.H/2 - border})
				worldLine(screen, cam, a, a.Add(sim.Vec2{Y: b.H + border*2}), 1, tint(ground, p.paving, 0.23))
			}
		}
	}
	// Broad planted beds give tree groups a landscape context instead of dots
	// floating on asphalt-colored emptiness.
	for _, tree := range w.Trees {
		if !cam.IsPointVisible(tree.Pos, tree.Radius*2) {
			continue
		}
		worldCircle(screen, cam, tree.Pos, tree.Radius*1.65, tint(p.ground, p.foliage, 0.23))
	}
}

func worldLine(screen *ebiten.Image, cam *sim.Camera, a, b sim.Vec2, width float64, c color.RGBA) {
	s, t := cam.WorldToScreen(a), cam.WorldToScreen(b)
	// Shared primitives keep the same camera transformation at every zoom/rotation.
	strokeWorldLine(screen, s, t, width*cam.Zoom, c)
}

func drawPavingJoints(screen *ebiten.Image, cam *sim.Camera, center, forward sim.Vec2, length, roadWidth float64, p scenePalette) {
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	position := cam.Pos.Sub(center).Dot(forward)
	span := math.Hypot(float64(cam.ScreenWidth), float64(cam.ScreenHeight)) / (2 * cam.Zoom)
	start := math.Max(-length/2, math.Floor((position-span)/64)*64)
	end := math.Min(length/2, position+span)
	for d := start; d < end; d += 64 {
		for _, side := range []float64{-1, 1} {
			a := center.Add(forward.Mul(d)).Add(right.Mul(side * (roadWidth/2 + 3)))
			worldLine(screen, cam, a, a.Add(right.Mul(side*(sim.ShoulderWidth-5))), 1.1, tint(p.paving, p.asphalt, .30))
		}
	}
}
