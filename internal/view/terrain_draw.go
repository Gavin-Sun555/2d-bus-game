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
			wave := math.Sin(float64(x)*0.049+math.Cos(float64(y)*0.037)*1.8) + math.Cos(float64(y)*0.049-math.Sin(float64(x)*0.037)*1.8)
			if wave > 0.35 {
				c = tint(c, p.grain, 0.28)
			} else if wave < -0.45 {
				c = tint(c, p.foliage, 0.22)
			}
			if ((x+y)/18)%2 == 0 {
				c = tint(c, p.grain, 0.08)
			}
			if hash%53 < 2 {
				c = tint(c, p.leafLight, 0.24)
			} else if hash%71 == 0 {
				c = tint(c, p.foliage, 0.26)
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
	if w.Theme != sim.MapThemeCustom {
		// Crisp roadside granite sidewalks and manicured green boulevard verges
		// frame each street while leaving the city blocks lushly landscaped.
		const lw = 52.0
		vergeCol := tint(p.ground, p.foliage, 0.45)
		for _, r := range w.Roads {
			dir := r.P2.Sub(r.P1)
			length := dir.Length()
			if length < 1 {
				continue
			}
			center := r.P1.Add(r.P2).Mul(0.5)
			if !cam.IsPointVisible(center, length/2+120) {
				continue
			}
			u := dir.Mul(1 / length)
			roadW := r.GetWidth(lw) + sim.ShoulderWidth*2
			drawRotatedBox(screen, cam, center, u.X, u.Y, length, roadW+52, vergeCol)
			drawRotatedBox(screen, cam, center, u.X, u.Y, length, roadW+34, p.paving)
		}
		for _, center := range w.Intersections {
			if w.IsAtRoundabout(center) {
				continue
			}
			box := w.JunctionAt(center, lw)
			if !cam.IsPointVisible(center, math.Hypot(box.HalfX, box.HalfY)+60) {
				continue
			}
			drawRotatedBox(screen, cam, center, 1, 0, box.HalfX*2+52, box.HalfY*2+52, vergeCol)
			drawRotatedBox(screen, cam, center, 1, 0, box.HalfX*2+34, box.HalfY*2+34, p.paving)
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
