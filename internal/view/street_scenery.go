package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type streetScenery struct {
	world     *sim.World
	laneWidth float64
	buildings []sim.Building
	trees     []sim.Tree
}

// Keep at most one derived scene per theme. Replacing/reloading a city releases
// the old cache; scenery never writes to World or consumes its random stream.
var streetScenes = map[sim.MapTheme]streetScenery{}

func sceneryFor(w *sim.World, laneWidth float64) streetScenery {
	if cache, ok := streetScenes[w.Theme]; ok && cache.world == w && cache.laneWidth == laneWidth {
		return cache
	}
	cache := streetScenery{world: w, laneWidth: laneWidth}
	if w.Theme == sim.MapThemeCustom {
		return cache
	}
	spacing := 650.0
	switch w.Theme {
	case sim.MapThemeMetropolis:
		spacing = 480
	case sim.MapThemeRiverValley:
		spacing = 880
	case sim.MapThemeTwinCities:
		spacing = 620
	}
	p := sceneColors(w.Theme)
	for roadIndex, road := range w.Roads {
		length := road.P2.Sub(road.P1).Length()
		if length < 400 {
			continue
		}
		forward := road.P2.Sub(road.P1).Normalize()
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		for d := 240.0; d < length-180; d += spacing {
			for _, side := range []float64{-1, 1} {
				seed := float64((roadIndex*29 + int(d/spacing)*17) % 11)
				width, height := 180+seed*8, 110+seed*4
				center := road.P1.Add(forward.Mul(d)).Add(right.Mul(side * (road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + height/2 + 50)))
				if math.Abs(forward.Y) > .5 {
					width, height = height, width
				}
				b := sim.Building{Pos: center, W: width, H: height, Color: p.paving, Roof: p.accent}
				if scenicLotClear(w, b, laneWidth, cache.buildings) {
					cache.buildings = append(cache.buildings, b)
				}
			}
		}
		for d := 150.0; d < length-100; d += 170 {
			for _, side := range []float64{-1, 1} {
				pos := road.P1.Add(forward.Mul(d)).Add(right.Mul(side * (road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + 45)))
				tree := sim.Building{Pos: pos, W: 35, H: 35}
				if scenicLotClear(w, tree, laneWidth, nil) {
					cache.trees = append(cache.trees, sim.Tree{Pos: pos, Radius: 13, Color: p.foliage})
				}
			}
		}
	}
	streetScenes[w.Theme] = cache
	return cache
}

func scenicLotClear(w *sim.World, b sim.Building, laneWidth float64, nearby []sim.Building) bool {
	for _, center := range w.Intersections {
		if w.IsAtRoundabout(center) {
			continue
		}
		box := w.JunctionAt(center, laneWidth)
		if math.Abs(b.Pos.X-center.X) < box.HalfX+b.W/2+45 && math.Abs(b.Pos.Y-center.Y) < box.HalfY+b.H/2+45 {
			return false
		}
	}
	for _, road := range w.Roads {
		padding := road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + 8
		if b.Pos.X+b.W/2 > math.Min(road.P1.X, road.P2.X)-padding && b.Pos.X-b.W/2 < math.Max(road.P1.X, road.P2.X)+padding &&
			b.Pos.Y+b.H/2 > math.Min(road.P1.Y, road.P2.Y)-padding && b.Pos.Y-b.H/2 < math.Max(road.P1.Y, road.P2.Y)+padding {
			return false
		}
	}
	for _, stop := range w.Stops {
		if math.Abs(b.Pos.X-stop.Pos.X) < b.W/2+130 && math.Abs(b.Pos.Y-stop.Pos.Y) < b.H/2+130 {
			return false
		}
	}
	for _, water := range w.Waters {
		if math.Abs(b.Pos.X-water.Center.X) < (b.W+water.Width)/2+20 && math.Abs(b.Pos.Y-water.Center.Y) < (b.H+water.Height)/2+20 {
			return false
		}
	}
	for _, river := range w.Rivers {
		if math.Abs(b.Pos.Y-river.Y) < (b.H+river.Height)/2+30 {
			return false
		}
	}
	for _, rb := range w.Roundabouts {
		if b.Pos.Distance(rb.Center) < rb.Radius+math.Hypot(b.W, b.H)/2+30 {
			return false
		}
	}
	for _, group := range [][]sim.Building{w.Buildings, nearby} {
		for _, other := range group {
			if math.Abs(b.Pos.X-other.Pos.X) < (b.W+other.W)/2+35 && math.Abs(b.Pos.Y-other.Pos.Y) < (b.H+other.H)/2+35 {
				return false
			}
		}
	}
	return true
}

func drawStreetScenery(screen *ebiten.Image, cam *sim.Camera, w *sim.World, plan bool, laneWidth float64) {
	cache := sceneryFor(w, laneWidth)
	for _, b := range cache.buildings {
		drawLotGarden(screen, cam, b, w.Theme, plan)
		drawBuilding(screen, cam, b, plan, w.Theme)
	}
	for _, t := range cache.trees {
		drawTree(screen, cam, t, plan, w.Theme)
	}
}
