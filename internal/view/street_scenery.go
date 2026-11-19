package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type streetScenery struct {
	world               *sim.World
	firstRoad           *sim.RoadSegment
	metroLots           []metroLot
	lotChunks           []scenicLotChunk
	laneWidth           float64
	placement           scenicPlacement
	trees               []sim.Tree
	treeChunks          []scenicTreeChunk
	railways            []*sim.Railway
	hasCanalCorridor    bool
	canalQuayX          float64
	canalCenterY        float64
	canalJoinY          float64
	canalLot            metroLot
	hasAirportCorridor  bool
	airportX            float64
	airportHalfW        float64
	hasCoastalPromenade bool
}

// Keep at most one derived scene per theme. Replacing/reloading a city releases
// the old cache; scenery never writes to World or consumes its random stream.
var streetScenes = map[sim.MapTheme]streetScenery{}

func sceneryFor(w *sim.World, laneWidth float64) streetScenery {
	var firstRoad *sim.RoadSegment
	if len(w.Roads) > 0 {
		firstRoad = &w.Roads[0]
	}
	if cache, ok := streetScenes[w.Theme]; ok && cache.world == w && cache.firstRoad == firstRoad && cache.laneWidth == laneWidth {
		return cache
	}
	cache := streetScenery{world: w, firstRoad: firstRoad, laneWidth: laneWidth}
	if w.Theme == sim.MapThemeCustom {
		return cache
	}
	cache.placement = newScenicPlacement(w, laneWidth)
	cache.railways = w.Railways()
	spacing := 340.0
	cache.addMetropolisLandmarks(w, laneWidth)
	p := sceneColors(w.Theme)
	for roadIndex, road := range w.Roads {
		length := road.P2.Sub(road.P1).Length()
		if length < 400 {
			continue
		}
		forward := road.P2.Sub(road.P1).Normalize()
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		startD, endMargin := 185.0, 145.0
		for d := startD; d < length-endMargin; d += spacing {
			for _, side := range []float64{-1, 1} {
				cache.addMetropolisFrontage(w, road, forward, right, d, side, laneWidth, int(d/spacing))
			}
		}
		treeStep := 115.0
		treeStart := 110.0
		for d := treeStart; d < length-95; d += treeStep {
			for _, side := range []float64{-1, 1} {
				vergeDist := 28.0
				pos := road.P1.Add(forward.Mul(d)).Add(right.Mul(side * (road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + vergeDist)))
				tree := sim.Building{Pos: pos, W: 30, H: 30}
				if cache.corridorsClear(tree) && cache.placement.clear(w, tree, false) {
					rad := 13.0 + float64((roadIndex+int(d))%3)*1.2
					cache.trees = append(cache.trees, sim.Tree{Pos: pos, Radius: rad, Color: p.foliage})
				}
			}
		}
		// Populate open ground, corner setbacks, and block interiors where no building
		// fits with landscaped multi-tree pocket park groves so no area looks barren.
		groveOffsets := [...]sim.Vec2{
			{X: -14, Y: -10},
			{X: 15, Y: -7},
			{X: 3, Y: 14},
			{X: -18, Y: 12},
		}
		groveRadii := [...]float64{15.5, 13.0, 16.5, 11.5}
		for d := 85.0; d < length-75; d += 175.0 {
			for _, side := range []float64{-1, 1} {
				for _, depthOff := range []float64{110.0, 260.0, 440.0} {
					center := road.P1.Add(forward.Mul(d)).Add(right.Mul(side * (road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + depthOff)))
					probe := sim.Building{Pos: center, W: 64, H: 64}
					if !cache.corridorsClear(probe) || !cache.placement.clear(w, probe, false) {
						continue
					}
					variant := metropolisVariant(center)
					count := 3 + variant%2
					for k := 0; k < count; k++ {
						idx := (variant + k) % len(groveOffsets)
						tp := center.Add(groveOffsets[idx])
						cache.trees = append(cache.trees, sim.Tree{
							Pos:    tp,
							Radius: groveRadii[idx],
							Color:  p.foliage,
						})
					}
				}
			}
		}
	}
	cache.placement = nil // Only used while placing the scenery.
	cache.lotChunks = groupScenicLots(cache.metroLots)
	cache.treeChunks = groupScenicTrees(cache.trees)
	streetScenes[w.Theme] = cache
	return cache
}

func drawStreetScenery(screen *ebiten.Image, cam *sim.Camera, w *sim.World, plan, night bool, laneWidth float64) {
	cache := sceneryFor(w, laneWidth)
	for _, chunk := range cache.lotChunks {
		if !cam.IsBoxVisible(chunk.min.X, chunk.min.Y, chunk.max.X, chunk.max.Y) {
			continue
		}
		for _, lot := range chunk.lots {
			drawMetropolisLot(screen, cam, lot, plan, night)
		}
	}
	// In planning mode or city overview, individual foliage crowns are subpixel (< 1.8px).
	if plan || cam.Zoom < 0.12 {
		return
	}
	for _, chunk := range cache.treeChunks {
		if !cam.IsBoxVisible(chunk.min.X, chunk.min.Y, chunk.max.X, chunk.max.Y) {
			continue
		}
		for _, t := range chunk.trees {
			drawTree(screen, cam, t, plan, w.Theme)
		}
	}
}

type scenicLotChunk struct {
	min, max sim.Vec2
	lots     []metroLot
}

func groupScenicLots(lots []metroLot) []scenicLotChunk {
	const chunkSize = 32
	chunks := make([]scenicLotChunk, 0, (len(lots)+chunkSize-1)/chunkSize)
	for start := 0; start < len(lots); start += chunkSize {
		group := lots[start:min(start+chunkSize, len(lots))]
		first := group[0].footprint
		chunk := scenicLotChunk{
			min:  sim.Vec2{X: first.Pos.X - first.W/2 - 40, Y: first.Pos.Y - first.H/2 - 40},
			max:  sim.Vec2{X: first.Pos.X + first.W/2 + 40, Y: first.Pos.Y + first.H/2 + 40},
			lots: group,
		}
		for _, lot := range group {
			fp := lot.footprint
			chunk.min.X = min(chunk.min.X, fp.Pos.X-fp.W/2-40)
			chunk.min.Y = min(chunk.min.Y, fp.Pos.Y-fp.H/2-40)
			chunk.max.X = max(chunk.max.X, fp.Pos.X+fp.W/2+40)
			chunk.max.Y = max(chunk.max.Y, fp.Pos.Y+fp.H/2+40)
		}
		chunks = append(chunks, chunk)
	}
	return chunks
}

type scenicTreeChunk struct {
	min, max sim.Vec2
	trees    []sim.Tree
}

// Keep generation order so overlapping trees render in the same order at
// every zoom. Consecutive trees are normally on the same stretch of road.
func groupScenicTrees(trees []sim.Tree) []scenicTreeChunk {
	const chunkSize = 64
	chunks := make([]scenicTreeChunk, 0, (len(trees)+chunkSize-1)/chunkSize)
	for start := 0; start < len(trees); start += chunkSize {
		group := trees[start:min(start+chunkSize, len(trees))]
		chunk := scenicTreeChunk{min: group[0].Pos, max: group[0].Pos, trees: group}
		for _, tree := range group {
			margin := tree.Radius*2 + 16
			chunk.min.X = min(chunk.min.X, tree.Pos.X-margin)
			chunk.min.Y = min(chunk.min.Y, tree.Pos.Y-margin)
			chunk.max.X = max(chunk.max.X, tree.Pos.X+margin)
			chunk.max.Y = max(chunk.max.Y, tree.Pos.Y+margin)
		}
		chunks = append(chunks, chunk)
	}
	return chunks
}
