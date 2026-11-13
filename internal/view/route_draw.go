package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// DrawRouteLine draws connecting route lines along the driving lanes on the side of the road.
// In Driver Mode (!isPlanMode), it strictly draws ONLY the route to the next target stop, eliminating road confusion.
// In Planning Mode (isPlanMode), it draws the full planned transit line with the upcoming segment highlighted.
func DrawRouteLine(r *sim.Route, screen *ebiten.Image, cam *sim.Camera, world *sim.World, isPlanMode bool, laneWidth ...float64) {
	if len(r.StopIDs) < 1 {
		return
	}

	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	if !isPlanMode {
		// DRIVER MODE: Only draw the suggested route leading to the next stop!
		targetStopID := r.GetCurrentTargetStopID()
		if targetStopID < 0 || targetStopID >= len(world.Stops) {
			return
		}

		prevIdx := (r.CurrentIndex - 1 + len(r.StopIDs)) % len(r.StopIDs)
		prevStopID := r.StopIDs[prevIdx]

		s1 := world.Stops[prevStopID]
		s2 := world.Stops[targetStopID]

		var path []sim.Vec2
		if s1.ID != s2.ID {
			path = sim.GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		} else if len(r.StopIDs) > 1 {
			path = []sim.Vec2{s1.Pos, s2.Pos}
		}

		if len(path) < 2 {
			return
		}

		lineColor := sim.RGBA(250, 204, 21, 240) // Glowing Gold for active navigation line
		drawSegmentPath(screen, cam, path, lineColor)
		return
	}

	// PLANNING MODE: Draw all planned segments so player can review full line
	for i := 0; i < len(r.StopIDs); i++ {
		currID := r.StopIDs[i]
		nextID := r.StopIDs[(i+1)%len(r.StopIDs)]

		s1 := world.Stops[currID]
		s2 := world.Stops[nextID]
		if s1.ID == s2.ID {
			continue
		}

		path := sim.GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		lineColor := sim.RGBA(r.Color.R, r.Color.G, r.Color.B, 130)
		if i == (r.CurrentIndex-1+len(r.StopIDs))%len(r.StopIDs) || (r.CurrentIndex == 0 && i == len(r.StopIDs)-1) {
			lineColor = sim.RGBA(250, 204, 21, 240) // Highlight upcoming segment in gold
		}
		drawSegmentPath(screen, cam, path, lineColor)
	}
}

func drawSegmentPath(screen *ebiten.Image, cam *sim.Camera, path []sim.Vec2, lineColor color.RGBA) {
	strokeWidth := float32(math.Max(2.5, 5.0*cam.Zoom))
	for sIdx := 0; sIdx < len(path)-1; sIdx++ {
		w1 := path[sIdx]
		w2 := path[sIdx+1]

		minX := min(w1.X, w2.X)
		maxX := max(w1.X, w2.X)
		minY := min(w1.Y, w2.Y)
		maxY := max(w1.Y, w2.Y)
		if !cam.IsBoxVisible(minX, minY, maxX, maxY) {
			continue
		}

		p1 := cam.WorldToScreen(w1)
		p2 := cam.WorldToScreen(w2)

		// Route line along the road
		strokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y),
			strokeWidth, lineColor, true)

		// Draw directional arrows along segments that are sufficiently long on screen
		segLen := w1.Distance(w2)
		if segLen*cam.Zoom > 22 {
			mid := p1.Add(p2).Mul(0.5)
			dir := p2.Sub(p1).Normalize()
			perp := sim.Vec2{X: -dir.Y, Y: dir.X}
			arrLen := math.Max(8.0, 12*cam.Zoom)
			arrWid := math.Max(4.5, 7*cam.Zoom)
			arr1 := mid.Sub(dir.Mul(arrLen)).Add(perp.Mul(arrWid))
			arr2 := mid.Sub(dir.Mul(arrLen)).Sub(perp.Mul(arrWid))
			arrLw := float32(math.Max(1.8, 3.0*cam.Zoom))
			strokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr1.X), float32(arr1.Y),
				arrLw, lineColor, true)
			strokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr2.X), float32(arr2.Y),
				arrLw, lineColor, true)
		}
	}
}
