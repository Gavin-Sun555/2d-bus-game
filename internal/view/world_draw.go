package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func DrawWorld(w *sim.World, screen *ebiten.Image, cam *sim.Camera, targetStopID int, laneWidth float64, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	drawTerrain(screen, cam, w)
	drawLandscape(screen, cam, w)
	for _, riv := range w.Rivers {
		covered := false
		for _, water := range w.Waters {
			if math.Abs(water.Center.Y-riv.Y) < 1 && water.Height >= riv.Height {
				covered = true
				break
			}
		}
		if !covered {
			drawRiver(screen, cam, riv, w.Theme)
		}
	}
	drawWaterBodies(screen, cam, w)
	drawScenicFoundations(screen, cam, w)

	// 3. Draw Road Network, Soft Shoulders & Hard Crash Barriers
	drawIntersectionPads(screen, cam, w, laneWidth, planMode)
	for _, r := range w.Roads {
		drawRoad(screen, cam, w, r, laneWidth, planMode)
	}
	drawIntersectionAsphalt(screen, cam, w, laneWidth)

	// 4. Draw Roundabouts (Rotary plazas and central monuments)
	drawRoundabouts(screen, cam, w, laneWidth)

	// 5. Draw Bridges (Railings, cable stays, stone arches, overpass viaducts)
	drawBridges(screen, cam, w, laneWidth)

	// 6. Draw Intersections & Crosswalks (skip in plan mode or when zoomed out)
	if !planMode && cam.Zoom >= 0.35 {
		drawCrosswalks(screen, cam, w, laneWidth)
	}

	// 7. Draw Bus Stops
	for _, s := range w.Stops {
		DrawStop(s, screen, cam, s.ID == targetStopID, false, planMode)
	}

	drawStreetScenery(screen, cam, w, planMode, laneWidth)

	// 8. Draw Buildings & Shadows
	for _, b := range w.Buildings {
		drawBuilding(screen, cam, b, planMode, w.Theme)
	}

	// 9. Draw Trees
	for _, t := range w.Trees {
		drawTree(screen, cam, t, planMode, w.Theme)
	}
}
