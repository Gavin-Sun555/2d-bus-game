package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func DrawWorld(w *sim.World, screen *ebiten.Image, cam *sim.Camera, targetStopID int, laneWidth float64, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]
	night := !planMode && w.Clock.IsNight()

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
	drawMetropolisCanalCorridor(screen, cam, w, laneWidth)
	drawCoastalPromenadeCorridor(screen, cam, w, laneWidth, planMode)
	drawRiverValleyEmbankments(screen, cam, w, laneWidth, planMode)
	drawScenicFoundations(screen, cam, w)
	for _, rail := range w.Railways() {
		drawRailGround(rail, screen, cam, planMode)
	}
	drawTwinCitiesAirportCorridor(screen, cam, w, laneWidth, planMode)

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
	drawMetropolisCanalOverpasses(screen, cam, w, laneWidth, planMode)
	for _, rail := range w.Railways() {
		drawRailLevelCrossings(rail, screen, cam, planMode)
		drawRailViaductGround(rail, screen, cam, planMode)
	}

	// 6. Draw Intersections & Crosswalks (skip in plan mode or when zoomed out)
	if !planMode && cam.Zoom >= 0.35 {
		drawCrosswalks(screen, cam, w, laneWidth)
	}
	drawStationCrossingGround(w, screen, cam, laneWidth)

	drawStreetScenery(screen, cam, w, planMode, night, laneWidth)

	// 7. Draw Bus Stops
	hover := -1
	if planMode {
		mx, my := ebiten.CursorPosition()
		hover = w.StopAtScreen(cam, sim.Vec2{X: float64(mx), Y: float64(my)}, 12)
	}
	for _, s := range w.Stops {
		DrawStopGeometry(s, screen, cam, s.ID == targetStopID, s.ID == hover, planMode)
	}
	for _, s := range w.Stops {
		DrawStopLabel(s, screen, cam, s.ID == targetStopID, s.ID == hover, planMode)
	}

	// 8. Draw Buildings & Shadows
	for _, b := range w.Buildings {
		drawBuilding(screen, cam, b, planMode, night, w.Theme)
	}

	// 9. Draw Trees
	for _, t := range w.Trees {
		drawTree(screen, cam, t, planMode, w.Theme)
	}
	drawStreetLampFixtures(screen, cam, w, laneWidth)
}
