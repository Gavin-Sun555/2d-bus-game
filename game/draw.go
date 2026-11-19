package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func (g *Game) Draw(screen *ebiten.Image) {
	g.drawWithVehicles(screen, g.Camera, g.Bus, g.TrafficMgr, g.AIBusMgr)
}

func (g *Game) drawWithVehicles(screen *ebiten.Image, camera *sim.Camera, bus *sim.Bus, cars *traffic.TrafficManager, fleet *traffic.AIBusManager) {
	if g.State == StateModeSelect {
		g.drawModeSelection(screen)
		return
	}
	if g.State == StateMainMenu {
		g.drawMainMenu(screen)
		g.drawCareerOffice(screen)
		g.drawSavePicker(screen)
		return
	}

	displayRoute := g.planningRoute()
	targetStopID := displayRoute.GetCurrentTargetStopID()
	laneWidth := g.Settings.GetLaneWidth()

	// 1. Draw World (Roads, Buildings, Trees, Rivers, Bus Stops)
	view.DrawWorld(g.World, screen, camera, targetStopID, laneWidth, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawIntersectionStopLines(g.TrafficLights, screen, camera, laneWidth)
	}

	if g.IsPlanMode && !g.HideOtherRoutes {
		for _, r := range g.activeLines() {
			if r != displayRoute {
				view.DrawRouteLine(r, screen, camera, g.World, true, laneWidth)
			}
		}
	}
	// 3. Draw Active Route lines on map (offset into driving lanes)
	view.DrawRouteLine(displayRoute, screen, camera, g.World, g.IsPlanMode, laneWidth)

	// 4. Draw AI Traffic Vehicles (Hidden during Planning Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		view.DrawTraffic(cars, screen, camera, g.Settings.TrafficDensity)
	}

	// 4.5 Draw Route AI Buses (Hidden during Planning Mode)
	if !g.IsPlanMode && g.AIBusMgr != nil {
		view.DrawFleet(fleet, screen, camera)
	}

	// 5. Draw Bus
	view.DrawBus(bus, screen, camera)

	view.DrawDayNightLighting(screen, camera, g.World, laneWidth, bus, cars, fleet, g.Settings.TrafficDensity, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawVehicleTurnSignals(screen, camera, bus, cars, fleet, g.Settings.TrafficDensity)
	}
	view.DrawBRTOverpasses(g.World, screen, camera, g.IsPlanMode)
	view.DrawElevatedStationCrossings(g.World, g.TrafficLights, screen, camera, laneWidth, g.IsPlanMode)
	view.DrawRailGates(g.World, g.TrafficLights, screen, camera, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawTrafficLights(g.TrafficLights, screen, camera, laneWidth, g.World.Railways())
	}
	view.DrawRailTraffic(g.World, g.TrafficLights, screen, camera, g.IsPlanMode)

	// 7. Draw HUD Overlay
	currentStop, _ := g.activeDockedStop()
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		g.HUD.TargetStopOccupied = g.IsStopOccupiedByOtherVehicle(g.World.Stops[targetStopID])
	} else {
		g.HUD.TargetStopOccupied = false
	}
	g.HUD.Draw(screen, bus, displayRoute, g.World, g.IsPlanMode, g.Cash, currentStop, g.Settings, g.stopBellRung, g.passengersAlighting, camera, g.TrafficLights)

	g.drawClockHUD(screen)
	g.drawNetworkPanel(screen)
	// 8. Draw Settings Modal (on top of everything)
	g.Settings.Draw(screen, camera)
	if g.OperationsPanel != nil {
		g.OperationsPanel.Draw(screen, g.Settings.Fare, g.Operations, g.World, g.activeLines()...)
	}
	g.drawCareerBadge(screen)
	g.drawCareerOffice(screen)
	g.drawSavePicker(screen)
	g.drawStopNameEditor(screen)
	g.drawStationSearch(screen)
}
