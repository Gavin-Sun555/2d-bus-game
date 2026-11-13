package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func (g *Game) Draw(screen *ebiten.Image) {
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
	view.DrawWorld(g.World, screen, g.Camera, targetStopID, laneWidth, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawIntersectionStopLines(g.TrafficLights, screen, g.Camera, laneWidth)
	}

	if g.IsPlanMode && !g.HideOtherRoutes {
		for _, r := range g.activeLines() {
			if r != displayRoute {
				view.DrawRouteLine(r, screen, g.Camera, g.World, true, laneWidth)
			}
		}
	}
	// 3. Draw Active Route lines on map (offset into driving lanes)
	view.DrawRouteLine(displayRoute, screen, g.Camera, g.World, g.IsPlanMode, laneWidth)

	// 4. Draw AI Traffic Vehicles (Hidden during Planning Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		view.DrawTraffic(g.TrafficMgr, screen, g.Camera, g.Settings.TrafficDensity)
	}

	// 4.5 Draw Route AI Buses (Hidden during Planning Mode)
	if !g.IsPlanMode && g.AIBusMgr != nil {
		view.DrawFleet(g.AIBusMgr, screen, g.Camera)
	}

	// 5. Draw Bus
	view.DrawBus(g.Bus, screen, g.Camera)

	view.DrawDayNightLighting(screen, g.Camera, g.World, laneWidth, g.Bus, g.TrafficMgr, g.AIBusMgr, g.Settings.TrafficDensity, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawVehicleTurnSignals(screen, g.Camera, g.Bus, g.TrafficMgr, g.AIBusMgr, g.Settings.TrafficDensity)
	}
	view.DrawBRTOverpasses(g.World, screen, g.Camera, g.IsPlanMode)
	view.DrawElevatedStationCrossings(g.World, g.TrafficLights, screen, g.Camera, laneWidth, g.IsPlanMode)
	view.DrawRailGates(g.World, g.TrafficLights, screen, g.Camera, g.IsPlanMode)
	if !g.IsPlanMode {
		view.DrawTrafficLights(g.TrafficLights, screen, g.Camera, laneWidth, g.World.Railways())
	}
	view.DrawRailTraffic(g.World, g.TrafficLights, screen, g.Camera, g.IsPlanMode)

	// 7. Draw HUD Overlay
	currentStop, _ := g.activeDockedStop()
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		g.HUD.TargetStopOccupied = g.IsStopOccupiedByOtherVehicle(g.World.Stops[targetStopID])
	} else {
		g.HUD.TargetStopOccupied = false
	}
	g.HUD.Draw(screen, g.Bus, displayRoute, g.World, g.IsPlanMode, g.Cash, currentStop, g.Settings, g.stopBellRung, g.passengersAlighting, g.Camera, g.TrafficLights)

	g.drawClockHUD(screen)
	g.drawNetworkPanel(screen)
	// 8. Draw Settings Modal (on top of everything)
	g.Settings.Draw(screen, g.Camera)
	if g.OperationsPanel != nil {
		g.OperationsPanel.Draw(screen, g.Settings.Fare, g.Operations, g.World, g.activeLines()...)
	}
	g.drawCareerBadge(screen)
	g.drawCareerOffice(screen)
	g.drawSavePicker(screen)
	g.drawStopNameEditor(screen)
	g.drawStationSearch(screen)
}
