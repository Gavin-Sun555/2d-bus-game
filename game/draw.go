package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func (g *Game) Draw(screen *ebiten.Image) {
	if g.State == StateMainMenu {
		g.drawMainMenu(screen)
		return
	}

	targetStopID := g.Route.GetCurrentTargetStopID()
	laneWidth := g.Settings.GetLaneWidth()

	// 1. Draw World (Roads, Buildings, Trees, Rivers, Bus Stops)
	view.DrawWorld(g.World, screen, g.Camera, targetStopID, laneWidth, g.IsPlanMode)

	// 2. Draw Traffic Lights and Stop Lines (Only in Driver Mode to eliminate lag and declutter map)
	if !g.IsPlanMode {
		view.DrawTrafficLights(g.TrafficLights, screen, g.Camera, laneWidth)
	}

	// 3. Draw Active Route lines on map (offset into driving lanes)
	view.DrawRouteLine(g.Route, screen, g.Camera, g.World, g.IsPlanMode, laneWidth)

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

	// 6. Draw Navigation Guide Arrow towards target stop in Driver Mode
	if !g.IsPlanMode && targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		targetStop := g.World.Stops[targetStopID]
		view.DrawNavArrow(g.Route, screen, g.Camera, g.Bus, targetStop)
	}

	// 7. Draw HUD Overlay
	var currentStop *sim.BusStop
	for _, s := range g.World.Stops {
		if s.ContainsBus(g.Bus) {
			currentStop = s
			break
		}
	}
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		g.HUD.TargetStopOccupied = g.IsStopOccupiedByOtherVehicle(g.World.Stops[targetStopID])
	} else {
		g.HUD.TargetStopOccupied = false
	}
	g.HUD.Draw(screen, g.Bus, g.Route, g.World, g.IsPlanMode, g.Cash, currentStop, g.Settings, g.stopBellRung, g.passengersAlighting, g.Camera, g.TrafficLights)

	// 8. Draw Settings Modal (on top of everything)
	g.Settings.Draw(screen, g.Camera)
}
