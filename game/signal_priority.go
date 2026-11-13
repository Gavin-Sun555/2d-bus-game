package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) updateBusSignalPriority(dt float64) {
	var crossings []sim.StationCrossing
	lw := g.Settings.GetLaneWidth()
	g.TrafficLights.ClearUTurnOccupants()
	if g.World != nil {
		crossings = g.World.StationCrossings()
		if center, ok := g.AutopilotDriver.UTurnClearance(g.Bus, g.World, lw); ok {
			g.TrafficLights.ObserveUTurnOccupant(center)
		}
		if g.AIBusMgr != nil {
			for _, ab := range g.AIBusMgr.Buses {
				if center, ok := ab.Driver.UTurnClearance(ab.Bus, g.World, lw); ok {
					g.TrafficLights.ObserveUTurnOccupant(center)
				}
			}
		}
	}
	if !g.Settings.BusSignalPriority {
		if g.TrafficLights.BusPriorityEnabled {
			g.TrafficLights.ObserveBusPriority(false, nil, g.Settings.GetLaneWidth(), crossings)
		}
		return
	}
	if !g.Settings.AutopilotEnabled && g.Route != nil && len(g.Route.StopIDs) > 0 {
		idx := g.Route.CurrentIndex
		if idx >= 0 && idx < len(g.Route.StopIDs) {
			from := g.Route.StopIDs[(idx-1+len(g.Route.StopIDs))%len(g.Route.StopIDs)]
			to := g.Route.StopIDs[idx]
			if from >= 0 && to >= 0 && from < len(g.World.Stops) && to < len(g.World.Stops) {
				g.AutopilotDriver.PrepareLeg(g.World, g.World.Stops[from], g.World.Stops[to], lw)
			}
		}
	}
	var storage [1 + traffic.MaxFleetBuses]sim.BusSignalObservation
	observations := storage[:0]
	// Manual driving uses the planned route's movement, just as the bus's
	// navigation display does; all dispatched buses receive equal priority.
	observations = append(observations, sim.BusSignalObservation{BusID: 0, Bus: g.Bus, Request: g.AutopilotDriver.SignalPriorityRequest(g.Bus, g.TrafficLights, lw)})
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			observations = append(observations, sim.BusSignalObservation{BusID: ab.ID, Bus: ab.Bus, Request: ab.Driver.SignalPriorityRequest(ab.Bus, g.TrafficLights, lw)})
		}
	}
	if g.TrafficMgr != nil && g.World != nil {
		density := g.Settings.TrafficDensity
		if g.TrafficLights.RefreshCarSignalDemand(dt, density) {
			g.TrafficMgr.ObserveSignalDemand(g.TrafficLights, g.World, lw, density)
		}
		g.TrafficLights.BeginSignalDemand()
	}
	g.TrafficLights.ObserveBusPriority(true, observations, lw, crossings)
}
