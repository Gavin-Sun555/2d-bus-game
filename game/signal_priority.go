package game

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

func (g *Game) updateBusSignalPriority() {
	if !g.Settings.BusSignalPriority {
		if g.TrafficLights.BusPriorityEnabled {
			g.TrafficLights.ObserveBusPriority(false, nil, g.Settings.GetLaneWidth())
		}
		return
	}
	lw := g.Settings.GetLaneWidth()
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
	observations := make([]sim.BusSignalObservation, 0, 49)
	// Manual driving uses the planned route's movement, just as the bus's
	// navigation display does; all dispatched buses receive equal priority.
	observations = append(observations, sim.BusSignalObservation{BusID: 0, Bus: g.Bus, Request: g.AutopilotDriver.SignalPriorityRequest(g.Bus, g.TrafficLights, lw)})
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			observations = append(observations, sim.BusSignalObservation{BusID: ab.ID, Bus: ab.Bus, Request: ab.Driver.SignalPriorityRequest(ab.Bus, g.TrafficLights, lw)})
		}
	}
	g.TrafficLights.ObserveBusPriority(true, observations, lw)
}
