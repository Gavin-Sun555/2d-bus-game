package game

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) initializeLineFleets() {
	playerModel := sim.BusStandard
	if g.Settings != nil {
		playerModel = g.Settings.GetPlayerModel()
	}
	for i := range g.LineFleets {
		g.LineFleets[i] = traffic.LineFleetSettings{SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}}
	}
	g.LineFleets[0] = traffic.LineFleetSettings{
		BusCount:    g.Settings.AIBusCount,
		PlayerModel: playerModel,
		SpeedRange:  sim.SpeedRange{MinKPH: g.Settings.AIBusMinSpeed, MaxKPH: g.Settings.AIBusMaxSpeed},
	}.Normalize()
}
func (g *Game) buildLineFleets(reset bool) {
	if reset {
		g.LineFleets[g.PlayerLine] = g.currentLineFleetSettings()
	}
	plans := make([]traffic.LineFleetPlan, 0, g.LineCount)
	for i, r := range g.activeLines() {
		plans = append(plans, traffic.LineFleetPlan{Route: r, Settings: g.LineFleets[i]})
	}
	var cars []*sim.TrafficCar
	if g.TrafficMgr != nil {
		cars = g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	}
	g.AIBusMgr.ConfigureLineFleets(plans, g.World, g.Settings.GetLaneWidth(), g.Bus, reset, cars)
}
func (g *Game) showPlayerLineSettings() {
	c := g.LineFleets[g.PlayerLine].Normalize()
	g.Settings.AIBusCount = c.BusCount
	g.Settings.AIBusMinSpeed = c.SpeedRange.MinKPH
	g.Settings.AIBusMaxSpeed = c.SpeedRange.MaxKPH
}
func (g *Game) syncCurrentLineSettings() {
	c := g.currentLineFleetSettings()
	if g.LineFleets[g.PlayerLine] != c {
		g.LineFleets[g.PlayerLine] = c
		g.buildLineFleets(false)
	}
}
func (g *Game) isPlayerAtRouteStart() bool {
	if g == nil || g.Bus == nil || g.Route == nil || len(g.Route.StopIDs) == 0 {
		return true
	}
	// If the route has advanced beyond the first stop, the bus is en route.
	if g.Route.CurrentIndex != 0 {
		return false
	}
	// Multi-lap service has completed at least one full lap.
	if g.Operations != nil && g.Operations.Vehicle(0) != nil && g.Operations.Vehicle(0).CompletedLaps > 0 {
		return false
	}
	// Has accumulated meaningful travel distance (> 50m).
	if g.Operations != nil && g.Operations.Vehicle(0) != nil && g.Operations.Vehicle(0).KM > 0.05 {
		return false
	}
	// Check physical location relative to the first stop of the route.
	firstStopID := g.Route.StopIDs[0]
	if firstStopID >= 0 && firstStopID < len(g.World.Stops) {
		firstStop := g.World.Stops[firstStopID]
		if firstStop != nil {
			if firstStop.ContainsBus(g.Bus) || g.Bus.Pos.Distance(firstStop.FrontBerthPosition()) < 120 {
				return true
			}
		}
	}
	// If operations has not recorded distance and speed is near 0.
	if (g.Operations == nil || g.Operations.Vehicle(0) == nil || g.Operations.Vehicle(0).KM == 0) && math.Abs(g.Bus.Speed) < 1.0 {
		return true
	}
	return false
}

func (g *Game) SetLineFleet(index int, c traffic.LineFleetSettings) {
	if index < 0 || index >= len(g.LineFleets) {
		return
	}
	g.LineFleets[index] = c.Normalize()
	if c.PlayerModel == sim.BusArticulated || c.Counts()[sim.BusArticulated] > 0 {
		sim.EnsureLongBusBays(g.World, g.Lines[index])
	}
	if index == g.PlayerLine {
		if g.Settings != nil {
			g.Settings.PlayerModel = c.PlayerModel
		}
		g.showPlayerLineSettings()
		if g.Bus != nil {
			if g.isPlayerAtRouteStart() {
				if g.Bus.SetModel(g.LineFleets[index].PlayerModel) {
					g.Bus.PendingModel = nil
					if g.Bus.Model == sim.BusArticulated {
						sim.EnsureLongBusBays(g.World, g.Lines[index])
						g.Bus.ResetArticulation()
					}
					if g.Bus.PassengerServiceRemaining() > 0 && g.Settings != nil {
						g.Bus.SetStationServiceRates(g.Settings.ModelPassengerRates(g.Bus.Model))
					}
				} else {
					g.Bus.RequestModel(g.LineFleets[index].PlayerModel)
				}
			} else {
				g.Bus.RequestModel(g.LineFleets[index].PlayerModel)
			}
		}
	}
	g.buildLineFleets(false)
}

// Legacy scalar settings remain a compatibility view; speed edits keep the mix.
func (g *Game) currentLineFleetSettings() traffic.LineFleetSettings {
	c := g.LineFleets[g.PlayerLine].Normalize()
	if c.BusCount != g.Settings.AIBusCount {
		c.BusCount = g.Settings.AIBusCount
		c.MixedModels = false
	}
	c.SpeedRange = sim.SpeedRange{MinKPH: g.Settings.AIBusMinSpeed, MaxKPH: g.Settings.AIBusMaxSpeed}
	return c.Normalize()
}
