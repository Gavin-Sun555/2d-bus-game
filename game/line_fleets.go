package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) initializeLineFleets() {
	for i := range g.LineFleets {
		g.LineFleets[i] = traffic.LineFleetSettings{SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}}
	}
	g.LineFleets[0] = traffic.LineFleetSettings{BusCount: g.Settings.AIBusCount, SpeedRange: sim.SpeedRange{MinKPH: g.Settings.AIBusMinSpeed, MaxKPH: g.Settings.AIBusMaxSpeed}}.Normalize()
}
func (g *Game) buildLineFleets(reset bool) {
	if reset {
		g.LineFleets[g.PlayerLine] = g.currentLineFleetSettings()
	}
	plans := make([]traffic.LineFleetPlan, 0, g.LineCount)
	for i, r := range g.activeLines() {
		plans = append(plans, traffic.LineFleetPlan{Route: r, Settings: g.LineFleets[i]})
	}
	g.AIBusMgr.ConfigureLineFleets(plans, g.World, g.Settings.GetLaneWidth(), g.Bus, reset)
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
func (g *Game) SetLineFleet(index int, c traffic.LineFleetSettings) {
	if index < 0 || index >= len(g.LineFleets) {
		return
	}
	g.LineFleets[index] = c.Normalize()
	if c.PlayerModel == sim.BusArticulated || c.Counts()[sim.BusArticulated] > 0 {
		sim.EnsureLongBusBays(g.World, g.Lines[index])
	}
	if index == g.PlayerLine {
		g.showPlayerLineSettings()
		if g.Bus != nil {
			g.Bus.RequestModel(g.LineFleets[index].PlayerModel)
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
