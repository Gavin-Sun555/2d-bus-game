package game

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

func (g *Game) resetOperations() {
	if g.World == nil || g.Route == nil || g.Settings == nil {
		return
	}
	g.Operations = sim.NewTransitOperations(g.World, g.Route, g.Settings.GetLaneWidth(), g.Settings.Fare)
	g.greenWaitTimers = nil
	g.greenWaitReports = nil
	if g.OperationsPanel != nil {
		g.OperationsPanel.SelectedVehicle = -1
		g.OperationsPanel.Scroll = 0
	}
	g.operationsRoute = append([]int(nil), g.Route.StopIDs...)
	g.operationsLaneWidth = g.Settings.GetLaneWidth()
	if g.AIBusMgr != nil {
		g.AIBusMgr.Operations = g.Operations
	}
}
func (g *Game) ensureOperations() {
	if g.Operations == nil || !intSlicesEqual(g.operationsRoute, g.Route.StopIDs) || g.operationsLaneWidth != g.Settings.GetLaneWidth() {
		g.resetOperations()
	}
	if g.Operations != nil {
		g.Operations.Policy = g.Settings.Fare.Normalize()
	}
	if g.AIBusMgr != nil {
		g.AIBusMgr.Operations = g.Operations
	}
}
func (g *Game) trackOperations(dt float64) {
	g.Operations.Track(0, g.Bus, g.Route, dt)
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			g.Operations.Track(ab.ID, ab.Bus, ab.Route, dt)
		}
	}
}
