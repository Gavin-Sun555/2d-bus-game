package game

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

func (g *Game) resetOperations() {
	if g.World == nil || g.Route == nil || g.Settings == nil {
		return
	}
	g.careerRevenueMark = 0
	g.Operations = sim.NewTransitOperations(g.World, g.Route, g.Settings.GetLaneWidth(), g.Settings.Fare)
	g.greenWaitTimers = nil
	g.greenWaitReports = nil
	if g.OperationsPanel != nil {
		g.OperationsPanel.SelectedVehicle = -1
		g.OperationsPanel.Scroll = 0
	}
	g.operationsRoute = append([]int(nil), g.operationsStops()...)
	g.operationsLaneWidth = g.Settings.GetLaneWidth()
	if g.AIBusMgr != nil {
		g.AIBusMgr.Operations = g.Operations
	}
}
func (g *Game) operationsStops() []int {
	if g.IsPlanMode && g.planningRoute() == g.Route && g.planModeInitialStops != nil {
		return g.planModeInitialStops
	}
	return g.Route.StopIDs
}

func (g *Game) ensureOperations() {
	if g.Operations == nil || !intSlicesEqual(g.operationsRoute, g.operationsStops()) || g.operationsLaneWidth != g.Settings.GetLaneWidth() {
		g.resetOperations()
	}
	if g.Operations != nil {
		g.Operations.Policy = g.Settings.Fare.Normalize()
		g.Operations.Distances = g.Operations.DistancesFor(g.Route)
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

func (g *Game) refreshOperationsPanel() {
	g.ensureOperations()
	g.syncPlayerSpeedLimit()
	g.trackOperations(0)
}
