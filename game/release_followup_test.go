package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"testing"
)

func logNearbyVehicles(t *testing.T, g *Game) {
	t.Helper()
	for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
		if c.Pos.Distance(g.Bus.Pos) < 350 {
			t.Logf("near car%d pos%v h%.4f speed%.2f turn%+v corridor%+v", c.ID, c.Pos, c.Heading, c.Speed, c.JunctionTurn, c.JunctionCorridor)
		}
	}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.Bus.Pos.Distance(g.Bus.Pos) < 350 {
			t.Logf("near AI%d pos%v h%.4f speed%.2f", ab.ID, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Speed)
		}
	}
}

func logStoppedFleet(t *testing.T, g *Game) {
	t.Helper()
	for _, ab := range g.AIBusMgr.Buses {
		if ab.Bus.Speed > 1 {
			continue
		}
		obs := g.AIBusMgr.GetObstacles(ab.ID)
		obs = append(obs, autopilot.ObserveBus(g.Bus))
		for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
		}
		diag := ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
		diag.Path = nil
		if len(diag.Blockers) > 5 {
			diag.Blockers = diag.Blockers[:5]
		}
		t.Logf("AI%d boundary%+v pos%v speed%.2f target%d diag%+v", ab.ID, ab.Bus.CheckBoundaries(g.World, g.Settings.GetLaneWidth()), ab.Bus.Pos, ab.Bus.Speed, ab.Route.GetCurrentTargetStopID(), diag)
	}
}
