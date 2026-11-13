package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
)

// Migrate only a never-driven, empty initial player bus pushed out of its
// station by old queue placement. Normal in-service saves keep their poses.
func (g *Game) repairInvalidInitialPlayerSpawn() {
	if g.Bus == nil || g.Route == nil || g.AutopilotDriver == nil || g.Operations == nil || g.Route.CurrentIndex != 0 || g.Route.TotalTrips != 0 || g.Bus.PassengerCount != 0 || math.Abs(g.Bus.Speed) > .1 {
		return
	}
	op := g.Operations.Vehicles[0]
	if op == nil || op.KM != 0 || op.CompletedLaps != 0 || op.Revenue != 0 {
		return
	}
	snapshot := g.AutopilotDriver.Snapshot()
	target := g.Route.GetCurrentTargetStopID()
	if snapshot.PathFromID != target || snapshot.PathToID != target || snapshot.Public.LastPathS > .01 || snapshot.Public.LastLateralErr <= g.Bus.OverallLength() {
		return
	}
	lw := g.Settings.GetLaneWidth()
	if traffic.SafeInitialBusPosition(g.Bus, g.World, lw) {
		return
	}
	start := g.World.Stops[g.Route.StopIDs[0]]
	g.queuePlayerAtStop(start)
	if !traffic.SafeInitialBusPosition(g.Bus, g.World, lw) {
		return
	}
	g.Operations.InterruptJourney(0)
	g.Camera.Pos, g.Camera.TargetPos = g.Bus.Pos, g.Bus.Pos
}
