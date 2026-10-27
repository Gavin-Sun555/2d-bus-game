package game

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestPlayerAutopilotPublishesRequiredMergeYield(t *testing.T) {
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 1, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated})
	g.Bus.SetModel(sim.BusArticulated)
	g.Bus.Pos = sim.Vec2{X: -7929.76, Y: -793.68}
	g.Bus.Heading = math.Pi / 2
	g.Bus.TrailerHeading = math.Pi / 2
	g.Bus.TrailerInitialized = true
	g.Bus.Speed = 0
	leader := g.AIBusMgr.Buses[0].Bus
	leader.Pos = sim.Vec2{X: -7904.46, Y: -563.17}
	leader.Heading = math.Pi / 2
	leader.TrailerHeading = 1.604
	leader.TrailerInitialized = true
	leader.Speed = 0
	leader.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{X: -7982}, Heading: math.Pi / 2, Remaining: .3, RightTurn: true}
	if _, ok := sim.BusMergeYieldLimit(g.Bus, leader, 52, false); !ok {
		t.Fatal("fixture does not leave a safe rear slot")
	}
	g.autopilotControls(1./60, 52)
	if g.Bus.YieldingToBus != leader {
		t.Fatal("player did not publish the safe slot to the merging AI bus")
	}
	g.manualControls()
	if g.Bus.YieldingToBus != nil {
		t.Fatal("manual driving retained an automatic yield guarantee")
	}
}
