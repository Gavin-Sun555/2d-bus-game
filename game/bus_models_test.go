package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"testing"
)

func TestLineModelsPersistAcrossSettingsResetAndSave(t *testing.T) {
	g := saveTestGame(t)
	g.SetTransitLineCount(2)
	c := traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{1, 2, 3}, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}}.Normalize()
	g.SetLineFleet(0, c)
	if g.Bus.Model != sim.BusStandard || g.Bus.PendingModel == nil || *g.Bus.PendingModel != sim.BusArticulated {
		t.Fatal("player did not receive line model")
	}
	g.Bus.SetModel(sim.BusArticulated) // Initial test vehicle, before simulating an in-service change.
	g.Settings.AIBusMaxSpeed = 70
	g.syncCurrentLineSettings()
	c.SpeedRange.MaxKPH = 70
	if g.LineFleets[0] != c {
		t.Fatal("speed edit lost mix")
	}
	g.ResetBusesToRouteStart()
	if g.LineFleets[0] != c || g.Bus.Model != sim.BusArticulated {
		t.Fatal("route reset lost models")
	}
	g.Bus.BoardPassengers(70, []int{g.Route.StopIDs[1]})
	c.PlayerModel = sim.BusStandard
	g.SetLineFleet(0, c)
	if g.Bus.PendingModel == nil || g.Bus.PassengerCount != 70 {
		t.Fatal("player downgrade removed riders")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.LineFleets[0] != c || g.Bus.Model != sim.BusArticulated || g.Bus.PendingModel == nil || *g.Bus.PendingModel != sim.BusStandard || g.Bus.PassengerCount != 70 {
		t.Fatal("save lost models, pending change or passengers")
	}
	counts := [sim.BusModelCount]int{}
	for _, b := range g.AIBusMgr.Buses {
		if b.LineIndex == 0 {
			counts[b.Bus.Model]++
		}
	}
	if counts != c.ModelCounts {
		t.Fatal("save lost fleet mix", counts)
	}
	g.SetLineFleet(1, traffic.LineFleetSettings{PlayerModel: sim.BusDoubleDecker, Model: sim.BusDoubleDecker, BusCount: 2, SpeedRange: c.SpeedRange})
	if !g.SelectPlayerLine(1) || g.Bus.Model != sim.BusDoubleDecker || g.Bus.Capacity != 80 {
		t.Fatal("select line lost player model")
	}
}

func TestNetworkModelButtonsRespectSharedLimit(t *testing.T) {
	g := saveTestGame(t)
	g.NetworkSelected = 0
	for _, action := range []int{18, 18, 20, 22, 22} {
		g.handleNetworkAction(action)
	}
	c := g.LineFleets[0]
	if c.Counts() != [sim.BusModelCount]int{2, 1, 2} || len(g.AIBusMgr.Buses) != 5 {
		t.Fatal("model count buttons", c)
	}
	for i := 0; i < 20; i++ {
		g.handleNetworkAction(22)
	}
	if g.LineFleets[0].BusCount != 8 {
		t.Fatal("mixed limit")
	}
	g.handleNetworkAction(23)
	if g.LineFleets[0].MixedModels || g.LineFleets[0].Counts() != [sim.BusModelCount]int{0, 8, 0} {
		t.Fatal("uniform selection")
	}
	g.handleNetworkAction(24)
	if g.Bus.Model != sim.BusStandard || g.Bus.PendingModel == nil || *g.Bus.PendingModel != sim.BusDoubleDecker {
		t.Fatal("player selection")
	}
}

func TestLoadedFleetDowngradeKeepsRidersAndCanBeCancelled(t *testing.T) {
	g := saveTestGame(t)
	c := traffic.LineFleetSettings{BusCount: 1, Model: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	g.SetLineFleet(0, c)
	b := g.AIBusMgr.Buses[0]
	b.Bus.BoardPassengers(75, []int{g.Route.StopIDs[1]})
	pos := b.Bus.Pos
	c.Model = sim.BusStandard
	g.SetLineFleet(0, c)
	if g.AIBusMgr.Buses[0] != b || b.Bus.Pos != pos || b.Bus.Model != sim.BusArticulated || b.Bus.PendingModel == nil || b.Bus.PassengerCount != 75 {
		t.Fatal("occupied fleet downgrade reset bus")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	b = g.AIBusMgr.Buses[0]
	if b.Bus.Model != sim.BusArticulated || b.Bus.PendingModel == nil || *b.Bus.PendingModel != sim.BusStandard || b.Bus.PassengerCount != 75 || b.Bus.Pos != pos {
		t.Fatal("load lost deferred fleet replacement")
	}
	c.Model = sim.BusDoubleDecker
	g.SetLineFleet(0, c)
	if b.Bus.Model != sim.BusArticulated || b.Bus.PendingModel == nil || *b.Bus.PendingModel != sim.BusDoubleDecker || b.Bus.PassengerCount != 75 {
		t.Fatal("changing pending choice lost passengers")
	}
	c.Model = sim.BusArticulated
	g.SetLineFleet(0, c)
	if b.Bus.PendingModel != nil || b.Bus.PassengerCount != 75 {
		t.Fatal("cannot cancel pending replacement")
	}
}

func TestPlayerFinishesLapBeforeTerminalVehicleHandover(t *testing.T) {
	g := saveTestGame(t)
	g.Route.StopIDs = []int{0, 2, 10, 0}
	g.Route.CurrentIndex = 1
	c := g.LineFleets[0]
	c.PlayerModel = sim.BusDoubleDecker
	g.SetLineFleet(0, c)
	g.Bus.Durability = 62
	g.Bus.BoardPassengers(45, []int{2})
	serve := func(id int, waiting int) {
		st := g.World.Stops[id]
		st.WaitingPassengers = waiting
		g.Bus.Pos = st.Pos
		g.Bus.Heading = st.Heading
		g.Bus.Speed = 0
		g.Bus.DoorsOpen = true
		g.Bus.DoorProgress = 1
		g.boardedThisStop = false
		g.updatePassengerExchange(false)
	}
	serve(2, 0)
	if g.Bus.PassengerCount != 0 || g.Bus.Model != sim.BusStandard || g.Bus.PendingModel == nil || g.Bus.Durability != 62 {
		t.Fatal("empty intermediate stop replaced bus")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Model != sim.BusStandard || g.Bus.PendingModel == nil {
		t.Fatal("load applied pending replacement")
	}
	serve(10, 0)
	if g.Bus.Model != sim.BusStandard {
		t.Fatal("changed before returning to terminal")
	}
	serve(0, 100)
	if g.Bus.Model != sim.BusDoubleDecker || g.Bus.PendingModel != nil || g.Bus.Durability != 100 || g.Bus.PassengerCount != 80 || g.Route.TotalTrips != 1 || g.World.Stops[0].WaitingPassengers != 20 {
		t.Fatalf("terminal exchange model%d riders%d trips%d", g.Bus.Model, g.Bus.PassengerCount, g.Route.TotalTrips)
	}
}

func TestPendingFleetRetirementSurvivesSave(t *testing.T) {
	g := saveTestGame(t)
	c := traffic.LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	g.SetLineFleet(0, c)
	b := g.AIBusMgr.Buses[0]
	b.Bus.BoardPassengers(20, []int{g.Route.StopIDs[1]})
	g.SetLineFleet(0, c.ChangeModelCount(sim.BusStandard, -1))
	if len(g.AIBusMgr.Buses) != 1 || !b.Retiring || b.Bus.PassengerCount != 20 {
		t.Fatal("retired immediately")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.LineFleets[0].BusCount != 0 || len(g.AIBusMgr.Buses) != 1 || !g.AIBusMgr.Buses[0].Retiring || g.AIBusMgr.Buses[0].Bus.PassengerCount != 20 {
		t.Fatal("save lost unfinished lap retirement")
	}
}

func TestArticulatedBentPoseAndLegacyGeometrySave(t *testing.T) {
	for _, legacy := range []bool{false, true} {
		t.Run(map[bool]string{false: "bent", true: "legacy"}[legacy], func(t *testing.T) {
			g := saveTestGame(t)
			g.Bus.SetModel(sim.BusArticulated)
			g.Bus.TrailerHeading = g.Bus.Heading - .3
			if legacy {
				g.Bus.GeometryVersion = 0
				g.Bus.TrailerInitialized = false
			}
			pos, h, tail := g.Bus.Pos, g.Bus.Heading, g.Bus.TrailerHeading
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			if g.Bus.Pos != pos || g.Bus.Heading != h || g.Bus.TrailerHeading != tail {
				t.Fatal("load changed articulated pose")
			}
			if legacy {
				if g.Bus.IsArticulated() || g.Bus.PendingModel == nil || *g.Bus.PendingModel != sim.BusArticulated {
					t.Fatal("old short bus grew in place")
				}
			} else if !g.Bus.IsArticulated() || !g.Bus.TrailerInitialized {
				t.Fatal("lost articulated geometry")
			}
		})
	}
}

// Exercise the actual city update loop, including fleet/traffic interactions and
// boundary damage, rather than just controller fixtures.
func TestArticulatedCityTrafficSoak(t *testing.T) {
	sim.SeedSimulation(20261023)
	g := saveTestGame(t)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	start := g.Bus.Pos
	starts := map[int]sim.Vec2{}
	for _, ab := range g.AIBusMgr.Buses {
		starts[ab.ID] = ab.Bus.Pos
	}
	for tick := 0; tick < 7200; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < 100 {
			t.Fatalf("player damage tick%d health%.1f pos%v heading%.2f tail%.2f target%d toasts%+v", tick, g.Bus.Durability, g.Bus.Pos, g.Bus.Heading, g.Bus.TailHeading(), g.Route.GetCurrentTargetStopID(), g.HUD.Toasts)
		}
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus.Durability < 100 {
				t.Fatalf("AI%d damage tick%d health%.1f pos%v", ab.ID, tick, ab.Bus.Durability, ab.Bus.Pos)
			}
		}
	}
	if g.Bus.Pos.Distance(start) < 1000 {
		t.Fatal("long player bus did not progress", g.Bus.Pos)
	}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.Bus.Pos.Distance(starts[ab.ID]) < 500 {
			t.Fatalf("AI%d failed to progress pos%v", ab.ID, ab.Bus.Pos)
		}
	}
}
