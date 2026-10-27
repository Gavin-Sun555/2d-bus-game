package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"testing"
)

func TestMultiLineDispatchAndPlayerSelection(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.AIBusCount = 8
	g.SetTransitLineCount(6)
	for i := 0; i < 6; i++ {
		count := 1
		if i == 0 {
			count = 3
		}
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: count, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
	}
	active, retiring := 0, 0
	for _, b := range g.AIBusMgr.Buses {
		if b.Retiring {
			retiring++
		} else {
			active++
		}
	}
	if active != 8 || retiring != 5 || len(g.AIBusMgr.Buses) != 13 {
		t.Fatal("fleet targets or pending retirements changed", active, retiring)
	}
	serviced := map[string]bool{}
	for _, b := range g.AIBusMgr.Buses {
		serviced[b.LineName] = true
		if b.Route == g.Route {
			t.Fatal("shared runtime cursor")
		}
		if !intSlicesEqual(b.Route.StopIDs, g.Lines[b.LineIndex].StopIDs) {
			t.Fatal("wrong itinerary")
		}
	}
	if len(serviced) != 6 {
		t.Fatal("not all six lines dispatched")
	}
	saved := g.AIBusMgr.Buses[0]
	pos := saved.Bus.Pos
	saved.Bus.PassengerCount = 3
	g.Operations.Revenue = 42
	g.Operations.Vehicle(0).Revenue = 12
	if !g.SelectPlayerLine(3) || g.Route != g.Lines[3] {
		t.Fatal("player line not selected")
	}
	if g.AIBusMgr.Buses[0] != saved || saved.Bus.Pos != pos || saved.Bus.PassengerCount != 3 {
		t.Fatal("selection reset another vehicle")
	}
	if g.Operations.Vehicle(0).Revenue != 12 {
		t.Fatal("selection lost player lifetime revenue")
	}
	if g.Operations.Revenue != 42 {
		t.Fatal("selection lost revenue")
	}
	g.World.Update(1, g.activeLines()...)
	for _, s := range g.World.Stops {
		served := false
		for _, r := range g.activeLines() {
			served = served || r.ContainsStop(s.ID)
		}
		if served && s.WaitingPassengers < 1 {
			t.Fatalf("served station %d lost passengers", s.ID)
		}
	}
	g.SetTransitLineCount(2)
	if g.PlayerLine >= 2 || len(g.activeLines()) != 2 {
		t.Fatal("invalid selection after reducing line count")
	}
	g.SetTransitLineCount(100)
	if g.LineCount != 6 {
		t.Fatal("upper bound")
	}
	g.SetTransitLineCount(0)
	if g.LineCount != 1 {
		t.Fatal("lower bound")
	}
}
func TestEditingSelectedLineLeavesOtherItineraries(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeRiverValley)
	g.SetTransitLineCount(4)
	old := append([]int(nil), g.Lines[0].StopIDs...)
	g.SelectPlayerLine(2)
	g.ReplanRoute([]int{12, 13, 14, 12})
	if !intSlicesEqual(g.Lines[0].StopIDs, old) || !intSlicesEqual(g.Lines[2].StopIDs, []int{12, 13, 14, 12}) {
		t.Fatal("editing changed the wrong line")
	}
}

func TestMultiLineFleetDepartsEachExpandedCity(t *testing.T) {
	for _, theme := range sim.BuiltinMapThemes() {
		t.Run(theme.ShortName(), func(t *testing.T) {
			sim.SeedSimulation(77)
			g := NewGame(1280, 720)
			g.EnterCity(theme)
			g.Settings.AIBusCount = 8
			g.SetTransitLineCount(6)
			for i := 0; i < 6; i++ {
				count := 1
				if i == 0 {
					count = 3
				}
				g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: count, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
			}
			start := map[int]sim.Vec2{}
			for _, b := range g.AIBusMgr.Buses {
				start[b.ID] = b.Bus.Pos
			}
			for step := 0; step < 1800; step++ {
				g.TrafficLights.Update(1. / 60)
				g.AIBusMgr.Update(1./60, nil, g.World, g.TrafficLights, nil, 52, 1)
			}
			for _, b := range g.AIBusMgr.Buses {
				if b.Bus.Pos.Distance(start[b.ID]) < 100 {
					t.Errorf("%s bus%d did not leave terminus: %v", b.LineName, b.ID, b.Bus.Pos)
				}
			}
		})
	}
}

func TestPlanNonPlayerLinePreservesOtherServices(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(3)
	for i := 0; i < 3; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 1, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
	}
	playerRoute, pos := g.Route, g.Bus.Pos
	g.Bus.PassengerCount = 5
	unrelated := g.AIBusMgr.Buses[0]
	unrelated.Bus.PassengerCount = 4
	unrelatedPos := unrelated.Bus.Pos
	if !g.PlanTransitLine(1) || g.Route != playerRoute || g.PlayerLine != 0 {
		t.Fatal("planning changed player service")
	}
	g.ClearPlannedRoute()
	if len(g.Lines[1].StopIDs) != 0 || len(g.Route.StopIDs) == 0 {
		t.Fatal("clear edited wrong service")
	}
	g.TogglePlanMode()
	if !g.IsPlanMode || g.PlanTransitLine(2) {
		t.Fatal("empty route should block finishing or switching editors")
	}
	for _, id := range []int{12, 13, 14, 12} {
		g.planningRoute().AddStop(id)
	}
	g.UndoLastPlannedStop()
	g.planningRoute().AddStop(12)
	g.TogglePlanMode()
	if g.IsPlanMode || g.Route != playerRoute || g.PlayerLine != 0 || g.Bus.Pos != pos || g.Bus.PassengerCount != 5 {
		t.Fatal("editing another service disturbed player bus")
	}
	if g.AIBusMgr.Buses[0] != unrelated || unrelated.Bus.Pos != unrelatedPos || unrelated.Bus.PassengerCount != 4 {
		t.Fatal("editing another service reset unrelated fleet")
	}
	for _, b := range g.AIBusMgr.Buses {
		if b.LineIndex == 1 && !intSlicesEqual(b.Route.StopIDs, []int{12, 13, 14, 12}) {
			t.Fatal("edited service did not receive its new route")
		}
	}
	if !g.PlanTransitLine(1) || !g.PlanTransitLine(2) || g.planningRoute() != g.Lines[2] {
		t.Fatal("cannot switch planning service")
	}
	g.SetTransitLineCount(1)
	if g.planningRoute() != g.Route {
		t.Fatal("editor retained an inactive service")
	}
}
