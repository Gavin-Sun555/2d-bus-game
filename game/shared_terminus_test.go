package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestSharedTerminusQueuesPlayerBehindExistingFleet(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		g := NewGame(1280, 720)
		g.EnterCity(sim.MapThemeMetropolis)
		g.SetTransitLineCount(2)
		st := g.World.Stops[234]
		st.Heading = h
		for i := 0; i < 2; i++ {
			g.Lines[i].Clear()
			for _, id := range []int{234, 236, 234} {
				g.Lines[i].AddStop(id)
			}
			g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 3, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
		}
		// Replanning and changing player service both used to reset the player
		// directly into an AI bus, letting collision separation push it sideways.
		g.PlanTransitLine(0)
		g.planModeRouteModified = true
		g.TogglePlanMode()
		checkStationQueue(t, g, st)
		u := sim.Vec2{X: math.Cos(st.Heading), Y: math.Sin(st.Heading)}
		if g.Bus.Pos.Sub(st.Pos).Dot(u) >= -120 {
			t.Fatal("replanning inserted player into an occupied terminus")
		}
		if !g.SelectPlayerLine(1) {
			t.Fatal("cannot select shared terminus")
		}
		g.PlanTransitLine(1)
		g.planModeRouteModified = true
		g.TogglePlanMode()
		checkStationQueue(t, g, st)
		g.ResetToStartingStation()
		checkStationQueue(t, g, st)
	}
}
func checkStationQueue(t *testing.T, g *Game, st *sim.BusStop) {
	t.Helper()
	u := sim.Vec2{X: math.Cos(st.Heading), Y: math.Sin(st.Heading)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	buses := []*sim.Bus{g.Bus}
	for _, ab := range g.AIBusMgr.Buses {
		buses = append(buses, ab.Bus)
	}
	for i, b := range buses {
		delta := b.Pos.Sub(st.Pos)
		if math.Abs(delta.Dot(r)) > .001 {
			t.Fatalf("bus%d dispatched beside queue: %v", i, b.Pos)
		}
		for j := i + 1; j < len(buses); j++ {
			if math.Abs(b.Pos.Sub(buses[j].Pos).Dot(u)) < (b.Length+buses[j].Length)/2+23.9 {
				t.Fatalf("buses%d/%d do not have a shared queue slot", i, j)
			}
		}
	}
}
func TestSharedTerminusWaitsThenDepartsInOrder(t *testing.T) {
	sim.SeedSimulation(712)
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(2)
	for _, line := range g.Lines[:2] {
		line.Clear()
		for _, id := range []int{234, 236, 234} {
			line.AddStop(id)
		}
	}
	for i := 0; i < 2; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
	}
	g.ResetBusesToRouteStart()
	g.TrafficMgr.Cars = nil
	st := g.World.Stops[234]
	u := sim.Vec2{X: math.Cos(st.Heading), Y: math.Sin(st.Heading)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for step := 0; step < 300; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		checkStationQueue(t, g, st)
	}
	g.Settings.AutopilotEnabled = true
	dispatched := map[int]bool{}
	for step := 0; step < 2400; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Route.CurrentIndex > 0 {
				dispatched[ab.ID] = true
				continue
			}
			if math.Abs(ab.Bus.Pos.Sub(st.Pos).Dot(r)) > 1 {
				t.Fatalf("bus%d pulled alongside waiting queue at step%d", ab.ID, step)
			}
		}
		if len(dispatched) == 4 {
			break
		}
	}
	if len(dispatched) != 4 {
		for _, ab := range g.AIBusMgr.Buses {
			t.Logf("bus%d pos%v target%d state%v speed%.1f", ab.ID, ab.Bus.Pos, ab.Route.CurrentIndex, ab.Driver.State, ab.Bus.Speed)
		}
		t.Fatalf("only %d/4 buses departed shared queue", len(dispatched))
	}
}

func TestLoadRepairsOldSideBySideTerminus(t *testing.T) {
	g := saveTestGame(t)
	g.SetTransitLineCount(2)
	for _, line := range g.Lines[:2] {
		line.Clear()
		for _, id := range []int{234, 236, 234} {
			line.AddStop(id)
		}
	}
	for i := 0; i < 2; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 50}})
	}
	g.ResetBusesToRouteStart()
	st := g.World.Stops[234]
	r := sim.Vec2{X: -math.Sin(st.Heading), Y: math.Cos(st.Heading)}
	blocked := g.AIBusMgr.Buses[2]
	blocked.Bus.Pos = st.Pos.Sub(r.Mul(40))
	blocked.Bus.BoardPassengers(3, []int{236})
	g.Cash = 241
	g.Operations.Revenue = 77
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	checkStationQueue(t, g, st)
	if g.Cash != 241 || g.Operations.Revenue != 77 || g.AIBusMgr.Buses[2].Bus.PassengerCount != 3 {
		t.Fatal("repair lost passengers or revenue")
	}
}
