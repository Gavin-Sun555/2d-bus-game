package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCurrentSlot1AI54DocksInsideBay(t *testing.T) {
	path := os.Getenv("BUS_AI54_SAVE")
	if path == "" {
		t.Skip("set BUS_AI54_SAVE")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	a := fleetBus(t, g, 54)
	bus := a.Bus
	health := bus.Durability
	a.Driver.BeforeRecovery = func() { t.Fatal("AI54 relocated") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == 54 {
			t.Fatal("AI54 reset", e)
		}
	}
	docked := map[int]bool{}
	for tick := 0; tick < 60*180; tick++ {
		before, open := bus.Pos, bus.DoorsOpen
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.1 || bus.Durability < health {
			t.Fatal("AI54 moved discontinuously or was damaged")
		}
		if open && bus.Pos.Distance(before) > .01 {
			t.Fatal("AI54 moved with doors open")
		}
		if a.Driver.State == autopilot.AutopilotStateDocked && bus.DoorsOpen && a.Driver.DockedStopID != 192 {
			id := a.Driver.DockedStopID
			if !docked[id] {
				stop := g.World.Stops[id]
				u := sim.UnitFromAngle(stop.Heading)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				extent := 0.
				for _, part := range bus.BodyParts() {
					for _, p := range sim.BodyCorners(part) {
						extent = math.Max(extent, math.Abs(p.Sub(stop.Pos).Dot(r)))
					}
				}
				t.Logf("AI54 docked stop%d at %.2fs; lateral extent %.3f / %.3f", id, float64(tick)/60, extent, stop.Width/2)
				if extent > stop.Width/2+1 {
					t.Fatal("AI54 obstructed travel lane at next arrival")
				}
				docked[id] = true
				if len(docked) == 2 {
					return
				}
			}
		}
	}
	t.Fatalf("AI54 did not complete two new bay arrivals: pos%v target%d served%v", bus.Pos, a.Route.GetCurrentTargetStopID(), docked)
}
