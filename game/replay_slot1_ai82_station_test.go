package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedSlot1AI82ReachesJingmingPark(t *testing.T) {
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", os.Getenv("BUS_SLOT1_AI82_SAVE"))
	g := loadSlot1LeftExitReplay(t)
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 82 {
			continue
		}
		const stopID = 211
		bus := a.Bus
		if a.Route.GetCurrentTargetStopID() != stopID {
			t.Fatal("AI82 is not approaching Jingming Park in this save")
		}
		a.Driver.BeforeRecovery = func() { t.Fatal("AI82 used position recovery") }
		g.AIBusMgr.BeforeRecovery = func(event sim.TrafficRecoveryEvent) {
			if event.ID == a.ID {
				t.Fatal("AI82 reset")
			}
		}
		health := bus.Durability
		docked := false
		for tick := 0; tick < 45*60; tick++ {
			before := bus.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.1 || bus.Durability < health {
				t.Fatalf("AI82 repositioned or collided at tick %d, %v -> %v, speed %.2f, durability %.2f -> %.2f", tick, before, bus.Pos, bus.Speed, health, bus.Durability)
			}
			if !docked && a.Driver.State == autopilot.AutopilotStateDocked && a.Driver.DockedStopID == stopID && bus.DoorsOpen {
				if !g.World.Stops[stopID].ContainsBus(bus) {
					t.Fatal("AI82 opened its doors outside the bay")
				}
				t.Logf("AI82 docked in %.2fs at %v", float64(tick+1)/60, bus.Pos)
				docked = true
			}
			if a.Route.GetCurrentTargetStopID() != stopID {
				if !docked {
					t.Fatal("AI82 skipped Jingming Park")
				}
				stop := g.World.Stops[stopID]
				if !bus.DoorsOpen && bus.Pos.Sub(stop.Pos).Dot(sim.UnitFromAngle(stop.Heading)) > 300 {
					t.Logf("AI82 departed normally in %.2fs", float64(tick+1)/60)
					return
				}
			}
		}
		t.Fatalf("AI82 remained at %v, heading %.3f, speed %.1f", bus.Pos, bus.Heading, bus.Speed)
	}
	t.Fatal("AI82 missing")
}
