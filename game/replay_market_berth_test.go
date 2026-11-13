package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedMarketBusBoardsAndDeparts(t *testing.T) {
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", os.Getenv("BUS_MARKET_BERTH_SAVE"))
	g := loadSlot1LeftExitReplay(t)
	const stopID = 186
	stop := g.World.Stops[stopID]
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 16 {
			continue
		}
		if a.Route.GetCurrentTargetStopID() != stopID {
			t.Fatal("AI16 is not approaching Chengming Market")
		}
		a.Driver.BeforeRecovery = func() { t.Fatal("AI16 used position recovery") }
		g.AIBusMgr.BeforeRecovery = func(event sim.TrafficRecoveryEvent) {
			if event.ID == a.ID {
				t.Fatal("AI16 reset")
			}
		}
		bus := a.Bus
		health := bus.Durability
		boarded := false
		for tick := 0; tick < 45*60; tick++ {
			before := bus.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.1 || bus.Durability < health {
				t.Fatalf("AI16 repositioned or collided at tick %d", tick)
			}
			if !boarded && a.Driver.State == autopilot.AutopilotStateDocked && a.Driver.DockedStopID == stopID && bus.DoorsOpen {
				if !stop.ContainsBus(bus) {
					t.Fatal("AI16 opened doors outside the platform")
				}
				boarded = true
			}
			if a.Route.GetCurrentTargetStopID() != stopID {
				if !boarded {
					t.Fatal("AI16 skipped the stop")
				}
				if !bus.DoorsOpen && bus.Pos.Sub(stop.Pos).Dot(sim.UnitFromAngle(stop.Heading)) > 300 {
					t.Logf("AI16 boarded and cleared in %.2fs", float64(tick+1)/60)
					return
				}
			}
		}
		t.Fatalf("AI16 remained blocked at %v", bus.Pos)
	}
	t.Fatal("AI16 missing")
}
