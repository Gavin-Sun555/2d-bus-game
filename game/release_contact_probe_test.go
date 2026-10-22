package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestReleaseContactProbe(t *testing.T) {
	if os.Getenv("BUSGAME_CONTACT_PROBE") != "1" {
		t.Skip("contact investigation")
	}
	for _, c := range []struct {
		theme      sim.MapTheme
		seed       int64
		mixed      bool
		start, end int
	}{{2, 502, false, 6040, 6150}, {0, 91, true, 6380, 6472}, {3, 94, true, 6660, 6890}} {
		t.Run(fmt.Sprint(c.theme), func(t *testing.T) {
			sim.SeedSimulation(c.seed)
			g := NewGame(1280, 720)
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			if !c.mixed {
				g.Settings.AutopilotBerserkMaxSpeed = 120
			}
			g.Settings.TrafficDensity = 4
			g.Settings.AIBusCount = 8
			if c.mixed {
				g.Settings.AICarMaxSpeed = 100
				g.Settings.AIBusMaxSpeed = 80
			}
			g.EnterCity(c.theme)
			for tick := 0; tick <= c.end; tick++ {
				if tick >= c.start && tick%10 == 0 {
					t.Logf("tick%d bus=%v h=%.3f v=%.1f lane=%.1f targetlane=%.1f targetv=%.1f state=%v wp=%d", tick, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset, g.AutopilotDriver.LastTargetSpeed, g.AutopilotDriver.State, g.AutopilotDriver.WaypointIdx)
					for _, car := range g.TrafficMgr.Cars {
						if car.Pos.Distance(g.Bus.Pos) < 140 {
							t.Logf(" car%d p=%v h=%.3f v=%.1f turn=%+v", car.ID, car.Pos, car.Heading, car.Speed, car.JunctionTurn)
						}
					}
				}
				health := g.Bus.Durability
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < health {
					t.Logf("HIT tick%d health%.0f", tick, g.Bus.Durability)
				}
			}
		})
	}
}
