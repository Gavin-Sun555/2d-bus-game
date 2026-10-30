package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestSavedSidecarSceneResumesStationService(t *testing.T) {
	path := os.Getenv("BUS_SIDECAR_SAVE")
	if path == "" {
		t.Skip("copied local save required")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprintf("seed%d", seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := NewGame(1280, 720)
			g.savePathOverride = path
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			sim.SeedSimulation(seed)
			start, target, health := g.Bus.Pos, g.Route.GetCurrentTargetStopID(), g.Bus.Durability
			var follower *sim.Bus
			var followRoute *sim.Route
			for _, a := range g.AIBusMgr.Buses {
				if a.ID == 4 {
					follower, followRoute = a.Bus, a.Route
				}
			}
			if follower == nil {
				t.Fatal("missing saved follower")
			}
			followStart, followTarget, followHealth := follower.Pos, followRoute.GetCurrentTargetStopID(), follower.Durability
			playerMoved, followerMoved := -1., -1.
			boarded, left := false, false
			carExists := func(id int) bool {
				for _, c := range g.TrafficMgr.Cars {
					if c.ID == id {
						return true
					}
				}
				return false
			}
			for tick := 0; tick < 9000; tick++ {
				before := g.Bus.Pos
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				elapsed := float64(tick+1) / 60
				if g.Bus.Pos.Distance(before) > g.Bus.MaxSpeedFwd/60+2 {
					t.Fatal("player relocated")
				}
				if g.Bus.Durability < health {
					t.Fatalf("recovery contact %.2fs", elapsed)
				}
				if follower.Pos.Distance(followStart) < 900 && follower.Durability < followHealth {
					t.Fatalf("follower contact %.2fs", elapsed)
				}
				if playerMoved < 0 && g.Bus.Pos.Distance(start) > 20 {
					playerMoved = elapsed
					if !carExists(41913) {
						t.Fatal("player recovery relied on deleting original sidecar")
					}
				}
				if followerMoved < 0 && follower.Pos.Distance(followStart) > 20 {
					followerMoved = elapsed
					if !carExists(41727) {
						t.Fatal("follower recovery relied on deleting original sidecar")
					}
				}
				if g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
					boarded = true
				}
				if boarded && !g.Bus.DoorsOpen && g.Route.GetCurrentTargetStopID() != target && g.Bus.Pos.Distance(g.World.Stops[target].Pos) > 200 {
					left = true
				}
				if left && followRoute.GetCurrentTargetStopID() != followTarget {
					if playerMoved < 0 || playerMoved > 5 || followerMoved < 0 || followerMoved > 8 {
						t.Fatalf("slow/unresolved original lock player%.2f follower%.2f", playerMoved, followerMoved)
					}
					if g.lastSkippedStopID == target {
						t.Fatal("target skipped instead of served")
					}
					t.Logf("player moved %.2fs AI4 moved %.2fs; player completed original service/departure, AI4 began service by %.2fs; health %.0f", playerMoved, followerMoved, elapsed, g.Bus.Durability)
					return
				}
			}
			t.Fatalf("service did not resume: moved%.2f/%.2f boarded%v left%v player%v AI4%v", playerMoved, followerMoved, boarded, left, g.Bus.Pos, follower.Pos)
		})
	}
}
