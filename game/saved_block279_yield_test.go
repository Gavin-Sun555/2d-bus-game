package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"strconv"
	"testing"
)

// Replay only the copied save. Follow every originally blocked bus through real
// boarding and departure, including the later player bus and red-light queues.
func TestSavedBlock279Yield(t *testing.T) {
	path := os.Getenv("BUS_BLOCK279_SAVE")
	if path == "" {
		t.Skip("BUS_BLOCK279_SAVE not set")
	}
	seeds := []int64{5, 17, 29}
	if v := os.Getenv("BUS_BLOCK279_SEED"); v != "" {
		seed, err := strconv.ParseInt(v, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
		seeds = []int64{seed}
	}
	for _, seed := range seeds {
		sim.SeedSimulation(seed)
		g := NewGame(1280, 720)
		g.savePathOverride = path
		if err := g.LoadGame(); err != nil {
			t.Fatal(err)
		}
		sim.SeedSimulation(seed)
		ids := map[int]bool{1: true, 2: true, 6: true, 7: true}
		distance := map[int]float64{}
		doors := map[int]bool{}
		served := map[int]float64{}
		serviceDistance := map[int]float64{}
		health := map[int]float64{}
		for _, a := range g.AIBusMgr.Buses {
			if ids[a.ID] {
				health[a.ID] = a.Bus.Durability
				if a.Route.GetCurrentTargetStopID() != 278 {
					t.Fatal("wrong saved target")
				}
			}
		}
		playerHealth := g.Bus.Durability
		for tick := 0; tick < 10800; tick++ {
			positions := map[int]sim.Vec2{}
			for _, a := range g.AIBusMgr.Buses {
				if ids[a.ID] {
					positions[a.ID] = a.Bus.Pos
				}
			}
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			if tick == 1200 {
				if _, ok := served[1]; !ok {
					t.Fatal("lead AI1 did not serve within20s")
				}
				if _, ok := served[2]; !ok {
					t.Fatal("lead AI2 did not serve within20s")
				}
			}
			if g.Bus.Durability < playerHealth {
				if g.Bus.Pos.Distance(g.World.Stops[278].Pos) < 700 {
					t.Fatalf("player contact in original station zone seed%d at %.2fs pos%v", seed, float64(tick)/60, g.Bus.Pos)
				}
				t.Logf("post-recovery contact outside original station: seed%d %.2fs pos%v health%.1f", seed, float64(tick)/60, g.Bus.Pos, g.Bus.Durability)
				playerHealth = g.Bus.Durability
			}
			for _, a := range g.AIBusMgr.Buses {
				if !ids[a.ID] {
					continue
				}
				step := a.Bus.Pos.Distance(positions[a.ID])
				distance[a.ID] += step
				if step > a.Bus.MaxSpeedFwd/60+1 {
					t.Fatal("queue recovery teleported bus")
				}
				if a.Bus.Durability < health[a.ID] {
					if a.Bus.Pos.Distance(g.World.Stops[278].Pos) < 700 {
						t.Fatalf("AI%d contact in original station zone seed%d at%.2fs", a.ID, seed, float64(tick)/60)
					}
					t.Logf("post-recovery AI%d contact outside original station %.2fs pos%v", a.ID, float64(tick)/60, a.Bus.Pos)
					health[a.ID] = a.Bus.Durability
				}
				if a.Bus.DoorsOpen && a.Driver.DockedStopID == 278 {
					doors[a.ID] = true
				}
				if _, done := served[a.ID]; !done && a.Route.GetCurrentTargetStopID() != 278 {
					if !doors[a.ID] {
						t.Fatalf("AI%d skipped boarding", a.ID)
					}
					served[a.ID] = float64(tick) / 60
					serviceDistance[a.ID] = distance[a.ID]
				}
			}
		}
		for id := range ids {
			if _, ok := served[id]; !ok || distance[id] < 500 || distance[id]-serviceDistance[id] < 300 {
				t.Fatalf("seed%d AI%d stuck: served%v travel%.1f", seed, id, served, distance[id])
			}
		}
		t.Logf("seed%d served times%v travel%v, all original buses served and departed300px; original station zone undamaged through180s", seed, served, distance)
	}
}
