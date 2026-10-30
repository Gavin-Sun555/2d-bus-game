package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

// Optional end-to-end replay of the read-only October 28 slot-one copy.
func TestSavedParallelBusDeadlock(t *testing.T) {
	path := os.Getenv("BUS_PARALLEL_SAVE")
	if path == "" {
		t.Skip("BUS_PARALLEL_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		sim.SeedSimulation(seed)
		g := NewGame(1280, 720)
		g.savePathOverride = path
		if err := g.LoadGame(); err != nil {
			t.Fatal(err)
		}
		sim.SeedSimulation(seed)
		var lead *sim.Bus
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == 15 {
				lead = a.Bus
			}
		}
		if lead == nil {
			t.Fatal("missing saved leading AI 15")
		}
		initialPlayer, initialLead := g.Bus.Pos, lead.Pos
		healthPlayer, healthLead := g.Bus.Durability, lead.Durability
		target := g.Route.GetCurrentTargetStopID()
		firstLead, firstPlayer, served := -1., -1., -1.
		doors := false
		for tick := 0; tick < 3600; tick++ {
			beforePlayer, beforeLead := g.Bus.Pos, lead.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			elapsed := float64(tick) / 60
			if g.Bus.Pos.Distance(beforePlayer) > g.Bus.MaxSpeedFwd/60+1 || lead.Pos.Distance(beforeLead) > lead.MaxSpeedFwd/60+1 {
				t.Fatal("recovery teleported a bus")
			}
			if g.Bus.Durability < healthPlayer || lead.Durability < healthLead {
				t.Fatalf("seed%d recovery caused contact at %.2fs: player %.1f/%.1f lead %.1f/%.1f", seed, elapsed, g.Bus.Durability, healthPlayer, lead.Durability, healthLead)
			}
			for _, own := range g.Bus.BodyParts() {
				for _, other := range lead.BodyParts() {
					if own.HalfLen <= 0 || other.HalfLen <= 0 {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(own, other); hit && depth > 1e-6 {
						t.Fatalf("seed%d physical bodies overlap at %.2fs: depth %g", seed, elapsed, depth)
					}
				}
			}
			if firstLead < 0 && lead.Pos.Distance(initialLead) > 20 {
				firstLead = elapsed
			}
			if firstPlayer < 0 && g.Bus.Pos.Distance(initialPlayer) > 20 {
				firstPlayer = elapsed
			}
			if g.Bus.DoorsOpen && g.AutopilotDriver.DockedStopID == target {
				doors = true
			}
			if served < 0 && g.Route.GetCurrentTargetStopID() != target {
				if !doors || g.lastSkippedStopID == target {
					t.Fatal("recovery skipped original station")
				}
				served = elapsed
			}
		}
		if firstLead < 0 || firstLead > 3 || firstPlayer < 0 || firstPlayer > 8 || served < 0 || served > 20 || g.Bus.Pos.Distance(initialPlayer) < 2000 || lead.Pos.Distance(initialLead) < 500 {
			t.Fatalf("seed%d lead%.2f player%.2f served%.2f final%v", seed, firstLead, firstPlayer, served, g.Bus.Pos)
		}
		t.Logf("seed%d AI15 resumed%.2fs player%.2fs served%.2fs; no contact through60s", seed, firstLead, firstPlayer, served)
	}
}
