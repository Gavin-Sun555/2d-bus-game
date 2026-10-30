package game

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Opt-in full traffic replay. Point at read-only copies, never the live saves.
func TestSavedSlotPairRecovery(t *testing.T) {
	dir := os.Getenv("BUS_SLOT_PAIR_DIR")
	if dir == "" {
		t.Skip("BUS_SLOT_PAIR_DIR not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		for _, slot := range []int{1, 3} {
			t.Run(fmt.Sprintf("slot%d/seed%d", slot, seed), func(t *testing.T) {
				sim.SeedSimulation(seed)
				g := NewGame(1280, 720)
				g.savePathOverride = filepath.Join(dir, fmt.Sprintf("slot%d-readonly.json", slot))
				if err := g.LoadGame(); err != nil {
					t.Fatal(err)
				}
				pos, health, target := g.Bus.Pos, g.Bus.Durability, g.Route.GetCurrentTargetStopID()
				firstMotion, advanced := -1, -1
				maxWait, wait := 0, 0
				for tick := 0; tick < 9000; tick++ {
					before := g.Bus.Pos
					if err := g.Update(); err != nil {
						t.Fatal(err)
					}
					if before.Distance(g.Bus.Pos) > 20 {
						t.Fatalf("unexpected position jump tick%d", tick)
					}
					if g.Bus.Durability < health {
						t.Fatalf("collision during recovery tick%d health%.1f -> %.1f", tick, health, g.Bus.Durability)
					}
					if firstMotion < 0 && pos.Distance(g.Bus.Pos) > 20 {
						firstMotion = tick
					}
					if g.Bus.Speed < 3 {
						wait++
						maxWait = max(maxWait, wait)
					} else {
						wait = 0
					}
					if g.Route.GetCurrentTargetStopID() != target && advanced < 0 {
						advanced = tick
					}
					// Continue at least 120s to catch the next station's queue/departure.
					if tick >= 7200 && advanced >= 0 && pos.Distance(g.Bus.Pos) > 2000 {
						break
					}
				}
				if firstMotion < 0 || firstMotion > 600 || advanced < 0 || pos.Distance(g.Bus.Pos) < 2000 {
					t.Fatalf("incomplete recovery: motion%d advanced%d distance%.1f", firstMotion, advanced, pos.Distance(g.Bus.Pos))
				}
				if maxWait > 3600 {
					t.Fatalf("new prolonged wait: %.2fs", float64(maxWait)/60)
				}
				t.Logf("motion %.2fs; next stop %.2fs; distance %.0fpx; longest stop %.2fs; health %.0f", float64(firstMotion)/60, float64(advanced)/60, pos.Distance(g.Bus.Pos), float64(maxWait)/60, g.Bus.Durability)
			})
		}
	}
}
