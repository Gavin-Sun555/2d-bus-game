package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedSlot1ParallelBusMergeClears(t *testing.T) {
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", os.Getenv("BUS_SLOT1_PARALLEL_MERGE_SAVE"))
	g := loadSlot1LeftExitReplay(t)
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 10 {
			continue
		}
		player, ai := g.Bus, a.Bus
		playerStart, aiStart := player.Pos, ai.Pos
		playerHealth, aiHealth := player.Durability, ai.Durability
		maxRetreat := 0.
		g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player bus repositioned") }
		a.Driver.BeforeRecovery = func() { t.Fatal("AI10 repositioned") }
		for tick := 0; tick < 40*60; tick++ {
			beforePlayer, beforeAI := player.Pos, ai.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			hit, _, depth := sim.CheckBusesCollision(player, ai)
			if (hit && depth > 1e-5) || player.Durability < playerHealth || ai.Durability < aiHealth ||
				player.Pos.Distance(beforePlayer) > math.Abs(player.Speed)/60+.1 ||
				ai.Pos.Distance(beforeAI) > math.Abs(ai.Speed)/60+.1 {
				t.Fatalf("collision or reposition at tick %d, player %v AI10 %v, overlap %.3f", tick, player.Pos, ai.Pos, depth)
			}
			maxRetreat = math.Max(maxRetreat, aiStart.Y-ai.Pos.Y)
			if player.Pos.Y > playerStart.Y+300 && ai.Pos.Y > aiStart.Y+150 {
				if maxRetreat < 20 {
					t.Fatal("AI10 did not reverse to create a gap")
				}
				t.Logf("AI10 reversed %.1fpx and both buses cleared in %.2fs", maxRetreat, float64(tick+1)/60)
				return
			}
		}
		t.Fatalf("pair still blocked: player %v AI10 %v, max retreat %.1f", player.Pos, ai.Pos, maxRetreat)
	}
	t.Fatal("AI10 missing")
}
