package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedOpposingLeftAndUTurnClear(t *testing.T) {
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", os.Getenv("BUS_UTURN_DEADLOCK_SAVE"))
	g := loadSlot1LeftExitReplay(t)
	var left, uTurn *sim.Bus
	for _, a := range g.AIBusMgr.Buses {
		switch a.ID {
		case 27:
			left = a.Bus
		case 33:
			uTurn = a.Bus
		}
	}
	if left == nil || uTurn == nil {
		t.Fatal("expected AI27 and AI33 in the capture")
	}
	startLeft, startUTurn := left.Pos, uTurn.Pos
	leftHealth, uTurnHealth := left.Durability, uTurn.Durability
	for tick := 0; tick < 40*60; tick++ {
		prevLeft, prevUTurn := left.Pos, uTurn.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		hit, _, _ := sim.CheckBusesCollision(left, uTurn)
		if hit || left.Durability < leftHealth || uTurn.Durability < uTurnHealth ||
			left.Pos.Distance(prevLeft) > math.Abs(left.Speed)/60+.05 ||
			uTurn.Pos.Distance(prevUTurn) > math.Abs(uTurn.Speed)/60+.05 {
			t.Fatalf("collision, damage or reposition during clearance at tick %d", tick)
		}
		if left.Pos.X < startLeft.X-500 && uTurn.Pos.Y < startUTurn.Y-500 {
			t.Logf("both buses cleared in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("junction still blocked, AI27 %v and AI33 %v", left.Pos, uTurn.Pos)
}
