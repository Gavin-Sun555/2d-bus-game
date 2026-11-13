package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestSavedSlot1SignalReleasesPlayer(t *testing.T) {
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", os.Getenv("BUS_SLOT1_SIGNAL_SAVE"))
	g := loadSlot1LeftExitReplay(t)
	var signal *sim.TrafficIntersection
	for _, inter := range g.TrafficLights.Intersections {
		if inter.Center == (sim.Vec2{X: -46800, Y: 21600}) {
			signal = inter
			break
		}
	}
	if signal == nil {
		t.Fatal("intersection missing")
	}
	start := g.Bus.Pos
	health := g.Bus.Durability
	phase := signal.Phase
	for tick := 0; tick < 45*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if signal.Phase != phase {
			t.Logf("t%.2f phase %d -> %d ages %v player %v", float64(tick+1)/60, phase, signal.Phase, signal.PriorityPhaseWait, g.Bus.Pos)
			phase = signal.Phase
		}
		if g.Bus.Durability < health {
			t.Fatal("player bus damaged at junction")
		}
		if g.Bus.Pos.X > start.X+1050 {
			t.Logf("player cleared in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("player remained at %v", g.Bus.Pos)
}
