package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

func TestSavedSlot1MassJamResolves(t *testing.T) {
	candidates := []string{
		filepath.Join("..", "dist", "slot1-jam-2026-11-03", "slot1.json"),
		filepath.Join("dist", "slot1-jam-2026-11-03", "slot1.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot1-jam-2026-11-03/slot1.json",
	}
	var savePath string
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			savePath = c
			break
		}
	}
	if savePath == "" {
		t.Skip("slot1 fixture not found")
	}

	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	// Capture initial positions of the queue head
	b47 := g.AIBusMgr.Buses[46] // ID 47
	b41 := g.AIBusMgr.Buses[40] // ID 41
	b46 := g.AIBusMgr.Buses[45] // ID 46
	b45 := g.AIBusMgr.Buses[44] // ID 45
	b42 := g.AIBusMgr.Buses[41] // ID 42

	init47X := b47.Bus.Pos.X
	init41X := b41.Bus.Pos.X
	init45Y := b45.Bus.Pos.Y
	init42Y := b42.Bus.Pos.Y

	// Count how many buses in the vertical avenue are stopped initially
	initialStoppedBuses := 0
	for _, b := range g.AIBusMgr.Buses {
		if math.Abs(b.Bus.Speed) < 2.0 && b.Bus.Pos.Y < -300 && b.Bus.Pos.X < -62000 {
			initialStoppedBuses++
		}
	}
	t.Logf("Initial stopped buses in vertical avenue: %d", initialStoppedBuses)

	// Simulate 45 seconds
	for tick := 1; tick <= 60*45; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}
	}

	t.Logf("After 45s: Bus47 X=%.1f (was %.1f), Bus41 X=%.1f (was %.1f), Bus46 Pos=(%.1f, %.1f), Bus45 Pos=(%.1f, %.1f), Bus42 Pos=(%.1f, %.1f)",
		b47.Bus.Pos.X, init47X, b41.Bus.Pos.X, init41X, b46.Bus.Pos.X, b46.Bus.Pos.Y, b45.Bus.Pos.X, b45.Bus.Pos.Y, b42.Bus.Pos.X, b42.Bus.Pos.Y)

	// Verify Bus 47 and Bus 41 moved significantly eastward
	if b47.Bus.Pos.X <= init47X+1000.0 {
		t.Fatalf("Bus 47 did not advance: X=%.1f", b47.Bus.Pos.X)
	}
	if b41.Bus.Pos.X <= init41X+1000.0 {
		t.Fatalf("Bus 41 did not advance: X=%.1f", b41.Bus.Pos.X)
	}

	// Verify Bus 46 entered the junction and turned east
	if b46.Bus.Pos.X <= -61000.0 {
		t.Fatalf("Bus 46 did not exit junction eastbound: Pos=%v", b46.Bus.Pos)
	}

	// Verify Bus 45 advanced through or into junction
	if b45.Bus.Pos.Y <= init45Y+300.0 {
		t.Fatalf("Bus 45 did not advance: Y=%.1f", b45.Bus.Pos.Y)
	}

	// Verify Bus 42 advanced
	if b42.Bus.Pos.Y <= init42Y+200.0 {
		t.Fatalf("Bus 42 did not advance: Y=%.1f", b42.Bus.Pos.Y)
	}

	// Verify the stopped queue on the vertical road drained significantly
	remainingStopped := 0
	for _, b := range g.AIBusMgr.Buses {
		if math.Abs(b.Bus.Speed) < 2.0 && b.Bus.Pos.Y < -300 && b.Bus.Pos.X < -62000 {
			remainingStopped++
		}
	}
	t.Logf("Remaining stopped buses in vertical avenue: %d (started with %d)", remainingStopped, initialStoppedBuses)
	if remainingStopped >= initialStoppedBuses/2 {
		t.Fatalf("Queue did not drain adequately: remaining %d vs initial %d", remainingStopped, initialStoppedBuses)
	}
}
