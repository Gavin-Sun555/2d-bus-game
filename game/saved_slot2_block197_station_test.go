package game

import (
	"os"
	"path/filepath"
	"testing"
)

func findSlot2Block197File() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-197block-2026-11-02-1150", "slot2.json"),
		filepath.Join("dist", "slot2-197block-2026-11-02-1150", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-197block-2026-11-02-1150/slot2.json",
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2Block197StationDeadlockDissolves verifies that the deadlock at Stop 6
// (opposite Stop 196 / Block 197 Northbound) in slot 2 dissolves cleanly:
// AIBus 6 (in the Stop 6 bay) escapes/departs onto Road 11,
// AIBus 5 (waiting in Lane 1) pulls into the bay and docks,
// and the player bus accelerates and cruises smoothly down Road 11.
func TestSavedSlot2Block197StationDeadlockDissolves(t *testing.T) {
	savePath := findSlot2Block197File()
	if savePath == "" {
		t.Skip("save file not found for slot2 block 197")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	b5 := g.AIBusMgr.Buses[5]
	b6 := g.AIBusMgr.Buses[6]
	if b5 == nil || b6 == nil {
		t.Fatalf("could not find AI buses 5 and 6")
	}

	initB5Y := b5.Bus.Pos.Y
	initB6Y := b6.Bus.Pos.Y

	for step := 1; step <= 1200; step++ { // 20 seconds at 60Hz
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at step %d: %v", step, err)
		}
	}

	// B6 should have departed from bay (Y ~ 859.5) and moved down Road 11 past Y=1200
	if b6.Bus.Pos.Y <= initB6Y+200 {
		t.Fatalf("B6 remained stuck at Stop 6: initial Y=%.1f, current Y=%.1f", initB6Y, b6.Bus.Pos.Y)
	}

	// B5 should have entered the Stop 6 bay (X moves from ~7719 to < 7690, Y moves from ~758 to > 840)
	if b5.Bus.Pos.Y <= initB5Y+50 || b5.Bus.Pos.X > 7700 {
		t.Fatalf("B5 failed to enter Stop 6 bay: pos=(%.1f, %.1f)", b5.Bus.Pos.X, b5.Bus.Pos.Y)
	}

	// Player bus should have cleared through the upstream signal and traveled down Road 11 past Y=400
	if g.Bus.Pos.Y < 400 {
		t.Fatalf("Player bus failed to advance: pos=(%.1f, %.1f)", g.Bus.Pos.X, g.Bus.Pos.Y)
	}
}
