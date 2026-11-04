package game

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func findSlot2HeadonFile() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-headon-2026-11-02-1227", "slot2.json"),
		filepath.Join("dist", "slot2-headon-2026-11-02-1227", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-headon-2026-11-02-1227/slot2.json",
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2HeadonDeadlockDissolves verifies that the head-to-head deadlock at intersection
// (0, 3600) between left-turning Bus 17 and straight-going Bus 47 dissolves cleanly:
// 1. After 10s of being locked head-to-head, Bus 17 (yielding left turn) reverses backward ~70px.
// 2. Bus 47's straight trajectory unblocks, allowing Bus 47 to cross intersection (0, 3600) Eastward.
// 3. Bus 17 completes its left turn South onto the avenue.
// 4. The 8-bus queue backing up to the player's bus dissolves, and the player bus moves forward.
func TestSavedSlot2HeadonDeadlockDissolves(t *testing.T) {
	savePath := findSlot2HeadonFile()
	if savePath == "" {
		t.Skip("save file not found for slot2 headon deadlock")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	var b17, b47 *sim.Bus
	var d17 *autopilot.AutopilotDriver
	var ab17, ab47 *traffic.AIBus

	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 17 {
			ab17 = ab
			b17 = ab.Bus
			d17 = ab.Driver
		} else if ab.ID == 47 {
			ab47 = ab
			b47 = ab.Bus
		}
	}

	if b17 == nil || b47 == nil || d17 == nil || ab17 == nil || ab47 == nil {
		t.Fatalf("could not find AI buses 17 and 47")
	}

	initB17Pos := b17.Pos
	initB47Pos := b47.Pos
	initPlayerX := g.Bus.Pos.X

	for step := 1; step <= 2100; step++ { // 35 seconds at 60Hz
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at step %d: %v", step, err)
		}
	}

	// 1. Bus 47 must have crossed intersection (0, 3600) and traveled East along Road (0, 3600)->(5400, 3600)
	if b47.Pos.X < 2000.0 {
		t.Fatalf("B47 remained stuck at intersection: initial X=%.1f, current X=%.1f", initB47Pos.X, b47.Pos.X)
	}

	// 2. Bus 17 must have completed its left turn and traveled South along Road (0, 3600)->(0, 7200)
	if b17.Pos.Y < 3800.0 {
		t.Fatalf("B17 remained stuck at intersection: initial Y=%.1f, current Y=%.1f", initB17Pos.Y, b17.Pos.Y)
	}

	// 3. Player's bus (and queue) must have unblocked and moved Westward
	if g.Bus.Pos.X > initPlayerX-50.0 {
		t.Fatalf("player bus remained queued: initial X=%.1f, current X=%.1f", initPlayerX, g.Bus.Pos.X)
	}
}

