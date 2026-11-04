package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

func findSlot2JamFile() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1105", "slot2.json"),
		filepath.Join("dist", "slot2-jam-2026-11-02-1105", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-jam-2026-11-02-1105/slot2.json",
		filepath.Join("..", "dist", "slot2-crash-2026-11-02", "slot2.json"),
		filepath.Join("dist", "slot2-crash-2026-11-02", "slot2.json"),
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2BridgeGridlockDissolves verifies that the 14-bus queue and civilian traffic
// deadlock on the Road 10 six-lane bridge in slot 2 dissolves smoothly:
// the lane-changing lead bus is permitted to clear the overlap padding,
// the following bus queue accelerates, and traffic flows freely.
func TestSavedSlot2BridgeGridlockDissolves(t *testing.T) {
	p := findSlot2JamFile()
	if p == "" {
		t.Skip("no save file found for slot2 jam")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

	bus4 := g.AIBusMgr.Buses[4]
	bus36 := g.AIBusMgr.Buses[36]
	initBus4X := bus4.Bus.Pos.X
	initBus36X := bus36.Bus.Pos.X

	// Simulate 30 seconds (allowing one full traffic light phase at intersection X=5400)
	for tick := 1; tick <= 60*30; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}
	}

	// Verify Bus 4 advanced significantly
	if bus4.Bus.Pos.X <= initBus4X+100.0 {
		t.Fatalf("Bus 4 did not advance: init X=%.1f, current Pos=%v", initBus4X, bus4.Bus.Pos)
	}

	// Verify Bus 36 accelerated and traversed the bridge
	if bus36.Bus.Pos.X <= initBus36X+500.0 {
		t.Fatalf("Bus 36 did not clear: init X=%.1f, current Pos=%v", initBus36X, bus36.Bus.Pos)
	}

	// Verify the stopped buses on the bridge queue have drained
	stoppedBusesOnBridge := 0
	for _, b := range g.AIBusMgr.Buses {
		if b.Bus.Pos.X >= 3000 && b.Bus.Pos.X <= 5500 && b.Bus.Pos.Y >= 0 && b.Bus.Pos.Y <= 150 {
			if math.Abs(b.Bus.Speed) < 2.0 {
				stoppedBusesOnBridge++
			}
		}
	}
	if stoppedBusesOnBridge > 2 {
		t.Fatalf("Bridge still jammed: %d buses stopped", stoppedBusesOnBridge)
	}
}
