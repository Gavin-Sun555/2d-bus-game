package game

import (
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

func findSlot2Station21File() string {
	_, source, _, _ := runtime.Caller(0)
	candidates := []string{
		filepath.Join(filepath.Dir(source), "..", "dist", "slot2-station21-jam-2026-11-03-0654", "slot2.json"),
		filepath.Join("..", "dist", "slot2-station21-jam-2026-11-03-0654", "slot2.json"),
		filepath.Join("dist", "slot2-station21-jam-2026-11-03-0654", "slot2.json"),
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2Station21DockingDissolves verifies that an AI bus arriving inside
// a station bay (e.g. Stop 32 / Coastal District 21 in slot 2) docks properly instead
// of stalling forever at speed=0 due to over-strict lateral tolerances.
func TestSavedSlot2Station21DockingDissolves(t *testing.T) {
	savePath := findSlot2Station21File()
	if savePath == "" {
		t.Skip("slot2 snapshot not found; skipping")
	}

	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	b20 := g.AIBusMgr.Buses[19]
	if b20 == nil || b20.ID != 20 {
		t.Fatalf("expected bus 20 at index 19")
	}

	// Within 60 frames (1 second), b20 must successfully enter docked state and open doors.
	dockedWithin1s := false
	for step := 1; step <= 60; step++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed: %v", err)
		}
		if b20.Bus.DoorsOpen && b20.Bus.PassengerService.Active {
			dockedWithin1s = true
			break
		}
	}

	if !dockedWithin1s {
		t.Fatalf("b20 failed to dock and open doors at Stop 32 within 1s: doorsOpen=%v, servActive=%v, state=%s",
			b20.Bus.DoorsOpen, b20.Bus.PassengerService.Active, b20.Driver.State.String())
	}
}
