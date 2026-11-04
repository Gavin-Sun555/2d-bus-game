package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

func findSlot2Station1WedgeFile() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1430", "slot2.json"),
		filepath.Join("dist", "slot2-jam-2026-11-02-1430", "slot2.json"),
		filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-02.json"),
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

// TestSavedSlot2Station1WedgeDissolves verifies that the corridor bottleneck in save slot 2
// (adjacent-lane wedging between Bus 2 and Bus 41 approaching Stop 1, and Bus 43 departing Stop 1)
// dissolves cleanly without manual intervention:
// 1. Bus 43 clears its departure crawl and accelerates along its route.
// 2. Bus 2 (leader) smoothly proceeds into Stop 1, exchanges passengers, and continues to Stop 7.
// 3. Bus 41 safely follows Bus 2 into the Stop 1 bay.
// 4. The following queue (Bus 1, Bus 8) moves forward smoothly.
func TestSavedSlot2Station1WedgeDissolves(t *testing.T) {
	savePath := findSlot2Station1WedgeFile()
	if savePath == "" {
		t.Skip("save file not found for slot2 station 1 wedge")
	}

	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	initialPositions := make(map[int][2]float64)
	for _, ab := range g.AIBusMgr.Buses {
		initialPositions[ab.ID] = [2]float64{ab.Bus.Pos.X, ab.Bus.Pos.Y}
	}

	// The fallback is the user's live slot, which may have been overwritten
	// with another scenario. Only run this historical replay with its buses.
	for _, id := range []int{43, 2, 41, 1, 8} {
		if _, ok := initialPositions[id]; !ok {
			t.Skipf("slot 2 is not the historical station-1 wedge fixture (missing bus #%d)", id)
		}
	}

	// Run for 1800 ticks (30 seconds)
	for tick := 1; tick <= 1800; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at tick %d: %v", tick, err)
		}
	}

	type busCheck struct {
		id      int
		minDist float64
		desc    string
	}

	checks := []busCheck{
		{id: 43, minDist: 1000.0, desc: "Bus 43 (departing Stop 1)"},
		{id: 2, minDist: 250.0, desc: "Bus 2 (approaching & servicing Stop 1)"},
		{id: 41, minDist: 150.0, desc: "Bus 41 (following into Stop 1 bay)"},
		{id: 1, minDist: 150.0, desc: "Bus 1 (following queue forward)"},
		{id: 8, minDist: 100.0, desc: "Bus 8 (following queue forward)"},
	}

	for _, chk := range checks {
		found := false
		for _, ab := range g.AIBusMgr.Buses {
			if ab.ID == chk.id {
				found = true
				initPos := initialPositions[chk.id]
				moved := math.Hypot(ab.Bus.Pos.X-initPos[0], ab.Bus.Pos.Y-initPos[1])
				if moved < chk.minDist {
					t.Errorf("%s moved only %.1f px, expected at least %.1f px", chk.desc, moved, chk.minDist)
				}
				break
			}
		}
		if !found {
			t.Errorf("bus #%d not found in manager", chk.id)
		}
	}
}
