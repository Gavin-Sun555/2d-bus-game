package game

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Replay Save Slot 1 from 2026-10-30 where articulated bus AI #1 was jammed
// entering Stop 228 (街区229·南行站) with 13 queued buses behind it.
func TestSavedSlot1Oct30JamRecovery(t *testing.T) {
	candidates := []string{
		filepath.Join("..", "dist", "slot1-jam-2026-10-30-1045", "slot1.json"),
		filepath.Join("dist", "slot1-jam-2026-10-30-1045", "slot1.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot1-jam-2026-10-30-1045/slot1.json",
	}
	var savePath string
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			savePath = c
			break
		}
	}
	if savePath == "" {
		wd, _ := os.Getwd()
		t.Skipf("save file not found, wd=%s", wd)
	}

	absPath, err := filepath.Abs(savePath)
	if err != nil {
		t.Fatal(err)
	}

	g := NewGame(1280, 720)
	g.savePathOverride = absPath
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

	type busTracker struct {
		lastPos          sim.Vec2
		distTraveled     float64
		initTarget       int
		reachedBoarding  bool
		departedStop     bool
		initialDurability float64
	}

	trackers := make(map[int]*busTracker)
	for _, a := range g.AIBusMgr.Buses {
		trackers[a.ID] = &busTracker{
			lastPos:          a.Bus.Pos,
			initTarget:       a.Route.GetCurrentTargetStopID(),
			initialDurability: a.Bus.Durability,
		}
	}

	playerInitHealth := g.Bus.Durability

	// Run 3600 ticks (60 seconds)
	for tick := 0; tick < 3600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d Update failed: %v", tick, err)
		}

		if g.Bus.Durability < playerInitHealth {
			t.Fatalf("player bus durability decreased at tick %d: %f < %f", tick, g.Bus.Durability, playerInitHealth)
		}

		for _, a := range g.AIBusMgr.Buses {
			tr := trackers[a.ID]
			if tr == nil {
				continue
			}

			step := a.Bus.Pos.Distance(tr.lastPos)
			if step > a.Bus.MaxSpeedFwd/60+10 {
				t.Fatalf("AI #%d moved too far in single tick (%f units) at tick %d", a.ID, step, tick)
			}
			tr.distTraveled += step
			tr.lastPos = a.Bus.Pos

			if a.Bus.Durability < tr.initialDurability-0.01 {
				t.Fatalf("AI #%d sustained damage at tick %d", a.ID, tick)
			}

			if a.Bus.DoorsOpen {
				tr.reachedBoarding = true
			}
			if a.Route.GetCurrentTargetStopID() != tr.initTarget {
				tr.departedStop = true
			}
		}
	}

	// AI #1 was the blocker at Stop 228. It must have recovered and cruised ahead.
	tr1 := trackers[1]
	if tr1 == nil || tr1.distTraveled < 500 {
		t.Fatalf("AI #1 did not recover and move forward: dist=%v", tr1)
	}
	if !tr1.departedStop {
		t.Fatalf("AI #1 did not advance target stop from Stop 228")
	}

	// AI #2 was queued behind Stop 228. It must have served Stop 228 and departed.
	tr2 := trackers[2]
	if tr2 == nil || !tr2.reachedBoarding {
		t.Fatalf("AI #2 did not reach boarding state at Stop 228")
	}
	if !tr2.departedStop {
		t.Fatalf("AI #2 did not depart Stop 228 after boarding")
	}
	if tr2.distTraveled < 300 {
		t.Fatalf("AI #2 did not cruise after departure: dist=%f", tr2.distTraveled)
	}
}
