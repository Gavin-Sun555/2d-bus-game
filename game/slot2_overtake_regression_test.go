package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestSaveSlot2BerserkOvertakesSlowerAI ensures that in Save Slot 2 on the 8-lane
// bus boulevard (城市拓展大道), the player's 120 km/h berserk autopilot changes lanes to
// Lane 2 to pass the slower AI bus #10 (traveling at ~80 km/h), accelerates to cruise speed,
// passes AI #10, and cleanly returns to the reserved bus lane (Lane 3).
func TestSaveSlot2BerserkOvertakesSlowerAI(t *testing.T) {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-oct30-1116", "slot2.json"),
		filepath.Join("dist", "slot2-oct30-1116", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-oct30-1116/slot2.json",
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
		t.Skipf("slot2 save file not found, wd=%s", wd)
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

	p := g.Bus
	ad := g.AutopilotDriver
	lw := g.Settings.GetLaneWidth()

	if !g.Settings.AutopilotBerserk || !ad.BerserkMode {
		t.Fatalf("expected berserk mode enabled, got settings=%v driver=%v", g.Settings.AutopilotBerserk, ad.BerserkMode)
	}

	var ai10 *sim.Bus
	for _, b := range g.AIBusMgr.Buses {
		if b.ID == 10 {
			ai10 = b.Bus
			break
		}
	}
	if ai10 == nil {
		t.Fatal("AI #10 not found in save slot 2")
	}

	// AI #10 must initially be ahead of the player in the same lane (heading West)
	if p.Pos.X <= ai10.Pos.X {
		t.Fatalf("expected player (X=%.1f) to be behind AI #10 (X=%.1f)", p.Pos.X, ai10.Pos.X)
	}

	laneChanged := false
	passedLeader := false
	returnedToLane := false

	for tick := 1; tick <= 350; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		// 1. Bus initiates lane change to Lane 2 (TargetLaneOffset = -lw)
		if ad.TargetLaneOffset <= -lw+1.0 {
			laneChanged = true
		}

		// 2. Bus passes AI #10 while heading West (Player X < AI X)
		if laneChanged && p.Pos.X < ai10.Pos.X {
			passedLeader = true
		}

		// 3. Bus returns to base lane (Lane 3) after completing the pass
		if passedLeader && math.Abs(ad.LaneOffset) < 1.0 && ad.OvertakeState == autopilot.OvertakeNone {
			returnedToLane = true
			break
		}
	}

	if !laneChanged {
		t.Errorf("player bus failed to initiate lane change to overtake AI #10: TargetLaneOffset=%.1f", ad.TargetLaneOffset)
	}
	if !passedLeader {
		t.Errorf("player bus failed to pass AI #10: PlayerX=%.1f AIX=%.1f", p.Pos.X, ai10.Pos.X)
	}
	if !returnedToLane {
		t.Errorf("player bus failed to return to base bus lane after passing: LaneOffset=%.1f OvertakeState=%d", ad.LaneOffset, ad.OvertakeState)
	}
}
