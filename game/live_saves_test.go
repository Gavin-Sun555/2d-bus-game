package game

import (
	"os"
	"path/filepath"
	"testing"
)

func TestLiveSave02(t *testing.T) {
	home, err := os.UserHomeDir()
	if err != nil {
		t.Fatal(err)
	}
	savePath := filepath.Join(home, "Library/Application Support/2d-bus-game/save-02.json")
	if _, err := os.Stat(savePath); os.IsNotExist(err) {
		t.Skip("Live save 02 not found on this machine")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	t.Logf("Initial: Pos=(%.1f, %.1f), Heading=%.4f (%.1f deg), Spd=%.1f, Gear=%v, AP=%v, Berserk=%v",
		g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Heading, g.Bus.Heading*180/3.14159, g.Bus.Speed,
		g.Bus.Gear, g.Settings.AutopilotEnabled, g.Settings.AutopilotBerserk)

	for step := 1; step <= 1200; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if step%60 == 0 {
			t.Logf("Step %d (%.1fs): Pos=(%.1f, %.1f), Heading=%.4f, Spd=%.1f, TargetSpd=%.1f, State=%v",
				step, float64(step)/60.0, g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Heading, g.Bus.Speed, g.AutopilotDriver.LastTargetSpeed, g.AutopilotDriver.State)
		}
	}
}

func TestLiveSave03(t *testing.T) {
	home, err := os.UserHomeDir()
	if err != nil {
		t.Fatal(err)
	}
	savePath := filepath.Join(home, "Library/Application Support/2d-bus-game/save-03.json")
	if _, err := os.Stat(savePath); os.IsNotExist(err) {
		t.Skip("Live save 03 not found on this machine")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	t.Logf("Initial Save 03: Pos=(%.1f, %.1f), Heading=%.4f (%.1f deg), Spd=%.1f, Gear=%v, AP=%v, Berserk=%v",
		g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Heading, g.Bus.Heading*180/3.14159, g.Bus.Speed,
		g.Bus.Gear, g.Settings.AutopilotEnabled, g.Settings.AutopilotBerserk)

	for step := 1; step <= 1200; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if step%60 == 0 {
			for _, ai := range g.AIBusMgr.Buses {
				if ai.ID >= 21 && ai.ID <= 24 {
					t.Logf("Step %d (%.1fs) AI#%d: Pos=(%.1f, %.1f), Spd=%.1f, Gear=%v, TgtSpd=%.1f, State=%v, Deadlock=%.1f, Active=%v",
						step, float64(step)/60.0, ai.ID, ai.Bus.Pos.X, ai.Bus.Pos.Y, ai.Bus.Speed, ai.Bus.Gear, ai.Driver.LastTargetSpeed, ai.Driver.State, ai.Driver.HeadonDeadlockTimer(), ai.Driver.HeadonRecoveryActive())
				}
			}
		}
	}
}
