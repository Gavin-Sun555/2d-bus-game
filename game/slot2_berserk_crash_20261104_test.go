package game

import (
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

// Optional local replay; the small observation-cache regression is covered by
// traffic.TestPlayerObstacleSnapshotWithoutAIBuses without an external save.
func TestSlot2BerserkCrashNovember4(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	p := filepath.Join(filepath.Dir(source), "../dist/slot2-berserk-crash-2026-11-04/slot2.json")
	if _, err := os.Stat(p); os.IsNotExist(err) {
		t.Skip("November 4 slot 2 snapshot unavailable")
	}
	for _, rewind := range []bool{false, true} {
		name := "resume"
		if rewind {
			name = "high-speed-approach"
		}
		t.Run(name, func(t *testing.T) {
			g := NewGame(1280, 720)
			g.savePathOverride = p
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			g.Settings.IsOpen = false
			if !g.Settings.AutopilotEnabled || !g.Settings.AutopilotBerserk || len(g.AIBusMgr.Buses) != 0 {
				t.Fatal("fixture no longer represents berserk driving without AI buses")
			}
			if rewind {
				g.Bus.Pos.X -= 900
				g.Bus.Speed = g.Settings.GetBerserkCruiseSpeed()
				g.Bus.CrashCooldown = 0
				g.AutopilotDriver.Reset()
			}
			initialDur := g.Bus.Durability
			initialPos := g.Bus.Pos
			slowed := false
			for tick := 0; tick < 60*20; tick++ {
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < initialDur {
					t.Fatalf("collision tick%d dur%.1f->%.1f pos%v speed%.1f", tick, initialDur, g.Bus.Durability, g.Bus.Pos, g.Bus.Speed)
				}
				if g.AutopilotDriver.LastTargetSpeed < g.Settings.GetBerserkCruiseSpeed()-20 {
					slowed = true
				}
			}
			if !slowed {
				t.Fatal("autopilot never responded to slower traffic")
			}
			if g.Bus.Pos.Distance(initialPos) < 1000 {
				t.Fatal("bus avoided crash by remaining stuck")
			}
			t.Logf("20 seconds without new damage: durability%.1f, distance%.0fpx", g.Bus.Durability, g.Bus.Pos.Distance(initialPos))
		})
	}
}
