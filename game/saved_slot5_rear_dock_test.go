package game

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
)

func TestSavedSlot5RearBerthImmediateDockBehindArticulatedBus(t *testing.T) {
	savePath := filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-05.json")
	if _, err := os.Stat(savePath); err != nil {
		t.Skip("save-05.json not found")
	}

	g := loadSaveForTest(t, savePath, 1440, 900)

	if err := g.Update(); err != nil {
		t.Fatalf("Update failed: %v", err)
	}

	if g.AutopilotDriver.State != autopilot.AutopilotStateDocked {
		t.Fatalf("expected player bus to dock immediately on tick 1 behind lead articulated bus, got state %s", g.AutopilotDriver.State)
	}
	if !g.Bus.DoorsOpen {
		t.Fatalf("expected player bus doors to open immediately on tick 1 while queued inside rear berth")
	}

	departed := false
	for i := 0; i < 600; i++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at tick %d: %v", i+2, err)
		}
		if g.AutopilotDriver.State == autopilot.AutopilotStateDeparting || (g.AutopilotDriver.State == autopilot.AutopilotStateCruising && g.Bus.Pos.X > -23300) {
			departed = true
			break
		}
	}
	if !departed {
		t.Fatalf("expected player bus to complete passenger service and depart station, final state=%s pos=%+v", g.AutopilotDriver.State, g.Bus.Pos)
	}
}
