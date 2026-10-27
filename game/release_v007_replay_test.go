package game

import (
	"os"
	"testing"
)

// Replays a captured, synthetic qualification scene without changing its file.
// Keeps short failure reproductions available after the original long run.
func TestRelease007CapturedSafetyReplay(t *testing.T) {
	path := os.Getenv("BUSGAME_RELEASE007_REPLAY_SAVE")
	if path == "" {
		t.Skip("set BUSGAME_RELEASE007_REPLAY_SAVE to a captured qualification scene")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	health := g.Bus.Durability
	for tick := 0; tick < 360; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatalf("contact tick%d position%v heading%.3f speed%.1f health%.0f->%.0f target%d events%+v", tick, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, health, g.Bus.Durability, g.Route.GetCurrentTargetStopID(), g.HUD.Toasts)
		}
	}
}
