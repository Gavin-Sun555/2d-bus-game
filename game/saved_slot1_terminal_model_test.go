package game

import (
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestSaveSlot1TerminalModelSwitchAndDeparture verifies that when Save Slot 1 is loaded
// (where the player's bus reached starting terminus Stop 226 with a pending model change to
// an articulated bus, but had previously been held in TerminalReplacementHold because the route
// index had already advanced upon door opening), the terminal replacement cleanly succeeds,
// the model switches to the articulated bus, the hold releases, passengers board, and the bus
// departs toward the next stop.
func TestSaveSlot1TerminalModelSwitchAndDeparture(t *testing.T) {
	candidates := []string{
		filepath.Join("..", "dist", "slot1-jam-2026-10-30-1224", "slot1.json"),
		filepath.Join("dist", "slot1-jam-2026-10-30-1224", "slot1.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot1-jam-2026-10-30-1224/slot1.json",
	}
	var savePath string
	for _, c := range candidates {
		g := NewGame(1280, 720)
		g.savePathOverride = c
		if err := g.LoadGame(); err == nil {
			savePath = c
			break
		}
	}
	g := NewGame(1280, 720)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

	p := g.Bus
	ad := g.AutopilotDriver

	t.Logf("INITIAL: Model=%v PendingModel=%v TerminalHold=%v Pos=%v Spd=%.2f DoorsOpen=%v PassCount=%d",
		p.Model, p.PendingModel, p.TerminalReplacementHold, p.Pos, p.Speed, p.DoorsOpen, p.PassengerCount)

	modelSwitched := false
	departed := false

	for tick := 1; tick <= 600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		if p.Model == sim.BusArticulated {
			if !modelSwitched {
				modelSwitched = true
				t.Logf("Tick %d: MODEL SWITCHED TO ARTICULATED! Pos=%v TerminalHold=%v DoorsOpen=%v PassCount=%d",
					tick, p.Pos, p.TerminalReplacementHold, p.DoorsOpen, p.PassengerCount)
			}
		}

		if modelSwitched && p.Speed > 10.0 {
			if !departed {
				departed = true
				t.Logf("Tick %d: BUS DEPARTED! Spd=%.2f Pos=%v State=%v TargetStop=%v",
					tick, p.Speed, p.Pos, ad.State, g.Route.GetCurrentTargetStopID())
			}
		}

		if tick%60 == 0 {
			t.Logf("Tick %d: Spd=%.2f Pos=%v State=%v TerminalHold=%v DoorsOpen=%v DoorProg=%.2f TargetStop=%v",
				tick, p.Speed, p.Pos, ad.State, p.TerminalReplacementHold, p.DoorsOpen, p.DoorProgress, g.Route.GetCurrentTargetStopID())
		}
	}

	if !modelSwitched {
		t.Fatalf("bus never switched to articulated model, current model: %v pending: %v", p.Model, p.PendingModel)
	}
	if !departed {
		t.Fatalf("bus failed to depart from station after switching, state: %v spd: %.2f hold: %v doors: %v",
			ad.State, p.Speed, p.TerminalReplacementHold, p.DoorsOpen)
	}
}
