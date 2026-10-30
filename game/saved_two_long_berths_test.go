package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestSavedSecondArticulatedBoardsWhileLeadServes(t *testing.T) {
	path := os.Getenv("BUS_TWO_LONG_BERTH_SAVE")
	if path == "" {
		t.Skip("copied 2026-10-29 08:44 save required")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	var rear *sim.Bus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 3 {
			rear = ab.Bus
		}
	}
	if rear == nil {
		t.Fatal("missing saved follower")
	}
	length := g.World.Stops[86].Length
	playerHealth, rearHealth := g.Bus.Durability, rear.Durability
	boardedTogether := false
	for tick := 0; tick < 5400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < playerHealth || rear.Durability < rearHealth {
			t.Fatalf("new contact at %.2fs", float64(tick)/60)
		}
		if g.Bus.DoorsOpen && rear.DoorsOpen && rear.PassengerService.Active && rear.PassengerService.StopID == 86 {
			if !boardedTogether {
				t.Logf("rear began service while lead doors open at %.2fs", float64(tick+1)/60)
			}
			boardedTogether = true
		}
	}
	if !boardedTogether {
		t.Fatal("second long bus never boarded with lead")
	}
	if g.World.Stops[86].Length != length {
		t.Fatal("platform was extended")
	}
	if rear.Pos.Distance(g.World.Stops[86].Pos) < 400 {
		t.Fatal("rear never departed after service")
	}
}
