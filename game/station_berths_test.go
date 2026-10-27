package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"testing"
)

func TestSavedTwoBerthsKeepIndependentBoardingPositions(t *testing.T) {
	g := saveTestGame(t)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}})
	st := g.World.Stops[g.Route.StopIDs[0]]
	others := []*sim.Bus{}
	for i, ab := range g.AIBusMgr.Buses {
		ab.Route.CurrentIndex = 0
		ab.Bus.SetModel(sim.BusModel(i))
		sim.PlaceBusInStationQueue(ab.Bus, st, others, 52)
		others = append(others, ab.Bus)
		ab.Bus.DoorsOpen = true
		ab.Bus.DoorProgress = 1
		ab.Driver.State = autopilot.AutopilotStateDocked
		ab.Driver.DockedStopID = st.ID
		ab.Driver.DwellTimer = 10
	}
	g.Bus.Pos = sim.Vec2{X: 10000, Y: 10000}
	positions := []sim.Vec2{others[0].Pos, others[1].Pos}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	for i, ab := range g.AIBusMgr.Buses {
		if ab.Bus.Pos != positions[i] || !ab.Bus.DoorsOpen || ab.Driver.DockedStopID != st.ID {
			t.Fatal("load moved/requeued an active berth")
		}
	}
	for _, stop := range g.World.Stops {
		if stop.Length != sim.StandardStopLength {
			t.Fatal("old station geometry after load")
		}
	}
}

func TestOldSaveUpgradesEveryPlatformWithoutMovingVehicles(t *testing.T) {
	g := saveTestGame(t)
	for _, stop := range g.World.Stops {
		stop.Length = 140
	}
	pos, cash := g.Bus.Pos, g.Cash
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Pos != pos || g.Cash != cash {
		t.Fatal("platform migration moved the running vehicle or changed cash")
	}
	for _, stop := range g.World.Stops {
		if stop.Length != sim.StandardStopLength {
			t.Fatal("legacy short platform survived load", stop.ID)
		}
	}
}
