package game

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot2-player-station-follow-20261116.json.gz
var slot2StationFollowScene []byte

func TestSavedSlot2PlayerFollowsAIIntoStation(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2StationFollowScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	verifySlot2StationFollow(t, g)
}

func TestCurrentSlot2PlayerFollowsAIIntoStation(t *testing.T) {
	path := os.Getenv("BUS_STATION_FOLLOW_SAVE")
	if path == "" {
		t.Skip("set BUS_STATION_FOLLOW_SAVE")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	verifySlot2StationFollow(t, g)
}

func verifySlot2StationFollow(t *testing.T, g *Game) {
	t.Helper()
	stopID := g.Route.GetCurrentTargetStopID()
	stop := g.World.Stops[stopID]
	u := sim.UnitFromAngle(stop.Heading)
	lead := fleetBus(t, g, 92)
	health, leadHealth := g.Bus.Durability, lead.Bus.Durability
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player relocated instead of docking") }
	lead.Driver.BeforeRecovery = func() { t.Fatal("AI lead relocated instead of leaving the station") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == 92 {
			t.Fatal("AI lead reset instead of leaving the station", e)
		}
	}
	boardedAt := -1
	for tick := 0; tick < 60*120; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health || lead.Bus.Durability < leadHealth {
			t.Fatal("station queue damaged a bus")
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.1 {
			t.Fatal("station approach relocated player")
		}
		if hit, _, _ := sim.CheckBusesCollision(g.Bus, lead.Bus); hit {
			t.Fatal("player hit lead bus")
		}
		if lead.Bus.DoorsOpen && g.Bus.Pos.Sub(lead.Bus.Pos).Dot(u) > -g.Bus.Length {
			t.Fatal("player overtook serving lead instead of following station queue")
		}
		if g.Bus.DoorsOpen && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == stopID && boardedAt < 0 {
			if !stop.ContainsBus(g.Bus) {
				t.Fatal("player boarded outside bay instead of following lead")
			}
			if !lead.Bus.DoorsOpen || tick > 60*15 {
				t.Fatal("player waited for lead departure despite a free rear berth")
			}
			if math.Abs(sim.NormalizeAngle(g.Bus.Heading-stop.Heading)) > .075 {
				t.Fatal("player did not straighten before docking")
			}
			boardedAt = tick
			t.Logf("player boarded behind serving AI at %.2fs pos%v", float64(tick)/60, g.Bus.Pos)
		}
		if boardedAt >= 0 && g.Route.GetCurrentTargetStopID() != stopID && g.Bus.Pos.Sub(stop.Pos).Dot(u) > 400 {
			t.Logf("player departed behind lead at %.2fs pos%v; both buses undamaged", float64(tick)/60, g.Bus.Pos)
			return
		}
	}
	t.Fatalf("player did not board and depart: boarded=%v pos%v speed%.2f target%d", boardedAt >= 0, g.Bus.Pos, g.Bus.Speed, g.Route.GetCurrentTargetStopID())
}
