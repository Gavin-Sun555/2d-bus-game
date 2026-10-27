package game

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func saveTestGame(t *testing.T) *Game {
	t.Helper()
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.savePathOverride = filepath.Join(t.TempDir(), "saves", "save.json")
	return g
}
func TestSaveLoadResumesNetwork(t *testing.T) {
	g := saveTestGame(t)
	g.SetTransitLineCount(3)
	for i := 0; i < 3; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 2, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 70}})
	}
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Cash = 876
	g.Bus.BoardPassengers(3, []int{g.Route.StopIDs[1]})
	g.Operations.Vehicle(0).Revenue = 27
	g.Operations.Revenue = 41
	for _, b := range g.AIBusMgr.Buses {
		b.Bus.BoardPassengers(2, []int{b.Route.StopIDs[1]})
	}
	for step := 0; step < 40; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	// Cross-vehicle links must reconnect to the restored live buses, not copies.
	g.Bus.YieldingToBus = g.AIBusMgr.Buses[0].Bus
	g.TrafficMgr.Cars[0].YieldingToBus = g.Bus
	g.TrafficMgr.Cars[0].PassingLeadBus = g.AIBusMgr.Buses[1].Bus
	before, err := json.Marshal(g.snapshot())
	if err != nil {
		t.Fatal(err)
	}
	if err = g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	path := g.savePathOverride
	g = NewGame(1280, 720)
	g.savePathOverride = path
	if g.State != StateMainMenu {
		t.Fatal("fresh game did not start at menu")
	}
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	after, err := json.Marshal(g.snapshot())
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(before, after) {
		os.WriteFile(filepath.Join(t.TempDir(), "before.json"), before, 0600)
		t.Fatal("saved state differs after load")
	}
	if g.Bus.YieldingToBus != g.AIBusMgr.Buses[0].Bus || g.TrafficMgr.Cars[0].YieldingToBus != g.Bus || g.TrafficMgr.Cars[0].PassingLeadBus != g.AIBusMgr.Buses[1].Bus {
		t.Fatal("vehicle references not reconnected")
	}
	if g.AIBusMgr.Operations != g.Operations || g.TrafficMgr.AIBusMgr != g.AIBusMgr || g.Operations.Vehicle(0).Revenue < 27 {
		t.Fatal("lost statistics wiring")
	}
	for step := 0; step < 120; step++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if err = g.SaveGame(); err != nil {
		t.Fatal(err)
	}
}
func TestSaveLoadPlanningAndCustomMap(t *testing.T) {
	g := saveTestGame(t)
	g.EnterCity(sim.MapThemeCustom)
	g.SetTransitLineCount(2)
	g.PlanTransitLine(1)
	g.ClearPlannedRoute()
	g.HideOtherRoutes = true
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if !g.IsPlanMode || g.planningRoute() != g.Lines[1] || len(g.Lines[1].StopIDs) != 0 || g.Route != g.Lines[0] || !g.HideOtherRoutes || g.World.CustomMap == nil {
		t.Fatal("planning or embedded custom map not restored")
	}
	g.planningRoute().AddStop(0)
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("could not finish restored route")
	}
}
func TestInvalidSaveLeavesCurrentGameUntouched(t *testing.T) {
	g := saveTestGame(t)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	good, err := os.ReadFile(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	originalWorld, originalBus, originalCash := g.World, g.Bus, g.Cash
	var envelope saveEnvelope
	if err = json.Unmarshal(good, &envelope); err != nil {
		t.Fatal(err)
	}
	envelope.Version++
	future, _ := json.Marshal(envelope)
	for _, raw := range [][]byte{[]byte("{"), future, bytes.Replace(good, []byte(`"checksum":"`), []byte(`"checksum":"wrong`), 1)} {
		if err = os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
			t.Fatal(err)
		}
		if err = g.LoadGame(); err == nil {
			t.Fatal("accepted damaged or future save")
		}
		if g.World != originalWorld || g.Bus != originalBus || g.Cash != originalCash {
			t.Fatal("failed load mutated game")
		}
	}
	os.Remove(g.savePathOverride)
	if err = g.LoadGame(); err == nil {
		t.Fatal("missing save accepted")
	}
	if err = g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	old, err := os.ReadFile(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	g.Bus.Speed = math.Inf(1)
	if err = g.SaveGame(); err == nil {
		t.Fatal("nonfinite state saved")
	}
	preserved, err := os.ReadFile(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(old, preserved) {
		t.Fatal("failed save destroyed previous slot")
	}
}

func TestSaveSemanticValidationIsTransactional(t *testing.T) {
	g := saveTestGame(t)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	good, err := os.ReadFile(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	for _, change := range []func(*gameSave){
		func(s *gameSave) { s.World = nil },
		func(s *gameSave) { s.Lines[0].StopIDs[0] = -1 },
		func(s *gameSave) { s.Player.Bus.PassengerCount = 5 },
		func(s *gameSave) { s.Player.Driver.Public.WaypointIdx = -1 },
		func(s *gameSave) { s.PlayerLine = 6 },
	} {
		if err = os.WriteFile(g.savePathOverride, good, 0600); err != nil {
			t.Fatal(err)
		}
		s, err := readSave(g.savePathOverride)
		if err != nil {
			t.Fatal(err)
		}
		change(&s)
		payload, err := json.Marshal(s)
		if err != nil {
			t.Fatal(err)
		}
		digest := sha256.Sum256(payload)
		raw, err := json.Marshal(saveEnvelope{Version: saveVersion, Checksum: hex.EncodeToString(digest[:]), Data: payload})
		if err != nil {
			t.Fatal(err)
		}
		if err = os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
			t.Fatal(err)
		}
		world, bus, active := g.World, g.Bus, sim.ActiveWorldForRoute
		if err = g.LoadGame(); err == nil {
			t.Fatal("accepted invalid saved state")
		}
		if g.World != world || g.Bus != bus || sim.ActiveWorldForRoute != active {
			t.Fatal("rejected save changed current game")
		}
	}
}

func TestSaveLoadAllCityThemes(t *testing.T) {
	for _, theme := range sim.BuiltinMapThemes() {
		t.Run(theme.ShortName(), func(t *testing.T) {
			g := saveTestGame(t)
			g.EnterCity(theme)
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			stops := len(g.World.Stops)
			g.EnterCity(sim.MapThemeCustom)
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			if g.World.Theme != theme || len(g.World.Stops) != stops {
				t.Fatal("wrong city restored")
			}
			expected := sim.BuildRouteDistances(g.World, g.Route, g.Settings.GetLaneWidth())
			if g.Operations.Distances.TotalKM != expected.TotalKM {
				t.Fatal("distance table retained previous city")
			}
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestSaveLoadDoesNotRepeatPassengerExchange(t *testing.T) {
	g := saveTestGame(t)
	g.Bus.DoorsOpen = true
	g.updatePassengerExchange(false)
	cash, revenue, passengers := g.Cash, g.Operations.Revenue, g.Bus.PassengerCount
	if passengers == 0 {
		t.Fatal("fixture did not board passengers")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	g.updatePassengerExchange(false)
	if g.Cash != cash || g.Operations.Revenue != revenue || g.Bus.PassengerCount != passengers {
		t.Fatal("loading an open-door bus repeated boarding or fares")
	}
}

func TestLoadLegacyTrafficPopulationKeepsTransitState(t *testing.T) {
	for _, pending := range []bool{false, true} {
		t.Run(fmt.Sprint(pending), func(t *testing.T) {
			g := saveTestGame(t)
			g.Settings.TrafficDensity = 4
			count := maxCompatibleSavedCars
			if pending {
				count = 800
			}
			g.TrafficMgr.BuildForWorld(g.World, count, g.Settings.GetLaneWidth())
			population := g.TrafficMgr.PopulationSnapshot()
			population.NextCarID = 2000
			if pending {
				population.PendingExits = make([]sim.Vec2, maxCompatibleSavedCars-count)
			}
			g.TrafficMgr.RestorePopulation(population)
			g.Cash = 13579
			g.Bus.BoardPassengers(3, []int{g.Route.StopIDs[1]})
			pos, heading := g.Bus.Pos, g.Bus.Heading
			firstID := g.TrafficMgr.Cars[0].ID
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			s := g.TrafficMgr.PopulationSnapshot()
			if len(g.TrafficMgr.Cars)+len(s.PendingExits) != sim.MaxTrafficCars {
				t.Fatal("legacy population not adapted to new limit")
			}
			if g.Cash != 13579 || g.Bus.PassengerCount != 3 || g.Bus.Pos != pos || g.Bus.Heading != heading || g.TrafficMgr.Cars[0].ID != firstID || s.NextCarID != 2000 {
				t.Fatal("population migration changed transit state or stable IDs")
			}
			for step := 0; step < 20; step++ {
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
			}
			if len(g.TrafficMgr.GetActiveCars(4)) > 864 {
				t.Fatal("population exceeded new capacity after load")
			}
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
		})
	}
}
