package game

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/hajimehoshi/ebiten/v2"
)

func newCareerGame(t *testing.T) *Game {
	t.Helper()
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	g.SelectMode(ModeCareer)
	return g
}
func TestCareerModeSessionsAndIndependentSlots(t *testing.T) {
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	g.EnterCity(sim.MapThemeMetropolis)
	g.Cash = 1234
	if err := g.SaveGameSlotWithName(0, "Fast origin"); err != nil {
		t.Fatal(err)
	}
	fastPath, _ := g.saveSlotPath(0)
	fastBytes, _ := os.ReadFile(fastPath)
	g.SelectMode(ModeCareer)
	if g.Career == nil || g.Cash != sim.CareerStartingCash || len(g.Career.Vehicles) != 1 {
		t.Fatal("bad starter")
	}
	slots, err := g.SaveSlots()
	if err != nil || slots[0].Exists {
		t.Fatal("Fast slot leaked into Career")
	}
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.State == StatePlaying {
		t.Fatal("locked city entered")
	}
	g.EnterCity(sim.MapThemeMetropolis)
	if len(g.AIBusMgr.Buses) != 0 || g.Bus.Model != sim.BusStandard || g.Bus.CatalogID != "xingtu-s40" {
		t.Fatal("free unowned fleet")
	}
	g.Career.DriverName = "独立生涯"
	if err := g.SaveGameSlotWithName(0, "Career origin"); err != nil {
		t.Fatal(err)
	}
	careerPath, _ := g.saveSlotPath(0)
	if careerPath == fastPath {
		t.Fatal("shared save path")
	}
	if data, _ := os.ReadFile(fastPath); string(data) != string(fastBytes) {
		t.Fatal("Career overwrote Fast")
	}
	g.SelectMode(ModeFast)
	if g.Cash != 1234 || g.Career != nil {
		t.Fatal("Career state leaked into Fast")
	}
	if err := g.LoadGameSlot(0); err != nil || g.Cash != 1234 {
		t.Fatal("Fast load failed", err)
	}
	g.SelectMode(ModeCareer)
	if g.Career.DriverName != "独立生涯" {
		t.Fatal("lost in-memory profile")
	}
	if err := g.LoadGameSlot(0); err != nil || g.Career.DriverName != "独立生涯" {
		t.Fatal("Career load failed", err)
	}
	// Copying a valid save into the wrong folder still cannot cross-load modes.
	if err := os.WriteFile(careerPath, fastBytes, 0600); err != nil {
		t.Fatal(err)
	}
	prior := g.Career
	if err := g.LoadGameSlot(0); err == nil || g.Career != prior {
		t.Fatal("cross-mode save accepted or damaged session")
	}
	if err := g.DeleteSaveSlot(0); err != nil {
		t.Fatal(err)
	}
	if data, _ := os.ReadFile(fastPath); string(data) != string(fastBytes) {
		t.Fatal("Career delete changed Fast")
	}
}
func TestCareerOwnedFleetAccountingAndPersistence(t *testing.T) {
	g := newCareerGame(t)
	g.Career.Balance = 6000000
	g.Cash = 6000000
	ids := []int{}
	for _, spec := range []string{"yuanchuan-s70", "xingtu-a100", "yuanchuan-d140"} {
		id, err := g.Career.Buy(spec)
		if err != nil {
			t.Fatal(err)
		}
		ids = append(ids, id)
		if err = g.assignCareerVehicle(id, 0); err != nil {
			t.Fatal(err)
		}
	}
	g.Cash = int(g.Career.Balance)
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.World.Theme != sim.MapThemeCoastalBay || len(g.AIBusMgr.Buses) != 3 {
		t.Fatal("owned dispatch failed")
	}
	seen := map[int]bool{g.Bus.OwnedVehicleID: true}
	for _, a := range g.AIBusMgr.Buses {
		if seen[a.Bus.OwnedVehicleID] || g.Career.Vehicle(a.Bus.OwnedVehicleID) == nil {
			t.Fatal("duplicate or unowned service bus")
		}
		seen[a.Bus.OwnedVehicleID] = true
	}
	// A kilometre for each dispatched vehicle charges its own tariff, once.
	balance := g.Career.Balance
	cost := 0.
	for _, b := range g.careerLiveBuses() {
		b.Speed = 1000 / sim.MetersPerPixel
		s, _ := sim.FindVehicleSpec(b.CatalogID)
		cost += s.CostPerKM
	}
	g.settleCareer(1, g.Cash, g.careerBoarded())
	if math.Abs(g.Career.Balance-(balance-cost)) > 1e-7 || math.Abs(g.Career.TotalKM-4) > 1e-7 {
		t.Fatal("incorrect operating ledger", g.Career.Balance, balance-cost)
	}
	for _, v := range g.Career.Vehicles {
		if v.KM != 1 || math.Abs(v.Condition-99.985) > 1e-8 {
			t.Fatal("individual odometer/condition", v)
		}
	}
	// Free-mode fleet controls cannot conjure vehicles in Career.
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, PlayerModel: sim.BusArticulated})
	g.Settings.AIBusCount = 8
	g.Settings.PlayerModel = sim.BusArticulated
	g.syncCurrentLineSettings()
	if g.LineFleets[0].BusCount != 3 || g.Bus.Model != sim.BusStandard {
		t.Fatal("free fleet bypass")
	}
	for _, b := range g.careerLiveBuses() {
		b.Speed = 0
	}
	g.Bus.Durability = 90
	g.syncCareerCondition()
	quote, _ := sim.FindVehicleSpec(g.Bus.CatalogID)
	before := g.Career.Balance
	if err := g.repairCareerVehicle(g.Bus.OwnedVehicleID); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Durability != 100 || g.Career.Balance != before-float64(quote.RepairQuote(90)) {
		t.Fatal("repair not charged")
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	saved, _ := json.Marshal(g.Career)
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	restored, _ := json.Marshal(g.Career)
	if string(saved) != string(restored) {
		t.Fatal("career ledger changed on load")
	}
	km, cash := g.Career.TotalKM, g.Career.Balance
	g.EnterCity(sim.MapThemeMetropolis)
	if g.Career.TotalKM != km || g.Career.Balance != cash || len(g.AIBusMgr.Buses) != 3 {
		t.Fatal("city change reset company")
	}
}
func TestCareerGarageSaveAndDispatchRestrictions(t *testing.T) {
	g := newCareerGame(t)
	g.openSavePicker(false)
	if !g.savePicker.open {
		t.Fatal("Career menu cannot open save picker")
	}
	g.savePicker.open = false
	g.Career.Balance = 5000000
	g.Cash = 5000000
	id, err := g.Career.Buy("xingtu-a100")
	if err != nil {
		t.Fatal(err)
	}
	if err = g.driveCareerVehicle(id); err != nil {
		t.Fatal(err)
	}
	g.Cash = int(g.Career.Balance)
	if err = g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if g.State != StateMainMenu {
		t.Fatal("garage save started gameplay")
	}
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Model != sim.BusArticulated || g.Bus.CatalogID != "xingtu-a100" {
		t.Fatal("selected player bus not restored")
	}
	if err = g.driveCareerVehicle(1); err == nil {
		t.Fatal("swapped active vehicle mid-drive")
	}
	g.Bus.Speed = 20
	g.Bus.Durability = 50
	if err = g.repairCareerVehicle(id); err == nil {
		t.Fatal("repair while moving")
	}
	// Achievements are independent of changing city and save slots.
	g.Bus.Speed = 0
	g.State = StateMainMenu
	g.Career.TotalKM = 10
	g.performCareerAction(careerButton{action: "claim", value: "km-10"})
	if !g.Career.Achievements["km-10"] {
		t.Fatal("claim failed")
	}
	if err = g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err = g.LoadGame(); err != nil || !g.Career.Achievements["km-10"] {
		t.Fatal("claim lost", err)
	}
}

func TestCareerSimulationEconomyResume(t *testing.T) {
	for _, specID := range []string{"xingtu-s40", "xingtu-a100", "xingtu-d100"} {
		for _, brt := range []bool{false, true} {
			t.Run(specID+map[bool]string{false: "/curb", true: "/brt"}[brt], func(t *testing.T) {
				sim.SeedSimulation(553)
				g := newCareerGame(t)
				g.Career.Balance = 20000000
				playerID := 1
				for _, s := range []string{"yuanchuan-s70", "xingtu-a100", "xingtu-d100"} {
					id, err := g.Career.Buy(s)
					if err != nil {
						t.Fatal(err)
					}
					if s == specID {
						playerID = id
					}
				}
				if err := g.driveCareerVehicle(playerID); err != nil {
					t.Fatal(err)
				}
				for _, v := range g.Career.Vehicles {
					if v.ID != playerID {
						if err := g.assignCareerVehicle(v.ID, 0); err != nil {
							t.Fatal(err)
						}
					}
				}
				g.Cash = int(g.Career.Balance)
				initialBalance := g.Career.Balance
				g.Settings.SetCenterBRT(sim.MapThemeMetropolis, brt)
				g.Settings.TrafficDensity = 1
				g.Settings.AutopilotEnabled = true
				g.Settings.AutopilotBerserk = true
				g.Settings.AutopilotBerserkMaxSpeed = 120
				g.Settings.TrafficFinesEnabled = false
				g.EnterCity(sim.MapThemeMetropolis)
				for tick := 0; tick < 3600; tick++ {
					if err := g.Update(); err != nil {
						t.Fatal(err)
					}
					for _, b := range g.careerLiveBuses() {
						v := g.Career.Vehicle(b.OwnedVehicleID)
						s, _ := sim.FindVehicleSpec(v.SpecID)
						if math.Abs(v.Condition-(100-v.KM*.015)) > 1e-5 {
							t.Fatalf("unexpected damage tick%d bus%d condition%.8f km%.5f", tick, v.ID, v.Condition, v.KM)
						}
						if b.Speed > sim.SpeedKPH(int(s.TopSpeedKPH))+1e-6 {
							t.Fatal("hardware speed limit bypass")
						}
					}
					if tick == 1799 {
						if err := g.SaveGame(); err != nil {
							t.Fatal(err)
						}
						before, _ := json.Marshal(g.Career)
						if err := g.LoadGame(); err != nil {
							t.Fatal(err)
						}
						after, _ := json.Marshal(g.Career)
						if string(before) != string(after) {
							t.Fatal("load changed career ledger")
						}
					}
				}
				if g.Career.TotalKM <= 0 || g.Career.OperatingCost <= 0 || g.Career.Passengers <= 0 {
					t.Fatal("inactive economy", g.Career)
				}
				if g.Career.Passengers != g.careerBoarded() || g.Career.Revenue != float64(g.Operations.Revenue) {
					t.Fatal("fares/boardings charged twice or lost")
				}
				expected := initialBalance + g.Career.Revenue - g.Career.OperatingCost
				if math.Abs(expected-g.Career.Balance) > 1e-5 {
					t.Fatal("ledger does not reconcile", expected, g.Career.Balance)
				}
				t.Logf("60s BRT%v %s: km%.3f pax%d revenue%.0f costs%.3f fleet%d", brt, specID, g.Career.TotalKM, g.Career.Passengers, g.Career.Revenue, g.Career.OperatingCost, len(g.AIBusMgr.Buses))
			})
		}
	}
}

func TestCareerInvalidSaveIsTransactional(t *testing.T) {
	g := newCareerGame(t)
	g.EnterCity(sim.MapThemeMetropolis)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	path, _ := g.savePath()
	good, _ := os.ReadFile(path)
	changes := []func(*gameSave){
		func(s *gameSave) { s.Career = nil },
		func(s *gameSave) { s.Career.Vehicles[0].SpecID = "unknown" },
		func(s *gameSave) {
			s.Career.Vehicles = append(s.Career.Vehicles, s.Career.Vehicles[0])
			s.Career.PeakFleet = 2
		},
		func(s *gameSave) { s.Player.Bus.OwnedVehicleID = 999 },
		func(s *gameSave) { s.Player.Bus.CatalogID = "lingfeng-s100" },
		func(s *gameSave) { s.Career.Balance += 1 },
		func(s *gameSave) { s.Mode = ModeFast },
	}
	for _, change := range changes {
		if err := os.WriteFile(path, good, 0600); err != nil {
			t.Fatal(err)
		}
		s, err := readSave(path)
		if err != nil {
			t.Fatal(err)
		}
		change(&s)
		payload, err := json.Marshal(s)
		if err != nil {
			t.Fatal(err)
		}
		digest := sha256.Sum256(payload)
		raw, err := json.Marshal(savefile.Envelope[SaveMetadata]{Version: savefile.Version, Checksum: hex.EncodeToString(digest[:]), Data: payload})
		if err != nil {
			t.Fatal(err)
		}
		if err = os.WriteFile(path, raw, 0600); err != nil {
			t.Fatal(err)
		}
		world, bus, profile, active := g.World, g.Bus, g.Career, sim.ActiveWorldForRoute
		if err = g.LoadGame(); err == nil {
			t.Fatal("invalid career save loaded")
		}
		if g.World != world || g.Bus != bus || g.Career != profile || sim.ActiveWorldForRoute != active {
			t.Fatal("bad save damaged current session")
		}
	}
}

func TestCareerMultiMapProgressAndMainMenuSaveLoad(t *testing.T) {
	g := newCareerGame(t)
	g.Career.DriverName = "全局进度车长"
	g.Career.Balance = 8000000
	g.Cash = 8000000

	// 1. Enter Metropolis and make city-specific progress (custom route, position, operations, stop queue).
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(2)
	metroCustomStops := []int{g.World.Stops[0].ID, g.World.Stops[2].ID, g.World.Stops[4].ID}
	g.Lines[1].StopIDs = append([]int(nil), metroCustomStops...)
	g.Lines[1].CurrentIndex = 1
	g.Bus.Pos.X += 321
	g.Bus.Pos.Y -= 123
	metroBusPos := g.Bus.Pos
	g.Operations.Revenue = 540
	g.careerRevenueMark = 540
	g.World.Stops[0].WaitingPassengers = 19

	// 2. Return to Career Main Menu, buy buses to unlock Coastal Bay & River Valley, dispatch to Line 1 & Line 2, claim achievement.
	g.Settings.ReturnToMenuRequested = true
	if err := g.Update(); err != nil {
		t.Fatal(err)
	}
	if g.State != StateMainMenu {
		t.Fatal("did not return to Career main menu")
	}
	for i, spec := range []string{"yuanchuan-s70", "xingtu-a100", "yuanchuan-d140", "lingfeng-s100"} {
		id, err := g.Career.Buy(spec)
		if err != nil {
			t.Fatal(err)
		}
		if err = g.assignCareerVehicle(id, i%2); err != nil {
			t.Fatal(err)
		}
	}
	g.Cash = int(g.Career.Balance)
	if !g.Career.MapUnlocked(sim.MapThemeCoastalBay) || !g.Career.MapUnlocked(sim.MapThemeRiverValley) {
		t.Fatal("expected Coastal Bay and River Valley to be unlocked")
	}
	g.performCareerAction(careerButton{action: "claim", value: "fleet-3"})
	if !g.Career.Achievements["fleet-3"] {
		t.Fatal("expected fleet-3 achievement claimed")
	}

	// 3. Enter Coastal Bay and make distinct progress there.
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.World.Theme != sim.MapThemeCoastalBay || len(g.AIBusMgr.Buses) != 4 {
		t.Fatalf("expected Coastal Bay with 4 AI buses, got theme=%v buses=%d", g.World.Theme, len(g.AIBusMgr.Buses))
	}
	coastalCustomStops := []int{g.World.Stops[1].ID, g.World.Stops[3].ID, g.World.Stops[5].ID}
	g.Lines[0].StopIDs = append([]int(nil), coastalCustomStops...)
	g.Lines[0].CurrentIndex = 2
	g.Bus.Pos.X -= 222
	g.Bus.Pos.Y += 111
	coastalBusPos := g.Bus.Pos
	g.Operations.Revenue = 880
	g.careerRevenueMark = 880
	g.World.Stops[1].WaitingPassengers = 27

	// 4. Return to Career Main Menu and save the entire Career progress into slot 2.
	g.HUD.ReturnToMenuRequested = true
	if err := g.Update(); err != nil {
		t.Fatal(err)
	}
	if err := g.SaveGameSlotWithName(2, "全图生涯存档"); err != nil {
		t.Fatal(err)
	}
	slots, err := g.SaveSlots()
	if err != nil || !slots[2].Exists || slots[2].Metadata == nil {
		t.Fatal("missing Career slot metadata", err)
	}
	if slots[2].Metadata.OwnedBuses != 5 || slots[2].Metadata.UnlockedCities != 3 || slots[2].Metadata.VisitedCities < 2 {
		t.Fatalf("unexpected Career metadata: %+v", slots[2].Metadata)
	}

	// 5. Load into a fresh Career game via the Main Menu save picker and verify all maps + company progress survive.
	fresh := NewGame(1280, 720)
	fresh.savePathOverride = g.savePathOverride
	fresh.SelectMode(ModeCareer)
	fresh.openSavePicker(true)
	fresh.selectSaveSlot(2)
	fresh.performSlotAction()
	if fresh.State != StateMainMenu {
		t.Fatalf("loading Career save from main menu picker should stay on StateMainMenu, got %v", fresh.State)
	}
	if fresh.Career.DriverName != "全局进度车长" || len(fresh.Career.Vehicles) != 5 || !fresh.Career.Achievements["fleet-3"] || !fresh.Career.MapUnlocked(sim.MapThemeRiverValley) {
		t.Fatalf("Career profile not fully restored: %+v", fresh.Career)
	}

	// Enter Metropolis: verify Metropolis's earlier progress + newly dispatched 4 AI buses are both present.
	fresh.EnterCity(sim.MapThemeMetropolis)
	if fresh.World.Theme != sim.MapThemeMetropolis || fresh.Bus.Pos != metroBusPos || fresh.Operations.Revenue != 540 || fresh.World.Stops[0].WaitingPassengers != 19 {
		t.Fatalf("Metropolis progress lost: pos=%v want=%v rev=%d wait=%d", fresh.Bus.Pos, metroBusPos, fresh.Operations.Revenue, fresh.World.Stops[0].WaitingPassengers)
	}
	if !intSlicesEqual(fresh.Lines[1].StopIDs, metroCustomStops) || fresh.Lines[1].CurrentIndex != 1 || len(fresh.AIBusMgr.Buses) != 4 {
		t.Fatalf("Metropolis lines or reconciled fleet mismatch: stops=%v idx=%d buses=%d", fresh.Lines[1].StopIDs, fresh.Lines[1].CurrentIndex, len(fresh.AIBusMgr.Buses))
	}

	// Switch to Coastal Bay: verify Coastal Bay's progress is also intact.
	fresh.EnterCity(sim.MapThemeCoastalBay)
	if fresh.World.Theme != sim.MapThemeCoastalBay || fresh.Bus.Pos != coastalBusPos || fresh.Operations.Revenue != 880 || fresh.World.Stops[1].WaitingPassengers != 27 {
		t.Fatalf("Coastal Bay progress lost: pos=%v want=%v rev=%d wait=%d", fresh.Bus.Pos, coastalBusPos, fresh.Operations.Revenue, fresh.World.Stops[1].WaitingPassengers)
	}
	if !intSlicesEqual(fresh.Lines[0].StopIDs, coastalCustomStops) || fresh.Lines[0].CurrentIndex != 2 || len(fresh.AIBusMgr.Buses) != 4 {
		t.Fatalf("Coastal Bay lines or fleet mismatch: stops=%v idx=%d buses=%d", fresh.Lines[0].StopIDs, fresh.Lines[0].CurrentIndex, len(fresh.AIBusMgr.Buses))
	}
}

func TestCareerSaveLoadMainMenuOnlyNotInGameSettings(t *testing.T) {
	g := newCareerGame(t)
	screen := ebiten.NewImage(1280, 720)
	defer screen.Dispose()

	// On Career Main Menu, both Save and Load buttons are rendered and openSavePicker works for both.
	g.Draw(screen)
	if g.menuSaveBounds.W <= 0 || g.menuSaveBounds.H <= 0 || g.menuContinueBounds.W <= 0 || g.menuContinueBounds.H <= 0 {
		t.Fatalf("Career main menu missing save/load buttons: save=%+v load=%+v", g.menuSaveBounds, g.menuContinueBounds)
	}
	g.openSavePicker(false)
	if !g.savePicker.open || g.savePicker.load {
		t.Fatal("expected Save picker to open on Career main menu")
	}
	g.savePicker.open = false
	g.openSavePicker(true)
	if !g.savePicker.open || !g.savePicker.load {
		t.Fatal("expected Load picker to open on Career main menu")
	}
	g.savePicker.open = false

	// In-game (StatePlaying) in Career mode: Settings panel hides Save/Load buttons and openSavePicker is disabled.
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.IsOpen = true
	g.Draw(screen)
	if g.Settings.HasSaveLoadButtons() {
		t.Fatal("Career in-game settings panel should not expose Save/Load buttons")
	}
	g.openSavePicker(false)
	if g.savePicker.open {
		t.Fatal("Career mode should not open Save picker during StatePlaying")
	}
	g.openSavePicker(true)
	if g.savePicker.open {
		t.Fatal("Career mode should not open Load picker during StatePlaying")
	}

	// Fast mode in-game Settings still exposes Save/Load buttons.
	g.SelectMode(ModeFast)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.IsOpen = true
	g.Draw(screen)
	if !g.Settings.HasSaveLoadButtons() {
		t.Fatal("Fast mode in-game settings panel should keep Save/Load buttons")
	}
}
