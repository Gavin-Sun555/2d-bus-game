package game

import (
	"bytes"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func autosaveTestGame(t *testing.T, mode PlayMode) *Game {
	t.Helper()
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	if mode == ModeCareer {
		g.SelectMode(mode)
	}
	g.EnterCity(sim.MapThemeMetropolis)
	return g
}
func setAutosaveCash(g *Game, cash int) {
	g.Cash = cash
	if g.isCareer() {
		g.Career.Balance = float64(cash)
	}
}
func TestAutosaveRotationIsolationAndRestore(t *testing.T) {
	for _, mode := range []PlayMode{ModeFast, ModeCareer} {
		t.Run(mode.String(), func(t *testing.T) {
			g := autosaveTestGame(t, mode)
			setAutosaveCash(g, 777)
			if err := g.SaveGameSlotWithName(7, "手动珍藏"); err != nil {
				t.Fatal(err)
			}
			manual, _ := g.saveSlotPath(7)
			original, _ := os.ReadFile(manual)
			header, err := savefile.Read[SaveMetadata](manual)
			if err != nil || header.Metadata == nil || header.Metadata.Mode != mode || header.Metadata.ModeName != mode.String() {
				t.Fatal("manual mode metadata missing", err)
			}
			base := time.Date(2026, 11, 4, 20, 0, 0, 0, time.UTC)
			for i := 0; i < 8; i++ {
				setAutosaveCash(g, 1000+i)
				if err := g.AutoSave("interval", base.Add(time.Duration(i)*AutoSaveInterval)); err != nil {
					t.Fatal(err)
				}
			}
			autos, err := g.AutoSaves()
			if err != nil || len(autos) != 5 {
				t.Fatal("retention", len(autos), err)
			}
			for rank, info := range autos {
				if info.Metadata == nil || info.Metadata.Mode != mode || info.Metadata.Map != sim.MapThemeMetropolis || info.Metadata.Source != "auto" {
					t.Fatal("missing autosave metadata", info)
				}
				if err := g.LoadAutoSave(info.Slot); err != nil {
					t.Fatal(err)
				}
				if g.Cash != 1007-rank || g.Mode != mode || g.activeSaveSlot != 7 {
					t.Fatal("wrong chronological restore or manual slot changed", rank, g.Cash, g.activeSaveSlot)
				}
			}
			data, _ := os.ReadFile(manual)
			if !bytes.Equal(original, data) {
				t.Fatal("autosaving changed manual slot")
			}
			other := ModeCareer
			if mode == ModeCareer {
				other = ModeFast
			}
			g.SelectMode(other)
			infos, err := g.AutoSaves()
			if err != nil || len(infos) != 0 {
				t.Fatal("autosaves leaked across modes", err)
			}
		})
	}
}
func TestAutosaveFiveMinuteClockAndFailurePreservesGenerations(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.EnableAutosaves()
	start := time.Now()
	g.lastAutosaveAt = start
	g.updateAutosave(start.Add(AutoSaveInterval - time.Second))
	s, _ := g.AutoSaves()
	if len(s) != 0 {
		t.Fatal("saved too early")
	}
	for i := 1; i <= 5; i++ {
		g.updateAutosave(start.Add(time.Duration(i) * AutoSaveInterval))
		g.finishIntervalAutosave(true)
	}
	s, _ = g.AutoSaves()
	if len(s) != 5 {
		t.Fatal("five minute clock", len(s))
	}
	before := map[string][]byte{}
	for i := 0; i < 5; i++ {
		path, _ := g.autoSavePath(i)
		before[path], _ = os.ReadFile(path)
	}
	g.Bus.Speed = math.NaN()
	failureTime := start.Add(6 * AutoSaveInterval)
	g.updateAutosave(failureTime)
	g.finishIntervalAutosave(true)
	for path, want := range before {
		got, _ := os.ReadFile(path)
		if !bytes.Equal(want, got) {
			t.Fatal("failed save destroyed generation")
		}
	}
	g.Bus.Speed = 0
	g.updateAutosave(failureTime.Add(10 * time.Second))
	if !g.lastAutosaveAt.Equal(start.Add(5 * AutoSaveInterval)) {
		t.Fatal("retry was not throttled")
	}
	g.updateAutosave(failureTime.Add(31 * time.Second))
	g.finishIntervalAutosave(true)
	if !g.lastAutosaveAt.Equal(failureTime.Add(31 * time.Second)) {
		t.Fatal("retry did not recover")
	}
	// Do not rotate a file while the player is choosing which autosave to load.
	g.openSavePicker(true)
	g.setSavePickerSource(true)
	last := g.lastAutosaveAt
	g.updateAutosave(last.Add(AutoSaveInterval))
	if !g.lastAutosaveAt.Equal(last) {
		t.Fatal("rotated an open load list")
	}
}
func TestAutosaveExitSavesBothSessionsExactlyOnce(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.EnableAutosaves()
	setAutosaveCash(g, 123)
	g.SelectMode(ModeCareer)
	g.Career.DriverName = "退出生涯"
	g.careerChanged = true // Still on the career city menu.
	g.SelectedMapTheme = sim.MapThemeCoastalBay
	if err := g.SaveOnExit(); err != nil {
		t.Fatal(err)
	}
	if g.State != StateMainMenu || g.SelectedMapTheme != sim.MapThemeCoastalBay {
		t.Fatal("exit save changed menu state or selection")
	}
	if err := g.SaveOnExit(); err != nil {
		t.Fatal(err)
	}
	for mode := ModeFast; mode <= ModeCareer; mode++ {
		target := g
		if mode != g.Mode {
			target = g.modeSessions[mode]
		}
		saves, err := target.AutoSaves()
		if err != nil || len(saves) != 1 {
			t.Fatal("exit duplicated or missed a mode", mode, len(saves), err)
		}
		if saves[0].Metadata.Reason != "exit" {
			t.Fatal("exit reason missing")
		}
		if err = target.LoadAutoSave(saves[0].Slot); err != nil {
			t.Fatal(err)
		}
		if mode == ModeFast && target.Cash != 123 {
			t.Fatal("wrong Fast snapshot")
		}
		if mode == ModeCareer && target.Career.DriverName != "退出生涯" {
			t.Fatal("wrong Career snapshot")
		}
	}
}
func TestSaveMetadataAndLegacyHeader(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.EnterCity(sim.MapThemeCoastalBay)
	g.Settings.SetCenterBRT(sim.MapThemeCoastalBay, true)
	g.EnterCity(sim.MapThemeCoastalBay)
	if err := g.SaveGameSlotWithName(0, "海滨线路"); err != nil {
		t.Fatal(err)
	}
	path, _ := g.savePath()
	envelope, err := savefile.Read[SaveMetadata](path)
	if err != nil {
		t.Fatal(err)
	}
	if envelope.Metadata == nil || envelope.Metadata.ModeName != "Fast" || envelope.Metadata.Map != sim.MapThemeCoastalBay || !envelope.Metadata.CenterBRT || envelope.Metadata.MapName == "" || envelope.Metadata.Source != "manual" {
		t.Fatal("metadata not persisted", envelope.Metadata)
	}
	envelope.Metadata = nil
	raw, _ := json.Marshal(envelope)
	if err = os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.Chtimes(path, time.Now().Add(time.Hour), time.Now().Add(time.Hour)); err != nil {
		t.Fatal(err)
	}
	slots, err := g.SaveSlots()
	if !slots[0].SavedAt.Equal(envelope.SavedAt) {
		t.Fatal("copied file time replaced actual save time")
	}
	if err != nil || slots[0].Metadata == nil || slots[0].Metadata.Map != sim.MapThemeCoastalBay || slots[0].Metadata.Mode != ModeFast {
		t.Fatal("legacy metadata fallback", err)
	}
	if err = g.LoadGame(); err != nil {
		t.Fatal("legacy file no longer loads", err)
	}
	if err = g.RenameSaveSlot(0, "改名"); err != nil {
		t.Fatal(err)
	}
	slots, _ = g.SaveSlots()
	if slots[0].Name != "改名" || slots[0].Metadata.Map != sim.MapThemeCoastalBay {
		t.Fatal("rename lost metadata")
	}
}
func TestAutosavePickerLoadUsesSelectedFileAndLeavesManualTarget(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	now := time.Now()
	for i := 0; i < 3; i++ {
		g.Cash = 40 + i
		if err := g.AutoSave("interval", now.Add(time.Duration(i)*time.Minute)); err != nil {
			t.Fatal(err)
		}
	}
	g.activeSaveSlot = 8
	g.openSavePicker(true)
	g.setSavePickerSource(true)
	if !g.savePicker.automatic || g.savePicker.slots[0].Metadata == nil {
		t.Fatal("auto tab not populated")
	}
	g.savePicker.selected = 1
	g.drawSavePicker(ebiten.NewImage(1280, 720))
	g.loadSelectedAutoSave()
	if g.Cash != 41 || g.activeSaveSlot != 8 || g.savePicker.open {
		t.Fatal("auto picker loaded wrong snapshot")
	}
	g.openSavePicker(true)
	g.setSavePickerSource(false)
	if g.savePicker.slots[0].Exists {
		t.Fatal("auto entry displayed as a manual slot")
	}
}
func TestAutosaveNeverWritesBeforePlayOrWhenDisabled(t *testing.T) {
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	g.ShowModeSelection()
	g.EnableAutosaves()
	g.updateAutosave(time.Now().Add(10 * time.Minute))
	if err := g.SaveOnExit(); err != nil {
		t.Fatal(err)
	}
	s, _ := g.AutoSaves()
	if len(s) != 0 {
		t.Fatal("empty startup game was autosaved")
	}
	g.SelectMode(ModeCareer)
	g.lastAutosaveAt = time.Now().Add(-time.Hour)
	g.updateAutosave(time.Now())
	if err := g.SaveOnExit(); err != nil {
		t.Fatal(err)
	}
	s, _ = g.AutoSaves()
	if len(s) != 0 {
		t.Fatal("unstarted Career menu overwrote old autosaves")
	}
	g = autosaveTestGame(t, ModeFast)
	g.lastAutosaveAt = time.Now().Add(-time.Hour)
	g.updateAutosave(time.Now())
	s, _ = g.AutoSaves()
	if len(s) != 0 {
		t.Fatal("headless/test game silently wrote autosaves")
	}
}

func TestAutosaveExitFailureCanRetry(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.EnableAutosaves()
	g.Bus.Speed = math.NaN()
	if err := g.SaveOnExit(); err == nil || g.exitAutosaved {
		t.Fatal("failed exit save was reported as complete")
	}
	g.Bus.Speed = 0
	if err := g.SaveOnExit(); err != nil {
		t.Fatal(err)
	}
	saves, err := g.AutoSaves()
	if err != nil || len(saves) != 1 || !g.exitAutosaved {
		t.Fatal("exit retry failed", err)
	}
}

func TestAutosaveCareerMenuRetainsJourneyWhenDispatchUnchanged(t *testing.T) {
	g := autosaveTestGame(t, ModeCareer)
	g.Bus.Pos.X += 123
	g.Route.CurrentIndex = 1
	pos := g.Bus.Pos
	g.State = StateMainMenu
	if err := g.AutoSave("interval", time.Now()); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Pos != pos || g.Route.CurrentIndex != 1 || g.State != StateMainMenu {
		t.Fatal("autosave reset the paused journey")
	}
	saves, _ := g.AutoSaves()
	if err := g.LoadAutoSave(saves[0].Slot); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Pos != pos || g.Route.CurrentIndex != 1 {
		t.Fatal("auto restore lost the paused journey")
	}
}

func TestIntervalAutosaveCapturesStateBeforeSimulationContinues(t *testing.T) {
	for _, mode := range []PlayMode{ModeFast, ModeCareer} {
		t.Run(mode.String(), func(t *testing.T) {
			g := autosaveTestGame(t, mode)
			g.EnableAutosaves()
			g.Bus.BoardPassengers(1, []int{g.Route.StopIDs[1]})
			g.World.Stops[0].WaitingDestinations = []int{g.Route.StopIDs[1]}
			want, err := json.Marshal(g.snapshot())
			if err != nil {
				t.Fatal(err)
			}
			g.updateAutosave(g.lastAutosaveAt.Add(AutoSaveInterval))
			if g.pendingAutosave == nil {
				t.Fatal("interval save was not queued")
			}

			g.Bus.Passengers[0].TargetStopID++
			g.World.Stops[0].WaitingDestinations[0]++
			g.World.Clock.ElapsedMinutes++
			g.Route.StopIDs[0]++
			g.TrafficLights.Intersections[0].Timer++
			if g.isCareer() {
				g.Career.Vehicles[0].Condition--
			}
			g.finishIntervalAutosave(true)
			path, err := g.autoSavePath(0)
			if err != nil {
				t.Fatal(err)
			}
			saved, err := savefile.Read[SaveMetadata](path)
			if err != nil {
				t.Fatal(err)
			}
			if !bytes.Equal(saved.Data, want) {
				t.Fatal("background save changed after the live state advanced")
			}
		})
	}
}
