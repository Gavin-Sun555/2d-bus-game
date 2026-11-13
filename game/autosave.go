package game

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
)

const MaxAutoSaves = 5
const AutoSaveInterval = 5 * time.Minute

func (g *Game) EnableAutosaves() { g.autosavesEnabled = true; g.lastAutosaveAt = time.Now() }
func (g *Game) canAutosave() bool {
	return g.sessionStarted || g.State == StatePlaying || (g.isCareer() && g.careerChanged)
}
func (g *Game) autoSavePath(slot int) (string, error) {
	if slot < 0 || slot >= MaxAutoSaves {
		return "", fmt.Errorf("invalid autosave slot")
	}
	manual, err := g.saveSlotPath(0)
	if err != nil {
		return "", err
	}
	stem := strings.TrimSuffix(filepath.Base(manual), filepath.Ext(manual))
	return filepath.Join(filepath.Dir(manual), "autosaves", fmt.Sprintf("%s-auto-%02d.json", stem, slot+1)), nil
}

// Physical slots stay fixed, avoiding a rename chain that could lose multiple
// generations if a write fails. The UI sorts these slots newest first.
func (g *Game) AutoSaves() ([]SaveSlotInfo, error) {
	g.finishIntervalAutosave(true)
	paths, err := g.autoSavePaths()
	if err != nil {
		return nil, err
	}
	return readAutoSaves(paths, true)
}

func (g *Game) autoSavePaths() ([MaxAutoSaves]string, error) {
	var paths [MaxAutoSaves]string
	for slot := range paths {
		path, err := g.autoSavePath(slot)
		if err != nil {
			return paths, err
		}
		paths[slot] = path
	}
	return paths, nil
}

func readAutoSaves(paths [MaxAutoSaves]string, includeDetails bool) ([]SaveSlotInfo, error) {
	var saves []SaveSlotInfo
	for i, path := range paths {
		st, err := os.Stat(path)
		if os.IsNotExist(err) {
			continue
		}
		if err != nil {
			return nil, err
		}
		if !st.Mode().IsRegular() {
			return nil, fmt.Errorf("autosave is not a regular file")
		}
		var name string
		var metadata *SaveMetadata
		var stamp time.Time
		if includeDetails {
			name, metadata, stamp = readSaveSlotHeader(path)
		} else if header, err := savefile.ReadHeader[SaveMetadata](path); err == nil {
			// Rotation needs only the date. Localized name validation stays
			// on the game thread, where the language setting can change.
			stamp = header.SavedAt
		}
		if stamp.IsZero() {
			stamp = st.ModTime()
		}
		saves = append(saves, SaveSlotInfo{Slot: i, Exists: true, SavedAt: stamp.Local(), Name: name, Metadata: metadata})
	}
	sort.Slice(saves, func(i, j int) bool {
		if saves[i].SavedAt.Equal(saves[j].SavedAt) {
			return saves[i].Slot < saves[j].Slot
		}
		return saves[i].SavedAt.After(saves[j].SavedAt)
	})
	return saves, nil
}
func (g *Game) AutoSave(reason string, now time.Time) error {
	if !g.canAutosave() {
		return nil
	}
	g.finishIntervalAutosave(true)
	paths, err := g.autoSavePaths()
	if err != nil {
		return err
	}
	snapshot, name, metadata := g.captureAutosave(reason)
	return writeAutosave(paths, snapshot, name, metadata, now)
}

func writeAutosave(paths [MaxAutoSaves]string, snapshot gameSave, name string, metadata *SaveMetadata, now time.Time) error {
	saves, err := readAutoSaves(paths, false)
	if err != nil {
		return err
	}
	slot := -1
	occupied := [MaxAutoSaves]bool{}
	for _, s := range saves {
		occupied[s.Slot] = true
	}
	for i, used := range occupied {
		if !used {
			slot = i
			break
		}
	}
	if slot < 0 {
		slot = saves[len(saves)-1].Slot
	}
	return writeSaveSnapshot(paths[slot], snapshot, name, metadata, now)
}

func (g *Game) captureAutosave(reason string) (gameSave, string, *SaveMetadata) {
	// snapshot records the live career city. Only the menu needs to reconcile
	// pending garage assignments before taking that snapshot.
	if g.isCareer() && g.State != StatePlaying {
		g.reconcileCareerCityFleet()
	}
	name := locale.T("自动存档", "Autosave")
	if reason == "exit" {
		name = locale.T("退出自动存档", "Exit autosave")
	}
	snapshot := g.snapshot()
	return snapshot, name, g.saveMetadata("auto", reason)
}

func (g *Game) careerSnapshotNeedsDispatch() bool {
	if !g.sessionStarted || g.Bus.OwnedVehicleID != g.Career.PlayerVehicleID {
		return true
	}
	live := map[int]bool{g.Bus.OwnedVehicleID: true}
	for _, a := range g.AIBusMgr.Buses {
		v := g.Career.Vehicle(a.Bus.OwnedVehicleID)
		if v == nil || v.Line != a.LineIndex {
			return true
		}
		live[v.ID] = true
	}
	for _, v := range g.Career.Vehicles {
		if v.ID != g.Career.PlayerVehicleID && v.Line >= 0 && !live[v.ID] {
			return true
		}
	}
	return false
}

type intervalAutosave struct {
	at   time.Time
	done chan error
}

func (g *Game) finishIntervalAutosave(wait bool) {
	pending := g.pendingAutosave
	if pending == nil {
		return
	}
	var err error
	if wait {
		err = <-pending.done
	} else {
		select {
		case err = <-pending.done:
		default:
			return
		}
	}
	g.pendingAutosave = nil
	if err != nil {
		g.HUD.AddWarningToast(locale.T("自动存档失败：", "Autosave failed: ") + err.Error())
		return
	}
	g.lastAutosaveAt = pending.at
	g.HUD.AddToast(locale.T("已自动存档（保留最近 5 份）", "Autosaved (latest 5 retained)"), true)
}

func (g *Game) updateAutosave(now time.Time) {
	g.finishIntervalAutosave(false)
	if g.pendingAutosave != nil || !g.autosavesEnabled || !g.canAutosave() || g.savePicker.open || g.stopNameEditor != nil {
		return
	}
	if g.lastAutosaveAt.IsZero() {
		g.lastAutosaveAt = now
		return
	}
	if now.Sub(g.lastAutosaveAt) < AutoSaveInterval || (!g.lastAutosaveAttempt.IsZero() && now.Sub(g.lastAutosaveAttempt) < 30*time.Second) {
		return
	}
	g.lastAutosaveAttempt = now
	paths, err := g.autoSavePaths()
	if err != nil {
		g.HUD.AddWarningToast(locale.T("自动存档失败：", "Autosave failed: ") + err.Error())
		return
	}
	snapshot, name, metadata := g.captureAutosave("interval")
	detached := detachedSaveSnapshot(snapshot)
	job := &intervalAutosave{at: now, done: make(chan error, 1)}
	g.pendingAutosave = job
	go func() {
		job.done <- writeAutosave(paths, detached, name, metadata, now)
	}()
}

// Save both independent in-memory sessions on a normal application exit.
func (g *Game) SaveOnExit() error {
	if !g.autosavesEnabled || g.exitAutosaved {
		return nil
	}
	var errs []error
	now := time.Now()
	for mode := ModeFast; mode <= ModeCareer; mode++ {
		target := g
		if mode != g.Mode {
			target = g.modeSessions[mode]
		}
		if target == nil || !target.canAutosave() {
			continue
		}
		if err := target.AutoSave("exit", now); err != nil {
			errs = append(errs, fmt.Errorf("%s: %w", mode.String(), err))
		}
	}
	if len(errs) > 0 {
		return errors.Join(errs...)
	}
	g.exitAutosaved = true
	return nil
}
func (g *Game) LoadAutoSave(slot int) error {
	g.finishIntervalAutosave(true)
	path, err := g.autoSavePath(slot)
	if err != nil {
		return err
	}
	return g.loadGamePath(path)
}
