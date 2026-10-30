package game

import (
	"bytes"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/hajimehoshi/ebiten/v2"
)

func TestTenSaveSlotsRemainIndependent(t *testing.T) {
	g := saveTestGame(t)
	legacy := g.savePathOverride
	paths := map[string]bool{}
	for slot := 0; slot < MaxSaveSlots; slot++ {
		g.Cash = 1000 + slot
		if err := g.SaveGameSlot(slot); err != nil {
			t.Fatal(err)
		}
		path, err := g.savePath()
		if err != nil {
			t.Fatal(err)
		}
		if paths[path] {
			t.Fatal("slots share a file")
		}
		paths[path] = true
	}
	if _, err := os.Stat(legacy); err != nil {
		t.Fatal("legacy filename lost", err)
	}
	slots, err := g.SaveSlots()
	if err != nil {
		t.Fatal(err)
	}
	// A fresh session can discover and load all ten files.
	fresh := NewGame(1280, 720)
	fresh.savePathOverride = legacy
	for slot, info := range slots {
		if !info.Exists || info.SavedAt.IsZero() || info.Slot != slot {
			t.Fatalf("bad slot metadata: %+v", info)
		}
		if err := fresh.LoadGameSlot(slot); err != nil {
			t.Fatal(err)
		}
		if fresh.Cash != 1000+slot || fresh.activeSaveSlot != slot {
			t.Fatalf("slot%d lost state", slot+1)
		}
	}
	before, err := os.ReadFile(legacy)
	if err != nil {
		t.Fatal(err)
	}
	fresh.Cash = 9999
	if err := fresh.SaveGame(); err != nil {
		t.Fatal(err)
	}
	after, err := os.ReadFile(legacy)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(before, after) {
		t.Fatal("saving current slot overwrote legacy slot")
	}
	if err := fresh.LoadGameSlot(9); err != nil || fresh.Cash != 9999 {
		t.Fatal("active slot not retained", err)
	}
}

func TestSaveSlotErrorsPreserveLiveState(t *testing.T) {
	g := saveTestGame(t)
	if err := g.SaveGameSlot(3); err != nil {
		t.Fatal(err)
	}
	bus, world, cash := g.Bus, g.World, g.Cash
	for _, slot := range []int{-1, MaxSaveSlots} {
		if err := g.SaveGameSlot(slot); err == nil {
			t.Fatal("accepted invalid slot")
		}
		if err := g.LoadGameSlot(slot); err == nil {
			t.Fatal("accepted invalid slot")
		}
	}
	if err := g.LoadGameSlot(7); err == nil {
		t.Fatal("loaded empty slot")
	}
	path, _ := g.saveSlotPath(8)
	if err := os.WriteFile(path, []byte("corrupt"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGameSlot(8); err == nil {
		t.Fatal("loaded corrupt slot")
	}
	// The destination is a directory, forcing the atomic rename to fail.
	path, _ = g.saveSlotPath(6)
	if err := os.Mkdir(path, 0700); err != nil {
		t.Fatal(err)
	}
	if err := g.SaveGameSlot(6); err == nil {
		t.Fatal("saved over a directory")
	}
	if g.activeSaveSlot != 3 || g.Bus != bus || g.World != world || g.Cash != cash {
		t.Fatal("failed operation replaced live state or selected slot")
	}
}

func TestSavePickerOverwriteAndPause(t *testing.T) {
	g := saveTestGame(t)
	g.Cash = 111
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	g.Cash = 222
	g.openSavePicker(false)
	g.performSlotAction()
	if !g.savePicker.open || !g.savePicker.confirm {
		t.Fatal("overwrite was not confirmed")
	}
	s, err := readSave(g.savePathOverride)
	if err != nil || s.Cash != 111 {
		t.Fatal("first click overwrote slot", err)
	}
	// All simulation state remains unchanged while the picker is open.
	before, err := json.Marshal(g.snapshot())
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 60; i++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	after, err := json.Marshal(g.snapshot())
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(before, after) {
		t.Fatal("simulation advanced inside picker")
	}
	g.performSlotAction()
	if g.savePicker.open {
		t.Fatal("successful save did not close picker")
	}
	s, err = readSave(g.savePathOverride)
	if err != nil || s.Cash != 222 {
		t.Fatal("confirmed overwrite did not save", err)
	}
	g.openSavePicker(false)
	g.performSlotAction()
	g.selectSaveSlot(1)
	if g.savePicker.confirm {
		t.Fatal("confirmation leaked into another slot")
	}
	g.performSlotAction()
	if g.activeSaveSlot != 1 {
		t.Fatal("empty slot did not save immediately")
	}
}

func TestSavePickerEmptyAndCorruptLoad(t *testing.T) {
	g := saveTestGame(t)
	g.openSavePicker(true)
	g.performSlotAction()
	if !g.savePicker.open || g.savePicker.message == "" {
		t.Fatal("empty load lacks feedback")
	}
	path, _ := g.saveSlotPath(2)
	if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte("broken"), 0600); err != nil {
		t.Fatal(err)
	}
	g.openSavePicker(true)
	g.selectSaveSlot(2)
	bus := g.Bus
	g.performSlotAction()
	if !g.savePicker.open || g.Bus != bus || g.activeSaveSlot != 0 || g.savePicker.message == "" {
		t.Fatal("failed load lost dialog or live game")
	}
}

func TestSavePickerDrawAndLayout(t *testing.T) {
	g := saveTestGame(t)
	for _, size := range [][2]int{{1280, 720}, {1280, 832}, {1920, 720}} {
		g.Width, g.Height = size[0], size[1]
		l := savePickerBounds(g.Width, g.Height)
		for i, r := range l.rows {
			if r.X < l.panel.X || r.Y < l.panel.Y || r.X+r.W > l.panel.X+l.panel.W || r.Y+r.H > l.cancel.Y {
				t.Fatal("row outside panel")
			}
			for j := 0; j < i; j++ {
				o := l.rows[j]
				if r.X < o.X+o.W && o.X < r.X+r.W && r.Y < o.Y+o.H && o.Y < r.Y+r.H {
					t.Fatal("overlapping rows")
				}
			}
		}
		g.openSavePicker(false)
		g.drawSavePicker(ebiten.NewImage(g.Width, g.Height))
		g.savePicker.confirm = true
		g.drawSavePicker(ebiten.NewImage(g.Width, g.Height))
		g.savePicker.confirm = false
		g.savePicker.confirmDelete = true
		g.drawSavePicker(ebiten.NewImage(g.Width, g.Height))
		g.beginSaveSlotRename()
		g.savePicker.nameInput = "早高峰铰接车测试"
		g.savePicker.nameCaret = len(g.savePicker.nameInput)
		g.drawSavePicker(ebiten.NewImage(g.Width, g.Height))
	}
}

func TestSaveSlotRenameAndDelete(t *testing.T) {
	g := saveTestGame(t)
	g.Cash = 5678

	// 1. Preset name on an empty slot in Save picker, then save.
	g.openSavePicker(false)
	g.selectSaveSlot(2)
	g.beginSaveSlotRename()
	g.savePicker.insertNameText("  早高峰1号线  ")
	if !g.commitSaveSlotRename() {
		t.Fatal("preset name rejected")
	}
	g.performSlotAction()
	if g.savePicker.open {
		t.Fatal("save with preset name did not finish")
	}

	slots, err := g.SaveSlots()
	if err != nil {
		t.Fatal(err)
	}
	if !slots[2].Exists || slots[2].Name != "早高峰1号线" {
		t.Fatalf("expected saved name 早高峰1号线, got %+v", slots[2])
	}
	savedAt := slots[2].SavedAt

	// 2. Rename existing slot from Load picker; timestamp and payload must remain intact.
	g.openSavePicker(true)
	g.selectSaveSlot(2)
	g.beginSaveSlotRename()
	g.savePicker.nameSelected = true
	g.savePicker.insertNameText("晚高峰备份")
	if !g.commitSaveSlotRename() {
		t.Fatal("rename existing slot failed")
	}
	slots, err = g.SaveSlots()
	if err != nil {
		t.Fatal(err)
	}
	if slots[2].Name != "晚高峰备份" || !slots[2].SavedAt.Equal(savedAt) {
		t.Fatalf("rename did not preserve timestamp or update name: %+v (was %v)", slots[2], savedAt)
	}

	// 3. Re-saving to the same slot without specifying a new name preserves its existing name.
	g.Cash = 6789
	if err := g.SaveGameSlot(2); err != nil {
		t.Fatal(err)
	}
	slots, err = g.SaveSlots()
	if err != nil {
		t.Fatal(err)
	}
	if slots[2].Name != "晚高峰备份" {
		t.Fatalf("overwriting slot lost existing name: %+v", slots[2])
	}

	// 4. Reject overly long or multiline names without mutating the saved name.
	if err := g.RenameSaveSlot(2, "一二三四五六七八九十一二三四五六七八九十一二三四五"); err == nil {
		t.Fatal("accepted name exceeding MaxSaveNameRunes")
	}
	if err := g.RenameSaveSlot(2, "bad\nname"); err == nil {
		t.Fatal("accepted multiline name")
	}
	slots, _ = g.SaveSlots()
	if slots[2].Name != "晚高峰备份" {
		t.Fatalf("failed rename mutated slot name: %+v", slots[2])
	}

	// 5. Two-step delete confirmation in picker.
	g.openSavePicker(true)
	g.selectSaveSlot(2)
	g.performDeleteSlot()
	if !g.savePicker.confirmDelete {
		t.Fatal("first delete click did not request confirmation")
	}
	if slots, _ = g.SaveSlots(); !slots[2].Exists {
		t.Fatal("first delete click removed file before confirmation")
	}
	// Switching slots cancels delete confirmation.
	g.selectSaveSlot(1)
	if g.savePicker.confirmDelete {
		t.Fatal("selecting another slot did not clear delete confirmation")
	}
	g.selectSaveSlot(2)
	g.performDeleteSlot()
	g.performDeleteSlot()
	if g.savePicker.confirmDelete || g.savePicker.slots[2].Exists {
		t.Fatal("confirmed delete did not remove slot")
	}
	if slots, _ = g.SaveSlots(); slots[2].Exists {
		t.Fatal("deleted slot still exists on disk")
	}
}

func TestSavePickerLoadFromMenu(t *testing.T) {
	g := saveTestGame(t)
	g.Cash = 4321
	if err := g.SaveGameSlot(9); err != nil {
		t.Fatal(err)
	}
	g.Cash = 1
	g.State = StateMainMenu
	g.openSavePicker(true)
	g.selectSaveSlot(9)
	g.performSlotAction()
	if g.State != StatePlaying || g.Cash != 4321 || g.activeSaveSlot != 9 || g.savePicker.open {
		t.Fatal("menu load did not restore selected slot")
	}
}
