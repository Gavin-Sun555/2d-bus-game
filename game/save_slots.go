package game

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

const MaxSaveSlots = 10

type SaveSlotInfo struct {
	Slot    int
	Exists  bool
	SavedAt time.Time
}

// Slot zero retains the original filename, so single-slot saves need no migration.
func (g *Game) saveSlotPath(slot int) (string, error) {
	if slot < 0 || slot >= MaxSaveSlots {
		return "", fmt.Errorf("invalid save slot: %d", slot+1)
	}
	base := g.savePathOverride
	if base == "" {
		dir, err := os.UserConfigDir()
		if err != nil {
			return "", err
		}
		base = filepath.Join(dir, "2d-bus-game", "save.json")
	}
	if slot == 0 {
		return base, nil
	}
	ext := filepath.Ext(base)
	return strings.TrimSuffix(base, ext) + fmt.Sprintf("-%02d", slot+1) + ext, nil
}

func (g *Game) SaveSlots() ([MaxSaveSlots]SaveSlotInfo, error) {
	var slots [MaxSaveSlots]SaveSlotInfo
	for i := range slots {
		slots[i].Slot = i
		path, err := g.saveSlotPath(i)
		if err != nil {
			return slots, err
		}
		info, err := os.Stat(path)
		if os.IsNotExist(err) {
			continue
		}
		if err != nil {
			return slots, err
		}
		if !info.Mode().IsRegular() {
			return slots, fmt.Errorf("save slot %d is not a regular file", i+1)
		}
		slots[i].Exists = true
		slots[i].SavedAt = info.ModTime()
	}
	return slots, nil
}

// A failed operation leaves the active slot and live simulation unchanged.
func (g *Game) SaveGameSlot(slot int) error {
	if _, err := g.saveSlotPath(slot); err != nil {
		return err
	}
	previous := g.activeSaveSlot
	g.activeSaveSlot = slot
	if err := g.SaveGame(); err != nil {
		g.activeSaveSlot = previous
		return err
	}
	return nil
}

func (g *Game) LoadGameSlot(slot int) error {
	if _, err := g.saveSlotPath(slot); err != nil {
		return err
	}
	previous := g.activeSaveSlot
	g.activeSaveSlot = slot
	if err := g.LoadGame(); err != nil {
		g.activeSaveSlot = previous
		return err
	}
	return nil
}
