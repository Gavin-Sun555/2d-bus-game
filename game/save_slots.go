package game

import (
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strings"
	"time"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
)

const MaxSaveSlots = 10
const MaxSaveNameRunes = 24

type SaveSlotInfo struct {
	Slot    int
	Exists  bool
	SavedAt time.Time
	Name    string
}

func validateSaveSlotName(name string) (string, error) {
	clean := strings.TrimSpace(name)
	for _, r := range clean {
		if r < 0x20 || r == 0x7f {
			return "", fmt.Errorf("%s", locale.T("存档名称不能包含换行或控制字符", "Save name cannot contain newlines or control characters"))
		}
	}
	if utf8.RuneCountInString(clean) > MaxSaveNameRunes {
		return "", fmt.Errorf("%s", locale.Tf("存档名称最多 %d 个字符", "Save name cannot exceed %d characters", MaxSaveNameRunes))
	}
	return clean, nil
}

func readSaveSlotName(path string) string {
	f, err := os.Open(path)
	if err != nil {
		return ""
	}
	defer f.Close()
	raw, err := io.ReadAll(io.LimitReader(f, maxSaveBytes+1))
	if err != nil || len(raw) > maxSaveBytes {
		return ""
	}
	var header struct {
		Version int    `json:"version"`
		Name    string `json:"name"`
	}
	if err := json.Unmarshal(raw, &header); err != nil || header.Version != saveVersion {
		return ""
	}
	clean, err := validateSaveSlotName(header.Name)
	if err != nil {
		return ""
	}
	return clean
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
		slots[i].Name = readSaveSlotName(path)
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

func (g *Game) SaveGameSlotWithName(slot int, name string) error {
	if _, err := g.saveSlotPath(slot); err != nil {
		return err
	}
	clean, err := validateSaveSlotName(name)
	if err != nil {
		return err
	}
	g.pendingSaveName = &clean
	defer func() { g.pendingSaveName = nil }()
	return g.SaveGameSlot(slot)
}

func (g *Game) RenameSaveSlot(slot int, name string) error {
	path, err := g.saveSlotPath(slot)
	if err != nil {
		return err
	}
	clean, err := validateSaveSlotName(name)
	if err != nil {
		return err
	}
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		return fmt.Errorf("%s", locale.T("这个槽位还没有存档。", "This slot is empty."))
	}
	if err != nil {
		return err
	}
	if !info.Mode().IsRegular() {
		return fmt.Errorf("save slot %d is not a regular file", slot+1)
	}
	envelope, err := readSaveEnvelope(path)
	if err != nil {
		return err
	}
	envelope.Name = clean
	data, err := json.Marshal(envelope)
	if err != nil {
		return err
	}
	if len(data) > maxSaveBytes {
		return fmt.Errorf("save exceeds size limit")
	}
	if err := writeSaveFileAtomically(path, data); err != nil {
		return err
	}
	_ = os.Chtimes(path, info.ModTime(), info.ModTime())
	return nil
}

func (g *Game) DeleteSaveSlot(slot int) error {
	path, err := g.saveSlotPath(slot)
	if err != nil {
		return err
	}
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		return fmt.Errorf("%s", locale.T("这个槽位还没有存档。", "This slot is empty."))
	}
	if err != nil {
		return err
	}
	if !info.Mode().IsRegular() {
		return fmt.Errorf("save slot %d is not a regular file", slot+1)
	}
	return os.Remove(path)
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
