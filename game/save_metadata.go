package game

import (
	"encoding/json"
	"math"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type SaveMetadata struct {
	Debug          *CongestionDebugReport `json:"debug,omitempty"`
	Mode           PlayMode               `json:"mode"`
	ModeName       string                 `json:"mode_name"`
	Map            sim.MapTheme           `json:"map"`
	MapName        string                 `json:"map_name"`
	CenterBRT      bool                   `json:"center_brt,omitempty"`
	LargeGrid      bool                   `json:"large_grid,omitempty"`
	Source         string                 `json:"source"`
	Reason         string                 `json:"reason,omitempty"`
	DriverName     string                 `json:"driver_name,omitempty"`
	Balance        int                    `json:"balance,omitempty"`
	OwnedBuses     int                    `json:"owned_buses,omitempty"`
	UnlockedCities int                    `json:"unlocked_cities,omitempty"`
	VisitedCities  int                    `json:"visited_cities,omitempty"`
}

func (g *Game) careerUnlockedCityCount() int {
	if !g.isCareer() {
		return 0
	}
	count := 0
	for _, th := range sim.BuiltinMapThemes() {
		if g.Career.MapUnlocked(th) {
			count++
		}
	}
	return count
}

func (g *Game) saveMetadata(source, reason string) *SaveMetadata {
	name := g.World.Theme.Name()
	if g.World.Theme == sim.MapThemeCustom && g.World.CustomMap != nil {
		name = g.World.CustomMap.Name
	}
	m := &SaveMetadata{Mode: g.Mode, ModeName: g.Mode.String(), Map: g.World.Theme, MapName: name, CenterBRT: g.World.CenterBRT, LargeGrid: g.World.LargeGrid, Source: source, Reason: reason}
	if g.isCareer() {
		m.DriverName = g.Career.DriverName
		m.Balance = int(math.Floor(g.Career.Balance))
		m.OwnedBuses = len(g.Career.Vehicles)
		m.UnlockedCities = g.careerUnlockedCityCount()
		m.VisitedCities = max(1, len(g.careerCities))
	}
	return m
}
func (m *SaveMetadata) label() string {
	if m == nil {
		return locale.T("地图未知", "Unknown map")
	}
	if m.Mode == ModeCareer {
		if m.OwnedBuses > 0 {
			return locale.Tf("车队 %d 辆 · 解锁 %d/4 城 · ¥%d", "%d buses · %d/4 cities · ¥%d", m.OwnedBuses, max(1, m.UnlockedCities), m.Balance)
		}
		return locale.T("生涯全局存档", "Career Progress")
	}
	name := m.Map.ShortName()
	if locale.CurrentLanguage == locale.LanguageEN {
		names := []string{"Metropolis", "Coastal Bay", "River Valley", "Emerald Lake", "Twin Districts", "Custom"}
		if int(m.Map) >= 0 && int(m.Map) < len(names) {
			name = names[m.Map]
		}
	}
	if m.Map == sim.MapThemeCustom && m.MapName != "" {
		name = m.MapName
	}
	return name
}
func readSaveSlotHeader(path string) (string, *SaveMetadata, time.Time) {
	header, err := savefile.ReadHeader[SaveMetadata](path)
	if err != nil {
		return "", nil, time.Time{}
	}
	clean, _ := validateSaveSlotName(header.Name)
	if header.Metadata == nil {
		// Older saves have no header metadata. Read only the fields needed for
		// the list; full validation still happens before restoring a game.
		var legacy struct {
			Mode  PlayMode
			World *struct {
				Theme                sim.MapTheme
				CenterBRT, LargeGrid bool
				CustomMap            *struct{ Name string }
			}
		}
		if json.Unmarshal(header.Data, &legacy) == nil && legacy.World != nil {
			w := legacy.World
			name := w.Theme.Name()
			if w.Theme == sim.MapThemeCustom && w.CustomMap != nil {
				name = w.CustomMap.Name
			}
			header.Metadata = &SaveMetadata{Mode: legacy.Mode, ModeName: legacy.Mode.String(), Map: w.Theme, MapName: name, CenterBRT: w.CenterBRT, LargeGrid: w.LargeGrid, Source: "manual"}
		}
	}
	return clean, header.Metadata, header.SavedAt
}
func (g *Game) saveSnapshotFile(path, name string, metadata *SaveMetadata, now time.Time) error {
	return writeSaveSnapshot(path, g.snapshot(), name, metadata, now)
}

func writeSaveSnapshot(path string, snapshot gameSave, name string, metadata *SaveMetadata, now time.Time) error {
	payload, err := json.Marshal(snapshot)
	if err != nil {
		return err
	}
	data, err := savefile.Encode(payload, name, metadata, now)
	if err != nil {
		return err
	}
	return savefile.Write(path, data)
}
