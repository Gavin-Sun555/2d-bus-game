package game

import (
	"encoding/json"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"strings"
	"unicode"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type stopLabels struct {
	Chinese string `json:"chinese,omitempty"`
	English string `json:"english,omitempty"`
}
type stopNameFile struct {
	Version int                   `json:"version"`
	Stops   map[string]stopLabels `json:"stops"`
}
type stopNameStore struct {
	path      string
	names     map[string]stopLabels
	loadError error
}

func validateStopLabels(n stopLabels) error {
	if !utf8.ValidString(n.Chinese) || !utf8.ValidString(n.English) {
		return fmt.Errorf("站名编码无效 / Invalid text encoding")
	}
	if utf8.RuneCountInString(n.Chinese) > 12 {
		return fmt.Errorf("中文站名最多12字 / Chinese name: max 12 characters")
	}
	if utf8.RuneCountInString(n.English) > 30 {
		return fmt.Errorf("英文站名最多30字符（含空格） / English name: max 30 characters incl. spaces")
	}
	for _, r := range n.Chinese {
		if unicode.IsControl(r) || unicode.Is(unicode.Cf, r) {
			return fmt.Errorf("站名不能包含换行或控制字符 / No line breaks or control characters")
		}
	}
	for _, r := range n.English {
		if r < 32 || r > 126 {
			return fmt.Errorf("英文名请使用英文字符 / Use English characters in the English name")
		}
	}
	return nil
}

// Road-center coordinates stay fixed when lane widths change. Include the map,
// stop ID, position and direction so unrelated maps/custom layouts cannot share
// a rename merely because they happen to reuse an integer stop ID.
func stopNameKey(w *sim.World, s *sim.BusStop) string {
	h := sim.NormalizeAngle(s.Heading)
	if math.Abs(h) < .0000005 {
		h = 0
	}
	if math.Abs(h+math.Pi) < .0000005 {
		h = math.Pi
	}
	return fmt.Sprintf("%d/%d/%.3f/%.3f/%.6f", w.Theme, s.ID, s.RoadCenter.X, s.RoadCenter.Y, h)
}

func readStopNames(path string) (*stopNameStore, error) {
	store := &stopNameStore{path: path, names: map[string]stopLabels{}}
	file, err := os.Open(path)
	if os.IsNotExist(err) {
		return store, nil
	}
	if err != nil {
		store.loadError = err
		return store, err
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(file, (8<<20)+1))
	if err == nil && len(raw) > 8<<20 {
		err = fmt.Errorf("station name file too large")
	}
	var data stopNameFile
	if err == nil {
		err = json.Unmarshal(raw, &data)
	}
	if err == nil && (data.Version != 1 || data.Stops == nil || len(data.Stops) > 65536) {
		err = fmt.Errorf("invalid station name file")
	}
	if err == nil {
		for _, n := range data.Stops {
			if err = validateStopLabels(n); err != nil {
				break
			}
		}
	}
	if err != nil {
		store.loadError = err
		return store, err
	}
	store.names = data.Stops
	return store, nil
}

// EnableStationNames loads native local preferences. Pure NewGame/replay tests
// do not read or overwrite the user's preferences unless explicitly enabled.
func (g *Game) EnableStationNames() error {
	dir, err := os.UserConfigDir()
	if err != nil {
		return err
	}
	return g.loadStopNames(filepath.Join(dir, "2d-bus-game", "station_names.json"))
}
func (g *Game) loadStopNames(path string) error {
	store, err := readStopNames(path)
	g.stopNames = store
	if err == nil {
		g.applyStopNames()
	}
	return err
}
func (g *Game) applyStopNames() {
	if g.stopNames == nil || g.World == nil {
		return
	}
	for _, s := range g.World.Stops {
		n := g.stopNames.names[stopNameKey(g.World, s)]
		s.CustomNameZH = n.Chinese
		s.CustomNameEN = n.English
	}
	for _, r := range g.Lines {
		if r != nil {
			r.InvalidateDescription()
		}
	}
	if g.Route != nil {
		g.Route.InvalidateDescription()
	}
	if g.AIBusMgr != nil {
		for _, a := range g.AIBusMgr.Buses {
			if a != nil && a.Route != nil {
				a.Route.InvalidateDescription()
			}
		}
	}
}

func (g *Game) renameStop(id int, n stopLabels) error {
	if g.stopNames == nil {
		return fmt.Errorf("本地站名配置不可用 / Station preferences unavailable")
	}
	if g.stopNames.loadError != nil {
		return fmt.Errorf("无法读取本地站名文件，未覆盖原文件 / Cannot read station preferences: %w", g.stopNames.loadError)
	}
	if g.World == nil || id < 0 || id >= len(g.World.Stops) {
		return fmt.Errorf("invalid station")
	}
	// Count spaces before trimming so the stated English limit includes spaces.
	if err := validateStopLabels(n); err != nil {
		return err
	}
	n.Chinese = strings.TrimSpace(n.Chinese)
	n.English = strings.TrimSpace(n.English)
	names := make(map[string]stopLabels, len(g.stopNames.names)+1)
	for k, v := range g.stopNames.names {
		names[k] = v
	}
	key := stopNameKey(g.World, g.World.Stops[id])
	if n == (stopLabels{}) {
		delete(names, key)
	} else {
		names[key] = n
	}
	data, err := json.MarshalIndent(stopNameFile{Version: 1, Stops: names}, "", "  ")
	if err != nil {
		return err
	}
	path := g.stopNames.path
	if err = os.MkdirAll(filepath.Dir(path), 0700); err != nil {
		return err
	}
	f, err := os.CreateTemp(filepath.Dir(path), ".station-names-*.tmp")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	if _, err = f.Write(data); err != nil {
		f.Close()
		return err
	}
	if err = f.Sync(); err != nil {
		f.Close()
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	if err = os.Rename(f.Name(), path); err != nil {
		return err
	}
	g.stopNames.names = names
	g.applyStopNames()
	return nil
}
