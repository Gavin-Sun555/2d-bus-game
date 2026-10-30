package game

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestStopNameLimitsAndPersistence(t *testing.T) {
	g := saveTestGame(t)
	path := filepath.Join(t.TempDir(), "prefs", "station_names.json")
	if err := g.loadStopNames(path); err != nil {
		t.Fatal(err)
	}
	n := stopLabels{Chinese: "人民广场公交换乘枢纽总站", English: "Central Square Bus Interchange"}
	if err := g.renameStop(278, n); err != nil {
		t.Fatal(err)
	}
	originalRaw := g.World.Stops[278].Name
	language := locale.CurrentLanguage
	defer locale.SetLanguage(language)
	locale.SetLanguage(locale.LanguageZH)
	if got := g.World.Stops[278].CleanName(); got != n.Chinese {
		t.Fatal(got)
	}
	locale.SetLanguage(locale.LanguageEN)
	if got := g.World.Stops[278].LocalizedName(); got != n.English {
		t.Fatal(got)
	}
	raw, _ := os.ReadFile(path)
	for _, invalid := range []stopLabels{{Chinese: n.Chinese + "多", English: n.English}, {Chinese: n.Chinese, English: n.English + " "}, {Chinese: "车站\n换行"}, {English: "Central\tBus"}, {English: "中文"}} {
		if err := g.renameStop(278, invalid); err == nil {
			t.Fatalf("accepted invalid label %+v", invalid)
		}
		after, _ := os.ReadFile(path)
		if !bytes.Equal(raw, after) || g.World.Stops[278].CustomNameZH != n.Chinese {
			t.Fatal("failed validation modified disk/live name")
		}
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	// Names are separate local preferences. A newer preference survives an old slot.
	newer := stopLabels{Chinese: "市民中心", English: "Civic Center"}
	if err := g.renameStop(278, newer); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.World.Stops[278].CustomNameZH != newer.Chinese || g.World.Stops[278].Name != originalRaw {
		t.Fatal("load restored stale or altered original name")
	}
	if err := g.renameStop(278, n); err != nil {
		t.Fatal(err)
	}
	g.SetMapTheme(sim.MapThemeCoastalBay)
	if g.World.Stops[0].CustomNameZH != "" {
		t.Fatal("rename leaked into another map")
	}
	g.SetMapTheme(sim.MapThemeMetropolis)
	if g.World.Stops[278].CustomNameEN != n.English {
		t.Fatal("map switch lost name")
	}
	// Fresh process-equivalent game loads the same preferences.
	fresh := NewGame(1280, 720)
	fresh.EnterCity(sim.MapThemeMetropolis)
	if err := fresh.loadStopNames(path); err != nil {
		t.Fatal(err)
	}
	if fresh.World.Stops[278].CustomNameZH != n.Chinese {
		t.Fatal("name lost on restart")
	}
	g.World.UpdateLaneWidth(78)
	g.applyStopNames()
	if g.World.Stops[278].CustomNameZH != n.Chinese {
		t.Fatal("lane width changed name identity")
	}
	if err := g.renameStop(278, stopLabels{}); err != nil {
		t.Fatal(err)
	}
	if g.World.Stops[278].CustomNameZH != "" || g.World.Stops[278].CustomNameEN != "" || g.World.Stops[278].Name != originalRaw {
		t.Fatal("reset did not restore original")
	}
}
func TestStopNamesFailedWriteAndCorruptFile(t *testing.T) {
	g := saveTestGame(t)
	dir := t.TempDir()
	path := filepath.Join(dir, "station_names.json")
	if err := os.WriteFile(path, []byte("broken"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := g.loadStopNames(path); err == nil {
		t.Fatal("corrupt preferences accepted")
	}
	if err := g.renameStop(0, stopLabels{Chinese: "车站"}); err == nil {
		t.Fatal("corrupt preferences overwritten")
	}
	if data, _ := os.ReadFile(path); string(data) != "broken" {
		t.Fatal("corrupt file not preserved")
	}
	// A path occupied by a directory forces atomic replacement to fail even as root.
	other := filepath.Join(dir, "other.json")
	if err := g.loadStopNames(other); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(other, 0700); err != nil {
		t.Fatal(err)
	}
	if err := g.renameStop(0, stopLabels{Chinese: "车站"}); err == nil {
		t.Fatal("write failure ignored")
	}
	if g.World.Stops[0].CustomNameZH != "" || len(g.stopNames.names) != 0 {
		t.Fatal("failed write changed live state")
	}
}
func TestStopNameMapGeometryAndRouteCache(t *testing.T) {
	g := saveTestGame(t)
	path := filepath.Join(t.TempDir(), "names.json")
	g.loadStopNames(path)
	id := g.Route.StopIDs[0]
	before := g.Route.GetRouteDescription(g.World)
	if err := g.renameStop(id, stopLabels{Chinese: "新站名", English: "New Station"}); err != nil {
		t.Fatal(err)
	}
	language := locale.CurrentLanguage
	defer locale.SetLanguage(language)
	locale.SetLanguage(locale.LanguageZH)
	if desc := g.Route.GetRouteDescription(g.World); desc == before || !strings.Contains(desc, "新站名") {
		t.Fatal("stale route description")
	}
	locale.SetLanguage(locale.LanguageEN)
	if desc := g.Route.GetRouteDescription(g.World); !strings.Contains(desc, "New Station") {
		t.Fatal("language switch kept stale route description")
	}
	oldKey := stopNameKey(g.World, g.World.Stops[id])
	g.World.Stops[id].RoadCenter.X += 100
	g.applyStopNames()
	if stopNameKey(g.World, g.World.Stops[id]) == oldKey || g.World.Stops[id].CustomNameZH != "" {
		t.Fatal("different custom map stop inherited name")
	}
}
func TestStopEditorDoesNotModifyRouteAndCancels(t *testing.T) {
	g := saveTestGame(t)
	g.loadStopNames(filepath.Join(t.TempDir(), "names.json"))
	g.TogglePlanMode()
	id := 278
	route := append([]int(nil), g.planningRoute().StopIDs...)
	busPos := g.Bus.Pos
	g.openStopNameEditor(id)
	if g.stopNameEditor == nil {
		t.Fatal("station outside route cannot be edited")
	}
	g.stopNameEditor.insert("测试车站")
	g.closeStopNameEditor()
	if g.World.Stops[id].CustomNameZH != "" || !intSlicesEqual(route, g.planningRoute().StopIDs) || g.Bus.Pos != busPos {
		t.Fatal("cancel changed world or routing")
	}
	g.openStopNameEditor(id)
	g.stopNameEditor.insert("测试车站")
	if !g.saveEditedStopName(false) || g.stopNameEditor != nil || g.World.Stops[id].CustomNameZH != "测试车站" {
		t.Fatal("save not applied")
	}
	if g.planModeRouteModified || !intSlicesEqual(route, g.planningRoute().StopIDs) || g.Bus.Pos != busPos {
		t.Fatal("station rename dispatched/reset fleet")
	}
	g.openStopNameEditor(id)
	g.stopNameEditor.selected = true
	g.stopNameEditor.insert(strings.Repeat("长", 13))
	if g.saveEditedStopName(false) || g.stopNameEditor == nil || g.World.Stops[id].CustomNameZH != "测试车站" {
		t.Fatal("invalid modal saved or closed")
	}
	g.closeStopNameEditor()
}
