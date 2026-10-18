package integration_test

import (
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestMapThemesRoundaboutsAndBridges(t *testing.T) {
	// 1. Test MapTheme enum, cycling across all 5 distinct maps and names
	m := sim.MapThemeMetropolis
	if m.DisplayName() != "现代都会 (Metropolis)" || m.ShortName() != "现代都会" {
		t.Fatalf("Unexpected display/short name for Metropolis: %s, %s", m.DisplayName(), m.ShortName())
	}
	c := m.Next()
	if c != sim.MapThemeCoastalBay || c.ShortName() != "海滨港湾" {
		t.Fatalf("Expected next theme to be Coastal Bay, got %v (%s)", c, c.ShortName())
	}
	rv := c.Next()
	if rv != sim.MapThemeRiverValley || rv.ShortName() != "溪谷江城" {
		t.Fatalf("Expected next theme to be River Valley, got %v (%s)", rv, rv.ShortName())
	}
	l := rv.Next()
	if l != sim.MapThemeEmeraldLake || l.ShortName() != "湖畔水乡" {
		t.Fatalf("Expected next theme to be Emerald Lake, got %v (%s)", l, l.ShortName())
	}
	tc := l.Next()
	if tc != sim.MapThemeTwinCities || tc.ShortName() != "高新双城" {
		t.Fatalf("Expected next theme to be Twin Cities, got %v (%s)", tc, tc.ShortName())
	}
	cust := tc.Next()
	if cust != sim.MapThemeCustom || cust.ShortName() != "自定义" {
		t.Fatalf("Expected next theme to be Custom Map, got %v (%s)", cust, cust.ShortName())
	}
	if cust.Next() != sim.MapThemeMetropolis {
		t.Fatalf("Expected cycle back to Metropolis, got %v", cust.Next())
	}

	laneW := 42.0

	// 2. Test Metropolis Map (5x5 grid with canal twin bridges)
	wm := sim.NewWorld(laneW, sim.MapThemeMetropolis)
	if wm.Theme != sim.MapThemeMetropolis {
		t.Fatalf("World theme mismatch: expected Metropolis, got %v", wm.Theme)
	}
	if len(wm.Roundabouts) == 0 {
		t.Fatal("Metropolis map should have at least 1 roundabout")
	}
	if len(wm.Bridges) < 2 {
		t.Fatalf("Metropolis map should have twin canal bridges, got %d", len(wm.Bridges))
	}
	if len(wm.Waters) == 0 {
		t.Fatal("Metropolis map should have water bodies (canal)")
	}
	if len(wm.Stops) != 12 {
		t.Fatalf("Metropolis map should have 12 stops, got %d", len(wm.Stops))
	}

	// 3. Test Coastal Bay Map (coastal boulevard & 1200m sea suspension bridge)
	wc := sim.NewWorld(laneW, sim.MapThemeCoastalBay)
	if wc.Theme != sim.MapThemeCoastalBay {
		t.Fatalf("World theme mismatch: expected Coastal Bay, got %v", wc.Theme)
	}
	if len(wc.Roundabouts) == 0 {
		t.Fatal("Coastal Bay map should have at least 1 roundabout")
	}
	if len(wc.Bridges) == 0 {
		t.Fatal("Coastal Bay map should have ocean bridge")
	}
	hasBeach := false
	for _, wb := range wc.Waters {
		if wb.HasBeach {
			hasBeach = true
			break
		}
	}
	if !hasBeach {
		t.Fatal("Coastal Bay map should have water body with sandy beach")
	}

	// 4. Test River Valley Map (Yangtze River gorge & twin cable-stayed bridges)
	wrv := sim.NewWorld(laneW, sim.MapThemeRiverValley)
	if wrv.Theme != sim.MapThemeRiverValley {
		t.Fatalf("World theme mismatch: expected River Valley, got %v", wrv.Theme)
	}
	if len(wrv.Bridges) < 2 {
		t.Fatalf("River Valley map should have twin grand bridges, got %d", len(wrv.Bridges))
	}
	if len(wrv.Waters) == 0 {
		t.Fatal("River Valley map should feature Yangtze water body")
	}
	if len(wrv.Stops) < 6 {
		t.Fatalf("River Valley map should have stops, got %d", len(wrv.Stops))
	}

	// 5. Test Emerald Lake Map (giant emerald lake, lake island & 4 stone arch bridges)
	wl := sim.NewWorld(laneW, sim.MapThemeEmeraldLake)
	if wl.Theme != sim.MapThemeEmeraldLake {
		t.Fatalf("World theme mismatch: expected Emerald Lake, got %v", wl.Theme)
	}
	if len(wl.Roundabouts) == 0 {
		t.Fatal("Emerald Lake map should have lake island roundabout")
	}
	if len(wl.Bridges) < 2 {
		t.Fatalf("Emerald Lake map should have stone arch bridges, got %d", len(wl.Bridges))
	}
	hasStoneArch := false
	for _, br := range wl.Bridges {
		if br.Style() == sim.BridgeStyleStoneArch {
			hasStoneArch = true
			break
		}
	}
	if !hasStoneArch {
		t.Fatal("Emerald Lake map should feature stone arch bridges")
	}

	// 6. Test Twin Cities Map (Central expressway viaduct overpass & grade-separated interchange)
	wtc := sim.NewWorld(laneW, sim.MapThemeTwinCities)
	if wtc.Theme != sim.MapThemeTwinCities {
		t.Fatalf("World theme mismatch: expected Twin Cities, got %v", wtc.Theme)
	}
	hasOverpass := false
	for _, br := range wtc.Bridges {
		if br.IsOverpass && br.Style() == sim.BridgeStyleOverpass {
			hasOverpass = true
			break
		}
	}
	if !hasOverpass {
		t.Fatal("Twin Cities map must feature a grade-separated highway overpass viaduct")
	}
	if len(wtc.Stops) < 6 {
		t.Fatalf("Twin Cities map should have stops, got %d", len(wtc.Stops))
	}

	// 7. Test Custom Map
	wcm := sim.NewWorld(laneW, sim.MapThemeCustom)
	if wcm.Theme != sim.MapThemeCustom {
		t.Fatalf("World theme mismatch: expected Custom, got %v", wcm.Theme)
	}
	if len(wcm.Stops) < 6 {
		t.Fatalf("Custom map should have at least 6 stops, got %d", len(wcm.Stops))
	}
	if len(wcm.Roads) < 8 {
		t.Fatalf("Custom map should have roads, got %d", len(wcm.Roads))
	}

	// 8. Test Roundabout roadway boundary detection vs center island barrier
	rb := wm.Roundabouts[0]
	roadwayRadius := (rb.InnerRadius + rb.Radius) / 2.0
	posOnRing := sim.Vec2{X: rb.Center.X + roadwayRadius, Y: rb.Center.Y}
	resRing := wm.CheckBoundaries(posOnRing, 1.5, laneW)
	if resRing.Zone == sim.ZoneHardBarrier {
		t.Fatalf("Bus on roundabout driving ring should not be in barrier! Res: %+v", resRing)
	}

	posInIsland := rb.Center
	resIsland := wm.CheckBoundaries(posInIsland, 1.5, laneW)
	if resIsland.Zone != sim.ZoneHardBarrier {
		t.Fatalf("Bus inside central island should hit barrier/boundary! Res: %+v", resIsland)
	}

	// 8. Test Bridge bounds (bus on bridge roadway is safe)
	br := wm.Bridges[0]
	posOnBridge := br.Pos
	resBridge := wm.CheckBoundaries(posOnBridge, 1.5, laneW)
	if resBridge.Zone == sim.ZoneHardBarrier {
		t.Fatalf("Bus on bridge roadway should be safe! Res: %+v", resBridge)
	}

	// 9. Test Overpass Viaduct bounds on Map 5
	overpassBr := wtc.Bridges[0]
	resOverpass := wtc.CheckBoundaries(overpassBr.Pos, 1.5, laneW)
	if resOverpass.Zone == sim.ZoneHardBarrier {
		t.Fatalf("Bus on highway overpass viaduct should be safe! Res: %+v", resOverpass)
	}

	// 10. Test Theme switching & Main Menu State
	g := game.NewGame(1280, 720)
	if g.State != game.StateMainMenu {
		t.Fatalf("Expected new game to start in StateMainMenu, got %v", g.State)
	}

	// Enter City
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.State != game.StatePlaying {
		t.Fatalf("Expected state to be StatePlaying after EnterCity, got %v", g.State)
	}
	if g.World.Theme != sim.MapThemeCoastalBay || g.Settings.MapTheme != sim.MapThemeCoastalBay {
		t.Fatalf("Game map theme switch to Coastal Bay failed")
	}

	// Enter River Valley
	g.EnterCity(sim.MapThemeRiverValley)
	if g.World.Theme != sim.MapThemeRiverValley || g.Settings.MapTheme != sim.MapThemeRiverValley {
		t.Fatalf("Game map theme switch to River Valley failed")
	}

	// Enter Twin Cities
	g.EnterCity(sim.MapThemeTwinCities)
	if g.World.Theme != sim.MapThemeTwinCities || g.Settings.MapTheme != sim.MapThemeTwinCities {
		t.Fatalf("Game map theme switch to Twin Cities failed")
	}

	// 11. Test HUD Menu button click returning to Main Menu
	hud := g.HUD
	hud.MenuBtnBounds = sim.Rect{X: 10, Y: 10, W: 120, H: 30}
	hud.MapThemeBtnBounds = hud.MenuBtnBounds
	clicked := hud.HandleMenuBtnClick(sim.Vec2{X: 20, Y: 20})
	if !clicked || !hud.ReturnToMenuRequested {
		t.Fatal("Expected HUD Menu button click to request return to menu")
	}

	// Update game should transition state back to StateMainMenu
	err := g.Update()
	if err != nil {
		t.Fatalf("Unexpected error from g.Update(): %v", err)
	}
	if g.State != game.StateMainMenu {
		t.Fatalf("Expected game state to return to StateMainMenu, got %v", g.State)
	}
}

func TestFiveDistinctCityMapsAndMainMenu(t *testing.T) {
	laneW := 52.0

	// 1. Verify 5 completely distinct maps with unique road topologies, stops, and geography
	type mapSpec struct {
		theme            sim.MapTheme
		expectedMinRoads int
		expectedMinStops int
		expectedBridges  int
		expectedWaters   int
	}

	specs := []mapSpec{
		{theme: sim.MapThemeMetropolis, expectedMinRoads: 10, expectedMinStops: 12, expectedBridges: 3, expectedWaters: 1},
		{theme: sim.MapThemeCoastalBay, expectedMinRoads: 9, expectedMinStops: 8, expectedBridges: 1, expectedWaters: 1},
		{theme: sim.MapThemeRiverValley, expectedMinRoads: 12, expectedMinStops: 8, expectedBridges: 2, expectedWaters: 1},
		{theme: sim.MapThemeEmeraldLake, expectedMinRoads: 10, expectedMinStops: 8, expectedBridges: 4, expectedWaters: 1},
		{theme: sim.MapThemeTwinCities, expectedMinRoads: 11, expectedMinStops: 8, expectedBridges: 1, expectedWaters: 1},
	}

	for _, sp := range specs {
		w := sim.NewWorld(laneW, sp.theme)
		if w.Theme != sp.theme {
			t.Fatalf("Theme mismatch for %v", sp.theme)
		}
		if len(w.Roads) < sp.expectedMinRoads {
			t.Fatalf("Map %s has insufficient roads: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Roads), sp.expectedMinRoads)
		}
		if len(w.Stops) < sp.expectedMinStops {
			t.Fatalf("Map %s has insufficient stops: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Stops), sp.expectedMinStops)
		}
		if len(w.Bridges) != sp.expectedBridges {
			t.Fatalf("Map %s bridge count mismatch: %d (expected %d)", sp.theme.DisplayName(), len(w.Bridges), sp.expectedBridges)
		}
		if len(w.Waters) < sp.expectedWaters {
			t.Fatalf("Map %s water bodies mismatch: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Waters), sp.expectedWaters)
		}
		if len(w.Intersections) == 0 {
			t.Fatalf("Map %s should dynamically compute intersections", sp.theme.DisplayName())
		}

		// Verify every road segment in the map is strictly orthogonal and clean (no unnatural jagged piecewise lines)
		for _, r := range w.Roads {
			isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
			isNS := math.Abs(r.P1.X-r.P2.X) < 1.0
			if !isEW && !isNS {
				t.Fatalf("Map %s has non-orthogonal unnatural road: %s (%+v -> %+v)", sp.theme.DisplayName(), r.Name, r.P1, r.P2)
			}
		}
	}

	// 2. Verify Map 5 Grade-Separated Highway Overpass Viaduct
	wTwin := sim.NewWorld(laneW, sim.MapThemeTwinCities)
	foundOverpass := false
	for _, br := range wTwin.Bridges {
		if br.IsOverpass && br.Style() == sim.BridgeStyleOverpass {
			foundOverpass = true
			if br.Length < 300 || br.Width < 336 {
				t.Fatalf("Overpass dimensions too small to cover 6-lane highway: length %f, width %f", br.Length, br.Width)
			}
		}
	}
	if !foundOverpass {
		t.Fatal("Map 5 must include a grade-separated highway overpass viaduct")
	}
	// Verify that the overpass at (0, 0) is recognized and does not generate an at-grade intersection
	if !wTwin.IsOverpassAt(sim.Vec2{X: 0, Y: 0}) {
		t.Fatal("Expected IsOverpassAt(0, 0) to be true for Map 5")
	}
	for _, ipt := range wTwin.Intersections {
		if ipt.Distance(sim.Vec2{X: 0, Y: 0}) < 10 {
			t.Fatalf("Map 5 must not generate an at-grade intersection on top of the overpass at (0, 0)")
		}
	}
	for _, riv := range wTwin.Rivers {
		if math.Abs(riv.Y) < 10 {
			t.Fatalf("Map 5 must not have a horizontal river conflict at Y=0")
		}
	}

	// 3. Main Menu System State Flow
	g := game.NewGame(1280, 720)
	if g.State != game.StateMainMenu {
		t.Fatalf("Game must boot directly into Main Menu State")
	}

	// Simulate selecting Map 4 (Emerald Lake) in Main Menu
	g.SelectedMapTheme = sim.MapThemeEmeraldLake
	g.EnterCity(g.SelectedMapTheme)
	if g.State != game.StatePlaying {
		t.Fatalf("Entering city must switch game state to StatePlaying")
	}
	if g.World.Theme != sim.MapThemeEmeraldLake {
		t.Fatalf("World theme must be Emerald Lake")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus must spawn inside Station Bay 0")
	}
	if g.Bus.Speed != 0 || !g.Bus.DoorsOpen {
		t.Fatalf("Bus at starting bay must be stopped with doors open")
	}

	// Simulate returning to Main Menu via in-game Settings
	g.Settings.IsOpen = true
	g.Settings.ReturnToMenuRequested = true
	err := g.Update()
	if err != nil {
		t.Fatalf("Update failed: %v", err)
	}
	if g.State != game.StateMainMenu {
		t.Fatalf("Settings ReturnToMenuRequested must return to StateMainMenu")
	}

	// Simulate selecting Map 5 (Twin Cities) and entering
	g.SelectedMapTheme = sim.MapThemeTwinCities
	g.EnterCity(g.SelectedMapTheme)
	if g.State != game.StatePlaying || g.World.Theme != sim.MapThemeTwinCities {
		t.Fatalf("Entering Twin Cities map failed")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus must reposition to Stop 0 of the new city")
	}

	// Simulate returning to Main Menu via HUD top-bar button
	g.HUD.ReturnToMenuRequested = true
	err = g.Update()
	if err != nil {
		t.Fatalf("Update failed: %v", err)
	}
	if g.State != game.StateMainMenu {
		t.Fatalf("HUD ReturnToMenuRequested must return to StateMainMenu")
	}
}

func TestCustomMapLoadingValidationAndReloading(t *testing.T) {
	// 1. Verify default preset map data integrity
	defaultMap := sim.GetDefaultCustomMapData()
	if defaultMap == nil {
		t.Fatal("GetDefaultCustomMapData returned nil")
	}
	if err := sim.ValidateCustomMapData(defaultMap); err != nil {
		t.Fatalf("Default custom map failed validation: %v", err)
	}
	if len(defaultMap.Stops) != 12 {
		t.Fatalf("Default custom map should have 12 stops, got %d", len(defaultMap.Stops))
	}
	if len(defaultMap.Roads) < 10 {
		t.Fatalf("Default custom map should have >= 10 roads, got %d", len(defaultMap.Roads))
	}
	if len(defaultMap.Roundabouts) == 0 {
		t.Fatal("Default custom map should have at least 1 roundabout")
	}
	if len(defaultMap.Bridges) == 0 {
		t.Fatal("Default custom map should have at least 1 bridge")
	}

	// 2. Verify all presets pass validation
	presets := sim.GetCustomMapPresets()
	if len(presets) < 3 {
		t.Fatalf("Expected at least 3 custom map presets, got %d", len(presets))
	}
	for i, preset := range presets {
		if err := sim.ValidateCustomMapData(preset); err != nil {
			t.Fatalf("Preset #%d (%s) failed validation: %v", i, preset.Name, err)
		}
		if len(preset.Stops) < 6 {
			t.Fatalf("Preset #%d (%s) should have >= 6 stops, got %d", i, preset.Name, len(preset.Stops))
		}
	}

	// 3. Verify validation error rejection
	// 3a. Empty name
	invalidMap := *defaultMap
	invalidMap.Name = ""
	if err := sim.ValidateCustomMapData(&invalidMap); err == nil {
		t.Fatal("Expected error for empty map name")
	}

	// 3b. Insufficient roads
	invalidMap = *defaultMap
	invalidMap.Roads = nil
	if err := sim.ValidateCustomMapData(&invalidMap); err == nil {
		t.Fatal("Expected error for missing roads")
	}

	// 3c. Non-orthogonal road
	invalidMap = *defaultMap
	invalidMap.Roads = []sim.CustomRoadData{
		{P1: [2]float64{0, 0}, P2: [2]float64{500, 0}, Lanes: 2, Name: "Road 1"},
		{P1: [2]float64{0, 0}, P2: [2]float64{500, 500}, Lanes: 2, Name: "Diagonal Road"},
	}
	if err := sim.ValidateCustomMapData(&invalidMap); err == nil || !strings.Contains(err.Error(), "orthogonal") {
		t.Fatalf("Expected non-orthogonal error, got %v", err)
	}

	// 3d. Duplicate stop ID
	invalidMap = *defaultMap
	invalidMap.Stops = []sim.CustomStopData{
		{ID: 1, Name: "Stop A", Pos: [2]float64{0, 0}, Lanes: 2},
		{ID: 1, Name: "Stop B", Pos: [2]float64{100, 0}, Lanes: 2},
	}
	if err := sim.ValidateCustomMapData(&invalidMap); err == nil || !strings.Contains(err.Error(), "duplicate") {
		t.Fatalf("Expected duplicate stop ID error, got %v", err)
	}

	// 4. Test file serialization roundtrip
	tmpDir, err := os.MkdirTemp("", "custom_map_test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	tmpFile := filepath.Join(tmpDir, "custom_map.json")
	if err := sim.SaveCustomMapToFile(tmpFile, defaultMap); err != nil {
		t.Fatalf("Failed to save custom map to file: %v", err)
	}

	loadedMap, err := sim.LoadCustomMapFromFile(tmpFile)
	if err != nil {
		t.Fatalf("Failed to load custom map from file: %v", err)
	}
	if loadedMap.Name != defaultMap.Name {
		t.Fatalf("Loaded map name mismatch: expected %s, got %s", defaultMap.Name, loadedMap.Name)
	}
	if len(loadedMap.Roads) != len(defaultMap.Roads) {
		t.Fatalf("Loaded road count mismatch: expected %d, got %d", len(defaultMap.Roads), len(loadedMap.Roads))
	}
	if len(loadedMap.Stops) != len(defaultMap.Stops) {
		t.Fatalf("Loaded stop count mismatch: expected %d, got %d", len(defaultMap.Stops), len(loadedMap.Stops))
	}

	// 5. Test World instantiation with custom map
	w := sim.NewWorld(48.0, sim.MapThemeCustom)
	if w.Theme != sim.MapThemeCustom {
		t.Fatalf("Expected world theme Custom, got %v", w.Theme)
	}
	if len(w.Stops) == 0 {
		t.Fatal("Custom world should have populated stops")
	}
	if len(w.Intersections) == 0 {
		t.Fatal("Custom world should have computed road intersections")
	}

	// 6. Test In-Game Custom Map Lifecycle & Reload
	g := game.NewGame(1280, 720)
	g.SelectedMapTheme = sim.MapThemeCustom
	g.EnterCity(g.SelectedMapTheme)
	if g.World.Theme != sim.MapThemeCustom {
		t.Fatalf("Expected Game to be on Custom Map theme")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus should spawn at stop 0 in custom map")
	}

	// Hot reload custom map
	err = g.ReloadCustomMap()
	if err != nil {
		t.Fatalf("ReloadCustomMap failed: %v", err)
	}
	if len(g.HUD.Toasts) == 0 {
		t.Fatal("ReloadCustomMap should issue HUD toast feedback")
	}
}
