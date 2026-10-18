package sim

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestCustomMapSerializationRoundtripAndValidation(t *testing.T) {
	data := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(data); err != nil {
		t.Fatalf("Default custom map data validation failed: %v", err)
	}

	// Validation constraints
	badData := *data
	badData.Name = ""
	if err := ValidateCustomMapData(&badData); err == nil {
		t.Errorf("Expected validation error for empty map name")
	}

	badRoads := *data
	badRoads.Roads = []CustomRoadData{{P1: [2]float64{0, 0}, P2: [2]float64{100, 0}, Lanes: 2}}
	if err := ValidateCustomMapData(&badRoads); err == nil {
		t.Errorf("Expected validation error for < 2 roads")
	}

	// colorToHex
	hexOpaque := colorToHex(RGBA(255, 128, 0, 255))
	if hexOpaque != "#FF8000" {
		t.Errorf("Expected #FF8000, got %s", hexOpaque)
	}
	hexAlpha := colorToHex(RGBA(255, 128, 0, 180))
	if hexAlpha != "#FF8000B4" {
		t.Errorf("Expected #FF8000B4, got %s", hexAlpha)
	}

	// Save and Load roundtrip via temporary directory
	tempDir := t.TempDir()
	tempPath := filepath.Join(tempDir, "test_custom_map.json")

	if err := SaveCustomMapToFile(tempPath, data); err != nil {
		t.Fatalf("SaveCustomMapToFile failed: %v", err)
	}

	loaded, err := LoadCustomMapFromFile(tempPath)
	if err != nil {
		t.Fatalf("LoadCustomMapFromFile failed: %v", err)
	}

	if loaded.Name != data.Name {
		t.Errorf("Loaded map name mismatch: got %s, expected %s", loaded.Name, data.Name)
	}
	if len(loaded.Roads) != len(data.Roads) {
		t.Errorf("Loaded roads count mismatch: got %d, expected %d", len(loaded.Roads), len(data.Roads))
	}

	// EnsureDefaultCustomMapFile on non-existing path creates the file
	ensurePath := filepath.Join(tempDir, "ensure_map.json")
	if err := EnsureDefaultCustomMapFile(ensurePath); err != nil {
		t.Fatalf("EnsureDefaultCustomMapFile failed: %v", err)
	}
	if _, err := os.Stat(ensurePath); os.IsNotExist(err) {
		t.Errorf("EnsureDefaultCustomMapFile failed to write file")
	}
}

func TestCustomMapStopTooCloseToIntersectionValidation(t *testing.T) {
	customMap := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(customMap); err != nil {
		t.Fatalf("Default custom map failed validation: %v", err)
	}

	// Deliberately place a stop right on an intersection (0, 0)
	invalidMap := *customMap
	invalidStops := make([]CustomStopData, len(customMap.Stops))
	copy(invalidStops, customMap.Stops)
	invalidStops[0].Pos = [2]float64{0, 0}
	invalidMap.Stops = invalidStops

	err := ValidateCustomMapData(&invalidMap)
	if err == nil {
		t.Fatalf("Expected validation error for stop at intersection (0, 0), got nil")
	}
	if !strings.Contains(err.Error(), "距离路口过近") {
		t.Fatalf("Expected error message about being too close to intersection, got: %v", err)
	}
}

// TestCustomMapParallelRoadSpacingValidation verifies that ValidateCustomMapData
// rejects maps containing parallel roads with distance < 400px and accepts well-spaced maps.
func TestCustomMapParallelRoadSpacingValidation(t *testing.T) {
	// Base valid map
	base := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(base); err != nil {
		t.Fatalf("Default custom map must pass validation: %v", err)
	}

	// 1. Add parallel EW road 200px away (too close)
	badEW := GetDefaultCustomMapData()
	badEW.Roads = append(badEW.Roads, CustomRoadData{
		P1:    [2]float64{-1000, -1000},
		P2:    [2]float64{1000, -1000},
		Lanes: 2,
		Name:  "平行过近加塞路 (200px away from Y=-1200)",
	})
	if err := ValidateCustomMapData(badEW); err == nil {
		t.Fatalf("Expected validation error for parallel EW roads 200px apart, got nil")
	}

	// 2. Add parallel NS road 300px away (too close)
	badNS := GetDefaultCustomMapData()
	badNS.Roads = append(badNS.Roads, CustomRoadData{
		P1:    [2]float64{300, -1000},
		P2:    [2]float64{300, 1000},
		Lanes: 2,
		Name:  "垂直平行过近加塞路 (300px away from X=0)",
	})
	if err := ValidateCustomMapData(badNS); err == nil {
		t.Fatalf("Expected validation error for parallel NS roads 300px apart, got nil")
	}

	// 3. Add parallel EW road 500px away (well spaced)
	goodEW := GetDefaultCustomMapData()
	goodEW.Roads = append(goodEW.Roads, CustomRoadData{
		P1:    [2]float64{-1000, -1700},
		P2:    [2]float64{1000, -1700},
		Lanes: 2,
		Name:  "安全间距平行外环路 (500px away from Y=-1200)",
	})
	if err := ValidateCustomMapData(goodEW); err != nil {
		t.Fatalf("Expected valid custom map with 500px parallel spacing, got error: %v", err)
	}
}
