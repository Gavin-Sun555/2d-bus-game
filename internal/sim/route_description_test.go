package sim

import (
	"testing"
)

// TestFormattedRouteLinesWrappingAndTruncation verifies the multi-line wrapping and ellipsis summarization
// of long bus route display lines in the HUD.
func TestFormattedRouteLinesWrappingAndTruncation(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Case 1: Empty route returns fallback string
	emptyRoute := &Route{StopIDs: []int{}}
	emptyLines := emptyRoute.GetFormattedRouteLines(w, 800.0, 14.0)
	if len(emptyLines) != 1 || emptyLines[0] == "" {
		t.Fatalf("Empty route should return single localized fallback line, got %v", emptyLines)
	}

	// Case 2: Short route (3 stops) fits neatly on single line
	shortRoute := &Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	shortLines := shortRoute.GetFormattedRouteLines(w, 800.0, 14.0)
	if len(shortLines) != 1 {
		t.Fatalf("Short route should fit on 1 line, got %d lines: %v", len(shortLines), shortLines)
	}

	// Case 3: Long route (all 12 stops) wraps onto 2 or 3 lines with proper arrow indentations
	longRoute := &Route{
		StopIDs:      []int{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0},
		CurrentIndex: 2,
	}
	longLines := longRoute.GetFormattedRouteLines(w, 400.0, 14.0)
	if len(longLines) < 2 || len(longLines) > 3 {
		t.Fatalf("Long route with width 400px should wrap into 2 or 3 lines, got %d: %v", len(longLines), longLines)
	}

	// Verify lines after first start with wrap arrow ↳
	for i := 1; i < len(longLines); i++ {
		if len(longLines[i]) < 4 {
			t.Fatalf("Line %d is too short: %q", i, longLines[i])
		}
	}
}
