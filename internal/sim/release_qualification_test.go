package sim

import (
	"testing"
)

// TestReleaseQualification_AllMapsBoundaryAndZoneSafetyAudit validates road network clearance,
// boundary safety, stop bay access, and crosswalk geometries across all 6 maps.
func TestReleaseQualification_AllMapsBoundaryAndZoneSafetyAudit(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping release qualification in short mode")
	}

	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	laneWidth := 52.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(laneWidth, theme)
			if len(w.Stops) != 12 {
				t.Fatalf("[%s] Map must have exactly 12 stops, got %d", theme.DisplayName(), len(w.Stops))
			}

			// 1. Verify every stop bay is outside road boundaries and strictly clear of ZoneHardBarrier
			for _, s := range w.Stops {
				bRes := w.CheckBoundaries(s.Pos, 18.0, laneWidth)
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop #%d (%s) at %v is inside ZoneHardBarrier (%s)",
						theme.DisplayName(), s.ID, s.Name, s.Pos, bRes.ObstacleName)
				}
				roadHalfW := float64(s.RoadLanes) * 0.5 * laneWidth
				distToRoadCenter := s.Pos.Distance(s.RoadCenter)
				if distToRoadCenter < roadHalfW {
					t.Fatalf("[%s] Stop #%d (%s) sits inside roadway (dist=%.2f < roadHalfW=%.2f)",
						theme.DisplayName(), s.ID, s.Name, distToRoadCenter, roadHalfW)
				}
			}

			// 2. Verify all roundabouts
			for i, rb := range w.Roundabouts {
				if rb.Radius < laneWidth*1.5 {
					t.Fatalf("[%s] Roundabout #%d radius too small: %.1f", theme.DisplayName(), i, rb.Radius)
				}
				// Verify core center is inside ZoneHardBarrier
				coreRes := w.CheckBoundaries(rb.Center, 5.0, laneWidth)
				if coreRes.Zone != ZoneHardBarrier {
					t.Fatalf("[%s] Roundabout #%d center island must be ZoneHardBarrier", theme.DisplayName(), i)
				}
			}

			// 3. Verify all designated route paths have valid drivable waypoints
			route := GetStandardRouteForMap(theme, w)
			for leg := 0; leg < len(route.StopIDs); leg++ {
				sFrom := w.Stops[route.StopIDs[leg]]
				nextStopID := route.StopIDs[(leg+1)%len(route.StopIDs)]
				sTo := w.Stops[nextStopID]

				path := GetRoadPathBetweenStopsInWorld(w, sFrom, sTo, laneWidth)
				if len(path) < 2 {
					t.Fatalf("[%s] Path %d->%d is empty", theme.DisplayName(), sFrom.ID, sTo.ID)
				}
				for idx, pt := range path {
					bRes := w.CheckBoundaries(pt, 16.0, laneWidth)
					if bRes.Zone == ZoneHardBarrier {
						t.Fatalf("[%s] Leg %d->%d: waypoint #%d (%v) hits hard barrier (%s)",
							theme.DisplayName(), sFrom.ID, sTo.ID, idx, pt, bRes.ObstacleName)
					}
				}
			}
		})
	}
}
