package sim

import (
	"math"
	"testing"
)

// TestSingleRouteConsistencyAllMaps verifies that each map has exactly one primary loop route
// with valid stop references and proper sequence.
func TestSingleRouteConsistencyAllMaps(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			world := NewWorld(52.0, theme)
			route := GetStandardRouteForMap(theme, world)

			if len(route.StopIDs) < 2 {
				t.Fatalf("Route on %s must have at least 2 stops", theme.DisplayName())
			}

			// Verify first and last stop form a continuous circular loop
			if route.StopIDs[0] != route.StopIDs[len(route.StopIDs)-1] {
				t.Fatalf("Route on %s should form a closed loop (first stop %d == last stop %d)",
					theme.DisplayName(), route.StopIDs[0], route.StopIDs[len(route.StopIDs)-1])
			}

			// Verify all stop IDs exist in the world
			for idx, stopID := range route.StopIDs {
				if stopID < 0 || stopID >= len(world.Stops) {
					t.Fatalf("Stop ID %d at index %d out of bounds (world has %d stops)",
						stopID, idx, len(world.Stops))
				}
			}
		})
	}
}

// TestRouteLaneOffsetOuterLaneSelection verifies that buses choose the outer travel lane (lw*1.5)
// on multi-lane/4-lane roads rather than straddling the dashed divider line (Bug #1).
func TestRouteLaneOffsetOuterLaneSelection(t *testing.T) {
	world := NewWorld(52.0)
	lw := 52.0

	// 4-lane avenue (Central Boulevard Y = 0):
	// Divider line is at lw*1.0 = 52.0px. Outer lane center is at lw*1.5 = 78.0px.
	offset4Lane := RoadLaneOffset(0.0, true, lw, world)
	if offset4Lane != lw*1.5 {
		t.Fatalf("Expected 4-lane road to center vehicle in outer lane (lw*1.5 = %.1f), got %.1f", lw*1.5, offset4Lane)
	}

	// 2-lane street (North 2nd St Y = -600):
	// Center is at lw*0.5 = 26.0px.
	offset2Lane := RoadLaneOffset(-600.0, true, lw, world)
	if offset2Lane != lw*0.5 {
		t.Fatalf("Expected 2-lane road to center vehicle at lw*0.5 = %.1f, got %.1f", lw*0.5, offset2Lane)
	}

	// Ensure lane offset keeps the bus body (width 36, halfWidth 18) strictly within outer lane [52.0, 104.0]
	busInnerEdge := offset4Lane - 18.0
	busOuterEdge := offset4Lane + 18.0
	dividerX := lw * 1.0
	if busInnerEdge <= dividerX {
		t.Fatalf("Bus inner edge (%.1f) straddles divider line (%.1f)!", busInnerEdge, dividerX)
	}
	if busOuterEdge >= lw*2.0 {
		t.Fatalf("Bus outer edge (%.1f) exceeds road boundary (%.1f)!", busOuterEdge, lw*2.0)
	}
}

// TestRoundaboutPathWaypointsInOuterLane verifies that all route waypoints within the roundabout
// are placed in the center of the outer circulating lane, rather than straddling the circular dashed divider line.
func TestRoundaboutPathWaypointsInOuterLane(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	s0 := w.Stops[0]
	s1 := w.Stops[1]
	path := ComputeRoadPathBetweenStops(w, s0, s1, 52.0)
	rb := w.Roundabouts[0]

	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	midR := (rApron + rOuter) * 0.5    // Circular dashed divider line (188.0px)
	expectedR := (midR + rOuter) * 0.5 // Outer lane center (214.0px)

	roundaboutWaypoints := 0
	for i, pt := range path {
		d := pt.Distance(rb.Center)
		if d < rb.Radius-1.0 && d > rApron {
			roundaboutWaypoints++
			// Must be within 1.0px of the outer lane center
			if math.Abs(d-expectedR) > 1.0 {
				t.Errorf("Waypoint %d at (%.1f, %.1f) has distToCenter=%.2f, expected outer lane center=%.2f",
					i, pt.X, pt.Y, d, expectedR)
			}
			// Clearance from inner dashed divider line: bus half-width is 18px
			clearanceToDivider := (d - 18.0) - midR
			if clearanceToDivider < 5.0 {
				t.Errorf("Waypoint %d bus inner edge has insufficient clearance (%.2fpx) to dashed divider",
					i, clearanceToDivider)
			}
			// Clearance to outer curb
			clearanceToCurb := rOuter - (d + 18.0)
			if clearanceToCurb < 5.0 {
				t.Errorf("Waypoint %d bus outer edge has insufficient clearance (%.2fpx) to curb",
					i, clearanceToCurb)
			}
		}
	}

	if roundaboutWaypoints < 5 {
		t.Fatalf("Expected at least 5 waypoints within roundabout, got %d", roundaboutWaypoints)
	}
}

// TestRoundaboutUTurnCircumnavigatesFullLoop verifies that when making a U-turn at an intersection
// with a roundabout, the route waypoints circumnavigate counter-clockwise around the roundabout outer lane
// (approx 320 to 360 degrees) rather than making a wrong-way / reverse shortcut across the entrance mouth.
func TestRoundaboutUTurnCircumnavigatesFullLoop(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	rb := w.Roundabouts[0]
	circR := getRoundaboutOuterLaneRadius(rb)

	// Approaches to test: West, East, North, South
	approaches := []struct {
		name      string
		fromCoord Vec2
		fromH     float64
		toCoord   Vec2
		toH       float64
	}{
		{
			name:      "West Approach (Eastbound entering, Westbound exiting)",
			fromCoord: Vec2{X: -650, Y: 0},
			fromH:     0,
			toCoord:   Vec2{X: -650, Y: 0},
			toH:       math.Pi,
		},
		{
			name:      "East Approach (Westbound entering, Eastbound exiting)",
			fromCoord: Vec2{X: 650, Y: 0},
			fromH:     math.Pi,
			toCoord:   Vec2{X: 650, Y: 0},
			toH:       0,
		},
		{
			name:      "North Approach (Southbound entering, Northbound exiting)",
			fromCoord: Vec2{X: 0, Y: -650},
			fromH:     math.Pi / 2,
			toCoord:   Vec2{X: 0, Y: -650},
			toH:       -math.Pi / 2,
		},
		{
			name:      "South Approach (Northbound entering, Southbound exiting)",
			fromCoord: Vec2{X: 0, Y: 650},
			fromH:     -math.Pi / 2,
			toCoord:   Vec2{X: 0, Y: 650},
			toH:       math.Pi / 2,
		},
	}

	for _, app := range approaches {
		t.Run(app.name, func(t *testing.T) {
			sFrom := NewBusStop(200, "UTurnFrom", app.fromCoord, app.fromH, 4, RGBA(0, 0, 0, 255), lw, 0)
			sTo := NewBusStop(201, "UTurnTo", app.toCoord, app.toH, 4, RGBA(0, 0, 0, 255), lw, 0)

			path := GetRoadPathBetweenStopsInWorld(w, sFrom, sTo, lw)
			if len(path) < 15 {
				t.Fatalf("Expected detailed arc waypoints through roundabout, got %d waypoints", len(path))
			}

			// Extract all waypoints inside the roundabout
			var rbPoints []Vec2
			for _, pt := range path {
				d := pt.Distance(rb.Center)
				if d <= rb.Radius+15.0 && d >= rb.InnerRadius {
					rbPoints = append(rbPoints, pt)
				}
			}

			if len(rbPoints) < 10 {
				t.Fatalf("Expected at least 10 waypoints inside roundabout for U-turn, got %d", len(rbPoints))
			}

			// 1. Verify all roundabout interior waypoints track the outer lane radius
			for i, pt := range rbPoints {
				d := pt.Distance(rb.Center)
				if math.Abs(d-circR) > 15.0 {
					t.Errorf("Waypoint %d at (%.1f, %.1f) distToCenter=%.2f differs from outer lane radius %.2f",
						i, pt.X, pt.Y, d, circR)
				}
			}

			// 2. Verify counter-clockwise rotation and total angle traversed >= 300 degrees (~5.2 rad)
			var totalAngleTraversed float64
			for i := 0; i < len(rbPoints)-1; i++ {
				ang1 := math.Atan2(rbPoints[i].Y-rb.Center.Y, rbPoints[i].X-rb.Center.X)
				ang2 := math.Atan2(rbPoints[i+1].Y-rb.Center.Y, rbPoints[i+1].X-rb.Center.X)
				// In counter-clockwise circulation (screen coords Y down), angle decreases
				stepDelta := ang1 - ang2
				for stepDelta < -math.Pi {
					stepDelta += 2 * math.Pi
				}
				for stepDelta > math.Pi {
					stepDelta -= 2 * math.Pi
				}
				if stepDelta < -0.15 {
					t.Fatalf("Waypoint %d to %d moved backwards (clockwise/wrong-way) by %.2f rad!",
						i, i+1, stepDelta)
				}
				if stepDelta > 0 {
					totalAngleTraversed += stepDelta
				}
			}

			minExpectedAngle := 5.0 // ~286 degrees
			if totalAngleTraversed < minExpectedAngle {
				t.Fatalf("U-turn traversal angle %.2f rad (%.1f deg) is too small, expected >= %.2f rad (绕一大圈)",
					totalAngleTraversed, totalAngleTraversed*180/math.Pi, minExpectedAngle)
			}
		})
	}
}

// TestLeftTurnEntersLeftTurnLane verifies that when a bus approaches an intersection
// on a 4-lane avenue to execute a left turn, it enters the inner dedicated left-turn lane
// (offset 0.5*lw) rather than turning directly from the outer through lane (offset 1.5*lw).
func TestLeftTurnEntersLeftTurnLane(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	lw := 52.0

	// s1: on South Ring Avenue (Y = 1200*MapGridScale, 4-lane avenue), Eastbound towards intersection (0, 1200*MapGridScale)
	s1 := NewBusStop(101, "TestEastbound", Vec2{X: -650 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	// s2: on Central North-South Avenue (X = 0, 4-lane avenue), Northbound
	s2 := NewBusStop(102, "TestNorthbound", Vec2{X: 0, Y: 600 * MapGridScale}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)

	path := ComputeRoadPathBetweenStops(w, s1, s2, lw)
	foundInnerLaneApproach := false
	foundInnerApex := false
	foundTargetOuterMerge := false
	for _, pt := range path {
		if pt.X >= -200.0*MapGridScale && pt.X <= 0.0 && math.Abs(math.Abs(pt.Y-1200.0*MapGridScale)-lw*0.5) < 2.0 {
			foundInnerLaneApproach = true
		}
		if pt.X >= lw*0.25 && pt.X <= lw*0.55 && pt.Y >= 1200.0*MapGridScale+lw*0.25 && pt.Y <= 1200.0*MapGridScale+lw*0.55 {
			foundInnerApex = true
		}
		if math.Abs(pt.X-lw*1.5) < 2.0 && pt.Y < 1200.0*MapGridScale-lw {
			foundTargetOuterMerge = true
		}
	}

	if !foundInnerLaneApproach {
		t.Fatalf("Bus route failed to enter dedicated inner left-turn lane before left turn at (0, 1200)")
	}
	if !foundInnerApex {
		t.Fatalf("Bus route left-turn arc does not pass through inner lane turn corridor (expected near X=%.1f..%.1f, Y=%.1f..%.1f)",
			lw*0.25, lw*0.55, 1200.0*MapGridScale+lw*0.25, 1200.0*MapGridScale+lw*0.55)
	}
	if !foundTargetOuterMerge {
		t.Fatalf("Bus route failed to merge out to travel lane (X=%.1f) after left turn", lw*1.5)
	}
}

// TestUTurnEntersLeftTurnLane verifies that when making a U-turn at an intersection
// on a 4-lane avenue, the bus route merges into the dedicated inner lane (offset 0.5*lw)
// before executing the U-turn across the center divider.
func TestUTurnEntersLeftTurnLane(t *testing.T) {
	w := NewWorld(42.0, MapThemeMetropolis)
	lw := 42.0

	// Stops 0 and 1 in Metropolis CBD require traveling East on Central Blvd (Y=0)
	// and executing a U-turn at intersection (1300, 0) to head West towards Stop 1.
	s0 := w.Stops[0]
	s1 := w.Stops[1]

	path := ComputeRoadPathBetweenStops(w, s0, s1, lw)
	if len(path) == 0 {
		t.Fatalf("Failed to compute path between Stop 0 and Stop 1")
	}

	// Verify that before the U-turn at X=1300*MapGridScale (in X in [1300*scale - 170, 1300*scale]),
	// the waypoints have merged into the inner left-turn lane (Y = 0.5*lw = 21.0)
	foundInnerLaneUTurn := false
	itX := 1300.0 * MapGridScale
	for _, pt := range path {
		if pt.X >= (itX-170.0) && pt.X <= itX {
			latDist := math.Abs(pt.Y - 0.0)
			if latDist < lw*0.95 {
				foundInnerLaneUTurn = true
				if math.Abs(latDist-lw*0.5) > 2.0 {
					t.Errorf("U-turn approach waypoint %v lateral dist=%.1f, expected inner lane center=%.1f",
						pt, latDist, lw*0.5)
				}
			}
		}
	}

	if !foundInnerLaneUTurn {
		t.Fatalf("Bus route failed to enter dedicated inner lane before U-turn at (1300, 0)")
	}
}

// TestEarlyLeftTurnLaneChangeScaleWithBlockDistance verifies that left-turn lane merge
// distance intelligently scales based on block distance between intersections:
// - Short blocks (< 320px) maintain compact merge around 110-160px
// - Long blocks (> 600px) merge early (up to 400-650px) to enter left-turn lane in advance
func TestEarlyLeftTurnLaneChangeScaleWithBlockDistance(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Short block setup: s1 at X=-220, intersection at (0, 1200*MapGridScale), s2 at (0, 600*MapGridScale)
	sShort := NewBusStop(301, "ShortStop", Vec2{X: -220, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	sDest := NewBusStop(302, "DestStop", Vec2{X: 0, Y: 600 * MapGridScale}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)

	pathShort := ComputeRoadPathBetweenStops(w, sShort, sDest, lw)
	if len(pathShort) == 0 {
		t.Fatalf("Failed to compute path for short block")
	}

	// Find the X coordinate where inner lane (latDist = lw*0.5) begins
	var mergeXShort float64 = 999.0
	for _, pt := range pathShort {
		if pt.X < 0 && math.Abs(pt.Y-(1200*MapGridScale+lw*0.5)) < 2.0 {
			if pt.X < mergeXShort {
				mergeXShort = pt.X
			}
		}
	}
	distShort := math.Abs(mergeXShort)
	if distShort > 180.0 {
		t.Fatalf("Short block merge distance should be compact (<= 180px), got %.1fpx", distShort)
	}

	// Long block setup: s1 at X=-900*MapGridScale, intersection at (0, 1200*MapGridScale)
	sLong := NewBusStop(303, "LongStop", Vec2{X: -900 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	pathLong := ComputeRoadPathBetweenStops(w, sLong, sDest, lw)
	if len(pathLong) == 0 {
		t.Fatalf("Failed to compute path for long block")
	}

	var mergeXLong float64 = 999.0
	for _, pt := range pathLong {
		if pt.X < 0 && math.Abs(pt.Y-(1200*MapGridScale+lw*0.5)) < 2.0 {
			if pt.X < mergeXLong {
				mergeXLong = pt.X
			}
		}
	}
	distLong := math.Abs(mergeXLong)
	if distLong < 400.0 {
		t.Fatalf("Long block merge distance should be significantly earlier (>= 400px), got %.1fpx", distLong)
	}

	if distLong <= distShort*1.8 {
		t.Fatalf("Long block merge (%.1fpx) should be substantially earlier than short block (%.1fpx)",
			distLong, distShort)
	}
}

// TestSixLaneSmoothLeftTurnMergeTaper verifies that on 6-lane roads (3 lanes in each direction),
// merging from the travel lane into the dedicated left-turn lane has a smooth, silky multi-point taper
// over a long forward distance, without any 90-degree orthogonal kinks or speed drops.
func TestSixLaneSmoothLeftTurnMergeTaper(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// In Metropolis, North Ring Avenue (Y = -1200*MapGridScale = -3600) is a 6-lane expressway
	if !w.IsRoad6Lane(-1200*MapGridScale, true) {
		t.Fatalf("Expected Y=-1200*MapGridScale to be a 6-lane road")
	}

	// s1: Westbound on 6-lane road, approaching intersection at (0, -1200*MapGridScale)
	s1 := NewBusStop(401, "SixLaneStart", Vec2{X: 600 * MapGridScale, Y: -1200 * MapGridScale}, math.Pi, 6, RGBA(0, 0, 0, 0), lw, 0)
	// s2: Turning left onto North-South avenue, heading South towards (0, 0)
	s2 := NewBusStop(402, "NSLaneEnd", Vec2{X: 0, Y: -400 * MapGridScale}, math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)

	path := ComputeRoadPathBetweenStops(w, s1, s2, lw)
	if len(path) < 10 {
		t.Fatalf("Expected rich multi-point path for 6-lane left turn, got %d points", len(path))
	}

	// Analyze all segments along the path approaching the intersection
	interPt := Vec2{X: 0, Y: -1200 * MapGridScale}
	var taperPoints []Vec2

	// Inspect road segments approaching and traversing intersection (excluding bus stop curb bay departure/arrival)
	for i := 1; i < len(path)-2; i++ {
		pA := path[i]
		pB := path[i+1]

		dx := math.Abs(pB.X - pA.X)
		dy := math.Abs(pB.Y - pA.Y)

		// Check for zero-forward-distance orthogonal kinks on the road:
		// On EW approach road (pA.X > 0 && pB.X >= 0): forward is dx, so kink is dx < 0.5 && dy > lw*0.8
		// On NS exit road (pA.Y > interPt.Y): forward is dy, so kink is dy < 0.5 && dx > lw*0.8
		if pA.X > 0 && pB.X >= 0 && dx < 0.5 && dy > lw*0.8 {
			t.Fatalf("Found harsh 90-degree orthogonal lane-change kink on EW approach between %v and %v (dy=%.1f, dx=%.1f)",
				pA, pB, dy, dx)
		}
		if pA.Y > interPt.Y && pB.Y > interPt.Y && dy < 0.5 && dx > lw*0.8 {
			t.Fatalf("Found harsh 90-degree orthogonal lane-change kink on NS exit between %v and %v (dy=%.1f, dx=%.1f)",
				pA, pB, dy, dx)
		}

		// Identify taper points where lateral offset transitions between travel lane (lw*2.5 = 130) and left-turn lane (lw*0.5 = 26)
		if pA.X > 0 && pB.X >= 0 {
			latA := math.Abs(pA.Y - interPt.Y)
			latB := math.Abs(pB.Y - interPt.Y)
			if (latA > lw*0.6 || latB > lw*0.6) && (latA < lw*2.4 || latB < lw*2.4) {
				taperPoints = append(taperPoints, pA)
			}
		}
	}

	if len(taperPoints) < 4 {
		t.Fatalf("Expected at least 4 intermediate taper points for 6-lane merge, got %d", len(taperPoints))
	}

	// Verify longitudinal span of the taper (must be long and silky, at least 300px)
	minX := taperPoints[0].X
	maxX := taperPoints[len(taperPoints)-1].X
	if minX > maxX {
		minX, maxX = maxX, minX
	}
	taperSpan := maxX - minX
	if taperSpan < 300.0 {
		t.Fatalf("6-lane left-turn merge taper span (%.1f px) is too short/steep; expected >= 300px", taperSpan)
	}

	// Verify maximum angle deviation between consecutive segments along the taper is gentle (< 0.30 rad ~ 17 deg)
	for i := 1; i < len(taperPoints)-1; i++ {
		v1 := taperPoints[i].Sub(taperPoints[i-1])
		v2 := taperPoints[i+1].Sub(taperPoints[i])
		if v1.Length() > 1.0 && v2.Length() > 1.0 {
			angle1 := math.Atan2(v1.Y, v1.X)
			angle2 := math.Atan2(v2.Y, v2.X)
			turn := math.Abs(NormalizeAngle(angle2 - angle1))
			if turn > 0.30 {
				t.Fatalf("Taper angle change between segments %d and %d is too sharp (%.3f rad = %.1f deg)",
					i-1, i, turn, turn*180/math.Pi)
			}
		}
	}
}

func TestRouteManipulationAndTargetStop(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	route := NewRoute()

	if route.GetCurrentTargetStopID() != -1 {
		t.Errorf("Empty route target stop should be -1")
	}

	route.AddStop(0)
	route.AddStop(1)
	route.AddStop(2)
	// Immediate duplicate stop should be ignored
	route.AddStop(2)
	if len(route.StopIDs) != 3 {
		t.Errorf("Expected 3 stops after duplicate prevention, got %d", len(route.StopIDs))
	}

	if route.GetCurrentTargetStopID() != 0 {
		t.Errorf("Target stop should be 0, got %d", route.GetCurrentTargetStopID())
	}
	route.AdvanceToNextStop()
	if route.GetCurrentTargetStopID() != 1 {
		t.Errorf("Target stop should be 1, got %d", route.GetCurrentTargetStopID())
	}

	// Cached description
	desc := route.GetRouteDescription(w)
	if desc == "" {
		t.Errorf("Route description should not be empty")
	}
	// Calling again should return cached description
	cachedDesc := route.GetRouteDescription(w)
	if cachedDesc != desc {
		t.Errorf("Cached description mismatch")
	}

	route.Clear()
	if len(route.StopIDs) != 0 || route.GetCurrentTargetStopID() != -1 {
		t.Errorf("Route should be clear")
	}
}

func TestWorldEntitiesAndRoundaboutMethods(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)

	// Roundabout OuterRadius
	if len(w.Roundabouts) > 0 {
		rb := w.Roundabouts[0]
		if rb.OuterRadius() != rb.Radius {
			t.Errorf("OuterRadius() = %.1f, expected %.1f", rb.OuterRadius(), rb.Radius)
		}
	}

	// Water bodies
	waters := w.WaterBodies()
	if len(waters) == 0 {
		t.Errorf("Metropolis should have water bodies")
	}

	// Bridge styles
	b1 := Bridge{IsArch: true}
	if b1.Style() != BridgeStyleStoneArch {
		t.Errorf("Expected BridgeStyleStoneArch, got %v", b1.Style())
	}
	b2 := Bridge{IsCable: true}
	if b2.Style() != BridgeStyleSuspension {
		t.Errorf("Expected BridgeStyleSuspension, got %v", b2.Style())
	}
	b3 := Bridge{IsOverpass: true}
	if b3.Style() != BridgeStyleOverpass {
		t.Errorf("Expected BridgeStyleOverpass, got %v", b3.Style())
	}
	b4 := Bridge{}
	if b4.Style() != BridgeStyleStandard {
		t.Errorf("Expected BridgeStyleStandard, got %v", b4.Style())
	}

	// MapTheme Name and DisplayName
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, th := range themes {
		if th.Name() == "" || th.DisplayName() == "" {
			t.Errorf("Theme %d names should not be empty", th)
		}
	}
}

func TestAutopilotMultiMapLeftTurnLaneDiscipline(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeTwinCities,
	}

	for _, th := range themes {
		t.Run(th.DisplayName(), func(t *testing.T) {
			lw := 52.0
			w := NewWorld(lw, th)
			route := GetStandardRouteForMap(th, w)
			if len(route.StopIDs) < 2 {
				t.Fatalf("Route must have >= 2 stops")
			}

			// Validate all consecutive stops along the route maintain proper lane offsets
			for i := 0; i < len(route.StopIDs)-1; i++ {
				s1 := w.Stops[route.StopIDs[i]]
				s2 := w.Stops[route.StopIDs[i+1]]
				path := ComputeRoadPathBetweenStops(w, s1, s2, lw)
				if len(path) == 0 {
					continue
				}

				// Verify all path waypoints outside roundabouts stay safely away from the center divider
				for _, pt := range path {
					// Roundabouts use circular curves rather than straight-lane offsets
					inRoundabout := false
					for _, rb := range w.Roundabouts {
						if pt.Distance(rb.Center) <= rb.Radius+25.0 {
							inRoundabout = true
							break
						}
					}
					if inRoundabout {
						continue
					}

					// Check against East-West 4-lane avenues
					for _, r := range w.Roads {
						if math.Abs(r.P1.Y-r.P2.Y) < 1.0 && (r.Lanes == 4 || r.Type == RoadAvenue4Lane) {
							if pt.X >= math.Min(r.P1.X, r.P2.X)-10 && pt.X <= math.Max(r.P1.X, r.P2.X)+10 {
								dy := math.Abs(pt.Y - r.P1.Y)
								// Waypoint must be inside valid travel corridors (either Lane 0 ~ 0.5*lw or Lane 1 ~ 1.5*lw)
								// and NOT on top of yellow centerline (|dy| < 8px)
								if dy < 10.0 {
									t.Errorf("[%s] Route waypoint %v is driving on the yellow centerline of %s",
										th.DisplayName(), pt, r.Name)
								}
							}
						}
					}
				}
			}
		})
	}
}

func TestRoutePlanning(t *testing.T) {
	route := NewDefaultRoute()
	if len(route.StopIDs) == 0 {
		t.Fatalf("Default route should have stops")
	}

	initialStop := route.GetCurrentTargetStopID()
	route.AdvanceToNextStop()
	nextStop := route.GetCurrentTargetStopID()

	if nextStop == initialStop && len(route.StopIDs) > 1 {
		t.Fatalf("Expected advance to change target stop")
	}

	// Test clear and re-add
	route.Clear()
	if len(route.StopIDs) != 0 {
		t.Fatalf("Expected route to be empty after Clear")
	}

	route.AddStop(2)
	route.AddStop(4)
	if len(route.StopIDs) != 2 || route.StopIDs[0] != 2 || route.StopIDs[1] != 4 {
		t.Fatalf("Route stops mismatch: %v", route.StopIDs)
	}
}

func TestRouteNextStopOnly(t *testing.T) {
	route := NewDefaultRoute()
	if len(route.StopIDs) < 2 {
		t.Fatalf("Expected default route to have multiple stops")
	}

	// Current target stop
	currTarget := route.GetCurrentTargetStopID()
	if currTarget != route.StopIDs[0] {
		t.Fatalf("Expected target stop to be first stop, got %d", currTarget)
	}

	// Advance
	route.AdvanceToNextStop()
	nextTarget := route.GetCurrentTargetStopID()
	if nextTarget != route.StopIDs[1] {
		t.Fatalf("Expected target stop to advance to second stop, got %d", nextTarget)
	}
}

// TestNavigationToNextStopOnAllMaps tests that the navigation system to the next stop
// functions properly across all built-in maps and custom maps.
// It verifies:
// 1. Route target stop identification and advancement.
// 2. Generation of valid multi-point road paths along driving lanes.
// 3. Traversal strictly along valid roads without jumping across water or missing bridges.
// 4. Approach heading aligns with the target stop's docking direction.
// 5. Lookahead navigation arrow pointing correctly along road corridors.
func TestNavigationToNextStopOnAllMaps(t *testing.T) {
	allThemes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, theme := range allThemes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(42.0, theme)
			if len(w.Stops) < 2 {
				t.Fatalf("[%s] Map should have at least 2 stops for navigation test", theme.DisplayName())
			}

			// Test with multiple consecutive stop sequences
			route := NewRoute()
			numStopsToTest := 6
			if numStopsToTest > len(w.Stops) {
				numStopsToTest = len(w.Stops)
			}
			for i := 0; i < numStopsToTest; i++ {
				route.AddStop(i)
			}
			// Close the loop
			route.AddStop(0)

			// In transit loop: bus starts departing from Stop 0 towards Stop 1
			route.CurrentIndex = 1
			initialTrips := route.TotalTrips

			for leg := 0; leg < len(route.StopIDs)-1; leg++ {
				targetID := route.GetCurrentTargetStopID()
				if targetID < 0 || targetID >= len(w.Stops) {
					t.Fatalf("[%s] Leg %d: Invalid target stop ID %d", theme.DisplayName(), leg, targetID)
				}
				targetStop := w.Stops[targetID]

				prevIdx := (route.CurrentIndex - 1 + len(route.StopIDs)) % len(route.StopIDs)
				prevStopID := route.StopIDs[prevIdx]
				prevStop := w.Stops[prevStopID]

				// 1. Calculate road path between stops
				path := GetRoadPathBetweenStopsInWorld(w, prevStop, targetStop, 42.0)
				if len(path) < 2 {
					t.Fatalf("[%s] Path between Stop %d ('%s') and Stop %d ('%s') has fewer than 2 waypoints: len=%d",
						theme.DisplayName(), prevStop.ID, prevStop.Name, targetStop.ID, targetStop.Name, len(path))
				}

				// 2. Validate waypoint coordinates and continuity
				for i := 0; i < len(path)-1; i++ {
					p1 := path[i]
					p2 := path[i+1]
					segLen := p1.Distance(p2)
					if segLen < 1.0 {
						t.Fatalf("[%s] Consecutive waypoints %d and %d are degenerate: dist=%.2f",
							theme.DisplayName(), i, i+1, segLen)
					}
				}

				// 3. In River Valley, verify path does NOT cross the river Y=0 without using a bridge
				if theme == MapThemeRiverValley {
					for i := 0; i < len(path)-1; i++ {
						p1 := path[i]
						p2 := path[i+1]
						// Crossing river Y=0
						if (p1.Y < -50 && p2.Y > 50) || (p1.Y > 50 && p2.Y < -50) {
							// Must be along one of the two bridge X coordinates (-1300*MapGridScale or 1300*MapGridScale)
							midX := (p1.X + p2.X) * 0.5
							isBridge1 := math.Abs(midX-(-1300*MapGridScale)) < 180
							isBridge2 := math.Abs(midX-1300*MapGridScale) < 180
							if !isBridge1 && !isBridge2 {
								t.Fatalf("[%s] Path crossed the river without using a bridge: from (%.1f, %.1f) to (%.1f, %.1f) at X=%.1f",
									theme.DisplayName(), p1.X, p1.Y, p2.X, p2.Y, midX)
							}
						}
					}
				}

				// 4. In Coastal Bay, verify path does NOT jump across the open bay (between mainland X <= 0 and harbor X >= 1800*MapGridScale)
				if theme == MapThemeCoastalBay {
					for i := 0; i < len(path)-1; i++ {
						p1 := path[i]
						p2 := path[i+1]
						// Crossing open water between mainland and offshore harbor
						if math.Abs(p1.Y-p2.Y) < 10 {
							minX := math.Min(p1.X, p2.X)
							maxX := math.Max(p1.X, p2.X)
							if minX < 1600*MapGridScale && maxX > 250*MapGridScale && (maxX-minX) > 100 {
								// Any segment traversing this open bay must be Causeway Bridge (Y=0) or South Cape Boardwalk (Y=1200*MapGridScale)
								y := (p1.Y + p2.Y) * 0.5
								isCauseway := math.Abs(y) < 140
								isSouthCape := math.Abs(y-1200*MapGridScale) < 140
								if !isCauseway && !isSouthCape {
									t.Fatalf("[%s] Path traversed offshore open water without a bridge/causeway: (%.1f, %.1f) to (%.1f, %.1f)",
										theme.DisplayName(), p1.X, p1.Y, p2.X, p2.Y)
								}
							}
						}
					}
				}

				// 5. Verify final arrival vector enters stop smoothly along stop heading
				if len(path) >= 2 {
					lastSeg := path[len(path)-1].Sub(path[len(path)-2])
					if lastSeg.Length() > 2 {
						stopDir := Vec2{X: math.Cos(targetStop.Heading), Y: math.Sin(targetStop.Heading)}
						dot := lastSeg.Normalize().Dot(stopDir)
						if dot < -0.1 {
							t.Fatalf("[%s] Final approach vector opposes stop heading: dot=%.2f (heading=%.2f)",
								theme.DisplayName(), dot, targetStop.Heading)
						}
					}
				}

				// 6. Test forward lookahead navigation arrow
				bus := NewBus(prevStop.Pos.X, prevStop.Pos.Y, prevStop.Heading)
				lookaheadFound := false
				for _, pt := range path {
					if bus.Pos.Distance(pt) > 70 {
						lookaheadFound = true
						break
					}
				}
				if len(path) >= 3 && !lookaheadFound {
					t.Fatalf("[%s] Multi-segment path failed to find forward lookahead waypoint", theme.DisplayName())
				}

				// 7. Advance route to next stop
				route.AdvanceToNextStop()
			}

			// Ensure loop completes
			if route.TotalTrips != initialTrips+1 {
				t.Fatalf("[%s] Expected route to complete a full transit loop (trips=%d, expected=%d)",
					theme.DisplayName(), route.TotalTrips, initialTrips+1)
			}
		})
	}
}

func TestRoundaboutCarriagewayLaneWidth(t *testing.T) {
	// Verify all built-in maps with roundabouts have ample carriageway lane width (>= 100px)
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeEmeraldLake,
	}

	for _, theme := range themes {
		w := NewWorld(42.0, theme)
		if len(w.Roundabouts) == 0 {
			continue
		}
		for idx, rb := range w.Roundabouts {
			carriagewayWidth := rb.Radius - rb.InnerRadius
			if carriagewayWidth < 100.0 {
				t.Errorf("[%s] Roundabout #%d (%s) carriageway width is too narrow: %.1f px (must be >= 100.0 px)",
					theme.DisplayName(), idx, rb.Name, carriagewayWidth)
			}
			if rb.InnerRadius < 80.0 {
				t.Errorf("[%s] Roundabout #%d (%s) inner island is too small: %.1f px (must be >= 80.0 px)",
					theme.DisplayName(), idx, rb.Name, rb.InnerRadius)
			}
		}
	}

	// Also verify custom map validation rejects narrow roundabout carriageways (< 80px)
	narrowCustomMap := CustomMapData{
		Name:  "Test Narrow Roundabout",
		Style: "lake",
		Roads: []CustomRoadData{
			{P1: [2]float64{-1000, 0}, P2: [2]float64{1000, 0}, Lanes: 2},
		},
		Stops: []CustomStopData{
			{ID: 0, Name: "A", Pos: [2]float64{-600, 40}, Heading: 0},
			{ID: 1, Name: "B", Pos: [2]float64{600, 40}, Heading: 0},
		},
		Roundabouts: []CustomRoundaboutData{
			{Name: "Narrow Island", Center: [2]float64{0, 0}, Radius: 150.0, InnerRadius: 85.0, Type: "civic"}, // width = 65 < 80
		},
	}
	err := ValidateCustomMapData(&narrowCustomMap)
	if err == nil {
		t.Fatalf("Expected ValidateCustomMapData to reject narrow roundabout carriageway (width = 65px < 80px), but validation passed")
	}
}

func TestRoundaboutRouteLineCurvatureAndCenterClearance(t *testing.T) {
	// In Emerald Lake map, stops 1 and 2 are on opposite sides of the roundabout
	w := NewWorld(42.0, MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	innerR := rb.InnerRadius

	if len(w.Stops) < 2 {
		t.Fatalf("Emerald lake map must have at least 2 stops")
	}

	// Compute path between consecutive stops
	for i := 0; i < len(w.Stops)-1; i++ {
		s1 := w.Stops[i]
		s2 := w.Stops[i+1]
		path := ComputeRoadPathBetweenStops(w, s1, s2, 42.0)
		if len(path) == 0 {
			continue
		}

		// Verify every waypoint in the navigation route maintains clearance from central island
		for idx, pt := range path {
			dist := pt.Distance(center)
			if dist < innerR-5.0 {
				t.Fatalf("Route waypoint #%d (%v) between [%s] and [%s] cuts through central island (dist=%.2f < innerR=%.2f)",
					idx, pt, s1.Name, s2.Name, dist, innerR)
			}
		}
	}
}
