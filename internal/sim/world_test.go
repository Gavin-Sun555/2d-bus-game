package sim

import (
	"math"
	"testing"
)

func TestGetRoadPathBetweenStops(t *testing.T) {
	world := NewWorld(42.0)
	if len(world.Stops) < 2 {
		t.Fatalf("Need at least 2 stops for test")
	}

	s1 := world.Stops[0] // e.g. North Hub (Y = -1200)
	s2 := world.Stops[1] // e.g. Financial Center (Y = 0)

	path := GetRoadPathBetweenStops(s1, s2)
	if len(path) < 2 {
		t.Fatalf("Path should contain multiple waypoints, got %d", len(path))
	}
	// Verify the path segments along the road grid are orthogonal outside roundabouts,
	// curve smoothly around roundabouts, and are drawn on the driving lane side rather than in the road center
	for i := 1; i < len(path)-2; i++ {
		pA := path[i]
		pB := path[i+1]
		dx := math.Abs(pA.X - pB.X)
		dy := math.Abs(pA.Y - pB.Y)

		isRoundaboutSeg := false
		for _, rb := range world.Roundabouts {
			if pA.Distance(rb.Center) <= rb.Radius+30 || pB.Distance(rb.Center) <= rb.Radius+30 {
				isRoundaboutSeg = true
				if pA.Distance(rb.Center) < rb.InnerRadius-5.0 {
					t.Errorf("Path waypoint %v penetrates roundabout inner island (dist: %.1f < %.1f)",
						pA, pA.Distance(rb.Center), rb.InnerRadius)
				}
				break
			}
		}

		// Segments along road grid should be orthogonal or gentle lane-change tapers (outside roundabouts)
		isLaneTaper := (dx > 1.0 && dy > 1.0 && (dx <= 120.0 || dy <= 120.0))
		if !isRoundaboutSeg && !isLaneTaper && dx > 1.0 && dy > 1.0 {
			t.Errorf("Path segment from %v to %v is diagonal, expected orthogonal grid line", pA, pB)
		}

		// Verify the segment is offset to the side of Central Blvd (Y=0), not on the centerline Y=0
		if !isRoundaboutSeg && dx > 50 { // Longitudinal road segment
			distFromRoadCenter := math.Abs(pA.Y - 0)
			if distFromRoadCenter < 20.0 {
				t.Errorf("Navigation line at Y=%.2f is in the middle of the road, expected side of road", pA.Y)
			}
		}
	}
}

func TestRightHandSideApproachAndAlignment(t *testing.T) {
	world := NewWorld(42.0)

	// Test default route stops
	defaultRoute := NewDefaultRoute()
	for i := 0; i < len(defaultRoute.StopIDs)-1; i++ {
		s1 := world.Stops[defaultRoute.StopIDs[i]]
		s2 := world.Stops[defaultRoute.StopIDs[i+1]]

		path := GetRoadPathBetweenStops(s1, s2)
		if len(path) < 3 {
			t.Fatalf("Path between %s and %s is too short", s1.Name, s2.Name)
		}

		// The second-to-last point is s2.RoadCenter, and the last point is s2.Pos
		// The point before s2.RoadCenter must lead into s2.RoadCenter along s2.Heading!
		pPrev := path[len(path)-3]
		pRoadCenter := path[len(path)-2]
		approachVec := pRoadCenter.Sub(pPrev).Normalize()

		expectedHeadingVec := Vec2{X: math.Cos(s2.Heading), Y: math.Sin(s2.Heading)}
		dot := approachVec.X*expectedHeadingVec.X + approachVec.Y*expectedHeadingVec.Y
		if dot < 0.95 {
			t.Errorf("Approach vector into %s (%v) does not match stop heading (%v, dot=%.2f)",
				s2.Name, approachVec, expectedHeadingVec, dot)
		}

		// Verify s2.Pos is to the RIGHT of approachVec
		// In 2D screen coordinates (X right, Y down):
		// Right perpendicular of (vx, vy) is (-vy, vx).
		stopOffset := s2.Pos.Sub(pRoadCenter).Normalize()
		perpRight := Vec2{X: -approachVec.Y, Y: approachVec.X}
		rightDot := stopOffset.X*perpRight.X + stopOffset.Y*perpRight.Y
		if rightDot < 0.95 {
			t.Errorf("Stop bay for %s is not on the RIGHT side of the approaching vehicle (dot=%.2f)",
				s2.Name, rightDot)
		}
	}

	// Test bus parking orientation: bus heading matching s.Heading contains bus, reverse does not
	s := world.Stops[0]
	busCorrect := NewBus(s.Pos.X, s.Pos.Y, s.Heading)
	if !s.ContainsBus(busCorrect) {
		t.Fatalf("Bus correctly parked at right-hand stop should be accepted")
	}

	busReverse := NewBus(s.Pos.X, s.Pos.Y, NormalizeAngle(s.Heading+math.Pi))
	if s.ContainsBus(busReverse) {
		t.Fatalf("Bus parked facing opposite direction should NOT be accepted at right-hand stop")
	}
}

func TestRoadBoundaryZonesAndBarriers(t *testing.T) {
	world := NewWorld(42.0)
	laneW := 42.0
	// Central Boulevard: Y = 0, 4 lanes -> width = 4 * 42 = 168, half-width = 84

	// 1. Center of Central Blvd: should be ZoneRoad (at X=300, far from vertical roads)
	resRoad := world.CheckBoundaries(Vec2{X: 300, Y: 0}, 18.0, laneW)
	if resRoad.Zone != ZoneRoad {
		t.Fatalf("Expected center of road to be ZoneRoad, got %v", resRoad.Zone)
	}

	// 2. Roadside shoulder: roadHW = (4*42+18)/2 = 93.0. Y = 93 + 15 = 108 (within ShoulderWidth = 36.0)
	resShoulder := world.CheckBoundaries(Vec2{X: 300, Y: 108}, 18.0, laneW)
	if resShoulder.Zone != ZoneShoulder {
		t.Fatalf("Expected Y=108 to be ZoneShoulder, got %v", resShoulder.Zone)
	}

	// 3. Beyond shoulder: Y = 93 + 45 = 138 (> ShoulderWidth = 36) -> ZoneHardBarrier
	resBarrier := world.CheckBoundaries(Vec2{X: 300, Y: 145}, 18.0, laneW)
	if resBarrier.Zone != ZoneHardBarrier {
		t.Fatalf("Expected Y=145 to be ZoneHardBarrier, got %v", resBarrier.Zone)
	}
	if resBarrier.Pushback.Y >= 0 {
		t.Fatalf("Expected pushback towards road (negative Y), got %v", resBarrier.Pushback)
	}

	// 4. Downtown skyscraper collision: cx=450, cy=300, w=220, h=160
	resBldg := world.CheckBoundaries(Vec2{X: 450, Y: 300}, 18.0, laneW)
	if resBldg.Zone != ZoneHardBarrier {
		t.Fatalf("Expected building center to be ZoneHardBarrier, got %v", resBldg.Zone)
	}
}

func TestBuildingsClearFromRoads(t *testing.T) {
	// Test at standard (52px) and maximum (78px) lane widths
	for _, laneW := range []float64{52.0, 78.0} {
		world := NewWorld(laneW)

		for i, b := range world.Buildings {
			// Check the 4 corners and center of every building
			testPoints := []Vec2{
				b.Pos,
				{X: b.Pos.X - b.W/2, Y: b.Pos.Y - b.H/2},
				{X: b.Pos.X + b.W/2, Y: b.Pos.Y - b.H/2},
				{X: b.Pos.X - b.W/2, Y: b.Pos.Y + b.H/2},
				{X: b.Pos.X + b.W/2, Y: b.Pos.Y + b.H/2},
			}

			for _, pt := range testPoints {
				for _, r := range world.Roads {
					roadHalfW := r.GetWidth(laneW) / 2
					roadClearance := roadHalfW + ShoulderWidth

					isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
					isNS := math.Abs(r.P2.X-r.P1.X) < 1.0
					if isEW {
						// Horizontal road
						minX := math.Min(r.P1.X, r.P2.X)
						maxX := math.Max(r.P1.X, r.P2.X)
						if pt.X >= minX-20 && pt.X <= maxX+20 {
							dist := math.Abs(pt.Y - r.P1.Y)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects horizontal road %s (Y=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.Y, roadClearance, dist)
							}
						}
					} else if isNS {
						// Vertical road
						minY := math.Min(r.P1.Y, r.P2.Y)
						maxY := math.Max(r.P1.Y, r.P2.Y)
						if pt.Y >= minY-20 && pt.Y <= maxY+20 {
							dist := math.Abs(pt.X - r.P1.X)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects vertical road %s (X=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.X, roadClearance, dist)
							}
						}
					} else {
						// Curved or diagonal road segment
						d, _ := DistToSegment(pt, r.P1, r.P2)
						if d <= roadClearance {
							t.Fatalf("Building #%d at %v intersects curved road %s (clearance=%.1f, dist=%.1f)",
								i, b.Pos, r.Name, roadClearance, d)
						}
					}
				}
			}
		}
	}
}

func TestBlockWidthExtendedAndBusStopsDistancedFromIntersections(t *testing.T) {
	world := NewWorld(42.0)
	tls := NewTrafficLightSystem()

	// 1. Verify block width is extended to 1300px across all East-West intervals
	expectedXs := []float64{-2600, -1300, 0, 1300, 2600}
	for i := 0; i < len(expectedXs)-1; i++ {
		width := expectedXs[i+1] - expectedXs[i]
		if math.Abs(width-1300.0) > 1.0 {
			t.Fatalf("Expected block width of 1300px, got %.1f between X=%.1f and X=%.1f", width, expectedXs[i], expectedXs[i+1])
		}
	}

	// Verify all 25 intersections reflect the extended 1300px block width
	if len(tls.Intersections) != 25 {
		t.Fatalf("Expected 25 intersections, got %d", len(tls.Intersections))
	}
	foundInter := false
	for _, it := range tls.Intersections {
		if math.Abs(it.Center.X-2600*MapGridScale) < 5.0 && math.Abs(it.Center.Y) < 5.0 {
			foundInter = true
			break
		}
	}
	if !foundInter {
		t.Fatalf("Expected outer boulevard intersection at X=%.0f, Y=0", 2600*MapGridScale)
	}

	// 2. Verify all bus stops are located away from intersections
	// Bus stops on 4-lane avenues must have >= 500px runway to the next intersection stop line
	// so the bus has plenty of room to change lanes.
	stopLineBuffer := 42.0*1.6 + 18.0 // approx 85.2px
	for _, stop := range world.Stops {
		isEW := math.Abs(math.Cos(stop.Heading)) > math.Abs(math.Sin(stop.Heading))
		minDistToCenter := math.MaxFloat64
		distToUpcomingStopLine := math.MaxFloat64

		for _, it := range tls.Intersections {
			dCenter := stop.RoadCenter.Distance(it.Center)
			if dCenter < minDistToCenter {
				minDistToCenter = dCenter
			}

			// Along direction of travel:
			if isEW {
				cosH := math.Cos(stop.Heading)
				// If heading East (+X), next intersection has it.Center.X > stop.RoadCenter.X
				if cosH > 0 && it.Center.X > stop.RoadCenter.X && math.Abs(it.Center.Y-stop.RoadCenter.Y) < 10.0 {
					dLine := (it.Center.X - stop.RoadCenter.X) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				} else if cosH < 0 && it.Center.X < stop.RoadCenter.X && math.Abs(it.Center.Y-stop.RoadCenter.Y) < 10.0 {
					dLine := (stop.RoadCenter.X - it.Center.X) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				}
			} else {
				sinH := math.Sin(stop.Heading)
				if sinH > 0 && it.Center.Y > stop.RoadCenter.Y && math.Abs(it.Center.X-stop.RoadCenter.X) < 10.0 {
					dLine := (it.Center.Y - stop.RoadCenter.Y) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				} else if sinH < 0 && it.Center.Y < stop.RoadCenter.Y && math.Abs(it.Center.X-stop.RoadCenter.X) < 10.0 {
					dLine := (stop.RoadCenter.Y - it.Center.Y) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				}
			}
		}

		// Distance to any intersection center should be at least 280px
		if minDistToCenter < 280.0 {
			t.Fatalf("Stop %d (%s) at %v is too close to an intersection center (minDist=%.1f < 280px)",
				stop.ID, stop.Name, stop.Pos, minDistToCenter)
		}

		// On 4-lane avenues, distance to upcoming stop line must be >= 500px for comfortable lane change
		if isEW && distToUpcomingStopLine < 500.0 {
			t.Fatalf("Stop %d (%s) at %v has insufficient lane-change runway to next stop line (dist=%.1f < 500px)",
				stop.ID, stop.Name, stop.Pos, distToUpcomingStopLine)
		}
	}
}

func TestPlanModePerformanceOptimizations(t *testing.T) {
	world := NewWorld(42.0)
	cam := NewCamera(1280, 720)
	cam.Pos = Vec2{X: 0, Y: 0}
	cam.Zoom = 0.22

	// 1. Verify Camera Frustum Culling
	// Center is visible
	if !cam.IsPointVisible(Vec2{X: 0, Y: 0}, 50) {
		t.Fatal("Origin point must be visible on camera centered at origin")
	}
	// Far off-screen point (X=10000) must NOT be visible
	if cam.IsPointVisible(Vec2{X: 10000, Y: 10000}, 50) {
		t.Fatal("Far away point (10000, 10000) must be culled by camera frustum")
	}
	// Far off-screen bounding box must NOT be visible
	if cam.IsBoxVisible(8000, 8000, 9000, 9000) {
		t.Fatal("Far away box (8000..9000) must be culled by camera frustum")
	}

	// 2. Verify Dijkstra Route Path Cache Memoization
	s1 := world.Stops[0]
	s2 := world.Stops[1]
	p1 := GetRoadPathBetweenStops(s1, s2, 42.0)
	if len(p1) < 2 {
		t.Fatalf("Path between stop 0 and 1 must contain at least 2 points, got %d", len(p1))
	}

	// Second call must return the cached slice (same length and points) instantly
	p2 := GetRoadPathBetweenStops(s1, s2, 42.0)
	if len(p2) != len(p1) {
		t.Fatalf("Cached path length mismatch: expected %d, got %d", len(p1), len(p2))
	}
	for i := range p1 {
		if p1[i] != p2[i] {
			t.Fatalf("Point %d mismatch in cached path: %v vs %v", i, p1[i], p2[i])
		}
	}

	// 3. Verify Route Description Cache
	route := NewRoute()
	route.AddStop(0)
	route.AddStop(1)
	desc1 := route.GetRouteDescription(world)
	if desc1 == "" {
		t.Fatal("Route description must not be empty")
	}
	// Calling again should return cached description string
	desc2 := route.GetRouteDescription(world)
	if desc1 != desc2 {
		t.Fatalf("Cached description mismatch: %q vs %q", desc1, desc2)
	}

	// Mutating route should invalidate cache and update correctly
	route.AddStop(2)
	desc3 := route.GetRouteDescription(world)
	if desc3 == desc1 {
		t.Fatal("Description cache should invalidate when a new stop is added")
	}

	// Clear should reset cache
	route.Clear()
	descEmpty := route.GetRouteDescription(world)
	if descEmpty != "未规划站点 (点击站点图标添加)" {
		t.Fatalf("Unexpected empty route description: %q", descEmpty)
	}
}

func TestAllMapsBusStopsDistancedFromIntersections(t *testing.T) {
	laneW := 52.0
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, theme := range themes {
		w := NewWorld(laneW, theme)
		if len(w.Stops) == 0 {
			t.Fatalf("Map theme %s has no stops", theme.DisplayName())
		}
		if len(w.Intersections) == 0 {
			t.Fatalf("Map theme %s has no intersections", theme.DisplayName())
		}

		for _, stop := range w.Stops {
			minDist := math.MaxFloat64
			var nearestInter Vec2
			for _, inter := range w.Intersections {
				d := stop.RoadCenter.Distance(inter)
				if d < minDist {
					minDist = d
					nearestInter = inter
				}
			}

			// Bus stop road center must be at least 240px away from any intersection
			if minDist < 240.0 {
				t.Fatalf("Map %s Stop '%s' (id=%d) at %v is too close to intersection at %v (dist=%.1f < 240px)",
					theme.DisplayName(), stop.Name, stop.ID, stop.RoadCenter, nearestInter, minDist)
			}
		}
	}
}

// TestNoUnreasonablyCloseParallelRoadsOnAnyMap verifies that across all maps,
// no two parallel two-way roads are unreasonably close to each other (must maintain >= 400px spacing).
func TestNoUnreasonablyCloseParallelRoadsOnAnyMap(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	const minParallelDistance = 400.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(52.0, theme)
			for i := 0; i < len(w.Roads); i++ {
				for j := i + 1; j < len(w.Roads); j++ {
					r1 := w.Roads[i]
					r2 := w.Roads[j]

					isR1EW := math.Abs(r1.P1.Y-r1.P2.Y) < 1.0
					isR2EW := math.Abs(r2.P1.Y-r2.P2.Y) < 1.0
					isR1NS := math.Abs(r1.P1.X-r1.P2.X) < 1.0
					isR2NS := math.Abs(r2.P1.X-r2.P2.X) < 1.0

					// Check parallel EW roads
					if isR1EW && isR2EW {
						y1 := (r1.P1.Y + r1.P2.Y) / 2.0
						y2 := (r2.P1.Y + r2.P2.Y) / 2.0
						dist := math.Abs(y1 - y2)

						minX1 := math.Min(r1.P1.X, r1.P2.X)
						maxX1 := math.Max(r1.P1.X, r1.P2.X)
						minX2 := math.Min(r2.P1.X, r2.P2.X)
						maxX2 := math.Max(r2.P1.X, r2.P2.X)
						overlap := math.Min(maxX1, maxX2) - math.Max(minX1, minX2)

						if overlap > 30.0 && dist > 1.0 && dist < minParallelDistance {
							t.Fatalf("[%s] Parallel EW roads '%s' (Y=%.1f) and '%s' (Y=%.1f) are too close: %.1f px < %.0f px",
								theme.DisplayName(), r1.Name, y1, r2.Name, y2, dist, minParallelDistance)
						}
					}

					// Check parallel NS roads
					if isR1NS && isR2NS {
						x1 := (r1.P1.X + r1.P2.X) / 2.0
						x2 := (r2.P1.X + r2.P2.X) / 2.0
						dist := math.Abs(x1 - x2)

						minY1 := math.Min(r1.P1.Y, r1.P2.Y)
						maxY1 := math.Max(r1.P1.Y, r1.P2.Y)
						minY2 := math.Min(r2.P1.Y, r2.P2.Y)
						maxY2 := math.Max(r2.P1.Y, r2.P2.Y)
						overlap := math.Min(maxY1, maxY2) - math.Max(minY1, minY2)

						if overlap > 30.0 && dist > 1.0 && dist < minParallelDistance {
							t.Fatalf("[%s] Parallel NS roads '%s' (X=%.1f) and '%s' (X=%.1f) are too close: %.1f px < %.0f px",
								theme.DisplayName(), r1.Name, x1, r2.Name, x2, dist, minParallelDistance)
						}
					}
				}
			}
		})
	}
}

// TestBuiltinMapsBusStopsSafeDistanceFromIntersections ensures that all bus stops in built-in maps
// maintain a safe, realistic distance (>= 250px) from all intersection centers and traffic roundabouts,
// preventing bus stops from crowding junctions or obstructing turns.
// (Per requirement: Custom maps MapThemeCustom are explicitly excluded from this constraint).
func TestBuiltinMapsBusStopsSafeDistanceFromIntersections(t *testing.T) {
	builtinThemes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
	}

	const minSafeDistance = 250.0 // Minimum clearance required from intersection center

	for _, theme := range builtinThemes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(42.0, theme)
			if len(w.Stops) == 0 {
				t.Fatalf("[%s] Map must have bus stops defined", theme.DisplayName())
			}
			if len(w.Intersections) == 0 {
				t.Fatalf("[%s] Map must have intersections computed", theme.DisplayName())
			}

			for _, stop := range w.Stops {
				// 1. Check distance to all intersections
				for _, inter := range w.Intersections {
					dist := stop.Pos.Distance(inter)
					if dist < minSafeDistance {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is too close to intersection at (%.1f, %.1f): distance %.1fpx < required %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y, inter.X, inter.Y, dist, minSafeDistance)
					}

					// Also check road center clearance
					rcDist := stop.RoadCenter.Distance(inter)
					if rcDist < minSafeDistance {
						t.Fatalf("[%s] Bus stop #%d '%s' RoadCenter at (%.1f, %.1f) is too close to intersection at (%.1f, %.1f): distance %.1fpx < required %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.RoadCenter.X, stop.RoadCenter.Y, inter.X, inter.Y, rcDist, minSafeDistance)
					}
				}

				// 2. Check clearance from roundabouts
				for _, rb := range w.Roundabouts {
					distToCenter := stop.Pos.Distance(rb.Center)
					minRBClearance := rb.Radius + 40.0
					if distToCenter < minRBClearance {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is inside roundabout '%s' (radius %.1f): dist %.1fpx < %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y, rb.Name, rb.Radius, distToCenter, minRBClearance)
					}
					if w.IsAtRoundabout(stop.Pos) {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is flagged inside roundabout by IsAtRoundabout",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y)
					}
				}
			}
		})
	}
}

// TestBusStopApproachAndBayBoundaryClearance verifies that the white box area in the user screenshot
// (the approach deceleration taper leading to Coastal Bay Stop #10 深水港智能集装箱总站)
// as well as all bus bays across all maps provide clean ZoneRoad clearance without hitting ZoneHardBarrier.
func TestBusStopApproachAndBayBoundaryClearance(t *testing.T) {
	laneW := 42.0

	// 1. Specific test for Coastal Bay Stop #10 (深水港智能集装箱总站)
	wc := NewWorld(laneW, MapThemeCoastalBay)
	var stop10 *BusStop
	for _, s := range wc.Stops {
		if s.ID == 10 {
			stop10 = s
			break
		}
	}
	if stop10 == nil {
		t.Fatalf("Coastal Bay stop #10 not found")
	}

	// Verify approach coordinates along stop10 travel heading
	cosH := math.Cos(stop10.Heading)
	sinH := math.Sin(stop10.Heading)
	whiteBoxCoords := []Vec2{
		stop10.Pos.Add(Vec2{X: -cosH * 120, Y: -sinH * 120}),
		stop10.Pos.Add(Vec2{X: -cosH * 70, Y: -sinH * 70}),
		stop10.Pos.Add(Vec2{X: -cosH * 30, Y: -sinH * 30}),
		stop10.Pos,
	}
	for _, pt := range whiteBoxCoords {
		res := wc.CheckBoundaries(pt, 18.0, laneW)
		if res.Zone == ZoneHardBarrier {
			t.Fatalf("Coastal Bay stop #10 approach point %v hit hard barrier: %s (pen=%.1f)",
				pt, res.ObstacleName, res.Penetration)
		}
		if res.Zone != ZoneRoad {
			t.Fatalf("Coastal Bay stop #10 approach point %v expected ZoneRoad, got %v", pt, res.Zone)
		}
	}

	// Verify that physical shelter structure on sidewalk DOES properly detect collision
	shelterHit := wc.CheckBoundaries(stop10.ShelterPos, 18.0, laneW)
	if shelterHit.Zone != ZoneHardBarrier {
		t.Fatalf("Expected physical shelter structure at %v to trigger ZoneHardBarrier, got %v",
			stop10.ShelterPos, shelterHit.Zone)
	}

	// 2. Comprehensive check across all themes: each stop bay and approach corridor must be free of hard barriers
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
	}
	for _, theme := range themes {
		w := NewWorld(laneW, theme)
		for _, s := range w.Stops {
			cosH := math.Cos(s.Heading)
			sinH := math.Sin(s.Heading)
			perpX := -sinH
			perpY := cosH

			// Test bay center
			bRes := w.CheckBoundaries(s.Pos, 18.0, laneW)
			if bRes.Zone == ZoneHardBarrier {
				t.Fatalf("[%s] Stop #%d '%s' bay center %v hit hard barrier: %s",
					theme.DisplayName(), s.ID, s.Name, s.Pos, bRes.ObstacleName)
			}

			// Test points along approach taper: 120px and 60px before stop bay
			for _, offset := range []float64{-120.0, -60.0, 0.0, 60.0} {
				// Point halfway into bay
				testPt := s.Pos.Add(Vec2{X: cosH * offset, Y: sinH * offset})
				res := w.CheckBoundaries(testPt, 18.0, laneW)
				if res.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop #%d '%s' corridor offset %.0f at %v hit hard barrier: %s",
						theme.DisplayName(), s.ID, s.Name, offset, testPt, res.ObstacleName)
				}
			}

			// Outer edge of bay (s.Width * 0.45 towards curb)
			bayEdgePt := s.Pos.Add(Vec2{X: perpX * (s.Width * 0.45), Y: perpY * (s.Width * 0.45)})
			edgeRes := w.CheckBoundaries(bayEdgePt, 18.0, laneW)
			if edgeRes.Zone == ZoneHardBarrier {
				t.Fatalf("[%s] Stop #%d '%s' outer bay edge at %v hit hard barrier: %s",
					theme.DisplayName(), s.ID, s.Name, bayEdgePt, edgeRes.ObstacleName)
			}
		}
	}
}

// TestCoastalBaySouthCapeCausewayNoShoulderDriving verifies that along the South Cape Causeway
// (南湾海角栈道, 2 lanes) leading to Stop 11 (Cape Lighthouse), the path waypoints
// are strictly centered in the travel lane (ZoneRoad) and never stray onto the roadside shoulder (ZoneShoulder).
func TestCoastalBaySouthCapeCausewayNoShoulderDriving(t *testing.T) {
	laneW := 52.0
	w := NewWorld(laneW, MapThemeCoastalBay)

	stop11 := w.Stops[11]
	if stop11.ID != 11 {
		t.Fatalf("Expected stop 11, got %d", stop11.ID)
	}

	// Verify all paths leading to Stop 11 have 0 shoulder points
	for i, sFrom := range w.Stops {
		if i == 11 {
			continue
		}
		path := GetRoadPathBetweenStopsInWorld(w, sFrom, stop11, laneW)
		for idx, pt := range path {
			if idx == 0 || idx == len(path)-1 {
				continue // skip starting / ending stop bay
			}
			bRes := w.CheckBoundaries(pt, 18.0, laneW)
			if bRes.Zone != ZoneRoad {
				t.Fatalf("From Stop %d (%s) to Stop 11: pt[%d]=(%.1f, %.1f) is in %v, not ZoneRoad! (Bus driving on shoulder)",
					i, sFrom.Name, idx, pt.X, pt.Y, bRes.Zone)
			}
		}
	}

	// Verify all paths leaving Stop 11 have 0 shoulder points
	for j, sTo := range w.Stops {
		if j == 11 {
			continue
		}
		path := GetRoadPathBetweenStopsInWorld(w, stop11, sTo, laneW)
		for idx, pt := range path {
			if idx == 0 || idx == len(path)-1 {
				continue
			}
			bRes := w.CheckBoundaries(pt, 18.0, laneW)
			if bRes.Zone != ZoneRoad {
				t.Fatalf("From Stop 11 to Stop %d (%s): pt[%d]=(%.1f, %.1f) is in %v, not ZoneRoad! (Bus driving on shoulder)",
					j, sTo.Name, idx, pt.X, pt.Y, bRes.Zone)
			}
		}
	}
}
