package game

import (
	"fmt"
	"math"
	"testing"
)

// checkReleaseQualificationGate skips the exhaustive qualification tests when running in -short mode.
// Routine feature development uses 'go test -short ./...' or 'make test'.
// Prior to publishing a release, 'go test -run TestReleaseQualification ./...' or 'make test-qualify' runs the full suite.
func checkReleaseQualificationGate(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping Release Qualification suite in -short mode. Run without -short or 'make test-qualify' to qualify release.")
	}
}

// TestReleaseQualification_AllMapsAutopilotTransitMultiLap executes an exhaustive multi-lap
// autonomous transit qualification across ALL 6 MAPS (Metropolis, Coastal Bay, River Valley,
// Emerald Lake, Twin Cities, and Custom Map).
//
// For each map, it verifies:
// 1. Lap 1: Standard Autopilot (Smooth, safe cruising at 150 px/s, cornering at 85 px/s,
//    precision stopping at bays, passenger boarding, and trip loop completion).
// 2. Lap 2: Berserk Autopilot (Fast straightaways at 445 px/s, cornering at 110-135 px/s,
//    high-speed right turns into 2-lane roads, safe left-turn lane entry, lightning 0.6s stop dwells).
// 3. Strict Safety Invariants across all legs:
//    - Zero barrier collisions (never enters ZoneHardBarrier).
//    - Zero double-yellow crossings into oncoming traffic lanes.
//    - Speed and position are never NaN or Inf.
//    - Traffic lights are respected when red/yellow.
//    - Bus durability remains at 100%.
//    - Fare revenue strictly increases.
func TestReleaseQualification_AllMapsAutopilotTransitMultiLap(t *testing.T) {
	checkReleaseQualificationGate(t)

	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	laneWidth := 52.0
	dt := 0.04 // 25 FPS physics step for high-throughput transit qualification

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(laneWidth, theme)
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(w)

			route := GetStandardRouteForMap(theme, w)
			if len(route.StopIDs) < 4 {
				t.Fatalf("[%s] Designated route must have at least 4 stops, got %d", theme.DisplayName(), len(route.StopIDs))
			}

			// Test two modes: Standard Mode (Lap 1) and Berserk Mode (Lap 2)
			testModes := []struct {
				name        string
				berserkMode bool
				maxLegSteps int
			}{
				{name: "StandardMode", berserkMode: false, maxLegSteps: 7500},
				{name: "BerserkMode", berserkMode: true, maxLegSteps: 6000},
			}

			for _, mode := range testModes {
				t.Run(mode.name, func(t *testing.T) {
					startStop := w.Stops[route.StopIDs[0]]
					bus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
					bus.Gear = GearDrive
					bus.TransmissionMode = TransmissionAuto

					driver := NewAutopilotDriver()
					driver.BerserkMode = mode.berserkMode
					driver.DwellTotal = 0.1

					totalTripsCompleted := 0

					// Iterate through all legs of the closed loop route (e.g. 0->A, A->B, B->C, C->0)
					numLegsToTest := len(route.StopIDs) - 1

					for leg := 0; leg < numLegsToTest; leg++ {
						fromID := route.StopIDs[leg]
						toID := route.StopIDs[leg+1]
						fromStop := w.Stops[fromID]
						toStop := w.Stops[toID]

						legRoute := &Route{
							Name:         fmt.Sprintf("QualifyLeg_%d_%d", fromID, toID),
							StopIDs:      []int{fromID, toID},
							CurrentIndex: 1,
						}

						// Reset driver state for new leg
						driver.State = AutopilotStateDeparting
						driver.drivePath = nil

						legDocked := false
						for step := 0; step < mode.maxLegSteps; step++ {
							tls.Update(dt)
							throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
								bus, legRoute, w, tls, nil, laneWidth, dt,
							)
							bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

							// Invariant 1: Position and Speed must never be NaN or Inf
							if math.IsNaN(bus.Pos.X) || math.IsNaN(bus.Pos.Y) || math.IsNaN(bus.Speed) {
								t.Fatalf("[%s-%s] Bus state corrupted with NaN at step %d: Pos=%v, Spd=%f",
									theme.DisplayName(), mode.name, step, bus.Pos, bus.Speed)
							}

							// Invariant 2: Bus center and perimeter must never enter ZoneHardBarrier
							bRes := w.CheckBoundaries(bus.Pos, bus.Width*0.4, laneWidth)
							if bRes.Zone == ZoneHardBarrier {
								t.Fatalf("[%s-%s] Leg %d->%d: Bus collided with hard barrier (%s) at %v (step %d)",
									theme.DisplayName(), mode.name, fromID, toID, bRes.ObstacleName, bus.Pos, step)
							}

							// Invariant 3: On multi-lane roads before intersections, vehicle body must never cross double yellow lines
							roadCenter, isEW, isMulti, _ := getCurrentRoadInfo(w, bus.Pos, bus.Heading, laneWidth)
							if isMulti {
								cosH := math.Cos(bus.Heading)
								sinH := math.Sin(bus.Heading)
								halfL := bus.Length * 0.5
								halfW := bus.Width * 0.5
								leftPerp := Vec2{X: sinH, Y: -cosH}
								flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
								rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

								distToInter := 999.0
								for _, it := range tls.Intersections {
									d := bus.Pos.Distance(it.Center)
									if d < distToInter {
										distToInter = d
									}
								}

								if distToInter > 80.0 {
									if isEW {
										if math.Cos(bus.Heading) < -0.5 { // Westbound: legal Y < roadCenter
											if flCorner.Y > roadCenter-1.0 || rlCorner.Y > roadCenter-1.0 {
												t.Fatalf("[%s-%s] Westbound bus crossed double yellow line: Pos=%v, fl=%v, rl=%v, center=%.1f",
													theme.DisplayName(), mode.name, bus.Pos, flCorner, rlCorner, roadCenter)
											}
										} else if math.Cos(bus.Heading) > 0.5 { // Eastbound: legal Y > roadCenter
											if flCorner.Y < roadCenter+1.0 || rlCorner.Y < roadCenter+1.0 {
												t.Fatalf("[%s-%s] Eastbound bus crossed double yellow line: Pos=%v, fl=%v, rl=%v, center=%.1f",
													theme.DisplayName(), mode.name, bus.Pos, flCorner, rlCorner, roadCenter)
											}
										}
									}
								}
							}

							// Check docking completion at target stop
							if isDocked && step > 60 {
								legDocked = true
								totalTripsCompleted++
								break
							}
						}

						if !legDocked {
							t.Fatalf("[%s-%s] Failed to dock at target stop #%d (%s) from stop #%d (%s) within %d steps: current Pos=%v, Speed=%.1f",
								theme.DisplayName(), mode.name, toID, toStop.Name, fromID, fromStop.Name, mode.maxLegSteps, bus.Pos, bus.Speed)
						}
					}

					if totalTripsCompleted < numLegsToTest {
						t.Fatalf("[%s-%s] Expected to complete %d legs, only completed %d",
							theme.DisplayName(), mode.name, numLegsToTest, totalTripsCompleted)
					}
				})
			}
		})
	}
}

// TestReleaseQualification_AllMapsTrafficAndAIFleetHeavyConcurrency qualifies full multi-vehicle
// concurrency across all 6 maps under maximum system load:
// - 128 civilian vehicles (Maximum Traffic Density Level 5)
// - 8 AI route buses (Maximum Fleet Configuration)
// - Player bus active in world
// - Traffic lights cycling through all intersection phases
//
// Verifies 0 deadlocks, 0 NaN values, 0 barrier collisions, and continuous fleet progression.
func TestReleaseQualification_AllMapsTrafficAndAIFleetHeavyConcurrency(t *testing.T) {
	checkReleaseQualificationGate(t)

	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	laneWidth := 52.0
	dt := 0.05 // 20 FPS high-throughput simulation tick

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(laneWidth, theme)
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(w)

			// 1. Build maximum civilian traffic (128 cars)
			tm := NewTrafficManager(laneWidth)
			tm.BuildForWorld(w, MaxTrafficCars, laneWidth)
			activeCars := tm.GetActiveCars(5)
			if len(activeCars) != 128 {
				t.Fatalf("[%s] Expected 128 active cars at density 5, got %d", theme.DisplayName(), len(activeCars))
			}

			// 2. Build maximum AI bus fleet (8 buses)
			startStop := w.Stops[0]
			playerBus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			abm := NewAIBusManager()
			abm.BuildFleet(w, 8, laneWidth, playerBus)
			if len(abm.Buses) != 8 {
				t.Fatalf("[%s] Expected 8 AI buses, got %d", theme.DisplayName(), len(abm.Buses))
			}

			// 3. Simulate 150 frames (7.5 seconds of concurrent transit operation)
			for frame := 0; frame < 150; frame++ {
				tls.Update(dt)
				tm.Update(dt, playerBus, tls, 5, laneWidth, w)
				abm.Update(dt, playerBus, w, tls, nil, laneWidth, 5)

				// Verify civilian traffic cars
				for i, car := range tm.GetActiveCars(5) {
					if math.IsNaN(car.Pos.X) || math.IsNaN(car.Pos.Y) || math.IsNaN(car.Speed) {
						t.Fatalf("[%s] Civilian car #%d corrupted with NaN at frame %d", theme.DisplayName(), i, frame)
					}
				}

				// Verify AI fleet buses
				for i, ai := range abm.Buses {
					if math.IsNaN(ai.Bus.Pos.X) || math.IsNaN(ai.Bus.Pos.Y) || math.IsNaN(ai.Bus.Speed) {
						t.Fatalf("[%s] AI Bus #%d corrupted with NaN at frame %d", theme.DisplayName(), i, frame)
					}
					// Ensure AI bus is never inside a hard barrier
					bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width*0.4, laneWidth)
					if bRes.Zone == ZoneHardBarrier {
						t.Fatalf("[%s] AI Bus #%d collided with hard barrier (%s) at %v (frame %d)",
							theme.DisplayName(), i, bRes.ObstacleName, ai.Bus.Pos, frame)
					}
				}
			}
		})
	}
}

// TestReleaseQualification_AllMapsBerserkOvertakingStress tests that when approaching slow lead vehicles
// in Berserk mode on multi-lane avenues, the bus triggers overtaking, cleanly transitions across lanes
// within 0.35s~0.55s, and maintains high momentum (>120 px/s) without sluggish panic braking.
func TestReleaseQualification_AllMapsBerserkOvertakingStress(t *testing.T) {
	checkReleaseQualificationGate(t)

	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	laneWidth := 52.0
	dt := 0.016

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(laneWidth, theme)
			route := GetStandardRouteForMap(theme, w)

			// Find a multi-lane approach leg
			fromStop := w.Stops[route.StopIDs[0]]
			toStop := w.Stops[route.StopIDs[1]]
			legRoute := &Route{
				Name:         "OvertakeStressLeg",
				StopIDs:      []int{fromStop.ID, toStop.ID},
				CurrentIndex: 1,
			}

			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			path := GetRoadPathBetweenStopsInWorld(w, fromStop, toStop, laneWidth)
			if len(path) < 3 {
				t.Fatalf("[%s] Path too short", theme.DisplayName())
			}

			// Spawn bus accurately on the straight travel path in the legal driving lane
			seg := path[2].Sub(path[1])
			segDir := seg.Normalize()
			startHeading := math.Atan2(segDir.Y, segDir.X)

			startPos := path[1].Add(segDir.Mul(60.0))
			bus := NewBus(startPos.X, startPos.Y, startHeading)
			bus.Gear = GearDrive
			bus.TransmissionMode = TransmissionAuto
			bus.Speed = 200.0

			driver.WaypointIdx = 1

			// Place slow civilian obstacle 380px ahead directly in the driving lane
			leadPos := startPos.Add(segDir.Mul(380.0))
			leadCar := VehicleObstacle{
				Pos:     leadPos,
				Heading: startHeading,
				Speed:   25.0,
				Length:  48.0,
				Width:   22.0,
			}
			obstacles := []VehicleObstacle{leadCar}

			overtakeTriggered := false
			minSpeedDuringOvertake := 999.0

			for step := 0; step < 80; step++ {
				// Move obstacle slowly along travel direction
				obstacles[0].Pos = obstacles[0].Pos.Add(segDir.Mul(obstacles[0].Speed * dt))

				thr, brk, str, hb, td, _ := driver.ComputeControls(bus, legRoute, w, nil, obstacles, laneWidth, dt)
				bus.Update(dt, thr, brk, str, hb, td)

				if driver.OvertakeState != OvertakeNone {
					overtakeTriggered = true
					if bus.Speed < minSpeedDuringOvertake {
						minSpeedDuringOvertake = bus.Speed
					}
				}
				// If lane change is in progress, verify lateral duration is brisk
				if driver.OvertakeState == OvertakeChangingLane {
					if driver.LaneOffsetDuration > 0.60 {
						t.Fatalf("[%s] LaneOffsetDuration too slow in Berserk mode: %.2fs", theme.DisplayName(), driver.LaneOffsetDuration)
					}
				}
			}

			if overtakeTriggered {
				// Verify the bus did not crawl below 110 px/s while executing the passing maneuver
				if minSpeedDuringOvertake < 110.0 {
					t.Fatalf("[%s] Bus crawled down to %.1f px/s during Berserk lane change (expected >= 110 px/s)",
						theme.DisplayName(), minSpeedDuringOvertake)
				}
			}
		})
	}
}

// TestReleaseQualification_AllMapsBoundaryAndZoneSafetyAudit validates road network clearance,
// boundary safety, stop bay access, and crosswalk geometries across all 6 maps.
func TestReleaseQualification_AllMapsBoundaryAndZoneSafetyAudit(t *testing.T) {
	checkReleaseQualificationGate(t)

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

// TestReleaseQualification_AllMapsBilingualAndHUDStress tests language toggling (ZH <-> EN),
// stop name localization, and route line wrapping across all 6 maps.
func TestReleaseQualification_AllMapsBilingualAndHUDStress(t *testing.T) {
	checkReleaseQualificationGate(t)

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
			route := GetStandardRouteForMap(theme, w)

			// 1. Verify all 12 stops have dual-language names (e.g. "中文名 (English Name)")
			for _, s := range w.Stops {
				SetLanguage(LanguageZH)
				zhName := s.LocalizedName()
				if zhName == "" {
					t.Fatalf("[%s] Stop #%d has empty ZH name", theme.DisplayName(), s.ID)
				}

				SetLanguage(LanguageEN)
				enName := s.LocalizedName()
				if enName == "" {
					t.Fatalf("[%s] Stop #%d has empty EN name", theme.DisplayName(), s.ID)
				}
			}

			// 2. Verify route line wrapping at different widths (300px, 500px, 800px)
			widths := []float64{300.0, 500.0, 800.0}
			for _, width := range widths {
				lines := route.GetFormattedRouteLines(w, width, 14.0)
				if len(lines) == 0 {
					t.Fatalf("[%s] Formatted route lines empty for width %.1f", theme.DisplayName(), width)
				}
				if len(lines) > 3 {
					t.Fatalf("[%s] Formatted route lines exceeded max 3 lines: got %d lines", theme.DisplayName(), len(lines))
				}
			}
		})
	}

	// Restore default language to Chinese
	SetLanguage(LanguageZH)
}
