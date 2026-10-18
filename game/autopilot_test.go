package game

import (
	"fmt"
	"math"
	"testing"
)

// TestPlayerBusAutopilotToggle verifies that AI autopilot for the player's bus can be toggled on and off cleanly.
func TestPlayerBusAutopilotToggle(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)

	// Initially disabled
	if g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot should be disabled by default")
	}

	// 1. Toggle via Settings
	g.Settings.AutopilotEnabled = true
	if !g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot failed to enable via Settings")
	}

	g.Settings.AutopilotEnabled = false
	if g.Settings.AutopilotEnabled {
		t.Fatalf("Autopilot failed to disable via Settings")
	}

	// 2. Toggle via HUD Dashboard Click
	btnPos := Vec2{
		X: g.HUD.AutopilotBounds.X + g.HUD.AutopilotBounds.W/2,
		Y: g.HUD.AutopilotBounds.Y + g.HUD.AutopilotBounds.H/2,
	}

	toggled, notice := g.HUD.HandleAutopilotClick(btnPos, g.Settings)
	if !toggled || !g.Settings.AutopilotEnabled {
		t.Fatalf("HandleAutopilotClick failed to toggle ON, notice=%s", notice)
	}

	// Click again to turn off
	toggled, notice = g.HUD.HandleAutopilotClick(btnPos, g.Settings)
	if !toggled || g.Settings.AutopilotEnabled {
		t.Fatalf("HandleAutopilotClick failed to toggle OFF, notice=%s", notice)
	}

	// Click outside bounds
	outsidePos := Vec2{X: 10, Y: 10}
	toggled, _ = g.HUD.HandleAutopilotClick(outsidePos, g.Settings)
	if toggled {
		t.Fatalf("Click outside bounds should not trigger toggle")
	}
}

// TestPlayerBusAutopilotDockingAndDeparture tests that when autopilot is active,
// the driver steers to the stop, docks in the yellow bay, opens doors, dwells, closes doors, and advances stop.
func TestPlayerBusAutopilotDockingAndDeparture(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)
	g.Settings.AutopilotEnabled = true

	// Route starts at Stop 0, target is Stop 2
	g.Route.StopIDs = []int{0, 2, 10, 8, 0}
	g.Route.CurrentIndex = 1
	targetStop := g.World.Stops[2] // Stop 2

	// Position bus 60px ahead of Stop 2 aligned with heading, approaching at moderate speed
	g.Bus.Pos = targetStop.Pos.Sub(Vec2{X: math.Cos(targetStop.Heading) * 60, Y: math.Sin(targetStop.Heading) * 60})
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 20.0
	g.Bus.DoorsOpen = false

	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.2 // Short dwell for fast test

	dt := 0.05
	docked := false
	doorsOpened := false
	doorsClosed := false

	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			g.Bus, g.Route, g.World, g.TrafficLights, nil, 52.0, dt,
		)
		g.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if isDocked {
			docked = true
		}
		if g.Bus.DoorsOpen {
			doorsOpened = true
		}
		if doorsOpened && !g.Bus.DoorsOpen && g.Bus.DoorProgress <= 0.05 {
			doorsClosed = true
			if driver.State != AutopilotStateDocked {
				break
			}
		}
	}

	if !docked {
		t.Fatalf("Autopilot failed to dock inside the bus stop bay")
	}
	if !doorsOpened {
		t.Fatalf("Autopilot failed to open doors while docked")
	}
	if !doorsClosed {
		t.Fatalf("Autopilot failed to close doors after dwelling")
	}
	if driver.State != AutopilotStateDeparting && driver.State != AutopilotStateCruising {
		t.Fatalf("Autopilot state should transition to Departing/Cruising after dwell, got %v", driver.State)
	}
}

// TestAutopilotObstacleAvoidance verifies that the autopilot slows down and brakes when a vehicle is ahead.
func TestAutopilotObstacleAvoidance(t *testing.T) {
	driver := NewAutopilotDriver()
	bus := NewBus(100, 100, 0) // Heading East
	bus.Speed = 40.0

	route := &Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	world := NewWorld(52.0)

	// Place an obstacle 40px straight ahead in the same lane
	obstacles := []VehicleObstacle{
		{
			Pos:     Vec2{X: 140, Y: 100},
			Heading: 0,
			Speed:   0,
			Length:  24,
			Width:   12,
		},
	}

	throttle, brake, _, _, _, _ := driver.ComputeControls(
		bus, route, world, nil, obstacles, 52.0, 0.05,
	)

	if throttle > 0.1 {
		t.Fatalf("Throttle should be 0 or near 0 when obstacle is immediately ahead, got %f", throttle)
	}
	if brake < 0.5 {
		t.Fatalf("Brake should be engaged when obstacle is immediately ahead, got %f", brake)
	}
}

// TestAIBusManagerConfigurableCounts verifies setting fleet counts for the route (0~8 buses).
func TestAIBusManagerConfigurableCounts(t *testing.T) {
	world := NewWorld(52.0)
	mgr := NewAIBusManager()

	// Initial default should be 0
	if mgr.GetLineCount() != 0 {
		t.Fatalf("Initial AI bus count should be 0, got %d", mgr.BusCount)
	}

	// 1. Build fleet with 8 buses
	mgr.BuildFleet(world, 8, 52.0)

	if mgr.GetLineCount() != 8 {
		t.Fatalf("Configured bus count mismatch: got %d, expected 8", mgr.BusCount)
	}

	if len(mgr.Buses) != 8 {
		t.Fatalf("Expected 8 total AI buses, got %d", len(mgr.Buses))
	}

	// 2. Test CycleLineCount (0 -> 1 -> ... -> 8 -> 0)
	mgr.SetLineCount(8, world, 52.0)
	next := mgr.CycleLineCount(world, 52.0)
	if next != 0 || mgr.GetLineCount() != 0 {
		t.Fatalf("CycleLineCount after 8 should wrap to 0, got %d", next)
	}

	next = mgr.CycleLineCount(world, 52.0)
	if next != 1 || mgr.GetLineCount() != 1 {
		t.Fatalf("CycleLineCount after 0 should advance to 1, got %d", next)
	}
}

// TestAIBusManagerCirculation verifies that spawned AI buses simulate smoothly without errors.
func TestAIBusManagerCirculation(t *testing.T) {
	world := NewWorld(52.0)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(world)
	tm := NewTrafficManager(52.0)

	mgr := NewAIBusManager()
	mgr.BuildFleet(world, 3, 52.0)

	if len(mgr.Buses) != 3 {
		t.Fatalf("Expected 3 AI buses, got %d", len(mgr.Buses))
	}

	// Simulate 100 frames
	playerBus := NewBus(world.Stops[0].Pos.X, world.Stops[0].Pos.Y, world.Stops[0].Heading)
	dt := 0.05

	for frame := 0; frame < 100; frame++ {
		mgr.Update(dt, playerBus, world, tls, tm, 52.0)

		for _, b := range mgr.Buses {
			if math.IsNaN(b.Bus.Pos.X) || math.IsNaN(b.Bus.Pos.Y) {
				t.Fatalf("AI Bus %d position is NaN at frame %d", b.ID, frame)
			}
			if math.Abs(b.Bus.Speed) > b.Bus.MaxSpeedFwd+10.0 {
				t.Fatalf("AI Bus %d speed %f exceeds max speed limit", b.ID, b.Bus.Speed)
			}
		}
	}
}

// TestAutopilotStationLaunchDoorsOpen verifies Bug 1:
// If the bus is stopped at a station with doors open and autopilot is activated,
// the autopilot automatically commands doors to close and starts driving towards the next stop.
func TestAutopilotStationLaunchDoorsOpen(t *testing.T) {
	world := NewWorld(52.0, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, world)
	route.CurrentIndex = 0 // Starts at Stop 0, target is Stop 2

	stop0 := world.Stops[route.StopIDs[0]]
	bus := NewBus(stop0.Pos.X, stop0.Pos.Y, stop0.Heading)
	bus.DoorsOpen = true
	bus.DoorProgress = 1.0 // Fully open
	bus.Speed = 0

	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.5 // Brief dwell
	dt := 0.05

	launched := false
	for step := 0; step < 150; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, world, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 && bus.Speed > 10.0 {
			launched = true
			break
		}
	}

	if bus.DoorsOpen {
		t.Fatalf("Bus doors should have closed automatically when launching, but are still open")
	}
	if !launched {
		t.Fatalf("Bus failed to start driving from the station after doors closed; final speed = %.2f", bus.Speed)
	}
}

// TestAutopilotStationLaunchDoorsClosed verifies that enabling autopilot when parked at a station
// with doors closed performs passenger boarding (dwells) and then launches towards the target.
func TestAutopilotStationLaunchDoorsClosed(t *testing.T) {
	world := NewWorld(52.0, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, world)
	route.CurrentIndex = 0

	stop0 := world.Stops[route.StopIDs[0]]
	bus := NewBus(stop0.Pos.X, stop0.Pos.Y, stop0.Heading)
	bus.DoorsOpen = false
	bus.DoorProgress = 0.0
	bus.Speed = 0

	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.3
	dt := 0.05

	doorsOpened := false
	doorsClosedAfterDwell := false
	launched := false

	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, world, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if bus.DoorsOpen {
			doorsOpened = true
		}
		if doorsOpened && !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			doorsClosedAfterDwell = true
		}
		if doorsClosedAfterDwell && bus.Speed > 10.0 {
			launched = true
			break
		}
	}

	if !doorsOpened {
		t.Fatalf("Autopilot should open doors for boarding when activating at a station")
	}
	if !doorsClosedAfterDwell {
		t.Fatalf("Autopilot should finish dwell and close doors")
	}
	if !launched {
		t.Fatalf("Bus failed to launch after dwelling at station; speed = %.2f", bus.Speed)
	}
}

// TestAutopilotNavigationNoUTurnNoSpinning verifies Bug 2:
// Autopilot driving along routes on all maps never commands sudden 180° U-turns or spins in circles.
func TestAutopilotNavigationNoUTurnNoSpinning(t *testing.T) {
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
				t.Skip("Route has insufficient stops")
			}

			startStop := world.Stops[route.StopIDs[0]]
			bus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			bus.Speed = 30.0

			driver := NewAutopilotDriver()
			driver.DwellTotal = 0.2
			driver.State = AutopilotStateCruising
			route.CurrentIndex = 1 // Head to first target stop

			dt := 0.05
			accumTurnAngle := 0.0
			prevHeading := bus.Heading

			for step := 0; step < 250; step++ {
				throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
					bus, route, world, nil, nil, 52.0, dt,
				)

				// Steering command must never be NaN or lock-to-lock erratic
				if math.IsNaN(steer) {
					t.Fatalf("Steer angle is NaN at step %d", step)
				}

				bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

				headingDelta := NormalizeAngle(bus.Heading - prevHeading)
				accumTurnAngle += math.Abs(headingDelta)
				prevHeading = bus.Heading

				// Single frame heading change must not exceed 0.2 rad (~11 degrees at 20fps)
				if math.Abs(headingDelta) > 0.35 {
					t.Fatalf("Sudden snap/U-turn detected at step %d: delta=%.3f rad", step, headingDelta)
				}
			}

			// In 250 steps (12.5 seconds), continuous spinning in circles would accumulate > 6*pi (~19 radians)
			if accumTurnAngle > 14.0 {
				t.Fatalf("Vehicle appears to be spinning in circles: accumTurnAngle=%.2f rad", accumTurnAngle)
			}
		})
	}
}

// TestCivilianCarsZeroShoulderDrivingAllMaps verifies Bug 3:
// Civilian AI vehicles driving on all city maps stay centered in lanes and 0 cars ever enter ZoneShoulder.
func TestCivilianCarsZeroShoulderDrivingAllMaps(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	laneW := 52.0
	dt := 1.0 / 60.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			world := NewWorld(laneW, theme)
			tm := NewTrafficManager(laneW)
			tm.BuildForWorld(world, 24, laneW)
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(world)
			bus := NewBus(5000, 5000, 0) // Park dummy bus far away

			shoulderViolations := 0
			var firstViolationPos Vec2
			var firstViolationName string

			for frame := 0; frame < 200; frame++ {
				tm.Update(dt, bus, tls, 2, laneW, world)

				for _, car := range tm.Cars {
					if car.InRoundabout {
						continue // Roundabout interior has distinct circular geometry
					}
					bRes := world.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone == ZoneShoulder || bRes.Zone == ZoneHardBarrier {
						shoulderViolations++
						if firstViolationPos == (Vec2{}) {
							firstViolationPos = car.Pos
							firstViolationName = bRes.ObstacleName
						}
					}
				}
			}

			if shoulderViolations > 0 {
				t.Fatalf("[%s] Detected %d instances of civilian AI cars driving on shoulder/barrier! First at %v (%s)",
					theme.DisplayName(), shoulderViolations, firstViolationPos, firstViolationName)
			}
		})
	}
}

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

// TestIntegrationFullGameplayAllMaps simulates full end-to-end gameplay across all 6 maps:
// verifies single-route architecture, fleet dispatch, station auto-launch, obstacle avoidance,
// zero-shoulder civilian traffic, and smooth simulation without NaN or panics.
func TestIntegrationFullGameplayAllMaps(t *testing.T) {
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
			g := NewGame(1280, 720)
			g.EnterCity(theme)

			// 1. Verify single standard route architecture
			if g.Route == nil || len(g.Route.StopIDs) < 2 {
				t.Fatalf("[%s] Standard loop route must have >= 2 stops", theme.DisplayName())
			}
			if g.Route.StopIDs[0] != g.Route.StopIDs[len(g.Route.StopIDs)-1] {
				t.Fatalf("[%s] Route must be a closed circular loop", theme.DisplayName())
			}

			// 2. Dispatch 2 AI route buses
			g.Settings.AIBusCount = 2
			g.Settings.AIBusFleetChanged = true

			// 3. Enable player autopilot while parked at initial station
			g.Settings.AutopilotEnabled = true

			initPos := g.Bus.Pos

			// 4. Simulate 350 game ticks of live gameplay (dwell cycle takes ~208 frames, then vehicle cruises down the street)
			for frame := 0; frame < 350; frame++ {
				err := g.Update()
				if err != nil {
					t.Fatalf("[%s] Frame %d: g.Update() returned error: %v", theme.DisplayName(), frame, err)
				}

				// Player bus position and speed validity
				if math.IsNaN(g.Bus.Pos.X) || math.IsNaN(g.Bus.Pos.Y) {
					t.Fatalf("[%s] Frame %d: Player bus position is NaN", theme.DisplayName(), frame)
				}
				if math.IsNaN(g.Bus.Speed) {
					t.Fatalf("[%s] Frame %d: Player bus speed is NaN", theme.DisplayName(), frame)
				}

				// Check player bus doesn't collide with barriers
				bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, g.Settings.GetLaneWidth())
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Frame %d: Player bus collided with hard barrier at %v (%s)",
						theme.DisplayName(), frame, g.Bus.Pos, bRes.ObstacleName)
				}

				// Check fleet buses
				if g.AIBusMgr != nil {
					for _, ab := range g.AIBusMgr.Buses {
						if math.IsNaN(ab.Bus.Pos.X) || math.IsNaN(ab.Bus.Pos.Y) {
							t.Fatalf("[%s] Frame %d: AI bus #%d position is NaN", theme.DisplayName(), frame, ab.ID)
						}
					}
				}

				// Check civilian cars do not drive on shoulder
				if g.TrafficMgr != nil {
					for _, c := range g.TrafficMgr.Cars {
						if c.InRoundabout {
							continue
						}
						cRes := g.World.CheckBoundaries(c.Pos, c.Width/2, g.Settings.GetLaneWidth())
						if cRes.Zone == ZoneShoulder || cRes.Zone == ZoneHardBarrier {
							t.Fatalf("[%s] Frame %d: Civilian car #%d driven on shoulder/barrier at %v",
								theme.DisplayName(), frame, c.ID, c.Pos)
						}
					}
				}
			}

			// 5. Verify the player bus successfully launched and moved away from the spawn station
			distMoved := g.Bus.Pos.Distance(initPos)
			if distMoved < 15.0 {
				t.Fatalf("[%s] Player bus failed to launch and move away from the stop (distMoved=%.2f < 15px)",
					theme.DisplayName(), distMoved)
			}
		})
	}
}

// TestAutopilotFullRouteLoopCompletionAllMaps validates that the autonomous bus driver navigates
// complete circular routes from start to finish across all 6 maps, accurately docking inside
// every station bay, servicing passenger boarding, and finishing with zero barrier and zero shoulder violations.
func TestAutopilotFullRouteLoopCompletionAllMaps(t *testing.T) {
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
			laneW := 52.0
			world := NewWorld(laneW, theme)
			route := GetStandardRouteForMap(theme, world)
			if len(route.StopIDs) < 2 {
				t.Fatalf("[%s] Standard loop route must have >= 2 stops", theme.DisplayName())
			}

			startStop := world.Stops[route.StopIDs[0]]
			bus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			driver := NewAutopilotDriver()
			driver.DwellTotal = 0.2 // Efficient dwell for fast test execution
			dt := 1.0 / 60.0

			var dockedStops []int
			prevState := driver.State
			barrierHits := 0
			shoulderHits := 0
			stuckTimer := 0.0
			maxStuck := 0.0

			// Simulate up to 750 seconds (plenty of time for 3x longer circuit at 60fps)
			maxFrames := 60 * 750
			for frame := 0; frame < maxFrames; frame++ {
				throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
					bus, route, world, nil, nil, laneW, dt,
				)
				bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

				if driver.State == AutopilotStateDocked && prevState != AutopilotStateDocked {
					dockedStops = append(dockedStops, driver.DockedStopID)
				}
				prevState = driver.State

				bRes := world.CheckBoundaries(bus.Pos, bus.Width/2, laneW)
				if bRes.Zone == ZoneHardBarrier {
					barrierHits++
				} else if bRes.Zone == ZoneShoulder {
					inBay := false
					for _, s := range world.Stops {
						if s.ContainsBus(bus) || s.Pos.Distance(bus.Pos) < s.Length/2 {
							inBay = true
							break
						}
					}
					if !inBay {
						shoulderHits++
						if shoulderHits <= 5 {
							t.Logf("[%s] shoulderHit #%d: bus.Pos=%v, state=%v, distToRoad=%v, obstacle=%s, zone=%v",
								theme.DisplayName(), shoulderHits, bus.Pos, driver.State, bRes.Penetration, bRes.ObstacleName, bRes.Zone)
						}
					}
				}

				if math.Abs(bus.Speed) < 2.0 && driver.State != AutopilotStateDocked {
					stuckTimer += dt
					if stuckTimer > maxStuck {
						maxStuck = stuckTimer
					}
				} else {
					stuckTimer = 0.0
				}

				// Terminate successfully once full loop is complete and next terminal cycle starts
				if route.TotalTrips >= 1 && len(dockedStops) > len(route.StopIDs) {
					break
				}
			}

			if route.TotalTrips < 1 {
				t.Fatalf("[%s] Autopilot failed to complete a full transit loop (trips=%d, docked=%v)",
					theme.DisplayName(), route.TotalTrips, dockedStops)
			}
			if barrierHits > 0 {
				t.Fatalf("[%s] Autopilot collided with hard barriers during route: %d times",
					theme.DisplayName(), barrierHits)
			}
			if shoulderHits > 0 {
				t.Fatalf("[%s] Autopilot drifted into shoulder during route: %d times",
					theme.DisplayName(), shoulderHits)
			}
			if maxStuck > 2.0 {
				t.Fatalf("[%s] Autopilot got stuck outside stations for %.1f seconds",
					theme.DisplayName(), maxStuck)
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
	offset4Lane := getRoadLaneOffset(0.0, true, lw, world)
	if offset4Lane != lw*1.5 {
		t.Fatalf("Expected 4-lane road to center vehicle in outer lane (lw*1.5 = %.1f), got %.1f", lw*1.5, offset4Lane)
	}

	// 2-lane street (North 2nd St Y = -600):
	// Center is at lw*0.5 = 26.0px.
	offset2Lane := getRoadLaneOffset(-600.0, true, lw, world)
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

// TestRoadCameraAutonomousClearance verifies that the bus in AI cruise mode drives normally past
// mid-block road surveillance cameras without stopping (Bug #2).
func TestRoadCameraAutonomousClearance(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)
	g.Settings.AutopilotEnabled = true
	g.Settings.TrafficDensity = 2 // Medium density (24 active cars out of 36)

	// Ensure no spawn definitions coincide directly with road camera locations
	for _, cam := range g.TrafficLights.RoadCameras {
		for _, c := range g.TrafficMgr.Cars {
			if c.Pos.Distance(cam.Pos) < 10.0 {
				t.Fatalf("Car %d spawns directly on top of road camera at (%.1f, %.1f)", c.ID, cam.Pos.X, cam.Pos.Y)
			}
		}
	}

	// Inactive cars (index >= 24) must never be added to active obstacle list
	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if len(activeCars) != 24 {
		t.Fatalf("Expected 24 active cars for density level 2, got %d", len(activeCars))
	}

	// Set active leg Stop 9 -> Stop 10 Eastbound along South Ring (Y=1200*MapGridScale)
	g.Route.StopIDs = []int{9, 10}
	g.Route.CurrentIndex = 1
	camPos := Vec2{X: -650 * MapGridScale, Y: 1200 * MapGridScale}
	g.Bus.Pos = Vec2{X: camPos.X - 200, Y: 1200*MapGridScale + 52.0*1.5}
	g.Bus.Heading = 0 // Eastbound
	g.Bus.Speed = 100.0
	g.AutopilotDriver.Reset()

	// Advance simulation past the camera gantry
	passedCamera := false
	for frame := 0; frame < 180; frame++ {
		_ = g.Update()
		if g.Bus.Pos.X > camPos.X+80.0 {
			passedCamera = true
			break
		}
		// Bus must never brake to a halt at the camera
		if g.Bus.Pos.X > camPos.X-100.0 && g.Bus.Pos.X < camPos.X+50.0 {
			if g.Bus.Speed < 30.0 {
				t.Fatalf("[Frame %d] Bus stopped near road camera: Pos=(%.1f, %.1f), Spd=%.1f",
					frame, g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Speed)
			}
		}
	}
	if !passedCamera {
		t.Fatalf("Bus failed to drive past road camera within 180 frames; final pos=(%.1f, %.1f)", g.Bus.Pos.X, g.Bus.Pos.Y)
	}
}

// TestRightTurnNormalDrivingAndOncomingExemption verifies that right turns proceed on red
// and oncoming vehicles stopped in opposite lanes do not cause unexpected stops (Bug #3).
func TestRightTurnNormalDrivingAndOncomingExemption(t *testing.T) {
	laneW := 52.0
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		IsMajor:        true,
		Phase:          PhaseNSGreen, // East-West approach has Red light
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	// 1. Through vehicle approaching red light must stop
	posApproach := Vec2{X: -140, Y: laneW * 1.5}
	shouldStopThru, _ := tls.CheckStopForVehicle(posApproach, 0, 130.0, laneW, false, false)
	if !shouldStopThru {
		t.Fatalf("Expected through vehicle to stop for red light")
	}

	// 2. Right-turning vehicle approaching red light is exempt and must NOT stop
	shouldStopRight, _ := tls.CheckStopForVehicle(posApproach, 0, 130.0, laneW, false, true)
	if shouldStopRight {
		t.Fatalf("Expected right-turning vehicle to proceed on red light (红灯允许右转)")
	}

	// 3. Test obstacle filtering when turning: an oncoming car stopped in the opposite lane
	// must NOT cause false-positive braking
	ad := NewAutopilotDriver()
	bus := NewBus(-50, 78, -2.80) // Turning right, nose pointed northwest
	bus.Speed = 60.0
	ad.drivePath = []Vec2{
		{X: -100, Y: 78},
		{X: -78, Y: 78},
		{X: -78, Y: -200},
	}
	ad.cumLen = []float64{0, 22, 300}
	ad.WaypointIdx = 1

	// Oncoming civilian car stopped at North approach opposite lane (X = -130, Y = -50, heading South)
	oncomingCar := VehicleObstacle{
		Pos:     Vec2{X: -130, Y: -50},
		Heading: math.Pi / 2, // South
		Speed:   0.0,
		Length:  46,
		Width:   22,
	}

	// Calculate distance from oncoming car to bus's travel path: should be > half widths buffer
	dToPath := minDistanceToPath(oncomingCar.Pos, ad.drivePath, ad.WaypointIdx)
	if dToPath < (bus.Width/2 + oncomingCar.Width/2 + 8.0) {
		t.Fatalf("Expected oncoming car in opposite lane to be outside travel corridor, got d=%.1f", dToPath)
	}
}

// TestRedLightStopBeforeZebraCrosswalk verifies that when stopping for a red light,
// the bus front bumper stops strictly behind the stop line and leaves buffer before zebra crossing (Bug #4).
func TestRedLightStopBeforeZebraCrosswalk(t *testing.T) {
	laneW := 52.0
	stopLineDist := laneW*1.6 + 18.0 // 101.2px
	crosswalkDist := stopLineDist - 25.0 // 76.2px
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		IsMajor:        true,
		Phase:          PhaseNSGreen, // East-West approach has Red light
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	// Place bus approaching stop line from West heading East (0)
	// Stop line is at X = -stopLineDist (-101.2). Zebra crosswalk is at X = -crosswalkDist (-76.2).
	bus := NewBus(-200, laneW*1.5, 0)
	bus.Speed = 60.0
	ad := NewAutopilotDriver()
	ad.drivePath = []Vec2{{X: -300, Y: laneW * 1.5}, {X: 300, Y: laneW * 1.5}}
	ad.cumLen = []float64{0, 600}

	// Simulate deceleration and stopping before red light
	dt := 0.016
	for step := 0; step < 240; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := ad.ComputeControls(
			bus,
			&Route{StopIDs: []int{0, 1}, CurrentIndex: 0},
			NewWorld(laneW),
			tls,
			nil,
			laneW,
			dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if bus.Speed < 0.1 && step > 60 {
			break
		}
	}

	// Check front bumper position
	frontBumperX := bus.Pos.X + math.Cos(bus.Heading)*(bus.Length*0.5)
	stopLineX := -stopLineDist
	zebraStartX := -crosswalkDist

	if frontBumperX > stopLineX {
		t.Fatalf("Bus front bumper (%.2f) crossed the stop line (%.2f)! Pressed onto zebra crossing!",
			frontBumperX, stopLineX)
	}

	bufferToZebra := zebraStartX - frontBumperX
	if bufferToZebra < 25.0 {
		t.Fatalf("Bus front bumper (%.2f) too close to zebra crosswalk (%.2f), buffer=%.2f < 25px",
			frontBumperX, zebraStartX, bufferToZebra)
	}
}

// TestRoundaboutPathWaypointsInOuterLane verifies that all route waypoints within the roundabout
// are placed in the center of the outer circulating lane, rather than straddling the circular dashed divider line.
func TestRoundaboutPathWaypointsInOuterLane(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	s0 := w.Stops[0]
	s1 := w.Stops[1]
	path := computeRoadPathBetweenStops(w, s0, s1, 52.0)
	rb := w.Roundabouts[0]

	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	midR := (rApron + rOuter) * 0.5 // Circular dashed divider line (188.0px)
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

// TestRoundaboutBusDrivesInOuterLane simulates an autopilot bus driving through the roundabout
// and verifies that the bus stays centered in the outer lane without straddling the middle divider line.
func TestRoundaboutBusDrivesInOuterLane(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	rb := w.Roundabouts[0]
	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	midR := (rApron + rOuter) * 0.5 // 188.0px
	_ = midR

	// Spawn approaching roundabout on West entrance avenue (at rb.Center.X - 550px)
	bus := NewBus(rb.Center.X-550, 78.0, 0)
	bus.Speed = 60.0
	ad := NewAutopilotDriver()

	r := &Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	dt := 0.016
	inRoundaboutCount := 0
	minDistToCenter := math.MaxFloat64
	maxDistToCenter := 0.0

	for step := 0; step < 800; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := ad.ComputeControls(
			bus, r, w, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		d := bus.Pos.Distance(rb.Center)

		// Check while bus is traversing within the circular carriageway
		if d < rb.Radius-15.0 && d > rApron+10.0 {
			inRoundaboutCount++
			if d < minDistToCenter {
				minDistToCenter = d
			}
			if d > maxDistToCenter {
				maxDistToCenter = d
			}

			// Bus center must remain strictly within the outer lane (midR=188 to rOuter=240)
			if d < midR+5.0 || d > rOuter-5.0 {
				t.Fatalf("Step %d: Bus center (dist=%.2f) strayed outside outer lane [%.2f, %.2f]! Pos=(%.1f, %.1f)",
					step, d, midR, rOuter, bus.Pos.X, bus.Pos.Y)
			}

			// Bus inner edge must not cross the circular dashed divider line into the inner lane
			busInnerEdgeDist := d - bus.Width*0.5
			if busInnerEdgeDist < midR {
				t.Fatalf("Step %d: Bus inner edge (dist=%.2f) crossed dashed divider (%.2f)! Pos=(%.1f, %.1f)",
					step, busInnerEdgeDist, midR, bus.Pos.X, bus.Pos.Y)
			}
		}

		if bus.Pos.X > rb.Center.X+rb.Radius+100 {
			break
		}
	}

	if inRoundaboutCount < 30 {
		t.Fatalf("Bus did not traverse roundabout enough steps, inRoundaboutCount=%d", inRoundaboutCount)
	}

	// Traversal bounds must remain strictly inside outer lane
	if minDistToCenter < midR+10.0 {
		t.Errorf("Minimum distance (%.2f) too close to dashed lane line (%.2f)", minDistToCenter, midR)
	}
	if maxDistToCenter > rOuter-10.0 {
		t.Errorf("Maximum distance (%.2f) too close to outer curb (%.2f)", maxDistToCenter, rOuter)
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

// TestRoundaboutNoDeadlockWithYieldingCar tests that a bus circulating inside a roundabout
// does not stop or deadlock with a civilian car waiting at the entrance branch yield line.
func TestRoundaboutNoDeadlockWithYieldingCar(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	s0 := w.Stops[0]
	s1 := w.Stops[1]
	_ = s0
	_ = s1
	rb := w.Roundabouts[0]

	// Place bus inside roundabout on the East side (radius 214px), heading North (-Pi/2)
	bus := NewBus(rb.Center.X+214.0, rb.Center.Y-20.0, -math.Pi/2)
	bus.Speed = 60.0

	// Place civilian car waiting at the East entrance yield line outside the roundabout
	yieldingCar := VehicleObstacle{
		Pos:     Vec2{X: rb.Center.X + 258.0, Y: rb.Center.Y - 40.0},
		Heading: math.Pi, // Facing West into roundabout
		Speed:   0.0,
		Length:  36.0,
		Width:   18.0,
	}

	r := &Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	driver := NewAutopilotDriver()

	dt := 0.016
	stuckSteps := 0

	for step := 0; step < 120; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, r, w, nil, []VehicleObstacle{yieldingCar}, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if bus.Speed < 1.0 {
			stuckSteps++
		}
	}

	if stuckSteps > 5 {
		t.Fatalf("Bus got stuck in deadlock for %d steps with vehicle waiting outside roundabout! Final pos=(%.1f, %.1f), speed=%.2f",
			stuckSteps, bus.Pos.X, bus.Pos.Y, bus.Speed)
	}

	// Bus should have successfully circulated North past the East entrance
	if bus.Pos.Y > rb.Center.Y-40.0 {
		t.Fatalf("Bus failed to progress past East entrance: Pos.Y=%.1f", bus.Pos.Y)
	}
}

// TestRoundaboutAICarsNoDeadlock tests that an AI car circulating inside a roundabout
// does not brake or deadlock with another AI car waiting at an entrance branch yield line.
func TestRoundaboutAICarsNoDeadlock(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	rb := w.Roundabouts[0]
	tm := NewTrafficManager(52.0)

	// Car 0: Yellow car circulating inside roundabout on outer lane near bottom (approaching East)
	car0 := tm.Cars[0]
	car0.InRoundabout = true
	car0.RoundaboutCenter = rb.Center
	car0.RoundaboutTargetR = 214.0
	car0.RoundaboutRadius = 214.0
	car0.RoundaboutOuterRadius = rb.Radius
	car0.RoundaboutAngle = math.Pi / 4 // 45 degrees
	car0.Pos = rb.Center.Add(Vec2{
		X: math.Cos(car0.RoundaboutAngle) * car0.RoundaboutRadius,
		Y: math.Sin(car0.RoundaboutAngle) * car0.RoundaboutRadius,
	})
	car0.Heading = NormalizeAngle(car0.RoundaboutAngle - math.Pi/2)
	car0.Speed = 60.0
	car0.TargetSpeed = 80.0
	car0.LaneIndex = 1

	// Car 1: Blue car approaching/waiting at the South entrance branch yield line
	car1 := tm.Cars[1]
	car1.InRoundabout = false
	car1.Pos = Vec2{X: rb.Center.X + 78.0, Y: rb.Center.Y + rb.Radius + 30.0}
	car1.Heading = -math.Pi / 2 // Heading North into roundabout
	car1.Speed = 0.0
	car1.TargetSpeed = 60.0
	car1.LaneIndex = 1

	bus := NewBus(1000, 1000, 0) // Distant dummy bus

	dt := 0.016
	stuckCount := 0

	for step := 0; step < 100; step++ {
		tm.updateSingleCar(dt, car0, 0, bus, nil, 2, 52.0, w)
		tm.updateSingleCar(dt, car1, 1, bus, nil, 2, 52.0, w)

		if car0.Speed < 1.0 {
			stuckCount++
		}
	}

	if stuckCount > 5 {
		t.Fatalf("Car 0 inside roundabout got stuck/deadlocked with car waiting at entrance! stuckCount=%d, speed=%.2f, pos=(%.1f, %.1f)",
			stuckCount, car0.Speed, car0.Pos.X, car0.Pos.Y)
	}

	// Car 0 must have advanced along the circular arc (angle decreased)
	if car0.RoundaboutAngle >= math.Pi/4 {
		t.Fatalf("Car 0 failed to circulate: RoundaboutAngle=%.2f >= %.2f", car0.RoundaboutAngle, math.Pi/4)
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

	path := computeRoadPathBetweenStops(w, s1, s2, lw)
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

	path := computeRoadPathBetweenStops(w, s0, s1, lw)
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

// TestRoundaboutAutopilotBrakesForEncroachingCar verifies that when an autopilot bus
// is circulating inside a roundabout, it properly brakes and stops before a car that is
// encroaching at the entrance mouth, rather than ignoring it or ramming into it.
func TestRoundaboutAutopilotBrakesForEncroachingCar(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	rb := w.Roundabouts[0]
	r := &Route{StopIDs: []int{1, 2}, CurrentIndex: 1}
	driver := NewAutopilotDriver()

	// Place bus circulating inside roundabout approaching East entrance (X=214, Y=+30), heading North (-Pi/2)
	bus := NewBus(rb.Center.X+214.0, rb.Center.Y+30.0, -math.Pi/2)
	bus.Speed = 40.0

	// Place a car encroaching at the East entrance mouth ahead of the bus (X=225, Y=-55)
	encroachingCar := VehicleObstacle{
		Pos:     Vec2{X: rb.Center.X + 225.0, Y: rb.Center.Y - 55.0},
		Heading: math.Pi, // Facing West
		Speed:   0.0,
		Length:  48.0,
		Width:   22.0,
	}

	dt := 0.016
	// Advance 80 frames: bus should recognize obstacle and brake to a stop before ramming
	for frame := 0; frame < 80; frame++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, r, w, nil, []VehicleObstacle{encroachingCar}, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)
	}

	if bus.Speed > 5.0 {
		t.Fatalf("Autopilot bus failed to brake for encroaching car! Speed = %.2f", bus.Speed)
	}
	distToCar := bus.Pos.Distance(encroachingCar.Pos)
	if distToCar < (bus.Length/2 + encroachingCar.Length/2) {
		t.Fatalf("Autopilot bus collided with encroaching car! Distance = %.2f px", distToCar)
	}
}

// TestRoundaboutCarsYieldToCirculatingBusAndAutopilotNoFines tests that:
// 1. A civilian traffic car approaching a roundabout yields and stops behind the yield line while a bus is circulating.
// 2. In Autopilot mode, any collision contact is zero-fine protected.
// 3. Roundabout collisions with entering vehicles do not fine the circulating bus.
func TestRoundaboutCarsYieldToCirculatingBusAndAutopilotNoFines(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.TrafficFinesEnabled = true
	g.Settings.AutopilotEnabled = true
	w := g.World
	rb := w.Roundabouts[0]

	// Bus circulating inside roundabout at radius 214px
	g.Bus.Pos = Vec2{X: rb.Center.X + 214.0, Y: rb.Center.Y - 20.0}
	g.Bus.Heading = -math.Pi / 2
	g.Bus.Speed = 40.0

	// Civilian car approaching from East road
	car := &TrafficCar{
		Pos:         Vec2{X: rb.Center.X + 300.0, Y: rb.Center.Y - 100.8},
		Heading:     math.Pi, // Westbound
		Speed:       80.0,
		TargetSpeed: 80.0,
		Length:      48.0,
		Width:       22.0,
		IsEW:        true,
	}
	g.TrafficMgr.Cars = []*TrafficCar{car}

	dt := 1.0 / 60.0
	// Simulate 120 frames: car should yield behind yield line (distToCenter > rb.Radius + 15)
	for i := 0; i < 120; i++ {
		g.TrafficMgr.updateSingleCar(dt, car, 0, g.Bus, nil, 1, 52.0, w)
	}

	distToCenter := car.Pos.Distance(rb.Center)
	if distToCenter < rb.Radius+15.0 {
		t.Fatalf("Car failed to yield at roundabout entrance! Distance to center = %.2f, expected >= %.2f",
			distToCenter, rb.Radius+15.0)
	}
	if car.Speed > 10.0 {
		t.Fatalf("Car should be stopped/braking at yield line, got speed = %.2f", car.Speed)
	}

	// 2. Autopilot mode collision protection test: force collision contact
	initialCash := g.Cash
	g.Bus.CrashCooldown = 0
	car.Pos = g.Bus.Pos.Add(Vec2{X: 10.0, Y: -10.0}) // Direct contact
	g.checkBusCarCollisions()

	if g.Cash != initialCash {
		t.Fatalf("Autopilot mode collision must NOT deduct fine! Expected %d, got %d", initialCash, g.Cash)
	}
}

// TestAutopilotBerserkModeStraightSpeedAndSafety tests the new Berserk Mode for the player bus:
// 1. Straightaway speed reaches up to 310 px/s (compared to standard 150 px/s);
// 2. Safely decelerates for corners, red lights, and vehicle obstacles;
// 3. Background AI buses and traffic cars remain in standard mode.
func TestAutopilotBerserkModeStraightSpeedAndSafety(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)

	w := g.World
	lw := 52.0
	driver := NewAutopilotDriver()

	// 1. Straight path along Y = 1200
	driver.pathWorld = w
	driver.pathFromID = 0
	driver.pathToID = 1
	driver.pathLane = lw
	driver.drivePath = []Vec2{
		{X: -1000, Y: 1200},
		{X: 1000, Y: 1200},
	}
	driver.cumLen = []float64{0, 2000.0}
	driver.WaypointIdx = 0

	route := NewRoute()
	route.StopIDs = []int{0, 1}
	route.CurrentIndex = 1

	b := NewBus(-600, 1200, 0) // Eastbound directly on the straight path
	b.Heading = 0
	b.Speed = 120.0

	// A. Standard Mode Test
	driver.BerserkMode = false
	throttleStd, _, _, _, _, _ := driver.ComputeControls(b, route, w, nil, nil, lw, 0.016)
	if driver.LastTargetSpeed > apCruiseSpeed+5.0 {
		t.Fatalf("Standard mode target speed should be around 150 px/s, got %.2f", driver.LastTargetSpeed)
	}

	// B. Berserk Mode Straightaway Test
	driver.BerserkMode = true
	b.Speed = 140.0
	throttleBerserk, _, _, _, _, _ := driver.ComputeControls(b, route, w, nil, nil, lw, 0.016)
	if driver.LastTargetSpeed < 440.0 {
		t.Fatalf("Berserk mode on straightaway must reach top speed >= 440 px/s (80 km/h), got %.2f", driver.LastTargetSpeed)
	}
	if throttleBerserk < throttleStd {
		t.Fatalf("Berserk mode throttle (%.2f) should be stronger than standard (%.2f)", throttleBerserk, throttleStd)
	}

	// C. Corner Deceleration Test: approaching a turn must slow down safely
	// Mock a path with a 90-degree turn ahead at 100px
	driver.drivePath = []Vec2{
		{X: -600, Y: 1200},
		{X: -500, Y: 1200},
		{X: -500, Y: 1400}, // 90 degree sharp turn
	}
	driver.cumLen = []float64{0, 100.0, 300.0}
	driver.WaypointIdx = 0
	b.Pos = Vec2{X: -580, Y: 1200}
	b.Speed = 280.0
	cornerLimit := driver.cornerSpeedLimit(20.0, 280.0)
	if cornerLimit > 170.0 {
		t.Fatalf("Approaching sharp corner at 80px must limit speed, got %.2f (expected <= 170)", cornerLimit)
	}

	// D. Red Light Safety Test: must slow to 0 before stop line under red light
	tls := NewTrafficLightSystem()
	inter := &TrafficIntersection{
		Center:  Vec2{X: -400, Y: 1200},
		Phase:   PhaseNSGreen, // EW is RED
		Timer:   0,
		IsMajor: true,
	}
	tls.Intersections = []*TrafficIntersection{inter}

	// Revert to straight path towards intersection
	driver.drivePath = []Vec2{{X: -800, Y: 1200}, {X: 200, Y: 1200}}
	driver.cumLen = []float64{0, 1000.0}
	driver.WaypointIdx = 0
	b.Pos = Vec2{X: -580, Y: 1200}
	b.Speed = 220.0

	_, brakeTL, _, _, _, _ := driver.ComputeControls(b, route, w, tls, nil, lw, 0.016)
	if driver.LastTargetSpeed > 100.0 {
		t.Fatalf("Must anticipate red light and decelerate target speed, got %.2f", driver.LastTargetSpeed)
	}
	if brakeTL <= 0.0 && driver.LastTargetSpeed > b.Speed {
		t.Fatalf("Must apply brake or reduce target speed when approaching red light from high speed")
	}

	// D2. Yellow Light Test: in Berserk Mode, approaching a yellow light must NOT stop (runs yellow light)
	interYellow := &TrafficIntersection{
		Center:  Vec2{X: -400, Y: 1200},
		Phase:   PhaseEWYellow, // EW is YELLOW
		Timer:   1.0,
		IsMajor: true,
	}
	tlsYellow := NewTrafficLightSystem()
	tlsYellow.Intersections = []*TrafficIntersection{interYellow}
	driver.BerserkMode = true
	b.Pos = Vec2{X: -580, Y: 1200}
	b.Speed = 220.0
	driver.ComputeControls(b, route, w, tlsYellow, nil, lw, 0.016)
	if driver.LastTargetSpeed < 200.0 {
		t.Fatalf("Berserk mode must run yellow light without stopping, got target speed %.2f", driver.LastTargetSpeed)
	}

	// But in Standard Mode, it must stop for yellow light:
	driver.BerserkMode = false
	driver.ComputeControls(b, route, w, tlsYellow, nil, lw, 0.016)
	if driver.LastTargetSpeed > 100.0 {
		t.Fatalf("Standard mode must stop for yellow light, got target speed %.2f", driver.LastTargetSpeed)
	}

	// E. Obstacle Avoidance Test: must decelerate and stop safely behind slower/stopped lead vehicle
	driver.BerserkMode = true
	stoppedCar := VehicleObstacle{
		Pos:     Vec2{X: -440, Y: 1200}, // 140px in front
		Heading: 0,
		Speed:   0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(b, route, w, nil, []VehicleObstacle{stoppedCar}, lw, 0.016)
	if driver.LastTargetSpeed > 130.0 {
		t.Fatalf("Must decelerate from high speed for lead vehicle at 140px, got target speed %.2f", driver.LastTargetSpeed)
	}

	// Close obstacle: within safe stop distance must command full stop (targetSpd = 0)
	closeCar := VehicleObstacle{
		Pos:     Vec2{X: -510, Y: 1200}, // 70px in front (< safeStopDist 85px)
		Heading: 0,
		Speed:   0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(b, route, w, nil, []VehicleObstacle{closeCar}, lw, 0.016)
	if driver.LastTargetSpeed > 0.0 {
		t.Fatalf("Must command full stop when lead vehicle is within safe distance, got %.2f", driver.LastTargetSpeed)
	}

	// F. Verify Other AI Buses remain in Standard Mode
	aiBusMgr := NewAIBusManager()
	aiBusMgr.SetLineCount(1, w, lw)
	if len(aiBusMgr.Buses) > 0 && aiBusMgr.Buses[0].Driver.BerserkMode {
		t.Fatalf("Background Route AI Bus must remain in standard mode (BerserkMode=false)")
	}

	// G. Settings & Toggle Verification
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = false

	// Cycle via Settings
	if !g.Settings.AutopilotBerserk {
		g.Settings.AutopilotBerserk = true
	}
	if !g.Settings.AutopilotBerserk {
		t.Fatalf("Berserk mode flag should be enabled")
	}
}

// TestBerserkModeOvertakingOnMultiLaneRoad verifies that on 4-lane roads (2 lanes per direction),
// the player bus in Berserk Mode can safely change lanes to overtake slower lead vehicles,
// while in Standard Mode or on single-lane roads it does not overtake.
func TestBerserkModeOvertakingOnMultiLaneRoad(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Stops on South Ring Avenue (Y=1200*MapGridScale, 4-lane avenue)
	s1 := NewBusStop(201, "AvenueStart", Vec2{X: -1000 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(202, "AvenueEnd", Vec2{X: 1000 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "OvertakeTestRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -600 * MapGridScale, Y: 1200*MapGridScale + lw*1.5} // In outer travel lane (offset 1.5*lw)
	bus.Heading = 0                                                       // Eastbound
	bus.Speed = 160.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()

	// Slow lead vehicle in same outer lane
	slowCar := VehicleObstacle{
		Pos:     Vec2{X: -600*MapGridScale + 90, Y: 1200*MapGridScale + lw*1.5}, // 90px in front
		Heading: 0,
		Speed:   50.0,
		Length:  48,
		Width:   22,
	}

	// 1. Standard Mode: should NOT overtake
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone || driver.LaneOffset != 0.0 {
		t.Fatalf("Standard mode must not initiate overtaking (state=%d, offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 2. Berserk Mode: initiates overtake into inner lane (-lw)
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate lane change (expected state %d, got %d)",
			OvertakeChangingLane, driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f (inner lane), got %.2f", lw, driver.TargetLaneOffset)
	}

	// Advance time until lane change is complete
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.08)
	}
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected OvertakePassing once offset reached, got state %d (offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}
	if math.Abs(driver.LaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Lane offset should be centered in passing lane (-%.1f), got %.2f", lw, driver.LaneOffset)
	}

	// Move bus ahead of the slow vehicle (overtake completed)
	bus.Pos.X = -380 // 130px ahead of slowCar at X=-510
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.5)
	if driver.OvertakeState != OvertakeReturning {
		t.Fatalf("Expected OvertakeReturning once vehicle passed, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Target offset after passing should return to 0, got %.2f", driver.TargetLaneOffset)
	}

	// Advance until returned to base lane
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, nil, lw, 0.08)
	}
	if driver.OvertakeState != OvertakeNone || math.Abs(driver.LaneOffset) > 0.1 {
		t.Fatalf("Expected return to OvertakeNone and 0 offset, got state %d, offset %.2f",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 3. Passing lane blocked: safety check
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	bus.Pos = Vec2{X: -600, Y: 1200 + lw*1.5}
	passingCar := VehicleObstacle{
		Pos:     Vec2{X: -550, Y: 1200 + lw*0.5}, // in passing lane
		Heading: 0,
		Speed:   80.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, passingCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Must not overtake when passing lane is occupied (got state %d)", driver.OvertakeState)
	}

	// 4. 2-lane road (1 lane per direction):
	// a. Standard mode: strictly prohibited
	driver.Reset()
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	bus.Pos = Vec2{X: 1300, Y: 400} // On 2-lane road
	bus.Heading = math.Pi / 2        // Southbound
	slowCar2Lane := VehicleObstacle{
		Pos:     Vec2{X: 1300, Y: 480},
		Heading: math.Pi / 2,
		Speed:   40.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar2Lane}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Standard mode must never overtake on 2-lane roads (got state %d)", driver.OvertakeState)
	}

	// b. Approaching intersection on 2-lane road: borrowing oncoming lane is prohibited
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar2Lane}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Berserk mode must not borrow lane near intersections on 2-lane roads (got state %d)", driver.OvertakeState)
	}
}

// TestBerserkModeBorrowLaneOvertakingOn2LaneRoad tests 借道超车 in Berserk Mode on 2-way 2-lane roads:
// 1. Standard mode: cannot borrow oncoming lane
// 2. Berserk mode with oncoming vehicle: blocked from borrowing lane
// 3. Berserk mode with oncoming lane clear: borrows oncoming lane (-lw) to overtake
// 4. Smooth lateral transition into oncoming lane
// 5. Passing slow car and safely returning to base lane (offset 0.0)
// 6. Active abort: if an oncoming car appears while in oncoming lane, aborts back to base lane
// 7. Verification across all Berserk top speed settings (60, 80, 100, 120 km/h)
func TestBerserkModeBorrowLaneOvertakingOn2LaneRoad(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Stops along 2-lane road "北二路" (Y = -600 * MapGridScale, Eastbound)
	// Road segment: P1: {-2600*MapGridScale, -600*MapGridScale}, P2: {2600*MapGridScale, -600*MapGridScale}, Lanes: 2, Type: RoadStreet2Lane
	// Travel lane (Eastbound) is at Y = roadY + lw*0.5
	// Oncoming lane (Westbound) is at Y = roadY - lw*0.5
	roadY := -600 * MapGridScale
	s1 := NewBusStop(401, "2LaneStart", Vec2{X: -1000 * MapGridScale, Y: roadY}, 0, 2, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(402, "2LaneEnd", Vec2{X: 1000 * MapGridScale, Y: roadY}, 0, 2, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "2LaneBorrowRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	startX := -600 * MapGridScale
	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: startX, Y: roadY + lw*0.5} // Base travel lane
	bus.Heading = 0                             // Eastbound
	bus.Speed = 160.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()

	// Slow blocker in our travel lane ahead
	slowCar := VehicleObstacle{
		Pos:     Vec2{X: startX + 90, Y: roadY + lw*0.5}, // 90px ahead
		Heading: 0,
		Speed:   40.0,
		Length:  48,
		Width:   22,
	}

	// 1. Standard Mode: should NOT borrow oncoming lane
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone || driver.LaneOffset != 0.0 {
		t.Fatalf("Standard mode must not borrow oncoming lane (state=%d, offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 2. Berserk Mode with oncoming vehicle in oncoming lane: MUST NOT borrow lane
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	oncomingCar := VehicleObstacle{
		Pos:     Vec2{X: startX + 400, Y: roadY - lw*0.5}, // Ahead in oncoming lane (400px ahead)
		Heading: math.Pi,                                 // Heading Westbound (oncoming!)
		Speed:   80.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, oncomingCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Must not borrow lane when oncoming lane has oncoming car (got state %d)", driver.OvertakeState)
	}

	// 3. Berserk Mode with oncoming lane CLEAR: initiates 借道超车
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate 借道超车 when oncoming lane is clear (got state %d)", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f (oncoming lane), got %.2f", lw, driver.TargetLaneOffset)
	}

	// 4. Progress into oncoming lane
	for i := 0; i < 20; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.05)
	}
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected OvertakePassing once arrived in oncoming lane, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.LaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Expected lane offset -%.1f in oncoming lane, got %.2f", lw, driver.LaneOffset)
	}

	// 5. Still alongside slowCar: should stay in passing lane
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Must remain in OvertakePassing while alongside blocker, got %d", driver.OvertakeState)
	}

	// 6. Passed slowCar: safely return to base lane
	bus.Pos.X = startX + 260 // 170px ahead of slowCar at startX+90 (beyond minSafeBehind ~147px)
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.5)
	if driver.OvertakeState != OvertakeReturning {
		t.Fatalf("Expected OvertakeReturning once vehicle passed, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Target offset after passing should return to 0, got %.2f", driver.TargetLaneOffset)
	}

	// 7. Advance until returned to base lane
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, nil, lw, 0.08)
	}
	if driver.OvertakeState != OvertakeNone || math.Abs(driver.LaneOffset) > 0.1 {
		t.Fatalf("Expected return to OvertakeNone and 0 offset, got state %d, offset %.2f",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 8. Active abort: if oncoming car appears while in oncoming lane, aborts to base lane
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	bus.Pos = Vec2{X: startX, Y: roadY - lw*0.5} // Physically in oncoming lane
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.OvertakeState = OvertakePassing
	imminentOncoming := VehicleObstacle{
		Pos:     Vec2{X: startX + 300, Y: roadY - lw*0.5}, // 300px ahead in oncoming lane
		Heading: math.Pi,                                 // Oncoming
		Speed:   90.0,
		Length:  48,
		Width:   22,
	}
	// Base lane is clear to return
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{imminentOncoming}, lw, 0.016)
	if driver.OvertakeState != OvertakeReturning || math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Must abort back to base lane when oncoming car appears (state=%d, target=%.2f)",
			driver.OvertakeState, driver.TargetLaneOffset)
	}

	// 9. Verify across all 4 Berserk top speed settings (60, 80, 100, 120 km/h)
	speedConfigs := []struct {
		kmh       int
		cruiseSpd float64
	}{
		{60, 333.0},
		{80, 445.0},
		{100, 555.0},
		{120, 667.0},
	}
	for _, sc := range speedConfigs {
		driver.Reset()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = sc.cruiseSpd
		driver.State = AutopilotStateCruising
		bus.Pos = Vec2{X: startX, Y: roadY + lw*0.5}
		bus.Speed = sc.cruiseSpd

		// With oncoming car: cannot borrow lane
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, oncomingCar}, lw, 0.016)
		if driver.OvertakeState != OvertakeNone {
			t.Fatalf("MaxSpeed %d: must not borrow lane with oncoming traffic (got %d)", sc.kmh, driver.OvertakeState)
		}

		// When oncoming lane is clear: can borrow lane
		driver.Reset()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = sc.cruiseSpd
		driver.State = AutopilotStateCruising
		bus.Pos = Vec2{X: startX, Y: roadY + lw*0.5}
		bus.Speed = sc.cruiseSpd

		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
		if driver.OvertakeState != OvertakeChangingLane {
			t.Fatalf("MaxSpeed %d: expected OvertakeChangingLane when oncoming lane clear, got %d", sc.kmh, driver.OvertakeState)
		}
		if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
			t.Fatalf("MaxSpeed %d: expected target offset -%.1f, got %.2f", sc.kmh, lw, driver.TargetLaneOffset)
		}
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

	pathShort := computeRoadPathBetweenStops(w, sShort, sDest, lw)
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
	pathLong := computeRoadPathBetweenStops(w, sLong, sDest, lw)
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

// TestSixLaneRoadGeometryAndNavigation validates 6-lane roads (3 lanes per direction / 6 lanes total):
// 1. Identification via IsRoad6Lane and GetRoadLanes across Metropolis, Coastal Bay, River Valley, and Twin Cities
// 2. Geometry width calculation (float64(Lanes)*laneWidth + 24.0)
// 3. Navigation lane offset: 2.5 * laneWidth on 6-lane roads vs 1.5*lw on 4-lane and 0.5*lw on 2-lane
// 4. Smooth multi-lane transitions from outer cruising lane (2.5*lw) to inner turn lane (0.5*lw)
// 5. Berserk overtaking on 6-lane expressways
func TestSixLaneRoadGeometryAndNavigation(t *testing.T) {
	lw := 52.0

	// 1. Map 1 (Metropolis): North Ring Expressway (Y = -1200 * MapGridScale) is 6-lane
	w1 := NewWorld(lw, MapThemeMetropolis)
	if !w1.IsRoad6Lane(-1200*MapGridScale, true) {
		t.Fatalf("Metropolis North Ring (Y=%.0f) must be 6-lane road", -1200*MapGridScale)
	}
	if lanes := w1.GetRoadLanes(-1200*MapGridScale, true); lanes != 6 {
		t.Fatalf("Expected 6 lanes at Y=%.0f in Metropolis, got %d", -1200*MapGridScale, lanes)
	}
	if off := getRoadLaneOffset(-1200*MapGridScale, true, lw, w1); math.Abs(off-lw*2.5) > 0.01 {
		t.Fatalf("Expected 6-lane lane offset 2.5*lw (%.1f), got %.2f", lw*2.5, off)
	}

	// 2. Map 2 (Coastal Bay): Central Grand Boulevard (Y = 0) is 6-lane
	w2 := NewWorld(lw, MapThemeCoastalBay)
	if !w2.IsRoad6Lane(0, true) {
		t.Fatalf("Coastal Bay Central Boulevard (Y=0) must be 6-lane road")
	}
	if lanes := w2.GetRoadLanes(0, true); lanes != 6 {
		t.Fatalf("Expected 6 lanes at Y=0 in Coastal Bay, got %d", lanes)
	}
	if off := getRoadLaneOffset(0, true, lw, w2); math.Abs(off-lw*2.5) > 0.01 {
		t.Fatalf("Expected 6-lane lane offset 2.5*lw (%.1f), got %.2f", lw*2.5, off)
	}

	// 3. Map 3 (River Valley): High Speed Rail Boulevard (Y = 1200*MapGridScale) and Century Suspension Bridge (X = 1300*MapGridScale) are 6-lane
	w3 := NewWorld(lw, MapThemeRiverValley)
	if !w3.IsRoad6Lane(1200*MapGridScale, true) {
		t.Fatalf("River Valley HSR Boulevard (Y=%.0f) must be 6-lane road", 1200*MapGridScale)
	}
	if !w3.IsRoad6Lane(1300*MapGridScale, false) {
		t.Fatalf("River Valley Century Suspension Bridge (X=%.0f) must be 6-lane road", 1300*MapGridScale)
	}

	// 4. Map 5 (Twin Cities): Central Elevated Expressway (Y = 0) is 6-lane
	w5 := NewWorld(lw, MapThemeTwinCities)
	if !w5.IsRoad6Lane(0, true) {
		t.Fatalf("Twin Cities Central Viaduct Expressway (Y=0) must be 6-lane road")
	}

	// 5. Road width validation:
	// 6-lane: 6*52 + 24 = 336
	// 4-lane: 4*52 + 24 = 232
	// 2-lane: 2*52 + 12 = 116
	r6 := RoadSegment{Lanes: 6, Type: RoadExpressway6Lane}
	r4 := RoadSegment{Lanes: 4, Type: RoadAvenue4Lane}
	r2 := RoadSegment{Lanes: 2, Type: RoadStreet2Lane}
	if w := r6.GetWidth(lw); math.Abs(w-(6*lw+24.0)) > 0.01 {
		t.Fatalf("Expected 6-lane width %.1f, got %.1f", 6*lw+24.0, w)
	}
	if w := r4.GetWidth(lw); math.Abs(w-(4*lw+24.0)) > 0.01 {
		t.Fatalf("Expected 4-lane width %.1f, got %.1f", 4*lw+24.0, w)
	}
	if w := r2.GetWidth(lw); math.Abs(w-(2*lw+12.0)) > 0.01 {
		t.Fatalf("Expected 2-lane width %.1f, got %.1f", 2*lw+12.0, w)
	}

	// 6. Navigation path left turn on 6-lane road: transitions from 2.5*lw to 0.5*lw
	// Westbound on Y=-1200*MapGridScale turning Left (South) onto X=0 avenue
	sStart := NewBusStop(401, "Start6Lane", Vec2{X: 800 * MapGridScale, Y: -1200 * MapGridScale}, math.Pi, 6, RGBA(0, 0, 0, 0), lw, 0)
	sTurn := NewBusStop(402, "TurnToAvenue", Vec2{X: 0, Y: -600 * MapGridScale}, math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
	path := computeRoadPathBetweenStops(w1, sStart, sTurn, lw)
	if len(path) == 0 {
		t.Fatalf("Failed to compute left-turn path on 6-lane road")
	}

	var hasOuterCruising bool
	var hasInnerLeftTurn bool
	for _, pt := range path {
		// Westbound outer cruising lane is at Y = -1200*MapGridScale - 2.5*lw
		if math.Abs(pt.Y-(-1200*MapGridScale-lw*2.5)) < 2.0 {
			hasOuterCruising = true
		}
		// Inner left turn lane is at Y = -1200*MapGridScale - 0.5*lw
		if math.Abs(pt.Y-(-1200*MapGridScale-lw*0.5)) < 2.0 {
			hasInnerLeftTurn = true
		}
	}
	if !hasOuterCruising {
		t.Fatalf("Expected path on 6-lane road to include outer cruising lane (Y=%.1f)", -1200*MapGridScale-lw*2.5)
	}
	if !hasInnerLeftTurn {
		t.Fatalf("Expected path on 6-lane road to transition to inner left-turn lane (Y=%.1f)", -1200*MapGridScale-lw*0.5)
	}

	// 7. Berserk mode overtaking on 6-lane road:
	wTest := NewWorld(lw, MapThemeMetropolis)
	s6Start := NewBusStop(501, "RingStart", Vec2{X: -1000 * MapGridScale, Y: -1200 * MapGridScale}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	s6End := NewBusStop(502, "RingEnd", Vec2{X: 1000 * MapGridScale, Y: -1200 * MapGridScale}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	wTest.Stops = []*BusStop{s6Start, s6End}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -600 * MapGridScale, Y: -1200*MapGridScale + lw*2.5} // In outer cruising lane
	bus.Heading = 0
	bus.Speed = 160.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	route := &Route{
		Name:         "Overtake6LaneRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	slowCar := VehicleObstacle{
		Pos:     Vec2{X: -600*MapGridScale + 90, Y: -1200*MapGridScale + lw*2.5},
		Heading: 0,
		Speed:   50.0,
		Length:  48,
		Width:   22,
	}

	// First step initializes path tracking and remaining distance
	driver.ComputeControls(bus, route, wTest, nil, nil, lw, 0.016)
	// Second step encounters slowCar on 6-lane road and initiates lane change
	driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate lane change on 6-lane road (state=%d)", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Expected target lane offset -%.1f into middle lane, got %.2f", lw, driver.TargetLaneOffset)
	}
}

// TestMajorIntersectionAutopilotLeftTurnRedLightStopping verifies that when turning left at a major intersection,
// the autopilot strictly respects the dedicated left-turn red signal (even when through traffic has a green light),
// never confuses lane merges for right turns, and comes to a complete, safe stop behind the stop line.
func TestMajorIntersectionAutopilotLeftTurnRedLightStopping(t *testing.T) {
	lw := 52.0
	inter := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseEWGreen, // Through is GREEN, Left arrow is RED!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
	stopLineDist, _, _, _ := inter.GetApproachDimensions(lw)

	// Verify signals for Eastbound approach:
	thruSig, leftSig := inter.GetSignalForApproach(0)
	if thruSig != SignalGreen {
		t.Fatalf("Expected through signal to be Green during PhaseEWGreen, got %v", thruSig)
	}
	if leftSig != SignalRed {
		t.Fatalf("Expected left signal to be Red during PhaseEWGreen, got %v", leftSig)
	}

	// 1. Direct CheckStopForVehicle test in dedicated left-turn lane (Y = lw*0.5 = 26.0)
	posApproachLeft := Vec2{X: -(stopLineDist + 50.0), Y: lw * 0.5} // bumper ~ -(stopLineDist + 2.0)
	bumperPos := posApproachLeft.Add(Vec2{X: 48, Y: 0})

	// When vehicle is in left-turn lane, it MUST stop for left-turn red light even if through is green:
	shouldStop, distToLine := tls.CheckStopForVehicle(bumperPos, 0, 140.0, lw, true, false)
	if !shouldStop {
		t.Fatalf("CheckStopForVehicle must return shouldStop=true for left turn when left arrow is Red")
	}
	if distToLine <= 0 {
		t.Fatalf("distToLine should be positive, got %.2f", distToLine)
	}

	// Even if wantRight was erroneously passed, a vehicle in the left-turn lane can NEVER turn right on red:
	shouldStopBogusRight, _ := tls.CheckStopForVehicle(bumperPos, 0, 140.0, lw, true, true)
	if !shouldStopBogusRight {
		t.Fatalf("CheckStopForVehicle must NEVER grant right-turn-on-red exemption to a vehicle turning left in left-turn lane")
	}

	// 2. Full Autopilot dynamic stopping simulation
	// Set up a route that turns left at the intersection:
	// Eastbound along road (Y = 78 outer -> merges to Y = 26 inner -> turns left at intersection center into Northbound road)
	driver := NewAutopilotDriver()
	driver.State = AutopilotStateCruising
	driver.drivePath = []Vec2{
		{X: -600, Y: lw * 1.5},
		{X: -350, Y: lw * 1.5},
		{X: -350, Y: lw * 0.5}, // Lane-change merge into dedicated left-turn lane
		{X: lw * 0.5, Y: lw * 0.5}, // Corner apex at inner lane junction
		{X: lw * 0.5, Y: -400}, // Exiting Northbound
	}
	driver.cumLen = make([]float64, len(driver.drivePath))
	for i := 1; i < len(driver.drivePath); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + driver.drivePath[i].Distance(driver.drivePath[i-1])
	}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -260, Y: lw * 0.5} // In left-turn lane, 260px before intersection center
	bus.Heading = 0
	bus.Speed = 140.0
	bus.Gear = GearDrive

	route := &Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
	w := NewWorld(lw, MapThemeMetropolis)

	// Simulate up to 300 steps (5 seconds) of approach:
	dt := 0.016
	for step := 0; step < 300; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, tls, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Check violation on every frame
		if violated, notice := tls.CheckPlayerViolation(bus, lw); violated {
			t.Fatalf("Frame %d: Autopilot ran red light on left turn! Notice: %s, pos: %v, speed: %.2f",
				step, notice, bus.Pos, bus.Speed)
		}

		if math.Abs(bus.Speed) < 0.5 && bus.Pos.X > -250 {
			break
		}
	}

	// Front bumper of bus must have stopped strictly before the stop line (X = -101.2)
	frontBumperX := bus.Pos.X + math.Cos(bus.Heading)*(bus.Length*0.5)
	if frontBumperX > -stopLineDist {
		t.Fatalf("Bus front bumper (%.2f) overshot stop line (-%.2f)! Ran red light!", frontBumperX, stopLineDist)
	}
	if math.Abs(bus.Speed) > 1.0 {
		t.Fatalf("Bus failed to come to a complete stop before red light, speed=%.2f", bus.Speed)
	}

	// 3. Signal transition to Left-Turn Green (PhaseEWLeftGreen): bus resumes and navigates left turn
	inter.Phase = PhaseEWLeftGreen
	_, leftSigNew := inter.GetSignalForApproach(0)
	if leftSigNew != SignalGreen {
		t.Fatalf("Expected left signal to be Green during PhaseEWLeftGreen, got %v", leftSigNew)
	}

	// Simulate up to 300 steps (5 seconds) of driving through the green left turn:
	for step := 0; step < 300; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, tls, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if violated, notice := tls.CheckPlayerViolation(bus, lw); violated {
			t.Fatalf("Frame %d: Violated traffic rule during lawful left turn! Notice: %s", step, notice)
		}

		if bus.Pos.Y < 0 {
			break
		}
	}

	// Verify bus successfully navigated through the intersection and is now heading North
	if bus.Pos.Y > 0 {
		t.Fatalf("Expected bus to have traversed into North road (Y < 0), got Y=%.1f", bus.Pos.Y)
	}
}

// TestBerserkModeMajorIntersectionLeftTurnRedLight verifies that even in Berserk mode (80 km/h),
// the bus still strictly obeys major intersection red left-turn arrows and decelerates in time.
func TestBerserkModeMajorIntersectionLeftTurnRedLight(t *testing.T) {
	lw := 52.0
	stopLineDist := lw*1.6 + 18.0

	inter := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseEWGreen, // Through Green, Left RED
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.drivePath = []Vec2{
		{X: -800, Y: lw * 0.5},
		{X: lw * 0.5, Y: lw * 0.5},
		{X: lw * 0.5, Y: -500},
	}
	driver.cumLen = make([]float64, len(driver.drivePath))
	for i := 1; i < len(driver.drivePath); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + driver.drivePath[i].Distance(driver.drivePath[i-1])
	}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -400, Y: lw * 0.5}
	bus.Heading = 0
	bus.Speed = 260.0 // Approaching at high speed
	bus.Gear = GearDrive

	route := &Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
	w := NewWorld(lw, MapThemeMetropolis)

	driver.pathWorld = w
	driver.pathFromID = 1
	driver.pathToID = 0
	driver.pathLane = lw

	dt := 0.016
	for step := 0; step < 300; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, tls, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if violated, notice := tls.CheckPlayerViolation(bus, lw); violated {
			t.Fatalf("Frame %d: Berserk mode ran red left light! Notice: %s, pos: %v", step, notice, bus.Pos)
		}

		if math.Abs(bus.Speed) < 0.5 && bus.Pos.X > -300 {
			break
		}
	}

	frontBumperX := bus.Pos.X + math.Cos(bus.Heading)*(bus.Length*0.5)
	if frontBumperX > -stopLineDist {
		t.Fatalf("Berserk mode bus front bumper (%.2f) overshot stop line (-%.2f)!", frontBumperX, stopLineDist)
	}
	if math.Abs(bus.Speed) > 1.0 {
		t.Fatalf("Berserk mode bus failed to come to a complete stop before red light, speed=%.2f", bus.Speed)
	}
}

// TestAutopilotMetropolisLeftTurnsWithTrafficLightsNoViolations runs the autopilot across
// full Metropolis intersections with active traffic light cycles and verifies 0 red-light violations.
func TestAutopilotMetropolisLeftTurnsWithTrafficLightsNoViolations(t *testing.T) {
	laneW := 52.0
	world := NewWorld(laneW, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, world)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(world)

	startStop := world.Stops[route.StopIDs[0]]
	bus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.1
	dt := 0.05 // 20 updates per second for fast simulation

	// Run for 3000 steps (150 seconds of simulated driving)
	for step := 0; step < 3000; step++ {
		tls.Update(dt)

		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, world, tls, nil, laneW, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if violated, notice := tls.CheckPlayerViolation(bus, laneW); violated {
			t.Fatalf("Step %d: Autopilot committed traffic violation: %s, bus pos: %v, speed: %.2f",
				step, notice, bus.Pos, bus.Speed)
		}
	}
}

// TestBerserkModeEarlyOvertakeInitiation verifies that in Berserk mode, the bus initiates lane change
// early at an ample distance (> 350px) ahead of a slow lead vehicle rather than closing in and tailgating,
// and completes the lane change with a silky smooth S-curve profile.
func TestBerserkModeEarlyOvertakeInitiation(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	s1 := NewBusStop(301, "AvenueStart", Vec2{X: -1200 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(302, "AvenueEnd", Vec2{X: 1200 * MapGridScale, Y: 1200 * MapGridScale}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "EarlyOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -700 * MapGridScale, Y: 1200*MapGridScale + lw*1.5}
	bus.Heading = 0
	bus.Speed = 260.0 // Fast cruise speed
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow lead vehicle 360px ahead (in the past, fwdMax was only 170-195px, so this was ignored until tailgating)
	leadCar := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X + 360.0, Y: 1200*MapGridScale + lw*1.5},
		Heading: 0,
		Speed:   45.0,
		Length:  48,
		Width:   22,
	}

	// 1. First frame: Berserk mode must detect blocker 360px away and initiate overtake immediately
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate lane change early at 360px distance (expected state %d, got %d)",
			OvertakeChangingLane, driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f, got %.2f", lw, driver.TargetLaneOffset)
	}

	// 2. Measure transition trajectory over time: must be a smooth S-curve and finish swiftly
	offsets := []float64{driver.LaneOffset}
	steps := 0
	for i := 0; i < 30; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.033)
		offsets = append(offsets, driver.LaneOffset)
		steps++
		if driver.OvertakeState == OvertakePassing {
			break
		}
	}

	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Lane change should finish smoothly within ~1.0s, but still in state %d (offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// Verify S-curve smoothstep easing: initial rate of lateral shift must be gentle, not a linear snap
	initialDelta := math.Abs(offsets[1] - offsets[0])
	peakDelta := 0.0
	for i := 1; i < len(offsets); i++ {
		d := math.Abs(offsets[i] - offsets[i-1])
		if d > peakDelta {
			peakDelta = d
		}
	}
	if initialDelta >= peakDelta*0.5 {
		t.Fatalf("Initial transition step (%.3f) is too sharp compared to peak step (%.3f); expected gentle S-curve easing",
			initialDelta, peakDelta)
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

	path := computeRoadPathBetweenStops(w, s1, s2, lw)
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

// TestBerserkModeNoPrematureReturnCollision verifies that when overtaking in Berserk mode:
// 1. The bus never prematurely starts returning into the base lane while the overtaken vehicle
//    is still ahead, alongside, or closely behind the bus (within the safe longitudinal buffer).
// 2. Approaching corner or stop bay does not trigger an unsafe cut-in if base lane is occupied.
// 3. Once safely clear (> 75px buffer behind bus tail), return to base lane can safely proceed.
// 4. During return maneuver, high cruise speed is maintained and any sudden obstruction in base
//    lane causes the bus to abort return and pull back into the passing lane.
func TestBerserkModeNoPrematureReturnCollision(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Eastbound 4-lane expressway on South Ring Avenue (Y = 1200*MapGridScale = 3600)
	roadY := 1200.0 * MapGridScale
	s1 := NewBusStop(0, "AvenueStart", Vec2{X: -2000, Y: roadY}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(1, "AvenueEnd", Vec2{X: 2000, Y: roadY}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "BerserkNoPrematureReturnRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1, // Heading towards s2
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Eastbound base lane (outer lane) is at roadY + lw*1.5
	baseLaneY := roadY + lw*1.5
	// Passing lane (inner lane) is at roadY + lw*0.5 (offset -lw)
	passingLaneY := roadY + lw*0.5

	bus := NewBus(-600, passingLaneY, 0)
	bus.Speed = 200.0

	// Run initial step to sync path and state
	driver.ComputeControls(bus, route, w, nil, nil, lw, 0.016)

	// Simulate that the bus has already pulled into the passing lane (-lw)
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.LaneOffsetProgress = 1.0
	driver.OvertakeState = OvertakePassing
	driver.OvertakeTimer = 2.0 // Has been passing for 2 seconds

	// 1. Overtaken vehicle is still slightly ahead (fwd = +40 px, in base lane)
	carAhead := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X + 40.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carAhead}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is still ahead (fwd=+40px)")
	}

	// 2. Overtaken vehicle is directly alongside the bus center (fwd = 0 px)
	carAlongside := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carAlongside}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is alongside (fwd=0px)")
	}

	// 3. Overtaken vehicle is slightly behind bus center (fwd = -70 px), within unsafe zone (< 75px buffer)
	carCloseBehind := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X - 70.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carCloseBehind}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is close behind (fwd=-70px, buffer too small)")
	}

	// 4. Overtaken vehicle is safely far behind (fwd = -180 px, clearance > 75px buffer)
	carFarBehind := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X - 180.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	throttle, brake, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carFarBehind}, lw, 0.016)
	if driver.OvertakeState != OvertakeReturning || driver.TargetLaneOffset != 0.0 {
		t.Fatalf("Expected bus to safely initiate return once overtaken vehicle is far behind, got state %d target %.1f",
			driver.OvertakeState, driver.TargetLaneOffset)
	}

	// Verify cruise speed is maintained during return (does not brake check vehicle behind)
	if brake > 0.0 {
		t.Fatalf("Bus should not brake during OvertakeReturning; expected brake=0.0, got %.2f", brake)
	}
	if throttle <= 0.0 && bus.Speed < apBerserkCruiseSpeed {
		t.Fatalf("Bus should maintain throttle during OvertakeReturning; got throttle=%.2f", throttle)
	}

	// 5. Test collision avoidance abort during return maneuver
	driver.OvertakeState = OvertakeReturning
	driver.LaneOffset = -lw * 0.5
	driver.TargetLaneOffset = 0.0
	driver.LaneOffsetProgress = 0.5
	bus.Pos.Y = roadY + lw*1.0 // mid-transition lateral position

	// Sudden danger in base lane right beside/ahead of bus
	dangerCar := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X + 20.0, Y: baseLaneY},
		Heading: 0,
		Speed:   50.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{dangerCar}, lw, 0.016)
	if driver.TargetLaneOffset != -lw || driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected bus to abort return and pull back to passing lane upon detecting danger, got state %d target %.1f",
			driver.OvertakeState, driver.TargetLaneOffset)
	}
}

// TestSixLaneStraightIntersectionChoosesClearLane verifies that on a 6-lane road going straight
// through an intersection:
// 1. If outer straight lane has queued cars and inner straight lane is empty, bus selects inner lane.
// 2. Bus maintains its straight lane through the intersection without forcing a merge to the rightmost lane.
// 3. Traffic lights in inner straight lane obey straight-through signals, not left-arrow signals.
func TestSixLaneStraightIntersectionChoosesClearLane(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// North Ring Avenue (Y = -1200*MapGridScale = -3600) is a 6-lane expressway in Metropolis
	roadY := -1200.0 * MapGridScale
	if !w.IsRoad6Lane(roadY, true) {
		t.Fatalf("Expected Y=%.1f to be a 6-lane road", roadY)
	}

	// Both stops on 6-lane road heading West (math.Pi), traversing intersection at (0, roadY)
	s1 := NewBusStop(0, "SixLaneStart", Vec2{X: 1200 * MapGridScale, Y: roadY}, math.Pi, 6, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(1, "SixLaneEnd", Vec2{X: -1200 * MapGridScale, Y: roadY}, math.Pi, 6, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "SixLaneStraightRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1, // Heading towards s2
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Westbound travel lane: for West heading, right-hand traffic is on North side (-Y)
	// Base drivePath is at roadY - lw*2.5 (outer through lane = -3600 - 130 = -3730)
	// Inner through lane is at roadY - lw*1.5 (-3600 - 78 = -3678), which is offset -lw from drivePath
	outerLaneY := roadY - lw*2.5
	innerLaneY := roadY - lw*1.5

	bus := NewBus(600, outerLaneY, math.Pi)
	bus.Speed = apBerserkCruiseSpeed * 0.9

	// Run initial step to sync path and waypoints
	driver.ComputeControls(bus, route, w, nil, nil, lw, 0.016)

	// Scenario 1: Outer through lane has queued stopped cars approaching intersection at X=100
	// Inner through lane is completely open!
	queueCar1 := VehicleObstacle{
		Pos:     Vec2{X: 320.0, Y: outerLaneY},
		Heading: math.Pi,
		Speed:   0.0, // Stopped in queue
		Length:  40.0,
		Width:   18.0,
	}
	queueCar2 := VehicleObstacle{
		Pos:     Vec2{X: 380.0, Y: outerLaneY},
		Heading: math.Pi,
		Speed:   0.0,
		Length:  40.0,
		Width:   18.0,
	}

	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{queueCar1, queueCar2}, lw, 0.016)

	// Verify that bus intelligently selects inner through lane (-lw) instead of getting stuck in outer lane
	if driver.TargetLaneOffset != -lw {
		t.Fatalf("Expected bus to select clear inner through lane (-lw=%.1f), got TargetLaneOffset=%.1f",
			-lw, driver.TargetLaneOffset)
	}

	// Advance lane offset transition to complete merge into inner through lane
	driver.LaneOffset = -lw
	driver.LaneOffsetProgress = 1.0
	bus.Pos.Y = innerLaneY

	// Step again with vehicle now in inner through lane
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{queueCar1, queueCar2}, lw, 0.016)

	// Verify that bus cruises in inner through lane and does NOT force itself back to the rightmost lane!
	if driver.TargetLaneOffset != -lw || driver.OvertakeState != OvertakeNone {
		t.Fatalf("Bus should cruise peacefully in inner through lane (state=%d target=%.1f)",
			driver.OvertakeState, driver.TargetLaneOffset)
	}

	// Scenario 2: Traffic Light System check in inner through lane
	// Create TLS with an intersection at (0, roadY)
	inter := &TrafficIntersection{
		ID:              0,
		Center:          Vec2{X: 0, Y: roadY},
		IsMajor:         true,
		ApproachHasRoad: [4]bool{true, true, true, true},
		ApproachLanes:   [4]int{6, 6, 6, 6},
		ApproachIs4Lane: [4]bool{true, true, true, true},
		Phase:           PhaseEWLeftGreen, // Left arrow is Green, Through is Red!
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	ewStop, _, _, _ := inter.GetApproachDimensions(lw)
	// Position bus approaching stop line in inner straight lane (Y = innerLaneY)
	// Bumper approaching stop line from the East (bumper is at X = ewStop + 24.0, stop line is at ewStop)
	bus.Pos = Vec2{X: ewStop + 70.0, Y: innerLaneY}
	bus.Heading = math.Pi
	bus.Speed = 120.0

	_, brakeLight, _, _, _, _ := driver.ComputeControls(bus, route, w, tls, nil, lw, 0.016)
	if brakeLight <= 0.0 {
		t.Fatalf("Bus going straight in inner through lane must stop for Red through light even when left-turn signal is Green")
	}

	// Now switch Through signal to Green, Left signal to Red
	inter.Phase = PhaseEWGreen // Through is Green, Left arrow is Red!

	throttleLight, _, _, _, _, _ := driver.ComputeControls(bus, route, w, tls, nil, lw, 0.016)
	if throttleLight <= 0.0 {
		t.Fatalf("Bus going straight in inner through lane must proceed when through signal is Green even when left-turn signal is Red")
	}
}

// TestAIBusFleetFrontBackQueueing verifies that multiple AI vehicles spawn strictly queued
// front-and-back behind the player bus at the starting terminal station (起点站), with zero
// lateral/vertical stacking, safe bumper clearance, and orderly departure progression.
func TestAIBusFleetFrontBackQueueing(t *testing.T) {
	themes := []MapTheme{MapThemeMetropolis, MapThemeCoastalBay, MapThemeCustom}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			lw := 52.0
			w := NewWorld(lw, theme)
			startStop := w.Stops[0]
			cosH := math.Cos(startStop.Heading)
			sinH := math.Sin(startStop.Heading)

			for fleetCount := 1; fleetCount <= 4; fleetCount++ {
				playerBus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
				abm := NewAIBusManager()
				abm.BuildFleet(w, fleetCount, lw, playerBus)

				if len(abm.Buses) != fleetCount {
					t.Fatalf("[%s] FleetCount %d: Expected %d buses, got %d", theme.DisplayName(), fleetCount, fleetCount, len(abm.Buses))
				}

				playerOBB := OBB{
					Center:  playerBus.Pos,
					HalfLen: playerBus.Length / 2,
					HalfWid: playerBus.Width / 2,
					Heading: playerBus.Heading,
				}

				// 1. Verify longitudinal queue positioning and zero lateral displacement
				prevBusPos := playerBus.Pos
				for i, ai := range abm.Buses {
					// Check heading alignment
					angleDiff := math.Abs(NormalizeAngle(ai.Bus.Heading - startStop.Heading))
					if angleDiff > 1e-4 {
						t.Fatalf("[%s] AI bus #%d heading (%f) does not match stop heading (%f)",
							theme.DisplayName(), i+1, ai.Bus.Heading, startStop.Heading)
					}

					// Check longitudinal & lateral relative displacement to previous vehicle in line
					delta := ai.Bus.Pos.Sub(prevBusPos)
					longDist := delta.X*cosH + delta.Y*sinH     // Should be negative (behind)
					latDist := math.Abs(-delta.X*sinH + delta.Y*cosH) // Must be 0 (no lateral/vertical stacking)

					if latDist > 1.0 {
						t.Fatalf("[%s] AI bus #%d has lateral offset %.2f px from queue line (should be 0, front-and-back)",
							theme.DisplayName(), i+1, latDist)
					}

					expectedSpacing := 120.0
					if math.Abs(longDist - (-expectedSpacing)) > 1.0 {
						t.Fatalf("[%s] AI bus #%d longitudinal spacing %.2f px != expected -%.2f px",
							theme.DisplayName(), i+1, longDist, expectedSpacing)
					}

					// Verify clear bumper-to-bumper buffer (23px > 20px)
					bumperGap := math.Abs(longDist) - (ai.Bus.Length/2 + playerBus.Length/2)
					if bumperGap < 20.0 {
						t.Fatalf("[%s] AI bus #%d bumper gap %.2f px is too close (< 20px)", theme.DisplayName(), i+1, bumperGap)
					}

					// Verify no OBB collision with player bus
					aiOBB := OBB{
						Center:  ai.Bus.Pos,
						HalfLen: ai.Bus.Length / 2,
						HalfWid: ai.Bus.Width / 2,
						Heading: ai.Bus.Heading,
					}
					collidesWithPlayer, _, depthPlayer := CheckOBBCollision(playerOBB, aiOBB)
					if collidesWithPlayer && depthPlayer > 0 {
						t.Fatalf("[%s] AI bus #%d overlaps with player bus (depth: %.2f px)", theme.DisplayName(), i+1, depthPlayer)
					}

					// Verify no OBB collision with preceding AI bus
					if i > 0 {
						prevAI := abm.Buses[i-1]
						prevAIOBB := OBB{
							Center:  prevAI.Bus.Pos,
							HalfLen: prevAI.Bus.Length / 2,
							HalfWid: prevAI.Bus.Width / 2,
							Heading: prevAI.Bus.Heading,
						}
						collidesPrev, _, depthPrev := CheckOBBCollision(prevAIOBB, aiOBB)
						if collidesPrev && depthPrev > 0 {
							t.Fatalf("[%s] AI bus #%d overlaps with AI bus #%d (depth: %.2f px)", theme.DisplayName(), i+1, i, depthPrev)
						}
					}

					// Verify road boundary clearance
					bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width/2, lw)
					if bRes.Zone == ZoneHardBarrier {
						t.Fatalf("[%s] AI bus #%d at %v intersects barrier (%s)", theme.DisplayName(), i+1, ai.Bus.Pos, bRes.ObstacleName)
					}

					prevBusPos = ai.Bus.Pos
				}

				// 2. Simulation check: while player is parked, AI buses must remain stopped and not deadlock
				tls := NewTrafficLightSystem()
				tls.BuildForWorld(w)
				dt := 0.05
				for frame := 0; frame < 30; frame++ {
					abm.Update(dt, playerBus, w, tls, nil, lw, 2)
					for _, b := range abm.Buses {
						if math.Abs(b.Bus.Speed) > 1.0 {
							t.Fatalf("[%s] AI bus #%d should stay stopped behind player in queue, speed=%f",
								theme.DisplayName(), b.ID, b.Bus.Speed)
						}
						if b.DeadlockTimer > 0 {
							t.Fatalf("[%s] AI bus #%d should not trigger deadlock while queuing behind station, timer=%f",
								theme.DisplayName(), b.ID, b.DeadlockTimer)
						}
					}
				}

				// 3. Dynamic departure progression:
				// When player departs (moves away from Stop 0), AI bus #1 must advance smoothly into Stop 0 bay
				if fleetCount >= 1 {
					bus1 := abm.Buses[0]
					oldDist := bus1.Bus.Pos.Distance(startStop.Pos)
					playerBus.Pos = startStop.Pos.Add(Vec2{X: cosH * 600.0, Y: sinH * 600.0}) // Player departs 600px ahead

					for f := 0; f < 80; f++ {
						abm.Update(dt, playerBus, w, tls, nil, lw, 2)
					}

					newDist := bus1.Bus.Pos.Distance(startStop.Pos)
					if newDist >= oldDist {
						t.Fatalf("[%s] AI bus #1 did not advance into Stop 0 bay when player departed (oldDist: %.1f, newDist: %.1f)",
							theme.DisplayName(), oldDist, newDist)
					}
				}
			}
		})
	}
}// TestBerserkModeOvertakeMaintainsSpeedInOpenSpace verifies that changing lanes in open space
// does not trigger drastic deceleration or heavy braking down to 55 px/s (~10 km/h),
// but instead maintains fast, smooth cruising speed.
func TestBerserkModeOvertakeMaintainsSpeedInOpenSpace(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	s1 := NewBusStop(601, "OpenAvenueStart", Vec2{X: -1200 * MapGridScale, Y: 0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(602, "OpenAvenueEnd", Vec2{X: 1200 * MapGridScale, Y: 0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "OpenOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := NewBus(1, 0, lw)
	bus.Pos = Vec2{X: -600 * MapGridScale, Y: lw * 1.5}
	bus.Heading = 0
	bus.Speed = 250.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow vehicle far ahead (350px away)
	leadCar := VehicleObstacle{
		Pos:     Vec2{X: bus.Pos.X + 350.0, Y: lw * 1.5},
		Heading: 0,
		Speed:   50.0,
		Length:  48,
		Width:   22,
	}

	// 1. Initial compute triggers lane change
	throttle, brake, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Expected OvertakeChangingLane, got %d", driver.OvertakeState)
	}

	// 2. Throughout the lane change transition, verify the bus maintains high speed (> 180 px/s)
	// and does not slam foot brake (brake > 0.35)
	for i := 0; i < 20; i++ {
		throttle, brake, _, _, _, _ = driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.033)
		if brake > 0.35 {
			t.Fatalf("Frame %d: Bus slammed brakes during lane change in open space (brake=%.2f, offset=%.2f, state=%d)",
				i, brake, driver.LaneOffset, driver.OvertakeState)
		}
		if throttle < 0.2 && bus.Speed < 200.0 {
			t.Fatalf("Frame %d: Bus throttled down unexpectedly during lane change (throttle=%.2f, speed=%.2f)",
				i, throttle, bus.Speed)
		}
	}
}

// TestBerserkModeRedLightNoLeftTurnLaneInvasion4Lane verifies that on a 4-lane avenue approaching an intersection,
// when going STRAIGHT and lead cars in the through lane are waiting at a red light,
// the bus NEVER changes lanes into the dedicated left-turn lane.
func TestBerserkModeRedLightNoLeftTurnLaneInvasion4Lane(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// Central Boulevard at Y=0 is a 4-lane avenue with dedicated left-turn lanes at intersections
	inter := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseNSGreen, // EW through light is Red!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	// Setup Eastbound route going straight through intersection (0, 0)
	s1 := NewBusStop(701, "EWStart", Vec2{X: -600 * MapGridScale, Y: lw * 1.5}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(702, "EWEnd", Vec2{X: 600 * MapGridScale, Y: lw * 1.5}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "RedLightStraightRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := NewBus(1, 0, lw)
	// Position bus 280px before stop line in outer through lane (Y = lw*1.5)
	bus.Pos = Vec2{X: -320.0, Y: lw * 1.5}
	bus.Heading = 0
	bus.Speed = 120.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Stopped car ahead in through lane waiting at red light (X = -150.0, speed = 0.0)
	stoppedCar := VehicleObstacle{
		Pos:     Vec2{X: -150.0, Y: lw * 1.5},
		Heading: 0,
		Speed:   0.0,
		Length:  48,
		Width:   22,
	}

	// Run multiple frames approaching the red light
	for frame := 0; frame < 30; frame++ {
		driver.ComputeControls(bus, route, w, tls, []VehicleObstacle{stoppedCar}, lw, 0.033)
		// Must NEVER initiate overtake into left-turn lane (-lw)
		if driver.TargetLaneOffset < -lw*0.5 {
			t.Fatalf("Frame %d: Bus invaded dedicated left-turn lane while intending to go straight at red light! TargetLaneOffset=%.2f",
				frame, driver.TargetLaneOffset)
		}
		if driver.OvertakeState == OvertakeChangingLane && driver.TargetLaneOffset != 0.0 {
			t.Fatalf("Frame %d: Bus initiated overtake at red light into non-straight lane (state=%d, targetOffset=%.2f)",
				frame, driver.OvertakeState, driver.TargetLaneOffset)
		}
	}
}

// TestBerserkModeRedLightStraightLaneSelection6Lane verifies that on a 6-lane road approaching an intersection,
// when going STRAIGHT and approaching a red light / traffic queue, the bus selects whichever through lane
// has fewer queued vehicles (between outer through lane and middle through lane), and NEVER enters the left-turn lane.
func TestBerserkModeRedLightStraightLaneSelection6Lane(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)

	// Metropolis North Ring (Y = -1200*MapGridScale = -3600) is 6-lane expressway
	roadCoord := -1200.0 * MapGridScale

	// Find intersection at (0, roadCoord)
	var inter *TrafficIntersection
	for _, in := range tls.Intersections {
		if math.Abs(in.Center.X) < 10.0 && math.Abs(in.Center.Y-roadCoord) < 10.0 {
			inter = in
			break
		}
	}
	if inter == nil {
		t.Fatalf("Could not find 6-lane intersection at (0, %f)", roadCoord)
	}
	inter.Phase = PhaseNSGreen // EW through light is Red!

	// Eastbound straight route on 6-lane road:
	// Base path is in outer through lane (offset 0.0, Y = roadCoord + lw*2.5)
	// Middle through lane is offset -lw (Y = roadCoord + lw*1.5)
	// Dedicated left-turn lane is offset -2*lw (Y = roadCoord + lw*0.5)
	s1 := NewBusStop(801, "SixLaneStart", Vec2{X: -600 * MapGridScale, Y: roadCoord + lw*2.5}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	s2 := NewBusStop(802, "SixLaneEnd", Vec2{X: 600 * MapGridScale, Y: roadCoord + lw*2.5}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{s1, s2}

	route := &Route{
		Name:         "SixLaneQueueSelectionRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	// Scenario A: Outer through lane has 2 queued cars waiting at red light; middle through lane has 0 cars.
	// Bus should select middle through lane (offset -lw).
	busA := NewBus(1, 0, lw)
	busA.Pos = Vec2{X: -350.0, Y: roadCoord + lw*2.5}
	busA.Heading = 0
	busA.Speed = 160.0
	busA.Gear = GearDrive

	driverA := NewAutopilotDriver()
	driverA.BerserkMode = true
	driverA.State = AutopilotStateCruising

	queuedCarsOuter := []VehicleObstacle{
		{Pos: Vec2{X: -140.0, Y: roadCoord + lw*2.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
		{Pos: Vec2{X: -200.0, Y: roadCoord + lw*2.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
	}

	driverA.ComputeControls(busA, route, w, tls, queuedCarsOuter, lw, 0.016)
	if driverA.TargetLaneOffset != -lw {
		t.Fatalf("Scenario A: Bus should select middle through lane (-%.1f) with fewer queued cars, got %.2f",
			lw, driverA.TargetLaneOffset)
	}

	// Scenario B: Middle through lane has 2 queued cars, outer lane has 0 cars.
	// Bus currently in middle through lane (offset -lw) should switch back to outer through lane (offset 0.0).
	busB := NewBus(2, 0, lw)
	busB.Pos = Vec2{X: -350.0, Y: roadCoord + lw*1.5}
	busB.Heading = 0
	busB.Speed = 160.0
	busB.Gear = GearDrive

	driverB := NewAutopilotDriver()
	driverB.BerserkMode = true
	driverB.State = AutopilotStateCruising
	driverB.LaneOffset = -lw
	driverB.TargetLaneOffset = -lw

	queuedCarsMiddle := []VehicleObstacle{
		{Pos: Vec2{X: -140.0, Y: roadCoord + lw*1.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
		{Pos: Vec2{X: -200.0, Y: roadCoord + lw*1.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
	}

	driverB.ComputeControls(busB, route, w, tls, queuedCarsMiddle, lw, 0.016)
	if driverB.TargetLaneOffset != 0.0 {
		t.Fatalf("Scenario B: Bus should select outer through lane (0.0) with fewer queued cars, got %.2f",
			driverB.TargetLaneOffset)
	}

	// Scenario C: Verify it NEVER enters dedicated left-turn lane (-2*lw)
	if driverA.TargetLaneOffset < -lw*1.5 || driverB.TargetLaneOffset < -lw*1.5 {
		t.Fatalf("Bus must NEVER enter dedicated left-turn lane (-2*lw) while intending to go straight!")
	}
}

// TestLeftTurnLaneChangeDoesNotCrossDoubleYellowIntoOncomingLane verifies that when a bus changes lanes
// into the dedicated left-turn lane (both in normal autopilot and Berserk mode), no part of the bus
// crosses over the center double-yellow divider into the oncoming lane (Bug: 变换车道去左转弯车道时压到对向车道).
func TestLeftTurnLaneChangeDoesNotCrossDoubleYellowIntoOncomingLane(t *testing.T) {
	lw := 52.0
	themes := []MapTheme{MapThemeCoastalBay}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(lw, theme)
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(w)
			// Find a stop leg that executes a left turn
			route := GetStandardRouteForMap(theme, w)
			testedLeg := false
			for legIdx := 0; legIdx < len(route.StopIDs); legIdx++ {
				fromStop := w.Stops[route.StopIDs[legIdx]]
				nextIdx := (legIdx + 1) % len(route.StopIDs)
				toStop := w.Stops[route.StopIDs[nextIdx]]

				legRoute := &Route{
					Name:         "TestLeftTurnLeg",
					StopIDs:      []int{fromStop.ID, toStop.ID},
					CurrentIndex: 1,
				}

				path := computeRoadPathBetweenStops(w, fromStop, toStop, lw)
				// Check if this leg contains a left turn
				hasLeftTurn := false
				for k := 1; k < len(path)-1; k++ {
					a := path[k].Sub(path[k-1])
					b := path[k+1].Sub(path[k])
					turn := NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
					if turn < -0.4 && turn > -2.8 {
						hasLeftTurn = true
						break
					}
				}
				if !hasLeftTurn {
					continue
				}

				bus := NewBus(fromStop.Pos.X, fromStop.Pos.Y, fromStop.Heading)
				driver := NewAutopilotDriver()
				driver.BerserkMode = true
				driver.DwellTotal = 0.1

				dt := 0.016
				maxSteps := 2000
				for step := 0; step < maxSteps; step++ {
					tls.Update(dt)
					throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
						bus, legRoute, w, tls, nil, lw, dt,
					)
					bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

					if isDocked && step > 60 {
						break
					}

					// Verify that when approaching intersections before the corner, the bus never crosses into oncoming traffic
					roadCenter, isEW, isMulti, _ := getCurrentRoadInfo(w, bus.Pos, bus.Heading, lw)
					if isMulti {
						// Double yellow line boundary: center divider is at roadCenter ± 6.0px.
						// Westbound (heading ~ Pi): legal side is North of center (Y < roadCenter).
						// Eastbound (heading ~ 0): legal side is South of center (Y > roadCenter).
						// Southbound (heading ~ Pi/2): legal side is West of center (X < roadCenter).
						// Northbound (heading ~ -Pi/2): legal side is East of center (X > roadCenter).
						cosH := math.Cos(bus.Heading)
						sinH := math.Sin(bus.Heading)

						// Calculate front-left and rear-left corner positions (the side closest to oncoming traffic)
						halfL := bus.Length * 0.5
						halfW := bus.Width * 0.5
						leftPerp := Vec2{X: sinH, Y: -cosH} // unit vector pointing towards the left of vehicle
						flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
						rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

						// Only check when vehicle is on the straight approach road before entering the intersection mouth (dist to intersection > 80px)
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
									if flCorner.Y > roadCenter-2.0 || rlCorner.Y > roadCenter-2.0 {
										t.Fatalf("Theme %s, Step %d: Westbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								} else if math.Cos(bus.Heading) > 0.5 { // Eastbound: legal Y > roadCenter
									if flCorner.Y < roadCenter+2.0 || rlCorner.Y < roadCenter+2.0 {
										t.Fatalf("Theme %s, Step %d: Eastbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								}
							} else {
								if math.Sin(bus.Heading) > 0.5 { // Southbound: legal X < roadCenter
									if flCorner.X > roadCenter-2.0 || rlCorner.X > roadCenter-2.0 {
										t.Fatalf("Theme %s, Step %d: Southbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								} else if math.Sin(bus.Heading) < -0.5 { // Northbound: legal X > roadCenter
									if flCorner.X < roadCenter+2.0 || rlCorner.X < roadCenter+2.0 {
										t.Fatalf("Theme %s, Step %d: Northbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								}
							}
						}
					}
				}

				testedLeg = true
				break
			}
			if !testedLeg {
				t.Fatalf("Expected to test at least one left-turn leg in theme %s", theme.DisplayName())
			}
		})
	}
}

// TestTrafficDensity72And128 verifies the 4th (72 cars) and 5th (128 cars) density options across all 6 maps.
func TestTrafficDensity72And128(t *testing.T) {
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
			lw := 52.0
			w := NewWorld(lw, theme)
			tm := NewTrafficManager(lw)
			tm.BuildForWorld(w, MaxTrafficCars, lw)

			if len(tm.Cars) != 128 {
				t.Fatalf("[%s] Expected 128 total cars generated, got %d", theme.DisplayName(), len(tm.Cars))
			}

			// Verify density levels: 1:12, 2:24, 3:36, 4:72, 5:128
			expectedCounts := map[int]int{
				1: 12,
				2: 24,
				3: 36,
				4: 72,
				5: 128,
			}
			for lvl, exp := range expectedCounts {
				active := tm.GetActiveCars(lvl)
				if len(active) != exp {
					t.Fatalf("[%s] Density level %d: expected %d active cars, got %d",
						theme.DisplayName(), lvl, exp, len(active))
				}
			}

			// Verify simulation step with 128 cars runs smoothly without NaN or panic
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(w)
			playerBus := NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, w.Stops[0].Heading)

			dt := 0.05
			for frame := 0; frame < 30; frame++ {
				tls.Update(dt)
				tm.Update(dt, playerBus, tls, 5, lw, w)
				for idx, c := range tm.GetActiveCars(5) {
					if math.IsNaN(c.Pos.X) || math.IsNaN(c.Pos.Y) || math.IsNaN(c.Speed) {
						t.Fatalf("[%s] Car #%d has NaN values at frame %d", theme.DisplayName(), idx, frame)
					}
				}
			}
		})
	}
}

// TestAIBusFleet8BusesAllThemes verifies configuring up to 8 AI buses across all 6 maps.
func TestAIBusFleet8BusesAllThemes(t *testing.T) {
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
			lw := 52.0
			w := NewWorld(lw, theme)
			tls := NewTrafficLightSystem()
			tls.BuildForWorld(w)

			startStop := w.Stops[0]
			playerBus := NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			abm := NewAIBusManager()
			abm.BuildFleet(w, 8, lw, playerBus)

			if len(abm.Buses) != 8 {
				t.Fatalf("[%s] Expected 8 AI buses in fleet, got %d", theme.DisplayName(), len(abm.Buses))
			}

			// Verify each bus is on ZoneRoad without barrier collisions
			for i, ai := range abm.Buses {
				bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width/2, lw)
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] AI bus #%d at %v intersects hard barrier (%s)",
						theme.DisplayName(), i+1, ai.Bus.Pos, bRes.ObstacleName)
				}
			}

			// Verify simulation step with 8 buses runs smoothly without NaN or panic
			dt := 0.05
			for frame := 0; frame < 30; frame++ {
				tls.Update(dt)
				abm.Update(dt, playerBus, w, tls, nil, lw, 5)
				for _, b := range abm.Buses {
					if math.IsNaN(b.Bus.Pos.X) || math.IsNaN(b.Bus.Pos.Y) || math.IsNaN(b.Bus.Speed) {
						t.Fatalf("[%s] AI Bus #%d has NaN at frame %d", theme.DisplayName(), b.ID, frame)
					}
				}
			}
		})
	}
}

// TestAutopilotRightTurnInto2LaneRoad verifies that when a bus makes a 90-degree right turn
// into a 2-lane road (1 lane in each direction, 52px wide), the vehicle's turning radius is
// sufficiently tight such that neither the bus center nor its front-left overhang crosses the
// road centerline into the oncoming traffic lane.
func TestAutopilotRightTurnInto2LaneRoad(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// In Metropolis:
	// Eastbound on 北二路 (Y = -1800.0, lane center Y = -1774.0)
	// Right turn onto 东环路 (X = 3900.0, 2-lane road, Southbound lane center X = 3874.0, Oncoming Northbound X > 3900.0).
	// Road centerline is X = 3900.0.
	legRoute := &Route{
		Name:         "TestRightTurnRoute",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	bus := NewBus(3600.0, -1774.0, 0.0) // 300px before intersection at (3900, -1800)
	bus.Gear = GearDrive
	bus.TransmissionMode = TransmissionAuto
	bus.Speed = 50.0

	dt := 0.016
	maxFLX := -1e9
	for step := 0; step < 600; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, legRoute, w, nil, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Calculate front-left corner coordinates of the bus envelope
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := Vec2{X: sinH, Y: -cosH}
		flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		if bus.Pos.X > 3800.0 {
			if flCorner.X > maxFLX {
				maxFLX = flCorner.X
			}
		}

		// Once bus has completed right turn and traveled downstream along X = 3874
		if bus.Pos.Y > -1400.0 {
			break
		}
	}

	roadCenterlineX := 3900.0
	if maxFLX > roadCenterlineX {
		t.Fatalf("Bus right turn intruded into oncoming traffic lane! Max front-left X=%.2f, centerline=%.2f (intrusion=%.2fpx)",
			maxFLX, roadCenterlineX, maxFLX-roadCenterlineX)
	}

	// Verify comfortable safety margin inside its own lane
	safetyMargin := roadCenterlineX - maxFLX
	if safetyMargin < 2.0 {
		t.Errorf("Safety margin to centerline too small: %.2fpx", safetyMargin)
	}
}

// TestBerserkModeOvertakeAccelerationAndSnappyLaneChange verifies that in Berserk mode,
// lane changes to pass slow lead vehicles do not suffer from sluggish crawling or premature deceleration.
// The lateral transition duration must be brisk (0.35s - 0.55s) and the target speed must remain high.
func TestBerserkModeOvertakeAccelerationAndSnappyLaneChange(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	legRoute := &Route{
		Name:         "OvertakeAccTest",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Bus driving along straight 6-lane avenue at 200 px/s in outer lane (Y = 130.0)
	bus := NewBus(-1000.0, 130.0, 0.0)
	bus.Gear = GearDrive
	bus.TransmissionMode = TransmissionAuto
	bus.Speed = 200.0

	// Lead car ahead driving slowly at 25 px/s
	leadCar := VehicleObstacle{
		Pos:     Vec2{X: -760.0, Y: 130.0},
		Heading: 0.0,
		Speed:   25.0,
		Length:  48.0,
		Width:   22.0,
	}
	obstacles := []VehicleObstacle{leadCar}

	dt := 0.016
	// Compute controls on initial step: driver detects lead car and triggers lane change
	thr, brk, _, _, _, _ := driver.ComputeControls(bus, legRoute, w, nil, obstacles, lw, dt)

	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Expected OvertakeState to be OvertakeChangingLane, got %d", driver.OvertakeState)
	}
	if driver.TargetLaneOffset != -lw {
		t.Fatalf("Expected TargetLaneOffset to be -%.1f (inner through lane), got %.1f", lw, driver.TargetLaneOffset)
	}

	// Lane change duration must be fast and brisk (between 0.35s and 0.55s)
	if driver.LaneOffsetDuration < 0.30 || driver.LaneOffsetDuration > 0.60 {
		t.Fatalf("LaneOffsetDuration in Berserk mode should be 0.35s-0.55s, got %.2fs", driver.LaneOffsetDuration)
	}

	// Throttle must remain active without unnecessary panic braking
	if brk > 0.0 || thr < 0.3 {
		t.Fatalf("Berserk mode initiated lane change with unnecessary braking! thr=%.2f, brk=%.2f", thr, brk)
	}

	// Step forward until lane offset transition is completed
	completedSteps := 0
	for step := 1; step < 60; step++ {
		thrStep, brkStep, strStep, hbStep, tdStep, _ := driver.ComputeControls(bus, legRoute, w, nil, obstacles, lw, dt)
		bus.Update(dt, thrStep, brkStep, strStep, hbStep, tdStep)

		if math.Abs(driver.LaneOffset-(-lw)) < 0.5 {
			completedSteps = step
			break
		}
	}

	if completedSteps == 0 {
		t.Fatalf("Lane change transition failed to complete within 60 steps (1s): current offset=%.2f", driver.LaneOffset)
	}

	transitionTime := float64(completedSteps) * dt
	if transitionTime > 0.60 {
		t.Fatalf("Lane change transition took too long: %.2fs > 0.60s", transitionTime)
	}

	// Verify bus maintained high momentum without dropping below 120 px/s
	if bus.Speed < 120.0 {
		t.Fatalf("Bus crawled down to %.1f px/s during lane change maneuver (expected >= 120 px/s)", bus.Speed)
	}
}

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

// TestOpposingLeftTurnsSimultaneousClearance verifies that when two opposing vehicles simultaneously
// make a left turn at the same intersection (one East->North, one West->South), the compact turning arc
// guarantees that they pass passenger-side to passenger-side with ample safety clearance (>45px centerline,
// >13px body clearance), with zero collisions and zero path crossing.
func TestOpposingLeftTurnsSimultaneousClearance(t *testing.T) {
	lw := 52.0
	w := &World{}
	w.Roads = []RoadSegment{
		{P1: Vec2{X: -1500, Y: 0}, P2: Vec2{X: 1500, Y: 0}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true},
		{P1: Vec2{X: 0, Y: -1500}, P2: Vec2{X: 0, Y: 1500}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true},
	}
	w.computeIntersections()

	// Intersection at (0, 0)
	inter := &TrafficIntersection{
		ID:                99,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseEWLeftGreen, // Left GREEN
		GreenDuration:     30.0,
		YellowDuration:    3.0,
		LeftGreenDuration: 30.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	// Stop 1: Eastbound on road Y=0 approaching (0,0)
	sEastIn := NewBusStop(0, "EastboundIn", Vec2{X: -600, Y: 0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sEastIn)
	sNorthOut := NewBusStop(1, "NorthboundOut", Vec2{X: 0, Y: -600}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sNorthOut)
	path1 := computeRoadPathBetweenStops(w, sEastIn, sNorthOut, lw)

	// Stop 2: Westbound on road Y=0 approaching (0,0)
	sWestIn := NewBusStop(2, "WestboundIn", Vec2{X: 600, Y: 0}, math.Pi, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sWestIn)
	sSouthOut := NewBusStop(3, "SouthboundOut", Vec2{X: 0, Y: 600}, math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sSouthOut)
	path2 := computeRoadPathBetweenStops(w, sWestIn, sSouthOut, lw)


	bus1 := NewBus(-320.0, 78.0, 0.0)
	bus1.Speed = 120.0
	route1 := &Route{StopIDs: []int{sEastIn.ID, sNorthOut.ID}, CurrentIndex: 1}
	driver1 := NewAutopilotDriver()
	driver1.BerserkMode = true
	driver1.State = AutopilotStateCruising
	driver1.drivePath = path1
	driver1.cumLen = make([]float64, len(path1))
	for i := 1; i < len(path1); i++ {
		driver1.cumLen[i] = driver1.cumLen[i-1] + path1[i].Distance(path1[i-1])
	}
	driver1.pathWorld = w
	driver1.pathLane = lw
	driver1.pathFromID = sEastIn.ID
	driver1.pathToID = sNorthOut.ID

	bus2 := NewBus(320.0, -78.0, math.Pi)
	bus2.Speed = 120.0
	route2 := &Route{StopIDs: []int{sWestIn.ID, sSouthOut.ID}, CurrentIndex: 1}
	driver2 := NewAutopilotDriver()
	driver2.BerserkMode = true
	driver2.State = AutopilotStateCruising
	driver2.drivePath = path2
	driver2.cumLen = make([]float64, len(path2))
	for i := 1; i < len(path2); i++ {
		driver2.cumLen[i] = driver2.cumLen[i-1] + path2[i].Distance(path2[i-1])
	}
	driver2.pathWorld = w
	driver2.pathLane = lw
	driver2.pathFromID = sWestIn.ID
	driver2.pathToID = sSouthOut.ID

	dt := 0.016
	minCenterDist := math.MaxFloat64
	minBodyClearance := math.MaxFloat64
	bothInsideIntersectionCount := 0
	bus1CompletedLeftTurn := false
	bus2CompletedLeftTurn := false

	// Run synchronized simulation for 600 frames (~10 seconds)
	for step := 0; step < 600; step++ {
		// Obstacles for bus 1 includes bus 2
		obs1 := []VehicleObstacle{{
			Pos:     bus2.Pos,
			Heading: bus2.Heading,
			Speed:   bus2.Speed,
			Length:  bus2.Length,
			Width:   bus2.Width,
		}}
		// Obstacles for bus 2 includes bus 1
		obs2 := []VehicleObstacle{{
			Pos:     bus1.Pos,
			Heading: bus1.Heading,
			Speed:   bus1.Speed,
			Length:  bus1.Length,
			Width:   bus1.Width,
		}}

		th1, br1, st1, hb1, tg1, _ := driver1.ComputeControls(bus1, route1, w, tls, obs1, lw, dt)
		th2, br2, st2, hb2, tg2, _ := driver2.ComputeControls(bus2, route2, w, tls, obs2, lw, dt)

		bus1.Update(dt, th1, br1, st1, hb1, tg1)
		bus2.Update(dt, th2, br2, st2, hb2, tg2)

		d := bus1.Pos.Distance(bus2.Pos)
		bodyClearance := d - (bus1.Width/2.0 + bus2.Width/2.0)

		// Check when both vehicles are within the intersection core zone (< 150px from center)
		d1 := bus1.Pos.Distance(inter.Center)
		d2 := bus2.Pos.Distance(inter.Center)

		if d1 < 120.0 && d2 < 120.0 {
			bothInsideIntersectionCount++
			if d < minCenterDist {
				minCenterDist = d
			}
			if bodyClearance < minBodyClearance {
				minBodyClearance = bodyClearance
			}
		}

		if bus1.Pos.Y < -50.0 && math.Sin(bus1.Heading) < -0.8 {
			bus1CompletedLeftTurn = true
		}
		if bus2.Pos.Y > 50.0 && math.Sin(bus2.Heading) > 0.8 {
			bus2CompletedLeftTurn = true
		}
	}

	if bothInsideIntersectionCount < 10 {
		t.Fatalf("Expected both buses to pass each other inside intersection core, but overlapped frames=%d",
			bothInsideIntersectionCount)
	}

	// Verify that minimum centerline distance is at least 45.0px (cleanly separated arcs)
	if minCenterDist < 45.0 {
		t.Fatalf("Opposing left-turn arcs too close: min centerline dist=%.1f < 45.0px (body clearance=%.1f)",
			minCenterDist, minBodyClearance)
	}
	if minBodyClearance < 9.0 {
		t.Fatalf("Opposing left-turn body clearance too small: %.1f < 9.0px", minBodyClearance)
	}

	// Verify both completed their left turns into target outgoing directions
	if !bus1CompletedLeftTurn {
		t.Fatalf("Bus 1 failed to complete left turn to North (heading never faced North)")
	}
	if !bus2CompletedLeftTurn {
		t.Fatalf("Bus 2 failed to complete left turn to South (heading never faced South)")
	}
}

// TestBerserkModeLargeIntersectionLeftTurnSilkySmooth verifies that at large 6-lane intersections,
// left turns in Berserk mode are executed with silky smoothness:
// 1. No violent throttle/brake hunting or sudden severe brake jerking.
// 2. High cornering speed momentum maintained (> 75 px/s).
// 3. Continuous smooth heading progression without angular kinks.
// 4. Zero traffic violations or overshooting double yellow lines.
func TestBerserkModeLargeIntersectionLeftTurnSilkySmooth(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	// s1: On 6-lane North Ring Expressway (Y = -3600), Westbound
	s1 := NewBusStop(len(w.Stops), "6LaneStart", Vec2{X: 600 * MapGridScale, Y: -1200 * MapGridScale}, math.Pi, 6, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, s1)
	// s2: Turning left onto North-South Avenue (X = 0), Southbound towards (0, 0)
	s2 := NewBusStop(len(w.Stops), "4LaneSouth", Vec2{X: 0, Y: -400 * MapGridScale}, math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, s2)

	path := computeRoadPathBetweenStops(w, s1, s2, lw)

	interCenter := Vec2{X: 0, Y: -1200 * MapGridScale}
	inter := &TrafficIntersection{
		ID:                95,
		Center:            interCenter,
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{6, 6, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseEWLeftGreen, // Left green light for approach
		GreenDuration:     40.0,
		YellowDuration:    3.0,
		LeftGreenDuration: 40.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	bus := NewBus(s1.Pos.X, s1.Pos.Y, s1.Heading)
	bus.Speed = 280.0 // Approaching at high speed in Berserk mode
	route := &Route{StopIDs: []int{s1.ID, s2.ID}, CurrentIndex: 1}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.drivePath = path
	driver.cumLen = make([]float64, len(path))
	for i := 1; i < len(path); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + path[i].Distance(path[i-1])
	}
	driver.pathWorld = w
	driver.pathLane = lw
	driver.pathFromID = s1.ID
	driver.pathToID = s2.ID

	dt := 0.016
	minTurnSpeed := math.MaxFloat64
	maxTurnSpeed := 0.0
	prevHeading := bus.Heading
	maxAngularRate := 0.0
	hardBrakeFramesDuringTurn := 0
	inTurnZoneCount := 0

	for step := 0; step < 700; step++ {
		th, br, st, hb, tg, isDocked := driver.ComputeControls(bus, route, w, tls, nil, lw, dt)
		bus.Update(dt, th, br, st, hb, tg)

		if isDocked && step > 60 {
			break
		}

		distToInter := bus.Pos.Distance(interCenter)
		if distToInter < 160.0 {
			inTurnZoneCount++
			if bus.Speed < minTurnSpeed {
				minTurnSpeed = bus.Speed
			}
			if bus.Speed > maxTurnSpeed {
				maxTurnSpeed = bus.Speed
			}
			// Measure angular steering rate in rad/s
			angDelta := math.Abs(NormalizeAngle(bus.Heading - prevHeading))
			angRate := angDelta / dt
			if angRate > maxAngularRate {
				maxAngularRate = angRate
			}
			// Check if emergency full braking (brake > 0.8) occurs while executing corner
			if br > 0.8 && bus.Speed > 50.0 {
				hardBrakeFramesDuringTurn++
			}
		}
		prevHeading = bus.Heading

		// Obey traffic signals
		if violated, notice := tls.CheckPlayerViolation(bus, lw); violated {
			t.Fatalf("Step %d: Left turn traffic violation: %s", step, notice)
		}
	}

	if inTurnZoneCount < 20 {
		t.Fatalf("Expected at least 20 steps inside turn zone, got %d", inTurnZoneCount)
	}

	// 1. Verify silky smooth cornering: minimum speed during turn should stay high (> 75 px/s)
	if minTurnSpeed < 75.0 {
		t.Fatalf("Berserk mode left turn slowed down excessively: min speed=%.1f px/s (expected >= 75.0 px/s)",
			minTurnSpeed)
	}

	// 2. Verify no violent hard brake stomping during the turn
	if hardBrakeFramesDuringTurn > 5 {
		t.Fatalf("Berserk mode left turn experienced %d frames of hard brake stomping (>0.8)",
			hardBrakeFramesDuringTurn)
	}

	// 3. Verify angular rate is smooth (no sudden snaps > 2.8 rad/s)
	if maxAngularRate > 2.8 {
		t.Fatalf("Berserk mode left turn steering too abrupt: max angular rate=%.2f rad/s (expected <= 2.8)",
			maxAngularRate)
	}

	// 4. Verify turn completed successfully into Southbound direction
	sinH := math.Sin(bus.Heading)
	if sinH < 0.70 {
		t.Fatalf("Failed to complete left turn onto Southbound avenue: heading=%.2f", bus.Heading)
	}
}
// TestBerserkModeConfigurableMaxSpeedsTurningAndLaneChange thoroughly tests the configurable
// Berserk mode top speeds (60, 80, 100, 120 km/h) and verifies:
// 1. Settings options, bounds, cycle logic, and cruise speed mapping;
// 2. Straightaway acceleration up to top speed with dynamic powertrain scaling;
// 3. Anticipated cornering deceleration for 90-degree turns without overshooting or instability;
// 4. Smooth overtaking and lane-change initiation and completion across all speeds.
func TestBerserkModeConfigurableMaxSpeedsTurningAndLaneChange(t *testing.T) {
	lw := 52.0

	// 1. Test Settings Configuration & Cycling
	s := NewSettings()
	if s.AutopilotBerserkMaxSpeed != 80 {
		t.Fatalf("Default Berserk max speed should be 80 km/h, got %d", s.AutopilotBerserkMaxSpeed)
	}
	if math.Abs(s.GetBerserkCruiseSpeed()-445.0) > 1.0 {
		t.Fatalf("Default Berserk cruise speed should be ~445.0 px/s, got %.2f", s.GetBerserkCruiseSpeed())
	}

	testSpeeds := []struct {
		kmh       int
		cruiseSpd float64
	}{
		{60, 333.0},
		{80, 445.0},
		{100, 555.0},
		{120, 667.0},
	}

	for _, ts := range testSpeeds {
		s.SetBerserkMaxSpeed(ts.kmh)
		if s.AutopilotBerserkMaxSpeed != ts.kmh {
			t.Fatalf("Expected Berserk speed %d, got %d", ts.kmh, s.AutopilotBerserkMaxSpeed)
		}
		if math.Abs(s.GetBerserkCruiseSpeed()-ts.cruiseSpd) > 1.0 {
			t.Fatalf("Expected Berserk cruise speed %.1f for %d km/h, got %.2f",
				ts.cruiseSpd, ts.kmh, s.GetBerserkCruiseSpeed())
		}
	}

	// Verify cycle sequence: 60 -> 80 -> 100 -> 120 -> 60
	s.SetBerserkMaxSpeed(60)
	if next := s.CycleBerserkMaxSpeed(); next != 80 {
		t.Fatalf("Expected cycle from 60 to 80, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 100 {
		t.Fatalf("Expected cycle from 80 to 100, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 120 {
		t.Fatalf("Expected cycle from 100 to 120, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 60 {
		t.Fatalf("Expected cycle from 120 to 60, got %d", next)
	}

	// 2. Subtests for Each Configured Speed
	for _, ts := range testSpeeds {
		t.Run(fmt.Sprintf("Speed_%d_kmh", ts.kmh), func(t *testing.T) {
			kmh := ts.kmh
			cruiseSpd := ts.cruiseSpd

			// -------------------------------------------------------------
			// A. Straightaway Top Speed & Powertrain Scaling
			// -------------------------------------------------------------
			wA := NewWorld(lw, MapThemeMetropolis)
			wA.Stops[1].Pos = Vec2{X: 3000, Y: 1200}
			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.pathWorld = wA
			driver.pathFromID = 0
			driver.pathToID = 1
			driver.pathLane = lw

			driver.drivePath = []Vec2{
				{X: -2000, Y: 1200},
				{X: 2000, Y: 1200},
			}
			driver.cumLen = []float64{0, 4000.0}
			driver.WaypointIdx = 0

			bus := NewBus(-1000, 1200, 0)
			bus.Speed = cruiseSpd * 0.6
			// Dynamic powertrain expansion in game.go
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			route := NewRoute()
			route.StopIDs = []int{0, 1}
			route.CurrentIndex = 1

			dt := 0.016
			// Initial target speed check
			driver.ComputeControls(bus, route, wA, nil, nil, lw, dt)
			if math.Abs(driver.LastTargetSpeed-cruiseSpd) > 2.0 {
				t.Fatalf("[%d km/h] Target speed on straightaway should be %.1f px/s, got %.2f",
					kmh, cruiseSpd, driver.LastTargetSpeed)
			}

			// Simulate forward acceleration
			maxSpeedReached := 0.0
			for step := 0; step < 200; step++ {
				th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, wA, nil, nil, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)
				if bus.Speed > maxSpeedReached {
					maxSpeedReached = bus.Speed
				}
			}
			if maxSpeedReached < cruiseSpd*0.95 {
				t.Fatalf("[%d km/h] Bus failed to accelerate to top speed: max speed=%.1f px/s (target=%.1f)",
					kmh, maxSpeedReached, cruiseSpd)
			}

			// -------------------------------------------------------------
			// B. Impact on Cornering: Deceleration & Smooth Turning
			// -------------------------------------------------------------
			// Build a path with a long straight leading into a 90-degree right turn
			cornerX := 0.0
			cornerY := 1200.0
			wB := NewWorld(lw, MapThemeMetropolis)
			wB.Stops[1].Pos = Vec2{X: cornerX, Y: cornerY + 1000}
			driverCorner := NewAutopilotDriver()
			driverCorner.BerserkMode = true
			driverCorner.BerserkMaxSpeed = cruiseSpd
			driverCorner.pathWorld = wB
			driverCorner.pathFromID = 0
			driverCorner.pathToID = 1
			driverCorner.pathLane = lw

			driverCorner.drivePath = []Vec2{
				{X: -1500, Y: cornerY},
				{X: cornerX, Y: cornerY},
				{X: cornerX, Y: cornerY + 600}, // 90-degree right turn at (0, 1200)
			}
			driverCorner.cumLen = []float64{0, 1500.0, 2100.0}
			driverCorner.WaypointIdx = 0

			busCorner := NewBus(-1200, cornerY, 0)
			busCorner.Speed = cruiseSpd
			busCorner.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// 1. Verify cornerSpeedLimit deceleration profile
			// Far away (> 1400px), limit should be Inf or >= cruiseSpd
			farLimit := driverCorner.cornerSpeedLimit(100.0, cruiseSpd)
			if farLimit < cruiseSpd {
				t.Fatalf("[%d km/h] Far corner speed limit should not brake prematurely: got %.2f (cruise=%.2f)",
					kmh, farLimit, cruiseSpd)
			}

			// 2. Drive towards the corner and verify it reaches safe cornering speed <= 95 px/s
			reachedSafeCornerSpeed := false
			enteredCorner := false
			completedTurn := false

			for step := 0; step < 500; step++ {
				th, br, st, hb, tg, _ := driverCorner.ComputeControls(busCorner, route, wB, nil, nil, lw, dt)
				busCorner.Update(dt, th, br, st, hb, tg)

				// Distance along X to corner apex
				distToApex := cornerX - busCorner.Pos.X
				if distToApex <= 30.0 && distToApex >= -30.0 {
					enteredCorner = true
					if busCorner.Speed <= 95.0 {
						reachedSafeCornerSpeed = true
					}
				}

				// Check if heading turned towards South (+Y)
				if math.Sin(busCorner.Heading) > 0.85 {
					completedTurn = true
					break
				}
			}

			if !enteredCorner {
				t.Fatalf("[%d km/h] Bus never reached the corner apex", kmh)
			}
			if !reachedSafeCornerSpeed {
				t.Fatalf("[%d km/h] Bus did not slow down sufficiently before entering corner: speed=%.1f px/s (expected <= 95.0)",
					kmh, busCorner.Speed)
			}
			if !completedTurn {
				t.Fatalf("[%d km/h] Bus failed to smoothly complete 90-degree turn: heading=%.2f",
					kmh, busCorner.Heading)
			}

			// -------------------------------------------------------------
			// C. Impact on Lane Changing & Overtaking
			// -------------------------------------------------------------
			yCenter := 1200.0 * MapGridScale
			sStart := NewBusStop(201, "AvenueStart", Vec2{X: -1500 * MapGridScale, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			sEnd := NewBusStop(202, "AvenueEnd", Vec2{X: 1500 * MapGridScale, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			wC := NewWorld(lw, MapThemeMetropolis)
			wC.Stops = []*BusStop{sStart, sEnd}

			routeC := &Route{
				Name:         "OvertakeTestRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driverOvertake := NewAutopilotDriver()
			driverOvertake.BerserkMode = true
			driverOvertake.BerserkMaxSpeed = cruiseSpd
			driverOvertake.State = AutopilotStateCruising

			busOvertake := NewBus(1, 0, lw)
			busOvertake.Pos = Vec2{X: -600 * MapGridScale, Y: yCenter + lw*1.5}
			busOvertake.Heading = 0
			busOvertake.Speed = cruiseSpd * 0.85
			busOvertake.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// Lead blocker moving at 50% cruise speed, positioned ahead within Berserk detection corridor
			blockerSpd := cruiseSpd * 0.50
			leadDist := math.Max(320.0, cruiseSpd*0.75)
			blocker := VehicleObstacle{
				Pos:     Vec2{X: busOvertake.Pos.X + leadDist, Y: yCenter + lw*1.5},
				Heading: 0,
				Speed:   blockerSpd,
				Length:  48,
				Width:   22,
			}

			// Initial control compute: should detect blocker and initiate lane change
			driverOvertake.ComputeControls(busOvertake, routeC, wC, nil, []VehicleObstacle{blocker}, lw, dt)
			if driverOvertake.OvertakeState != OvertakeChangingLane {
				t.Fatalf("[%d km/h] Expected OvertakeChangingLane for slow blocker (spd=%.1f), got state %d",
					kmh, blockerSpd, driverOvertake.OvertakeState)
			}
			if math.Abs(driverOvertake.TargetLaneOffset - (-lw)) > 1.0 {
				t.Fatalf("[%d km/h] Expected target lane offset -%.1f (inner passing lane), got %.2f",
					kmh, lw, driverOvertake.TargetLaneOffset)
			}

			// Simulate full overtaking maneuver: change lane -> pass -> return
			statePassingReached := false
			stateReturningReached := false
			maneuverCompleted := false
			collided := false

			for step := 0; step < 600; step++ {
				// Blocker moves forward at its steady speed
				blocker.Pos.X += blocker.Speed * dt

				th, br, st, hb, tg, _ := driverOvertake.ComputeControls(
					busOvertake, routeC, wC, nil, []VehicleObstacle{blocker}, lw, dt,
				)
				busOvertake.Update(dt, th, br, st, hb, tg)

				// Collision check
				distToBlocker := busOvertake.Pos.Distance(blocker.Pos)
				if distToBlocker < (busOvertake.Length/2 + blocker.Length/2 - 5.0) {
					// Check bounding box overlap
					if math.Abs(busOvertake.Pos.Y-blocker.Pos.Y) < (busOvertake.Width/2 + blocker.Width/2) {
						collided = true
						t.Fatalf("[%d km/h] Step %d: Bus collided with lead vehicle! dist=%.1f",
							kmh, step, distToBlocker)
					}
				}

				if driverOvertake.OvertakeState == OvertakePassing {
					statePassingReached = true
				}
				if driverOvertake.OvertakeState == OvertakeReturning {
					stateReturningReached = true
				}
				if stateReturningReached && driverOvertake.OvertakeState == OvertakeNone && math.Abs(driverOvertake.LaneOffset) < 1.0 {
					maneuverCompleted = true
					break
				}
			}

			if collided {
				t.Fatalf("[%d km/h] Overtaking maneuver resulted in collision", kmh)
			}
			if !statePassingReached {
				t.Fatalf("[%d km/h] Overtaking maneuver never reached OvertakePassing (current state=%d, offset=%.2f)",
					kmh, driverOvertake.OvertakeState, driverOvertake.LaneOffset)
			}
			if !stateReturningReached {
				t.Fatalf("[%d km/h] Overtaking maneuver never reached OvertakeReturning", kmh)
			}
			if !maneuverCompleted {
				t.Fatalf("[%d km/h] Overtaking maneuver did not return to base lane cleanly: state=%d, offset=%.2f",
					kmh, driverOvertake.OvertakeState, driverOvertake.LaneOffset)
			}
		})
	}
}

// TestOvertakingLookaheadAndNoCollisionWithSlowBlocker explicitly tests the dynamic advance lookahead (提前量)
// and collision-free lane changing when approaching slow or stopped lead obstacles.
// It verifies that:
// 1. Blocker vehicles are detected well in advance (提前量) based on relative closing speed.
// 2. The bus initiates lane change in advance, clearing the blocker without colliding.
// 3. In tight / close-quarters scenarios, the bus enforces longitudinal obstacle braking while
//    lateral clearance is not yet achieved (latDist < minClearLat), eliminating collisions before fully clearing.
func TestOvertakingLookaheadAndNoCollisionWithSlowBlocker(t *testing.T) {
	lw := 47.5
	dt := 1.0 / 60.0

	// Test across all 4 Berserk speeds: 60, 80, 100, 120 km/h
	for _, kmh := range []int{60, 80, 100, 120} {
		t.Run(fmt.Sprintf("%d_kmh_Lookahead_Collision_Test", kmh), func(t *testing.T) {
			cruiseSpd := float64(kmh) * 5.0
			yCenter := 1200.0 * MapGridScale

			sStart := NewBusStop(301, "StopStart", Vec2{X: -2000, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			sEnd := NewBusStop(302, "StopEnd", Vec2{X: 2000, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			w := NewWorld(lw, MapThemeMetropolis)
			w.Stops = []*BusStop{sStart, sEnd}

			route := &Route{
				Name:         "OvertakeLookaheadRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.State = AutopilotStateCruising
			driver.drivePath = []Vec2{
				{X: -2000, Y: yCenter + lw*1.5},
				{X: 2000, Y: yCenter + lw*1.5},
			}
			driver.cumLen = []float64{0, 4000.0}
			driver.pathWorld = w
			driver.pathFromID = sStart.ID
			driver.pathToID = sEnd.ID
			driver.pathLane = lw
			driver.pathSynced = true
			driver.WaypointIdx = 0

			bus := NewBus(1, 0, lw)
			bus.Pos = Vec2{X: -1000, Y: yCenter + lw*1.5}
			bus.Heading = 0
			bus.Speed = cruiseSpd
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// Lead blocker moving at 35 px/s (slow vehicle)
			blockerSpd := 35.0
			closingSpeed := cruiseSpd - blockerSpd
			// Blocker positioned ahead: dynamic lookahead should detect it
			blockerDist := closingSpeed*1.8 + 120.0
			blocker := VehicleObstacle{
				Pos:     Vec2{X: bus.Pos.X + blockerDist, Y: yCenter + lw*1.5},
				Heading: 0,
				Speed:   blockerSpd,
				Length:  48,
				Width:   22,
			}

			// Step 1: Initial check must detect blocker and initiate lane change
			driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
			if driver.OvertakeState != OvertakeChangingLane {
				t.Fatalf("[%d km/h] Expected OvertakeChangingLane for slow blocker (dist=%.1f, closingSpd=%.1f), got %d",
					kmh, blockerDist, closingSpeed, driver.OvertakeState)
			}

			// Step 2: Step forward simulation until lane change completes or passes
			minLongitudinalGap := 9999.0
			collided := false
			for step := 0; step < 500; step++ {
				blocker.Pos.X += blocker.Speed * dt

				th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)

				dx := blocker.Pos.X - bus.Pos.X
				dy := math.Abs(bus.Pos.Y - blocker.Pos.Y)
				if dx > 0 && dx < minLongitudinalGap {
					minLongitudinalGap = dx
				}

				// Check physical box overlap collision:
				contactDistX := bus.Length/2 + blocker.Length/2 - 2.0
				contactDistY := bus.Width/2 + blocker.Width/2 - 2.0
				if math.Abs(dx) < contactDistX && dy < contactDistY {
					collided = true
					t.Fatalf("[%d km/h] Step %d: Bus collided with lead vehicle! dx=%.1f, dy=%.1f",
						kmh, step, dx, dy)
				}

				if driver.OvertakeState == OvertakePassing && bus.Pos.X > blocker.Pos.X+80.0 {
					break
				}
			}

			if collided {
				t.Fatalf("[%d km/h] Lane change resulted in collision", kmh)
			}
			t.Logf("[%d km/h] Overtaking success! Min longitudinal gap before lateral clearance: %.1f px",
				kmh, minLongitudinalGap)
		})
	}

	// Tight / close-quarters scenario: blocker at 180px while bus is in mid-overtake transition
	// Verifies bus decelerates behind blocker rather than accelerating into it
	t.Run("Close_Range_Lane_Change_No_Collision", func(t *testing.T) {
		cruiseSpd := 400.0 // 80 km/h
		yCenter := 1200.0 * MapGridScale

		sStart := NewBusStop(303, "Start3", Vec2{X: -2000, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
		sEnd := NewBusStop(304, "End3", Vec2{X: 2000, Y: yCenter}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
		w := NewWorld(lw, MapThemeMetropolis)
		w.Stops = []*BusStop{sStart, sEnd}
		route := &Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = cruiseSpd
		driver.State = AutopilotStateCruising
		driver.drivePath = []Vec2{{X: -2000, Y: yCenter + lw*1.5}, {X: 2000, Y: yCenter + lw*1.5}}
		driver.cumLen = []float64{0, 4000.0}
		driver.pathWorld = w
		driver.pathFromID = sStart.ID
		driver.pathToID = sEnd.ID
		driver.pathLane = lw
		driver.pathSynced = true

		bus := NewBus(1, 0, lw)
		bus.Pos = Vec2{X: -1000, Y: yCenter + lw*1.5}
		bus.Heading = 0
		bus.Speed = 200.0
		bus.MaxSpeedFwd = 460.0

		// Blocker placed 160px ahead, moving at 40px/s
		blocker := VehicleObstacle{
			Pos:     Vec2{X: bus.Pos.X + 160.0, Y: yCenter + lw*1.5},
			Heading: 0,
			Speed:   40.0,
			Length:  48,
			Width:   22,
		}

		driver.OvertakeState = OvertakeChangingLane
		driver.TargetLaneOffset = -lw

		// Compute controls: must NOT accelerate past, targetSpd must decelerate behind blocker
		th, br, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
		if th > 0.5 && br == 0.0 {
			t.Fatalf("Expected bus to brake/decelerate behind close blocker during lane change, got throttle=%.2f, brake=%.2f",
				th, br)
		}

		// Step forward and ensure no collision occurs
		for step := 0; step < 300; step++ {
			blocker.Pos.X += blocker.Speed * dt
			th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
			bus.Update(dt, th, br, st, hb, tg)

			dx := blocker.Pos.X - bus.Pos.X
			dy := math.Abs(bus.Pos.Y - blocker.Pos.Y)
			contactDistX := bus.Length/2 + blocker.Length/2 - 2.0
			contactDistY := bus.Width/2 + blocker.Width/2 - 2.0
			if math.Abs(dx) < contactDistX && dy < contactDistY {
				t.Fatalf("Step %d: Collision in close-range lane change! dx=%.1f, dy=%.1f", step, dx, dy)
			}
			if bus.Pos.X > blocker.Pos.X+80.0 {
				break
			}
		}
	})
}

// TestBerserkModeStationDockingWithWaitingPassengers verifies that when passengers are waiting
// at a station, the bus in Berserk mode (at 60, 80, 100, 120 km/h) reliably decelerates,
// pulls into the stop bay, and docks successfully without overshooting.
func TestBerserkModeStationDockingWithWaitingPassengers(t *testing.T) {
	lw := 47.5
	dt := 1.0 / 60.0

	for _, kmh := range []int{60, 80, 100, 120} {
		t.Run(fmt.Sprintf("%d_kmh_Station_Docking_Test", kmh), func(t *testing.T) {
			cruiseSpd := float64(kmh) * 5.0
			yRoad := 800.0

			// Stop 0 at -1500, Stop 1 at 0
			// Bay center at Y = yRoad + lw*1.5 + bayOffset (~yRoad + 115)
			s0 := NewBusStop(0, "DepartStop", Vec2{X: -1500, Y: yRoad + lw*1.5 + 44.0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			s1 := NewBusStop(1, "ArriveStop", Vec2{X: 0, Y: yRoad + lw*1.5 + 44.0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
			s1.WaitingPassengers = 6 // Station has waiting passengers!

			w := NewWorld(lw, MapThemeMetropolis)
			w.Stops = []*BusStop{s0, s1}

			route := &Route{
				Name:         "StationDockingRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.State = AutopilotStateCruising
			driver.drivePath = []Vec2{
				{X: -1500, Y: yRoad + lw*1.5},
				{X: 0, Y: yRoad + lw*1.5},
				s1.Pos,
			}
			driver.cumLen = []float64{0, 1500.0, 1500.0 + s1.Pos.Distance(Vec2{X: 0, Y: yRoad + lw*1.5})}

			bus := NewBus(1, 0, lw)
			bus.Pos = Vec2{X: -600, Y: yRoad + lw*1.5}
			bus.Heading = 0
			bus.Speed = cruiseSpd
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			docked := false
			for step := 0; step < 800; step++ {
				th, br, st, hb, tg, isDocked := driver.ComputeControls(bus, route, w, nil, nil, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)

				if isDocked || driver.State == AutopilotStateDocked {
					docked = true
					break
				}

				// If bus overshoots far past the stop without docking, fail early
				if bus.Pos.X > s1.Pos.X+120.0 {
					t.Fatalf("[%d km/h] Bus blew past the station without docking! bus.Pos=%v, stop.Pos=%v, speed=%.1f",
						kmh, bus.Pos, s1.Pos, bus.Speed)
				}
			}

			if !docked {
				t.Fatalf("[%d km/h] Bus never docked at the station with waiting passengers", kmh)
			}
			t.Logf("[%d km/h] Station docking successful! Final bus speed=%.1f, pos=%v, stop=%v",
				kmh, bus.Speed, bus.Pos, s1.Pos)
		})
	}
}

// TestBerserkModePassengerBoardingAndDockingCycle ensures that in Berserk mode,
// at various approach speeds, the bus reliably docks, opens doors, boards waiting passengers,
// finishes dwell, closes doors, and departs smoothly.
func TestBerserkModePassengerBoardingAndDockingCycle(t *testing.T) {
	testSpeeds := []float64{25.0, 20.0, 15.0, 8.0}
	for _, approachSpeed := range testSpeeds {
		t.Run(fmt.Sprintf("ApproachSpeed_%.1f", approachSpeed), func(t *testing.T) {
			g := NewGame(1280, 720)
			g.EnterCity(MapThemeMetropolis)
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			g.AutopilotDriver.BerserkMode = true

			// Route from Stop 0 to Stop 2
			g.Route.StopIDs = []int{0, 2, 10, 8, 0}
			g.Route.CurrentIndex = 1
			targetStop := g.World.Stops[2]
			targetStop.WaitingPassengers = 8

			// Place bus approaching Stop 2 with given speed
			g.Bus.Pos = targetStop.Pos.Sub(Vec2{X: math.Cos(targetStop.Heading) * 70, Y: math.Sin(targetStop.Heading) * 70})
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = approachSpeed
			g.Bus.DoorsOpen = false
			g.Bus.Passengers = nil
			g.Bus.PassengerCount = 0

			g.AutopilotDriver.DwellTotal = 0.4
			g.AutopilotDriver.State = AutopilotStateCruising

			dt := 0.016
			dockedSeen := false
			doorsOpenedSeen := false
			passengersBoardedSeen := false
			doorsClosedSeen := false

			for frame := 0; frame < 300; frame++ {
				laneW := 52.0
				th, br, st, hb, tg, isDocked := g.AutopilotDriver.ComputeControls(
					g.Bus, g.Route, g.World, g.TrafficLights, nil, laneW, dt,
				)

				g.Bus.Update(dt, th, br, st, hb, tg)

				var activeStop *BusStop
				for _, s := range g.World.Stops {
					isAutopilotDockedAtThisStop := g.Settings.AutopilotEnabled && g.AutopilotDriver != nil &&
						g.AutopilotDriver.State == AutopilotStateDocked && g.AutopilotDriver.DockedStopID == s.ID &&
						(s.ContainsBus(g.Bus) || s.Pos.Distance(g.Bus.Pos) < s.Length/2+65.0)
					if s.ContainsBus(g.Bus) || isAutopilotDockedAtThisStop {
						activeStop = s
						break
					}
				}

				if activeStop != nil {
					if g.Bus.DoorsOpen && !g.boardedThisStop {
						g.boardedThisStop = true
						g.Bus.RemovePassengersAlightingAt(activeStop.ID)
						boardSpace := g.Bus.Capacity - g.Bus.PassengerCount
						boardCount := activeStop.WaitingPassengers
						if boardCount > boardSpace {
							boardCount = boardSpace
						}
						if boardCount > 0 {
							candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
							actualBoarded := g.Bus.BoardPassengers(boardCount, candidates)
							activeStop.WaitingPassengers -= actualBoarded
							if actualBoarded > 0 {
								passengersBoardedSeen = true
							}
						}
						if activeStop.ID == g.Route.GetCurrentTargetStopID() {
							g.Route.AdvanceToNextStop()
						}
					}
				} else {
					g.boardedThisStop = false
				}

				if isDocked || g.AutopilotDriver.State == AutopilotStateDocked {
					dockedSeen = true
				}
				if g.Bus.DoorsOpen {
					doorsOpenedSeen = true
				}
				if doorsOpenedSeen && !g.Bus.DoorsOpen && g.Bus.DoorProgress <= 0.05 {
					doorsClosedSeen = true
					if g.AutopilotDriver.State != AutopilotStateDocked {
						break
					}
				}
			}

			if !dockedSeen {
				t.Fatalf("Bus failed to dock at station at approach speed %.1f", approachSpeed)
			}
			if !doorsOpenedSeen {
				t.Fatalf("Bus doors failed to open while docked at approach speed %.1f", approachSpeed)
			}
			if !passengersBoardedSeen {
				t.Fatalf("Passengers failed to board bus while docked in Berserk mode at approach speed %.1f", approachSpeed)
			}
			if !doorsClosedSeen {
				t.Fatalf("Bus doors failed to close after dwell at approach speed %.1f", approachSpeed)
			}
			if g.Bus.PassengerCount == 0 {
				t.Fatalf("Passenger count must be > 0, got %d", g.Bus.PassengerCount)
			}
			if targetStop.WaitingPassengers >= 8 {
				t.Fatalf("Waiting passengers at stop should have decreased from 8, got %d", targetStop.WaitingPassengers)
			}
		})
	}
}

// TestAutopilotDockingWithLateralOffset verifies that a bus pulling in with a slight lateral offset
// (e.g. 20px off center) is correctly recognized by ContainsBus and activeStop, boarding passengers.
func TestAutopilotDockingWithLateralOffset(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)
	s0 := g.World.Stops[0]
	s0.WaitingPassengers = 5

	// Position bus parked in bay with 20px lateral offset
	sinH := math.Sin(s0.Heading)
	cosH := math.Cos(s0.Heading)
	perpX := -sinH
	perpY := cosH
	g.Bus.Pos = s0.Pos.Add(Vec2{X: perpX * 20.0, Y: perpY * 20.0})
	g.Bus.Heading = s0.Heading
	g.Bus.Speed = 0.0
	g.Bus.DoorsOpen = false
	g.Bus.DoorProgress = 0.0
	g.Bus.Passengers = nil
	g.Bus.PassengerCount = 0

	if !s0.ContainsBus(g.Bus) {
		t.Fatalf("Bus with 20px lateral offset in bay should be accepted by ContainsBus")
	}

	// Press E to open doors in manual mode
	g.Bus.Update(0.016, 0, 0, 0, false, true)
	if !g.Bus.DoorsOpen {
		t.Fatalf("Doors should open when speed is 0 and toggleDoors is true")
	}

	// Run boarding logic
	var activeStop *BusStop
	for _, s := range g.World.Stops {
		if s.ContainsBus(g.Bus) {
			activeStop = s
			break
		}
	}
	if activeStop == nil {
		t.Fatalf("activeStop should be detected for 20px lateral offset bus in bay")
	}

	candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
	actual := g.Bus.BoardPassengers(activeStop.WaitingPassengers, candidates)
	if actual != 5 {
		t.Fatalf("Expected 5 passengers boarded, got %d", actual)
	}
}

// TestUnifiedMaxSpeedLimitManualAndBerserk verifies that the configurable top speed limit
// (60, 80, 100, 120 km/h) applies consistently to BOTH manual driving and AI Berserk mode.
func TestUnifiedMaxSpeedLimitManualAndBerserk(t *testing.T) {
	s := NewSettings()

	// 1. Verify default is 80 km/h
	if s.AutopilotBerserkMaxSpeed != 80 {
		t.Fatalf("Default max speed should be 80 km/h, got %d", s.AutopilotBerserkMaxSpeed)
	}
	if math.Abs(s.GetBusMaxSpeed()-445.0) > 1.0 {
		t.Fatalf("Default bus max speed should be 445.0 px/s (80 km/h), got %.2f", s.GetBusMaxSpeed())
	}
	if math.Abs(s.GetBerserkCruiseSpeed()-445.0) > 1.0 {
		t.Fatalf("Default berserk cruise speed should be 445.0 px/s (80 km/h), got %.2f", s.GetBerserkCruiseSpeed())
	}

	testCases := []struct {
		kmh          int
		manualMaxSpd float64
		cruiseSpd    float64
	}{
		{60, 334.0, 333.0},
		{80, 445.0, 445.0},
		{100, 556.0, 555.0},
		{120, 667.0, 667.0},
	}

	for _, tc := range testCases {
		s.SetMaxSpeed(tc.kmh)
		if s.AutopilotBerserkMaxSpeed != tc.kmh {
			t.Fatalf("Expected speed %d, got %d", tc.kmh, s.AutopilotBerserkMaxSpeed)
		}
		if math.Abs(s.GetBusMaxSpeed()-tc.manualMaxSpd) > 0.1 {
			t.Fatalf("[%d km/h] Expected bus manual max speed %.1f, got %.2f", tc.kmh, tc.manualMaxSpd, s.GetBusMaxSpeed())
		}
		if math.Abs(s.GetBerserkCruiseSpeed()-tc.cruiseSpd) > 0.1 {
			t.Fatalf("[%d km/h] Expected berserk cruise speed %.1f, got %.2f", tc.kmh, tc.cruiseSpd, s.GetBerserkCruiseSpeed())
		}

		// 2. Simulate manual driving kinematics
		bus := NewBus(100, 100, 0)
		bus.MaxSpeedFwd = s.GetBusMaxSpeed()

		// Full throttle forward for 5 seconds
		dt := 0.016
		for step := 0; step < 350; step++ {
			bus.Update(dt, 1.0, 0.0, 0.0, false, false)
		}

		// Verify speed capped at MaxSpeedFwd
		if bus.Speed > bus.MaxSpeedFwd+0.01 {
			t.Fatalf("[%d km/h] Manual bus exceeded speed limit! Speed=%.2f, Max=%.2f", tc.kmh, bus.Speed, bus.MaxSpeedFwd)
		}

		// Verify converted KM/H on HUD matches target speed limit
		hudKmh := int(math.Abs(bus.Speed) * 0.18)
		if hudKmh != tc.kmh {
			t.Fatalf("[%d km/h] Expected HUD speed %d km/h, got %d km/h (raw speed: %.2f)", tc.kmh, tc.kmh, hudKmh, bus.Speed)
		}
	}

	// 3. Test Cycling Sequence: 60 -> 80 -> 100 -> 120 -> 60
	s.SetMaxSpeed(60)
	if next := s.CycleMaxSpeed(); next != 80 {
		t.Fatalf("Expected cycle 60 -> 80, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 100 {
		t.Fatalf("Expected cycle 80 -> 100, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 120 {
		t.Fatalf("Expected cycle 100 -> 120, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 60 {
		t.Fatalf("Expected cycle 120 -> 60, got %d", next)
	}
}

// TestBerserkMode3LaneOvertakingOn6LaneRoad verifies that in Berserk Mode on 6-lane roads (3 lanes per direction),
// when both the right lane (offset 0.0) and middle lane (offset -lw) have vehicles, and the leftmost lane (offset -2*lw)
// is clear, the bus changes into the leftmost lane to overtake, safely passes, and returns once clear.
func TestBerserkMode3LaneOvertakingOn6LaneRoad(t *testing.T) {
	lw := 52.0
	wTest := NewWorld(lw, MapThemeMetropolis)
	s6Start := NewBusStop(601, "RingEastStart", Vec2{X: -1000 * MapGridScale, Y: -1200 * MapGridScale}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	s6End := NewBusStop(602, "RingEastEnd", Vec2{X: 1000 * MapGridScale, Y: -1200 * MapGridScale}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
	wTest.Stops = []*BusStop{s6Start, s6End}

	route := &Route{
		Name:         "Overtake3LaneRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	// 1. Bus in outer lane (0.0): car in outer lane (0.0) AND car in middle lane (-lw), left lane clear
	t.Run("OuterAndMiddleBlocked_ChangesToLeftmostLane", func(t *testing.T) {
		bus := NewBus(1, 0, lw)
		bus.Pos = Vec2{X: -600 * MapGridScale, Y: -1200*MapGridScale + lw*2.5} // In outer lane (0.0)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		// Car in outer lane (0.0) ahead
		carOuter := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 90, Y: -1200*MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		// Car in middle lane (-lw) ahead
		carMiddle := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 110, Y: -1200*MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   55.0,
			Length:  48,
			Width:   22,
		}

		// Init path
		driver.ComputeControls(bus, route, wTest, nil, nil, lw, 0.016)

		// Step with both outer and middle lanes occupied
		driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{carOuter, carMiddle}, lw, 0.016)

		if driver.OvertakeState != OvertakeChangingLane {
			t.Fatalf("Expected OvertakeChangingLane when both right lanes blocked, got state %d", driver.OvertakeState)
		}
		expectedTarget := -2.0 * lw
		if math.Abs(driver.TargetLaneOffset-expectedTarget) > 1.0 {
			t.Fatalf("Expected TargetLaneOffset %.1f (-2*lw), got %.2f", expectedTarget, driver.TargetLaneOffset)
		}
	})

	// 2. Bus currently in middle lane (-lw): lead car in middle lane ahead, outer lane blocked, leftmost clear
	t.Run("MiddleLaneBlocked_ChangesToLeftmostLane", func(t *testing.T) {
		bus := NewBus(1, 0, lw)
		bus.Pos = Vec2{X: -600 * MapGridScale, Y: -1200*MapGridScale + lw*1.5} // In middle lane (-lw)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		// Car in middle lane (-lw) ahead
		carMiddle := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 95, Y: -1200*MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		// Car in outer lane (0.0) alongside
		carOuter := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 10, Y: -1200*MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   45.0,
			Length:  48,
			Width:   22,
		}

		driver.ComputeControls(bus, route, wTest, nil, nil, lw, 0.016)
		driver.LaneOffset = -lw
		driver.TargetLaneOffset = -lw

		driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{carMiddle, carOuter}, lw, 0.016)

		if driver.OvertakeState != OvertakeChangingLane {
			t.Fatalf("Expected OvertakeChangingLane from middle lane, got state %d", driver.OvertakeState)
		}
		expectedTarget := -2.0 * lw
		if math.Abs(driver.TargetLaneOffset-expectedTarget) > 1.0 {
			t.Fatalf("Expected TargetLaneOffset %.1f (-2*lw), got %.2f", expectedTarget, driver.TargetLaneOffset)
		}
	})

	// 3. Passing in leftmost lane: does not brake for cars in right lanes, and returns to middle lane when cleared
	t.Run("LeftmostLanePassingAndReturning", func(t *testing.T) {
		bus := NewBus(1, 0, lw)
		bus.Pos = Vec2{X: -600 * MapGridScale, Y: -1200*MapGridScale + lw*0.5} // In leftmost lane (-2*lw)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising
		driver.LaneOffset = -2.0 * lw
		driver.TargetLaneOffset = -2.0 * lw
		driver.OvertakeState = OvertakePassing
		driver.OvertakeTimer = 1.0

		// Init path
		driver.ComputeControls(bus, route, wTest, nil, nil, lw, 0.016)
		driver.LaneOffset = -2.0 * lw
		driver.TargetLaneOffset = -2.0 * lw
		driver.OvertakeState = OvertakePassing
		driver.OvertakeTimer = 1.0

		// Cars in middle and outer lane are alongside/ahead
		carOuter := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 50, Y: -1200*MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddle := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 60, Y: -1200*MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   55.0,
			Length:  48,
			Width:   22,
		}

		throttle, brake, _, _, _, _ := driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{carOuter, carMiddle}, lw, 0.016)
		// Should NOT brake; bus is in clear leftmost lane!
		if brake > 0.1 {
			t.Fatalf("Bus in leftmost lane should not brake for vehicles in adjacent lanes, got brake %.2f", brake)
		}
		if throttle < 0.2 {
			t.Fatalf("Bus in leftmost lane should maintain throttle, got %.2f", throttle)
		}
		if driver.OvertakeState != OvertakePassing {
			t.Fatalf("Bus should remain in OvertakePassing while alongside cars, got %d", driver.OvertakeState)
		}

		// Now cars have been passed (behind bus)
		carOuterBehind := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale - 160, Y: -1200*MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddleBehind := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale - 150, Y: -1200*MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   55.0,
			Length:  48,
			Width:   22,
		}

		driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{carOuterBehind, carMiddleBehind}, lw, 0.016)
		if driver.OvertakeState != OvertakeReturning {
			t.Fatalf("Bus should transition to OvertakeReturning once obstacles cleared, got %d", driver.OvertakeState)
		}
		if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 && math.Abs(driver.TargetLaneOffset-0.0) > 1.0 {
			t.Fatalf("Bus should return to through lane (-lw or 0.0), got TargetLaneOffset %.2f", driver.TargetLaneOffset)
		}
	})

	// 4. Non-Berserk Standard Mode: must NOT use leftmost lane (-2*lw) to overtake
	t.Run("StandardMode_DoesNotUseLeftmostLane", func(t *testing.T) {
		bus := NewBus(1, 0, lw)
		bus.Pos = Vec2{X: -600 * MapGridScale, Y: -1200*MapGridScale + lw*2.5}
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = false // Standard mode!
		driver.State = AutopilotStateCruising

		carOuter := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 90, Y: -1200*MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddle := VehicleObstacle{
			Pos:     Vec2{X: -600*MapGridScale + 110, Y: -1200*MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   55.0,
			Length:  48,
			Width:   22,
		}

		driver.ComputeControls(bus, route, wTest, nil, nil, lw, 0.016)
		driver.ComputeControls(bus, route, wTest, nil, []VehicleObstacle{carOuter, carMiddle}, lw, 0.016)

		if driver.TargetLaneOffset <= -lw*1.5 {
			t.Fatalf("Standard mode must NOT target leftmost lane (-2*lw), got %.2f", driver.TargetLaneOffset)
		}
	})
}

// TestBerserkModeOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode, when overtaking a vehicle
// on a multi-lane road where the upcoming intersection requires a left turn:
// 1. After passing or while approaching, the bus does NOT return to the outer right lane.
// 2. The bus seamlessly continues forward directly into the dedicated left-turn lane.
// 3. The bus NEVER overshoots across the double yellow line into the oncoming lane during or after the merge taper.
func TestBerserkModeOvertakingIntoUpcomingLeftTurn(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	w.Roundabouts = nil

	// Eastbound along Y = 0 (Metropolis Central Boulevard, 4-lane avenue), turning North (Left) onto X = 0 (North-South avenue).
	// Legal travel is at Y > 0 (outer lane = 1.5*lw = 78, left-turn lane = 0.5*lw = 26).
	// Centerline is at Y = 0.0; oncoming Westbound traffic is at Y < 0.
	sStart := NewBusStop(0, "StartEast", Vec2{X: -600 * MapGridScale, Y: 0}, 0, 4, RGBA(0, 0, 0, 0), lw, 0) // X = -1800
	sEnd := NewBusStop(1, "EndNorth", Vec2{X: 0, Y: -600 * MapGridScale}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*BusStop{sStart, sEnd}

	route := &Route{
		Name:         "LeftTurnOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := NewBus(-1400, lw*1.5, 0) // in outer lane (Y = 78), 1400px before X=0
	bus.Speed = 160.0
	bus.Gear = GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow vehicle in outer lane ahead
	slowCar := VehicleObstacle{
		Pos:     Vec2{X: -1050, Y: lw * 1.5},
		Heading: 0,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	tls := &TrafficLightSystem{}
	inter := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             PhaseEWLeftGreen, // Left green
		GreenDuration:     15.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 15.0,
	}
	tls.Intersections = []*TrafficIntersection{inter}

	// Step through simulation
	dt := 0.016
	passedObstacle := false

	for step := 0; step < 400; step++ {
		slowCar.Pos.X += slowCar.Speed * dt
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, tls, []VehicleObstacle{slowCar}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Calculate front-left and rear-left corners (closest to centerline)
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := Vec2{X: sinH, Y: -cosH}
		flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
		rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		// Double yellow line check: while approaching intersection before the mouth (>80px),
		// the bus must NEVER cross into oncoming traffic (legal side is Y > 0).
		distToInter := math.Abs(bus.Pos.X)
		if distToInter > 80.0 {
			if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
				t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, LaneOffset=%.2f, TargetLaneOffset=%.2f",
					step, bus.Pos, flCorner, rlCorner, driver.LaneOffset, driver.TargetLaneOffset)
			}
		}

		// Once bus has passed the obstacle (bus.Pos.X > slowCar.Pos.X + 60):
		if bus.Pos.X > slowCar.Pos.X+60.0 {
			passedObstacle = true
			// Bus must NOT return to outer lane (Y must not veer back towards 78)
			if driver.OvertakeState == OvertakeReturning && driver.TargetLaneOffset == 0.0 && bus.Pos.Y > lw*1.2 {
				t.Fatalf("Step %d: Bus attempted to return to outer lane despite upcoming left turn! Pos=%v", step, bus.Pos)
			}
		}

		// Stop when bus reaches intersection mouth
		if bus.Pos.X >= -70.0 {
			break
		}
	}

	if !passedObstacle {
		t.Fatalf("Bus failed to overtake slowCar before intersection")
	}

	// Verify that the bus positioned safely in the dedicated left-turn lane (Y near 0.5*lw = 26)
	if bus.Pos.Y > lw*1.0 {
		t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f (expected near %.1f)", bus.Pos.Y, lw*0.5)
	}
}

// TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode on a 6-lane road:
// When overtaking or cruising where the upcoming intersection requires a left turn:
// TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode on a 6-lane road:
// When overtaking or cruising where the upcoming intersection requires a left turn:
// 1. The bus does NOT swerve into oncoming traffic (Y < 0).
// 2. The bus does NOT perform an S-curve (swerving right toward outer lanes then swerving left).
// 3. The bus smoothly transitions into the dedicated left-turn lane (Y ~ 0.5*lw = 26).
func TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn(t *testing.T) {
	lw := 52.0

	t.Run("OvertakingTrafficIntoUpcomingLeftTurn", func(t *testing.T) {
		w := NewWorld(lw, MapThemeCoastalBay)
		w.Roundabouts = nil

		sStart := NewBusStop(0, "StartEast6Lane", Vec2{X: -600 * MapGridScale, Y: 0}, 0, 6, RGBA(0, 0, 0, 0), lw, 0) // X = -1800
		sEnd := NewBusStop(1, "EndNorth", Vec2{X: 0, Y: -600 * MapGridScale}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
		w.Stops = []*BusStop{sStart, sEnd}

		route := &Route{
			Name:         "LeftTurn6LaneOvertakeRoute",
			StopIDs:      []int{0, 1},
			CurrentIndex: 1,
		}

		bus := NewBus(-1400, lw*2.5, 0) // in outer lane (Y = 130), 1400px before X=0
		bus.Speed = 220.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		slowOuter := VehicleObstacle{
			Pos:     Vec2{X: -950, Y: lw * 2.5},
			Heading: 0,
			Speed:   35.0,
			Length:  48,
			Width:   22,
		}
		slowMid := VehicleObstacle{
			Pos:     Vec2{X: -900, Y: lw * 1.5},
			Heading: 0,
			Speed:   35.0,
			Length:  48,
			Width:   22,
		}
		obstacles := []VehicleObstacle{slowOuter, slowMid}

		tls := &TrafficLightSystem{}
		inter := &TrafficIntersection{
			ID:                0,
			Center:            Vec2{X: 0, Y: 0},
			IsMajor:           true,
			ApproachHasRoad:   [4]bool{true, true, true, true},
			ApproachLanes:     [4]int{6, 4, 6, 4},
			ApproachIs4Lane:   [4]bool{true, true, true, true},
			Phase:             PhaseEWLeftGreen,
			GreenDuration:     15.0,
			YellowDuration:    2.5,
			LeftGreenDuration: 15.0,
		}
		tls.Intersections = []*TrafficIntersection{inter}

		dt := 0.016
		passedObstacle := false
		enteredLeftTurnCorridor := false

		for step := 0; step < 500; step++ {
			for i := range obstacles {
				obstacles[i].Pos.X += obstacles[i].Speed * dt
			}
			throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
				bus, route, w, tls, obstacles, lw, dt,
			)
			bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

			cosH := math.Cos(bus.Heading)
			sinH := math.Sin(bus.Heading)
			halfL := bus.Length * 0.5
			halfW := bus.Width * 0.5
			leftPerp := Vec2{X: sinH, Y: -cosH}
			flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
			rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

			distToInter := math.Abs(bus.Pos.X)
			if distToInter > 80.0 {
				if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
					t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v",
						step, bus.Pos, flCorner, rlCorner)
				}
			}

			if distToInter <= 600.0 && bus.Pos.Y <= lw*1.15 {
				enteredLeftTurnCorridor = true
			}
			if enteredLeftTurnCorridor && distToInter > 80.0 {
				if bus.Pos.Y > lw*1.35 {
					t.Fatalf("Step %d: Bus swerved back right (S-curve) after entering left-turn corridor! Pos=%v",
						step, bus.Pos)
				}
			}

			if bus.Pos.X > obstacles[1].Pos.X+60.0 {
				passedObstacle = true
			}
			if bus.Pos.X >= -70.0 {
				break
			}
		}

		if !passedObstacle {
			t.Fatalf("Bus failed to overtake slow vehicles before intersection")
		}
		if bus.Pos.Y > lw*1.0 {
			t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f", bus.Pos.Y)
		}
	})

	t.Run("MiddleLaneApproachSmoothTransitionNoSCurve", func(t *testing.T) {
		w := NewWorld(lw, MapThemeCoastalBay)
		w.Roundabouts = nil

		sStart := NewBusStop(0, "StartEast6Lane", Vec2{X: -600, Y: 0}, 0, 6, RGBA(0, 0, 0, 0), lw, 0)
		sEnd := NewBusStop(1, "EndNorth", Vec2{X: 0, Y: -600}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)
		w.Stops = []*BusStop{sStart, sEnd}

		route := &Route{
			Name:         "LeftTurn6LaneRoute",
			StopIDs:      []int{0, 1},
			CurrentIndex: 1,
		}

		bus := NewBus(-500, lw*1.5, 0) // in middle lane (Y = 78), 500px before X=0
		bus.Speed = 220.0
		bus.Gear = GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising
		driver.LaneOffset = -lw
		driver.TargetLaneOffset = -lw
		driver.lastTargetLaneOffset = -lw

		tls := &TrafficLightSystem{}
		inter := &TrafficIntersection{
			ID:                0,
			Center:            Vec2{X: 0, Y: 0},
			IsMajor:           true,
			ApproachHasRoad:   [4]bool{true, true, true, true},
			ApproachLanes:     [4]int{6, 4, 6, 4},
			ApproachIs4Lane:   [4]bool{true, true, true, true},
			Phase:             PhaseEWLeftGreen,
			GreenDuration:     15.0,
			YellowDuration:    2.5,
			LeftGreenDuration: 15.0,
		}
		tls.Intersections = []*TrafficIntersection{inter}

		dt := 0.016
		enteredLeftTurnCorridor := false

		for step := 0; step < 500; step++ {
			throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
				bus, route, w, tls, nil, lw, dt,
			)
			bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

			cosH := math.Cos(bus.Heading)
			sinH := math.Sin(bus.Heading)
			halfL := bus.Length * 0.5
			halfW := bus.Width * 0.5
			leftPerp := Vec2{X: sinH, Y: -cosH}
			flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
			rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))



			distToInter := math.Abs(bus.Pos.X)
			if distToInter > 80.0 {
				if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
					t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v",
						step, bus.Pos, flCorner, rlCorner)
				}
			}

			if bus.Pos.Y <= lw*1.15 {
				enteredLeftTurnCorridor = true
			}
			if enteredLeftTurnCorridor && distToInter > 80.0 {
				if bus.Pos.Y > lw*1.35 {
					t.Fatalf("Step %d: Bus swerved back right (S-curve) after entering left-turn corridor! Pos=%v",
						step, bus.Pos)
				}
			}

			if bus.Pos.X >= -70.0 {
				break
			}
		}

		if bus.Pos.Y > lw*1.0 {
			t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f", bus.Pos.Y)
		}
	})
}

// TestAIBusLeftTurnNotBlockedByOncomingBusInLeftTurnLane verifies that an AI bus making a left
// turn at an intersection does NOT falsely brake or deadlock due to an oncoming vehicle waiting
// in the dedicated left-turn lane across the road centerline.
func TestAIBusLeftTurnNotBlockedByOncomingBusInLeftTurnLane(t *testing.T) {
	lw := 26.0
	w := NewWorld(lw, MapThemeTwinCities)
	route := GetStandardRouteForMap(MapThemeTwinCities, w)

	// In TwinCities, the major intersection is at X=3900, Y=0.
	// AI#8 is turning left from Northbound avenue (X=3900) into Westbound expressway lanes (Y=-26).
	// Mid-turn, its position is near (3880, -10) with Heading ~2.7 rad (facing South-West).
	turningBus := NewBus(3880, -10, 2.7)
	turningBus.Speed = 30.0

	driver := NewAutopilotDriver()
	driver.State = AutopilotStateCruising
	// Path tracks left turn curve into westbound lane Y=-26
	driver.drivePath = []Vec2{
		{X: 3900, Y: 40},
		{X: 3880, Y: -10},
		{X: 3840, Y: -26},
		{X: 3700, Y: -26},
		{X: 3500, Y: -26},
	}
	driver.cumLen = make([]float64, len(driver.drivePath))
	for i := 1; i < len(driver.drivePath); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + driver.drivePath[i].Distance(driver.drivePath[i-1])
	}
	driver.WaypointIdx = 1

	// Player bus is stopped in the Eastbound left turn lane at Y=+26, X=3845, Heading=0.0
	playerObstacle := VehicleObstacle{
		Pos:     Vec2{X: 3845, Y: 26},
		Heading: 0.0,
		Speed:   0.0,
		Length:  48,
		Width:   24,
	}

	throttle, brake, _, _, _, _ := driver.ComputeControls(
		turningBus, route, w, nil, []VehicleObstacle{playerObstacle}, lw, 0.016,
	)

	// AI bus must NOT slam on brakes or set targetSpd to 0
	if brake > 0.5 {
		t.Fatalf("Turning AI bus falsely braked (brake=%.2f, throttle=%.2f) due to oncoming vehicle in opposite lane",
			brake, throttle)
	}
	if driver.LastTargetSpeed <= 0.5 {
		t.Fatalf("Target speed became %.2f (expected positive speed to complete left turn)", driver.LastTargetSpeed)
	}

	// Also verify AIBusManager deadlock timer is not suppressed by this oncoming vehicle
	aiBus := &AIBus{
		ID:     8,
		Bus:    turningBus,
		Route:  route,
		Driver: driver,
	}
	// An opposing AI bus that is jammed against aiBus
	otherBus := NewBus(3860, 10, 0.5)
	otherBus.Speed = 0.0
	otherAIBus := &AIBus{
		ID:     5,
		Bus:    otherBus,
		Route:  route,
		Driver: NewAutopilotDriver(),
	}
	playerBus := NewBus(3845, 26, 0.0)
	playerBus.Speed = 0.0

	abm := NewAIBusManager()
	abm.Buses = []*AIBus{aiBus, otherAIBus}

	// When turning bus is stationary and jammed against otherAIBus, deadlock timer should advance
	// because playerBus is oncoming traffic and not a same-direction leader blocking the bus
	turningBus.Speed = 0.0
	abm.Update(0.1, playerBus, w, nil, nil, lw)
	if aiBus.DeadlockTimer < 0.05 {
		t.Fatalf("DeadlockTimer failed to increment (got %.2f), playerBlocking falsely suppressed deadlock resolution",
			aiBus.DeadlockTimer)
	}
}

// TestAutopilotBerserkRoundaboutOvertakingDoesNotExitRoundabout verifies that in Berserk mode,
// when a bus encounters a slower vehicle ahead in a roundabout, it overtakes via the inner passing
// lane without falsely steering outwards or exiting the roundabout into unselected side avenues.
func TestAutopilotBerserkRoundaboutOvertakingDoesNotExitRoundabout(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, w)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.ensurePath(w, fromStop, toStop, lw)

	// Find the roundabout
	var rb *Roundabout
	for i := range w.Roundabouts {
		if w.Roundabouts[i].Center.Distance(Vec2{X: 0, Y: 0}) < 10.0 {
			rb = &w.Roundabouts[i]
			break
		}
	}
	if rb == nil {
		t.Fatalf("Civic roundabout not found in Metropolis")
	}

	// Find a waypoint in the roundabout along the bottom curve (near South road, Y > 0)
	var wpIdx int = -1
	for idx, pt := range driver.drivePath {
		if pt.Distance(rb.Center) <= rb.Radius && pt.Y > 100.0 && pt.X < -20.0 {
			wpIdx = idx
			break
		}
	}
	if wpIdx < 0 {
		t.Fatalf("Could not locate bottom waypoint inside roundabout")
	}

	startPt := driver.drivePath[wpIdx]
	nextPt := driver.drivePath[wpIdx+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 60.0
	driver.WaypointIdx = wpIdx

	// Place a slower blocker ahead in the outer lane of the roundabout
	blockerWP := driver.drivePath[wpIdx+3]
	blocker := VehicleObstacle{
		Pos:     blockerWP,
		Heading: heading,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// 1. TargetLaneOffset must NEVER be positive in a roundabout
		if driver.TargetLaneOffset > 0.1 {
			t.Fatalf("Step %d: TargetLaneOffset became positive (%.2f) in roundabout, shifting outwards into exit!",
				step, driver.TargetLaneOffset)
		}
		if driver.LaneOffset > 0.1 {
			t.Fatalf("Step %d: LaneOffset became positive (%.2f) in roundabout!", step, driver.LaneOffset)
		}

		// 2. Bus position must strictly remain inside the roundabout, not exiting south
		distToCenter := bus.Pos.Distance(rb.Center)
		if distToCenter > rb.Radius+15.0 {
			t.Fatalf("Step %d: Bus exited roundabout! Pos=%v, Dist=%.2f (Radius=%.2f)",
				step, bus.Pos, distToCenter, rb.Radius)
		}

		// Invariant: Bus must not drive into south avenue (Y > 240)
		if bus.Pos.Y > rb.Radius+15.0 {
			t.Fatalf("Step %d: Bus drove into south exit avenue! Pos=%v", step, bus.Pos)
		}
	}

	// Verify that the driver initiated an overtake into the inner lane (negative offset)
	if driver.TargetLaneOffset >= 0.0 && driver.LaneOffset >= 0.0 && driver.OvertakeState == OvertakeNone {
		t.Fatalf("Expected Berserk mode to attempt overtaking in the roundabout")
	}
}

// TestAutopilotBerserkStationApproachOvertakesIfRunwayPermits verifies that when there is sufficient runway,
// the Berserk autopilot overtakes the lead vehicle ahead, pulls cleanly in front, returns to offset 0.0,
// and docks at the station bay without colliding.
func TestAutopilotBerserkStationApproachOvertakesIfRunwayPermits(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.ensurePath(w, fromStop, toStop, lw)

	// Pick a straight section with plenty of runway before the stop (~650px)
	startWp := 55 // Waypoint on the long straight Southbound approach to Stop 2 (~685px runway)

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 70.0
	driver.WaypointIdx = startWp

	// Lead bus 100px ahead in the same base lane
	leadPt := startPt.Add(Vec2{X: math.Cos(heading) * 100.0, Y: math.Sin(heading) * 100.0})
	leadBus := VehicleObstacle{
		Pos:     leadPt,
		Heading: heading,
		Speed:   20.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	overtookSuccessfully := false
	docked := false

	for step := 0; step < 2400; step++ {
		// Move lead bus forward slowly, slowing down if user bus has cleanly pulled in front
		if bus.Pos.Y > leadBus.Pos.Y+75.0 && leadBus.Speed > 0.0 {
			leadBus.Speed = math.Max(0.0, leadBus.Speed-25.0*dt)
		}
		leadBus.Pos = leadBus.Pos.Add(Vec2{X: math.Cos(leadBus.Heading) * leadBus.Speed * dt, Y: math.Sin(leadBus.Heading) * leadBus.Speed * dt})

		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Check for physical body collision using Separating Axis Theorem (SAT)
		dCol := leadBus.Pos.Sub(bus.Pos)
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		fwdCol := math.Abs(dCol.X*cosH + dCol.Y*sinH)
		latCol := math.Abs(-dCol.X*sinH + dCol.Y*cosH)

		cosL := math.Cos(leadBus.Heading)
		sinL := math.Sin(leadBus.Heading)
		fwdLead := math.Abs(dCol.X*cosL + dCol.Y*sinL)
		latLead := math.Abs(-dCol.X*sinL + dCol.Y*cosL)

		halfL := bus.Length/2 + leadBus.Length/2
		halfW := bus.Width/2 + leadBus.Width/2

		if (fwdCol < halfL && latCol < halfW) && (fwdLead < halfL && latLead < halfW) {
			t.Fatalf("Step %d: Physical collision detected between bus and lead bus! fwdCol=%.2f, latCol=%.2f, fwdLead=%.2f, latLead=%.2f",
				step, fwdCol, latCol, fwdLead, latLead)
		}

		if driver.OvertakeState == OvertakePassing || driver.OvertakeState == OvertakeReturning {
			overtookSuccessfully = true
		}

		if isDocked || driver.State == AutopilotStateDocked {
			docked = true
			break
		}
	}

	if !overtookSuccessfully {
		t.Fatalf("Expected autopilot to initiate and execute overtaking when runway permitted")
	}
	if !docked && bus.Pos.Distance(toStop.Pos) > 40.0 {
		t.Fatalf("Bus did not reach destination stop bay cleanly: busPos=%v, stopPos=%v, dist=%.2f, speed=%.2f, docked=%v, state=%v",
			bus.Pos, toStop.Pos, bus.Pos.Distance(toStop.Pos), bus.Speed, docked, driver.State)
	}
}

// TestAutopilotBerserkStationApproachWaitsBehindWhenRunwayInsufficient verifies that when approaching
// a station stop without sufficient runway (e.g. lead vehicle slow near stop), the Berserk autopilot
// does NOT initiate an overtake and instead safely queues behind the lead vehicle.
func TestAutopilotBerserkStationApproachWaitsBehindWhenRunwayInsufficient(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.ensurePath(w, fromStop, toStop, lw)

	// Pick a start point only ~280px before the stop (insufficient runway to complete an overtake)
	targetWp := len(driver.drivePath) - 1
	var startWp int = -1
	totalPathLen := driver.cumLen[targetWp]
	for i := 0; i < targetWp; i++ {
		rem := totalPathLen - driver.cumLen[i]
		if rem <= 280.0 {
			startWp = i
			break
		}
	}
	if startWp < 0 {
		t.Fatalf("Could not find start waypoint with ~280px runway")
	}

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 45.0
	driver.WaypointIdx = startWp

	// Lead bus stopped or crawling 110px ahead near the station bay
	leadPt := startPt.Add(Vec2{X: math.Cos(heading) * 110.0, Y: math.Sin(heading) * 110.0})
	leadBus := VehicleObstacle{
		Pos:     leadPt,
		Heading: heading,
		Speed:   0.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	for step := 0; step < 250; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Must NOT switch to passing lane when runway is insufficient
		if driver.OvertakeState == OvertakeChangingLane || driver.TargetLaneOffset < -10.0 {
			t.Fatalf("Step %d: Autopilot initiated overtaking without sufficient runway! State=%v, TargetOffset=%.2f",
				step, driver.OvertakeState, driver.TargetLaneOffset)
		}

		// Check for collision
		dist := bus.Pos.Distance(leadBus.Pos)
		minClearDist := bus.Length/2 + leadBus.Length/2 + 4.0
		if dist < minClearDist {
			t.Fatalf("Step %d: Collision detected! Dist=%.2f, MinClear=%.2f", step, dist, minClearDist)
		}
	}

	// Bus must have stopped safely behind lead vehicle
	if bus.Speed > 2.0 {
		t.Fatalf("Expected bus to stop safely behind lead bus, got speed=%.2f", bus.Speed)
	}
}

// TestAutopilotBerserkStationApproachWaitsAlongsideAndEntersBayAfterLeadLeaves verifies that when
// the bus is approaching in the adjacent passing lane while the curb bay is occupied by a lead bus,
// the autopilot maintains its lane, does not steer into the lead bus, slows/stops alongside, and
// once the lead bus departs, smoothly enters the bay and docks.
func TestAutopilotBerserkStationApproachWaitsAlongsideAndEntersBayAfterLeadLeaves(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)
	route := GetStandardRouteForMap(MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.ensurePath(w, fromStop, toStop, lw)

	targetWp := len(driver.drivePath) - 1
	var startWp int = -1
	totalPathLen := driver.cumLen[targetWp]
	for i := 0; i < targetWp; i++ {
		rem := totalPathLen - driver.cumLen[i]
		if rem <= 240.0 {
			startWp = i
			break
		}
	}
	if startWp < 0 {
		t.Fatalf("Could not find start waypoint with ~220px runway")
	}

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	// Bus starts in adjacent passing lane (-lw offset)
	hDir := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	rightDir := Vec2{X: -hDir.Y, Y: hDir.X}
	busPos := startPt.Add(rightDir.Mul(-lw))

	bus := NewBus(busPos.X, busPos.Y, heading)
	bus.Speed = 40.0
	driver.WaypointIdx = startWp
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.OvertakeState = OvertakePassing

	// Lead AI bus stopped at the destination stop bay
	leadBus := VehicleObstacle{
		Pos:     toStop.Pos,
		Heading: toStop.Heading,
		Speed:   0.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	// Phase 1: Wait alongside the lead bus for 200 steps
	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Must not have docked in the middle of the road while in adjacent lane
		if isDocked || driver.State == AutopilotStateDocked {
			t.Fatalf("Step %d: Falsely docked in adjacent lane!", step)
		}

		// Must not cut into the lead bus (TargetLaneOffset must stay negative / waiting)
		if driver.TargetLaneOffset > -15.0 {
			t.Fatalf("Step %d: Swerved into occupied bay while lead bus was present! TargetLaneOffset=%.2f",
				step, driver.TargetLaneOffset)
		}

		// Must not collide with lead bus
		dist := bus.Pos.Distance(leadBus.Pos)
		minClearDist := bus.Length/2 + leadBus.Length/2 + 2.0
		if dist < minClearDist {
			t.Fatalf("Step %d: Collision with lead bus in bay! Dist=%.2f, MinClear=%.2f",
				step, dist, minClearDist)
		}
	}

	// Bus should be stopped or nearly stopped alongside the bay
	if bus.Speed > 5.0 {
		t.Fatalf("Expected bus to wait alongside at low speed/stopped, got Speed=%.2f", bus.Speed)
	}

	// Phase 2: Lead bus departs (clear obstacles)
	docked := false
	for step := 0; step < 300; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if isDocked || driver.State == AutopilotStateDocked {
			docked = true
			break
		}
	}

	if !docked {
		t.Fatalf("After lead bus departed, bus failed to enter the bay and dock cleanly! BusPos=%v, StopPos=%v, LaneOffset=%.2f",
			bus.Pos, toStop.Pos, driver.LaneOffset)
	}
}

// TestAutopilotBerserkRightTurnInto2LaneDoesNotOvertakeIntoOppositeShoulder verifies that when the bus
// executes a right turn into a 2-way 2-lane road with a slow vehicle downstream, it does NOT initiate
// borrow-lane overtaking during the turn or before straightening out (which would swing into the opposite shoulder).
// Once straightened out in the travel lane, it initiates borrow-lane overtaking, passes the slow vehicle,
// and returns safely without ever touching the opposite shoulder.
func TestAutopilotBerserkRightTurnInto2LaneDoesNotOvertakeIntoOppositeShoulder(t *testing.T) {
	lw := 52.0
	w := NewWorld(lw, MapThemeMetropolis)

	legRoute := &Route{
		Name:         "TestRightTurnRoute",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Bus starts 300px before right turn intersection at (3900, -1800)
	bus := NewBus(3600.0, -1774.0, 0.0)
	bus.Gear = GearDrive
	bus.TransmissionMode = TransmissionAuto
	bus.Speed = 60.0

	// Downstream slow vehicle on 2-lane road (Southbound lane center X = 3874.0)
	slowCar := VehicleObstacle{
		Pos:     Vec2{X: 3874.0, Y: -1450.0},
		Heading: math.Pi / 2,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	oppShoulderX := 3900.0 + lw // 3952.0 (East shoulder of 2-lane road)
	maxFLX := -1e9
	overtakeInitiatedAfterStraight := false

	for step := 0; step < 800; step++ {
		// Update slow vehicle
		slowCar.Pos.Y += slowCar.Speed * dt

		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, legRoute, w, nil, []VehicleObstacle{slowCar}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// 1. Verify physical boundaries: bus must never be on shoulder or crash barrier
		bRes := w.CheckBoundaries(bus.Pos, bus.Width/2, lw)
		if bRes.Zone == ZoneShoulder || bRes.Zone == ZoneHardBarrier || bus.OnShoulder {
			t.Fatalf("Step %d: Bus drove onto shoulder/barrier! Pos=%v, Zone=%v, OnShoulder=%v, State=%v, LaneOffset=%.2f",
				step, bus.Pos, bRes.Zone, bus.OnShoulder, driver.OvertakeState, driver.LaneOffset)
		}

		// 2. Track front-left and rear-left corner coordinates on the 2-lane road
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := Vec2{X: sinH, Y: -cosH}

		flCorner := bus.Pos.Add(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
		rlCorner := bus.Pos.Sub(Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		if bus.Pos.Y > -1750.0 && bus.Pos.X > 3800.0 {
			if flCorner.X > maxFLX {
				maxFLX = flCorner.X
			}
			if rlCorner.X > maxFLX {
				maxFLX = rlCorner.X
			}
			// Strict opposite shoulder check: bus body must NEVER cross or touch opposite shoulder (3952.0)
			if flCorner.X >= oppShoulderX || rlCorner.X >= oppShoulderX {
				t.Fatalf("Step %d: Bus corner penetrated opposite shoulder! maxFLX=%.2f, oppShoulderX=%.2f, busPos=%v, Heading=%.2f, State=%v",
					step, maxFLX, oppShoulderX, bus.Pos, bus.Heading, driver.OvertakeState)
			}
		}

		// 3. During the right turn itself (before vehicle has cleared the intersection turn at Y = -1650):
		// Autopilot must NOT initiate borrow-lane overtaking!
		if bus.Pos.Y < -1650.0 {
			if driver.OvertakeState != OvertakeNone {
				t.Fatalf("Step %d: Initiated overtaking prematurely during right turn! State=%v, Heading=%.2f, Pos=%v",
					step, driver.OvertakeState, bus.Heading, bus.Pos)
			}
		} else {
			// Once straightened out in the Southbound lane, borrow-lane overtake should initiate
			if driver.OvertakeState == OvertakeChangingLane || driver.OvertakeState == OvertakePassing {
				overtakeInitiatedAfterStraight = true
			}
		}

		// Finish once passed slow vehicle
		if bus.Pos.Y > slowCar.Pos.Y+120.0 {
			break
		}
	}

	if !overtakeInitiatedAfterStraight {
		t.Fatalf("Expected borrow-lane overtake to initiate after completing the right turn and straightening out")
	}
}


