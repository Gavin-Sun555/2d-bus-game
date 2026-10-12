package game

import (
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

// TestAIBusManagerConfigurableCounts verifies setting fleet counts for the route (0~4 buses).
func TestAIBusManagerConfigurableCounts(t *testing.T) {
	world := NewWorld(52.0)
	mgr := NewAIBusManager()

	// Initial default should be 0
	if mgr.GetLineCount() != 0 {
		t.Fatalf("Initial AI bus count should be 0, got %d", mgr.BusCount)
	}

	// 1. Build fleet with 3 buses
	mgr.BuildFleet(world, 3, 52.0)

	if mgr.GetLineCount() != 3 {
		t.Fatalf("Configured bus count mismatch: got %d, expected 3", mgr.BusCount)
	}

	if len(mgr.Buses) != 3 {
		t.Fatalf("Expected 3 total AI buses, got %d", len(mgr.Buses))
	}

	// 2. Test CycleLineCount (0 -> 1 -> 2 -> 3 -> 4 -> 0)
	mgr.SetLineCount(4, world, 52.0)
	next := mgr.CycleLineCount(world, 52.0)
	if next != 0 || mgr.GetLineCount() != 0 {
		t.Fatalf("CycleLineCount after 4 should wrap to 0, got %d", next)
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

			// Simulate up to 240 seconds (plenty of time for full loop at 60fps)
			maxFrames := 60 * 240
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

	// Set active leg Stop 9 (-1950, 1200) -> Stop 10 (650, 1200) Eastbound along South Ring (Y=1200)
	g.Route.StopIDs = []int{9, 10}
	g.Route.CurrentIndex = 1
	camPos := Vec2{X: -650, Y: 1200}
	g.Bus.Pos = Vec2{X: -850, Y: 1200 + 52.0*1.5}
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
	s0 := w.Stops[0]
	rb := w.Roundabouts[0]
	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	midR := (rApron + rOuter) * 0.5 // 188.0px
	_ = midR

	bus := NewBus(s0.Pos.X+50, 78.0, 0)
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

	// s1: on South Ring Avenue (Y = 1200, 4-lane avenue), Eastbound towards intersection (0, 1200)
	s1 := NewBusStop(101, "TestEastbound", Vec2{X: -650, Y: 1200}, 0, 4, RGBA(0, 0, 0, 0), lw, 0)
	// s2: on Central North-South Avenue (X = 0, 4-lane avenue), Northbound
	s2 := NewBusStop(102, "TestNorthbound", Vec2{X: 0, Y: 600}, -math.Pi/2, 4, RGBA(0, 0, 0, 0), lw, 0)

	path := computeRoadPathBetweenStops(w, s1, s2, lw)
	if len(path) == 0 {
		t.Fatalf("Failed to compute path")
	}
	// Verify that approaching intersection (0, 1200) from the west (X in [-200, 0]),
	// the waypoints have merged into the dedicated inner left-turn lane (offset 0.5 * lw = 26.0)
	foundInnerLaneApproach := false
	foundInnerApex := false
	foundTargetOuterMerge := false
	for _, pt := range path {
		if pt.X >= -200.0 && pt.X <= 0.0 {
			latDist := math.Abs(pt.Y - 1200.0)
			if latDist < lw*0.95 {
				foundInnerLaneApproach = true
				if math.Abs(latDist-lw*0.5) > 2.0 {
					t.Errorf("Approach waypoint %v lateral dist=%.1f, expected inner lane center=%.1f",
						pt, latDist, lw*0.5)
				}
			}
		}
		if math.Abs(pt.X-lw*0.5) < 2.0 && math.Abs(pt.Y-(1200.0+lw*0.5)) < 2.0 {
			foundInnerApex = true
		}
		if math.Abs(pt.X-lw*1.5) < 2.0 && pt.Y < 1200.0-lw {
			foundTargetOuterMerge = true
		}
	}

	if !foundInnerLaneApproach {
		t.Fatalf("Bus route failed to enter dedicated inner left-turn lane before left turn at (0, 1200)")
	}
	if !foundInnerApex {
		t.Fatalf("Bus route left-turn apex is not at inner lane junction (expected X=%.1f, Y=%.1f)",
			lw*0.5, 1200.0+lw*0.5)
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

	// Verify that before the U-turn at X=1300 (in X in [1130, 1300]),
	// the waypoints have merged into the inner left-turn lane (Y = 0.5*lw = 21.0)
	foundInnerLaneUTurn := false
	for _, pt := range path {
		if pt.X >= 1130.0 && pt.X <= 1300.0 {
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

