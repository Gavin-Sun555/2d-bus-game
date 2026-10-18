package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestAIBusManagerConfigurableCounts verifies setting fleet counts for the route (0~8 buses).
func TestAIBusManagerConfigurableCounts(t *testing.T) {
	world := sim.NewWorld(52.0)
	mgr := traffic.NewAIBusManager()

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
	world := sim.NewWorld(52.0)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(world)
	tm := traffic.NewTrafficManager(52.0)

	mgr := traffic.NewAIBusManager()
	mgr.BuildFleet(world, 3, 52.0)

	if len(mgr.Buses) != 3 {
		t.Fatalf("Expected 3 AI buses, got %d", len(mgr.Buses))
	}

	// Simulate 100 frames
	playerBus := sim.NewBus(world.Stops[0].Pos.X, world.Stops[0].Pos.Y, world.Stops[0].Heading)
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

func TestBackgroundBusesAndBerserkSettingsRemainIndependent(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	w, lw := g.World, 52.0
	// F. Verify Other AI Buses remain in Standard Mode
	aiBusMgr := traffic.NewAIBusManager()
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

// TestAIBusFleetFrontBackQueueing verifies that multiple AI vehicles spawn strictly queued
// front-and-back behind the player bus at the starting terminal station (起点站), with zero
// lateral/vertical stacking, safe bumper clearance, and orderly departure progression.
func TestAIBusFleetFrontBackQueueing(t *testing.T) {
	themes := []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeCustom}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			lw := 52.0
			w := sim.NewWorld(lw, theme)
			startStop := w.Stops[0]
			cosH := math.Cos(startStop.Heading)
			sinH := math.Sin(startStop.Heading)

			for fleetCount := 1; fleetCount <= 4; fleetCount++ {
				playerBus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
				abm := traffic.NewAIBusManager()
				abm.BuildFleet(w, fleetCount, lw, playerBus)

				if len(abm.Buses) != fleetCount {
					t.Fatalf("[%s] FleetCount %d: Expected %d buses, got %d", theme.DisplayName(), fleetCount, fleetCount, len(abm.Buses))
				}

				playerOBB := sim.OBB{
					Center:  playerBus.Pos,
					HalfLen: playerBus.Length / 2,
					HalfWid: playerBus.Width / 2,
					Heading: playerBus.Heading,
				}

				// 1. Verify longitudinal queue positioning and zero lateral displacement
				prevBusPos := playerBus.Pos
				for i, ai := range abm.Buses {
					// Check heading alignment
					angleDiff := math.Abs(sim.NormalizeAngle(ai.Bus.Heading - startStop.Heading))
					if angleDiff > 1e-4 {
						t.Fatalf("[%s] AI bus #%d heading (%f) does not match stop heading (%f)",
							theme.DisplayName(), i+1, ai.Bus.Heading, startStop.Heading)
					}

					// Check longitudinal & lateral relative displacement to previous vehicle in line
					delta := ai.Bus.Pos.Sub(prevBusPos)
					longDist := delta.X*cosH + delta.Y*sinH           // Should be negative (behind)
					latDist := math.Abs(-delta.X*sinH + delta.Y*cosH) // Must be 0 (no lateral/vertical stacking)

					if latDist > 1.0 {
						t.Fatalf("[%s] AI bus #%d has lateral offset %.2f px from queue line (should be 0, front-and-back)",
							theme.DisplayName(), i+1, latDist)
					}

					expectedSpacing := 120.0
					if math.Abs(longDist-(-expectedSpacing)) > 1.0 {
						t.Fatalf("[%s] AI bus #%d longitudinal spacing %.2f px != expected -%.2f px",
							theme.DisplayName(), i+1, longDist, expectedSpacing)
					}

					// Verify clear bumper-to-bumper buffer (23px > 20px)
					bumperGap := math.Abs(longDist) - (ai.Bus.Length/2 + playerBus.Length/2)
					if bumperGap < 20.0 {
						t.Fatalf("[%s] AI bus #%d bumper gap %.2f px is too close (< 20px)", theme.DisplayName(), i+1, bumperGap)
					}

					// Verify no OBB collision with player bus
					aiOBB := sim.OBB{
						Center:  ai.Bus.Pos,
						HalfLen: ai.Bus.Length / 2,
						HalfWid: ai.Bus.Width / 2,
						Heading: ai.Bus.Heading,
					}
					collidesWithPlayer, _, depthPlayer := sim.CheckOBBCollision(playerOBB, aiOBB)
					if collidesWithPlayer && depthPlayer > 0 {
						t.Fatalf("[%s] AI bus #%d overlaps with player bus (depth: %.2f px)", theme.DisplayName(), i+1, depthPlayer)
					}

					// Verify no OBB collision with preceding AI bus
					if i > 0 {
						prevAI := abm.Buses[i-1]
						prevAIOBB := sim.OBB{
							Center:  prevAI.Bus.Pos,
							HalfLen: prevAI.Bus.Length / 2,
							HalfWid: prevAI.Bus.Width / 2,
							Heading: prevAI.Bus.Heading,
						}
						collidesPrev, _, depthPrev := sim.CheckOBBCollision(prevAIOBB, aiOBB)
						if collidesPrev && depthPrev > 0 {
							t.Fatalf("[%s] AI bus #%d overlaps with AI bus #%d (depth: %.2f px)", theme.DisplayName(), i+1, i, depthPrev)
						}
					}

					// Verify road boundary clearance
					bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width/2, lw)
					if bRes.Zone == sim.ZoneHardBarrier {
						t.Fatalf("[%s] AI bus #%d at %v intersects barrier (%s)", theme.DisplayName(), i+1, ai.Bus.Pos, bRes.ObstacleName)
					}

					prevBusPos = ai.Bus.Pos
				}

				// 2. Simulation check: while player is parked, AI buses must remain stopped and not deadlock
				tls := sim.NewTrafficLightSystem()
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
					playerBus.Pos = startStop.Pos.Add(sim.Vec2{X: cosH * 600.0, Y: sinH * 600.0}) // Player departs 600px ahead

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
}

// TestAIBusFleet8BusesAllThemes verifies configuring up to 8 AI buses across all 6 maps.
func TestAIBusFleet8BusesAllThemes(t *testing.T) {
	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			lw := 52.0
			w := sim.NewWorld(lw, theme)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)

			startStop := w.Stops[0]
			playerBus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			abm := traffic.NewAIBusManager()
			abm.BuildFleet(w, 8, lw, playerBus)

			if len(abm.Buses) != 8 {
				t.Fatalf("[%s] Expected 8 AI buses in fleet, got %d", theme.DisplayName(), len(abm.Buses))
			}

			// Verify each bus is on ZoneRoad without barrier collisions
			for i, ai := range abm.Buses {
				bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width/2, lw)
				if bRes.Zone == sim.ZoneHardBarrier {
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

func TestAIBusManagerObstaclesAndFleetCounts(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	abm := traffic.NewAIBusManager()

	abm.BuildFleet(w, 3, 52.0)
	if abm.TotalActiveBuses() != 3 {
		t.Errorf("Expected 3 active buses in fleet, got %d", abm.TotalActiveBuses())
	}

	// All obstacles
	allObs := abm.GetObstacles(-1)
	if len(allObs) != 3 {
		t.Errorf("Expected 3 obstacles, got %d", len(allObs))
	}

	// Exclude one bus
	firstID := abm.Buses[0].ID
	filteredObs := abm.GetObstacles(firstID)
	if len(filteredObs) != 2 {
		t.Errorf("Expected 2 obstacles excluding bus #%d, got %d", firstID, len(filteredObs))
	}

	// Set count to 0 (clears fleet)
	abm.SetLineCount(0, w, 52.0)
	if abm.TotalActiveBuses() != 0 {
		t.Errorf("Fleet count should be 0, got %d", abm.TotalActiveBuses())
	}
}

func TestAIBusDestinationBasedExchange(t *testing.T) {
	world := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	abm := traffic.NewAIBusManager()
	abm.SetLineCount(1, world, 52.0)

	if len(abm.Buses) < 1 {
		t.Fatalf("Expected at least 1 AI bus")
	}

	ab := abm.Buses[0]
	// AI bus route is [0, 2, 10, 8, 0]
	ab.Route.CurrentIndex = 0
	targetStopID := ab.Route.GetCurrentTargetStopID() // Stop 0
	st := world.Stops[targetStopID]
	st.WaitingPassengers = 10

	// Dock bus and open doors
	ab.Bus.DoorsOpen = true
	ab.LastExchangedStopID = -1

	// Simulate exchange directly
	candidates := ab.Route.GetUpcomingCandidateStopIDs(ab.Route.CurrentIndex)
	if len(candidates) == 0 {
		t.Fatalf("Expected candidates for AI bus")
	}
	for _, id := range candidates {
		if id == targetStopID {
			t.Fatalf("AI bus candidate should not include current stop ID %d", targetStopID)
		}
	}

	boarded := ab.Bus.BoardPassengers(5, candidates)
	if boarded != 5 || ab.Bus.PassengerCount != 5 {
		t.Fatalf("Expected 5 boarded on AI bus, got %d", boarded)
	}

	// When docking at next stop, passengers targeting it alight
	nextStopID := candidates[0]
	countForNext := ab.Bus.CountPassengersAlightingAt(nextStopID)
	removed := ab.Bus.RemovePassengersAlightingAt(nextStopID)
	if removed != countForNext {
		t.Fatalf("AI bus alighting count mismatch: %d vs %d", removed, countForNext)
	}
}

func TestAIBusFleetCapacityAndBoardingLogic(t *testing.T) {
	world := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	abm := traffic.NewAIBusManager()
	abm.SetLineCount(4, world, 52.0)

	if len(abm.Buses) != 4 {
		t.Fatalf("Expected 4 AI buses, got %d", len(abm.Buses))
	}

	// 1. Verify every AI bus has 45 capacity
	for _, ab := range abm.Buses {
		if ab.Bus.Capacity != 45 {
			t.Fatalf("AI Bus #%d capacity expected 45, got %d", ab.ID, ab.Bus.Capacity)
		}
	}

	// 2. Test AI Bus boarding all waiting passengers when space is available (no 6-passenger cap)
	ab := abm.Buses[0]
	targetStopID := ab.Route.GetCurrentTargetStopID()
	st := world.Stops[targetStopID]
	st.WaitingPassengers = 25
	ab.Bus.Pos = st.Pos
	ab.Bus.Heading = st.Heading
	ab.Bus.Speed = 0
	ab.Bus.DoorsOpen = true
	ab.Bus.DoorProgress = 1.0
	ab.LastExchangedStopID = -1
	ab.Driver.State = autopilot.AutopilotStateDocked
	ab.Driver.DockedStopID = targetStopID
	ab.Driver.DwellTimer = 3.0
	ab.Driver.DwellFinished = false

	// Run AI bus update when docked at station
	abm.Update(0.1, nil, world, nil, nil, 52.0)

	// Since capacity is 45 and bus was empty, all 25 waiting passengers must board
	if ab.Bus.PassengerCount != 25 {
		t.Fatalf("Expected all 25 passengers to board AI bus, got %d", ab.Bus.PassengerCount)
	}
	if st.WaitingPassengers != 0 {
		t.Fatalf("Expected 0 passengers remaining at stop, got %d", st.WaitingPassengers)
	}
	if ab.PassengersCarried != 25 {
		t.Fatalf("Expected PassengersCarried to be 25, got %d", ab.PassengersCarried)
	}

	// 3. Test boarding when waiting passengers exceed empty capacity
	// Now bus has 25 passengers, so empty capacity is 45 - 25 = 20.
	// Put 30 waiting passengers at next stop.
	ab.Route.AdvanceToNextStop()
	nextStopID := ab.Route.GetCurrentTargetStopID()
	nextStop := world.Stops[nextStopID]
	nextStop.WaitingPassengers = 45
	ab.Bus.Pos = nextStop.Pos
	ab.Bus.Heading = nextStop.Heading
	ab.Bus.DoorsOpen = true
	ab.Bus.DoorProgress = 1.0
	ab.LastExchangedStopID = -1
	ab.Driver.DockedStopID = nextStopID
	ab.Driver.DwellTimer = 3.0
	ab.Driver.DwellFinished = false

	abm.Update(0.1, nil, world, nil, nil, 52.0)

	// The bus alights passengers destined for nextStop, then boards up to full 45 capacity
	if ab.Bus.PassengerCount != 45 {
		t.Fatalf("Expected AI bus to reach full 45 capacity, got %d", ab.Bus.PassengerCount)
	}
	boardedAtNext := ab.PassengersCarried - 25
	if nextStop.WaitingPassengers != 45-boardedAtNext {
		t.Fatalf("Expected %d passengers remaining at stop, got %d", 45-boardedAtNext, nextStop.WaitingPassengers)
	}
}

func TestTrafficCarAIBusNoOverlap_ApproachingQueue(t *testing.T) {
	tm := traffic.NewTrafficManager(52.0)
	abm := traffic.NewAIBusManager()
	tm.AIBusMgr = abm

	// AI bus stopped at a stop line heading East in Lane 0 (Y = 26.0)
	aiBus := sim.NewBus(300.0, 26.0, 0.0)
	aiBus.Speed = 0.0
	ab := &traffic.AIBus{
		ID:     1,
		Bus:    aiBus,
		Route:  sim.NewRoute(),
		Driver: autopilot.NewAutopilotDriver(),
	}
	abm.Buses = []*traffic.AIBus{ab}

	// Traffic car behind AI bus in the same lane traveling East (Lane 0, Y = 26.0)
	car := &sim.TrafficCar{
		Pos:         sim.Vec2{X: 180.0, Y: 26.0},
		Heading:     0.0,
		Speed:       120.0,
		TargetSpeed: 120.0,
		Length:      48.0,
		Width:       22.0,
		IsEW:        true,
		RoadCenter:  0.0,
		LaneIndex:   0,
	}
	tm.Cars = []*sim.TrafficCar{car}

	// Simulate car approaching AI bus over 2 seconds (120 frames at 60 FPS)
	dt := 1.0 / 60.0
	for step := 0; step < 120; step++ {
		tm.Update(dt, nil, nil, 1, 52.0)

		// Assert that at every single frame, car never overlaps with AI bus
		busOBB := sim.OBB{Center: aiBus.Pos, HalfLen: aiBus.Length / 2.0, HalfWid: aiBus.Width / 2.0, Heading: aiBus.Heading}
		carOBB := sim.OBB{Center: car.Pos, HalfLen: car.Length / 2.0, HalfWid: car.Width / 2.0, Heading: car.Heading}
		collides, _, depth := sim.CheckOBBCollision(busOBB, carOBB)
		if collides && depth > 0.001 {
			t.Fatalf("Step %d: Car overlapped with AI bus! Depth: %.2f, Car Pos: %+v, Bus Pos: %+v", step, depth, car.Pos, aiBus.Pos)
		}
	}

	// Car should have come to a firm stop behind the AI bus
	if car.Speed > 1.0 {
		t.Fatalf("Expected car to be stopped behind AI bus, got speed: %.2f", car.Speed)
	}

	bumperDist := (aiBus.Pos.X - aiBus.Length/2.0) - (car.Pos.X + car.Length/2.0)
	if bumperDist < 20.0 {
		t.Fatalf("Expected bumper distance >= 20px, got: %.2f", bumperDist)
	}
}

func TestGameUpdate_TrafficCarDoesNotOverlapStoppedAIBus(t *testing.T) {
	g := game.NewGame(1280, 720)
	g.State = game.StatePlaying
	g.Settings.AIBusCount = 2
	g.AIBusMgr.BuildFleet(g.World, g.Settings.AIBusCount, 52.0, g.Bus)

	if len(g.AIBusMgr.Buses) == 0 {
		t.Fatalf("Expected AI buses to be spawned in game")
	}

	// Pick first AI bus and stop it in Lane 0 (Y = 26.0)
	targetAI := g.AIBusMgr.Buses[0]
	targetAI.Bus.Pos = sim.Vec2{X: 250.0, Y: 26.0}
	targetAI.Bus.Heading = 0.0
	targetAI.Bus.Speed = 0.0

	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if len(activeCars) == 0 {
		t.Fatalf("Expected active cars in game")
	}

	// Place active car behind the AI bus heading in same direction in Lane 0 (Y = 26.0)
	car := activeCars[0]
	car.Pos = sim.Vec2{X: 160.0, Y: 26.0}
	car.Heading = 0.0
	car.Speed = 100.0
	car.TargetSpeed = 100.0
	car.IsEW = true
	car.RoadCenter = 0.0
	car.LaneIndex = 0

	for frame := 0; frame < 60; frame++ {
		// Stop AI bus firmly each frame to simulate red light queue wait
		targetAI.Bus.Speed = 0.0

		_ = g.Update()

		// Verify no overlap
		busOBB := sim.OBB{Center: targetAI.Bus.Pos, HalfLen: targetAI.Bus.Length / 2.0, HalfWid: targetAI.Bus.Width / 2.0, Heading: targetAI.Bus.Heading}
		carOBB := sim.OBB{Center: car.Pos, HalfLen: car.Length / 2.0, HalfWid: car.Width / 2.0, Heading: car.Heading}
		collides, _, depth := sim.CheckOBBCollision(busOBB, carOBB)
		if collides && depth > 0.001 {
			t.Fatalf("Frame %d: Traffic car overlapped with stopped AI bus! Depth: %.2f", frame, depth)
		}
	}
}

// TestStationApproachAIBusDoesNotDespawnIn4Seconds verifies that an AI bus near a bus stop
// is NOT prematurely despawned after 4.0s (Point 1: station area timeout uses full traffic light cycle 38.0s).
func TestStationApproachAIBusDoesNotDespawnIn4Seconds(t *testing.T) {
	g := game.NewGame(1920, 1080)
	g.State = game.StatePlaying
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true

	// Pick a station in the route
	targetStopID := g.Route.StopIDs[1]
	targetStop := g.World.Stops[targetStopID]

	// Position player bus approaching the stop in adjacent lane
	h := targetStop.Heading
	cosH := math.Cos(h)
	sinH := math.Sin(h)
	perpX := -sinH
	perpY := cosH

	// Player bus is 120px behind stop along heading, shifted laterally
	g.Bus.Pos = targetStop.Pos.Sub(sim.Vec2{X: cosH * 120.0, Y: sinH * 120.0}).Add(sim.Vec2{X: perpX * -40.0, Y: perpY * -40.0})
	g.Bus.Heading = h
	g.Bus.Speed = 0.0

	// Position an AI bus stopped in the curb lane at the entrance of the stop bay
	aiBusPos := targetStop.Pos.Sub(sim.Vec2{X: cosH * 100.0, Y: sinH * 100.0})
	aiDriver := autopilot.NewAutopilotDriver()
	aiRoute := sim.GetStandardRouteForMap(g.Settings.MapTheme, g.World)
	aiRoute.CurrentIndex = 1
	aiBus := &traffic.AIBus{
		ID:     1,
		Bus:    sim.NewBus(aiBusPos.X, aiBusPos.Y, h),
		Route:  aiRoute,
		Driver: aiDriver,
	}
	aiBus.Bus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{aiBus}

	// Advance route index and load passengers so we can verify route is not lost
	g.Route.CurrentIndex = 1
	g.Bus.PassengerCount = 10

	// Simulate for 5.0 seconds (> IntersectionDeadlockThreshold = 4.0s)
	totalFrames := int(5.0 * 60)
	for i := 0; i < totalFrames; i++ {
		g.Bus.Speed = 0.0
		aiBus.Bus.Speed = 0.0
		_ = g.Update()
	}

	// 1. Point 1: The AI bus at the station must NOT have been prematurely despawned after 4.0s!
	if aiBus.Bus.Pos.Distance(aiBusPos) > 20.0 {
		t.Fatalf("AI bus at station approach was prematurely despawned within 4.0s! AI Bus moved to %v", aiBus.Bus.Pos)
	}

	// 2. The player bus must remain on active leg 1 with passengers intact
	if g.Route.CurrentIndex != 1 {
		t.Fatalf("Player bus was wrongfully reset to route index %d; should remain on active leg 1", g.Route.CurrentIndex)
	}
	if g.Bus.PassengerCount != 10 {
		t.Fatalf("Player bus lost passengers: got %d, expected 10", g.Bus.PassengerCount)
	}
}
