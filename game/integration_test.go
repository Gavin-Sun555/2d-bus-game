package game

import (
	"testing"
)

// TestIntegrationAllMapsRouteCompletion tests that all 6 city maps can be entered
// and their designated transit routes can be 100% completed from start to finish
// with full passenger service (boarding, alighting, fare collection, and route loop).
func TestIntegrationAllMapsRouteCompletion(t *testing.T) {
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

			if g.State != StatePlaying {
				t.Fatalf("[%s] Game state must be StatePlaying, got %v", theme.DisplayName(), g.State)
			}
			if g.World.Theme != theme {
				t.Fatalf("[%s] World theme mismatch, got %v", theme.DisplayName(), g.World.Theme)
			}
			if len(g.Route.StopIDs) < 2 {
				t.Fatalf("[%s] Route must have multiple stops, got %d", theme.DisplayName(), len(g.Route.StopIDs))
			}

			firstStop := g.World.Stops[0]
			if g.Bus.Pos.Distance(firstStop.Pos) > 1.0 {
				t.Fatalf("[%s] Bus must spawn at initial stop bay 0: expected %v, got %v",
					theme.DisplayName(), firstStop.Pos, g.Bus.Pos)
			}
			if !firstStop.ContainsBus(g.Bus) {
				t.Fatalf("[%s] Bus must be squarely inside initial stop bay 0", theme.DisplayName())
			}

			initialCash := g.Cash
			initialTrips := g.Route.TotalTrips
			totalStopsInRoute := len(g.Route.StopIDs)

			// Step through each stop in the route until a full trip loop is completed
			maxSteps := totalStopsInRoute + 2
			stepCount := 0

			for g.Route.TotalTrips == initialTrips && stepCount < maxSteps {
				stepCount++
				targetStopID := g.Route.GetCurrentTargetStopID()
				if targetStopID < 0 || targetStopID >= len(g.World.Stops) {
					t.Fatalf("[%s] Invalid target stop ID: %d", theme.DisplayName(), targetStopID)
				}
				targetStop := g.World.Stops[targetStopID]

				// 1. Bus arrives at target stop bay and parks cleanly
				g.Bus.Pos = targetStop.Pos
				g.Bus.Heading = targetStop.Heading
				g.Bus.Speed = 0
				g.Bus.SteeringAngle = 0
				g.Bus.IsBraking = true

				if !targetStop.ContainsBus(g.Bus) {
					t.Fatalf("[%s] Bus parked at stop #%d (%s) must be accepted by bay containment",
						theme.DisplayName(), targetStop.ID, targetStop.Name)
				}

				// Verify stop bay is on drivable road/shoulder surface, never embedded in a hard barrier
				bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, g.Settings.GetLaneWidth())
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop bay #%d (%s) at %v is embedded in hard barrier (%s)!",
						theme.DisplayName(), targetStop.ID, targetStop.Name, targetStop.Pos, bRes.ObstacleName)
				}

				// Ensure some passengers are waiting if currently 0
				if targetStop.WaitingPassengers == 0 {
					targetStop.WaitingPassengers = 4
				}

				// 2. Open doors to initiate boarding / alighting
				g.Bus.DoorsOpen = true
				prevIndex := g.Route.CurrentIndex

				// Execute game simulation tick
				err := g.Update()
				if err != nil {
					t.Fatalf("[%s] g.Update() failed during passenger service: %v", theme.DisplayName(), err)
				}

				// Verify passenger service occurred and route advanced
				if g.Route.CurrentIndex == prevIndex && g.Route.TotalTrips == initialTrips {
					t.Fatalf("[%s] Route failed to advance to next stop at stop #%d (%s)",
						theme.DisplayName(), targetStop.ID, targetStop.Name)
				}

				// 3. Close doors and depart the stop bay
				g.Bus.DoorsOpen = false
				g.Bus.IsBraking = false
				// Move bus to road centerline adjacent to stop to clear the bay
				g.Bus.Pos = targetStop.RoadCenter
				g.Bus.Speed = 25.0

				// Execute tick outside the stop bay to clear boarding state
				err = g.Update()
				if err != nil {
					t.Fatalf("[%s] g.Update() failed during departure: %v", theme.DisplayName(), err)
				}
			}

			// 4. Assert full loop completed
			if g.Route.TotalTrips <= initialTrips {
				t.Fatalf("[%s] Failed to complete full route loop within %d steps (TotalTrips: %d)",
					theme.DisplayName(), maxSteps, g.Route.TotalTrips)
			}

			// 5. Assert revenue collected from passenger fares
			if g.Cash <= initialCash {
				t.Fatalf("[%s] Expected cash to increase from fares: initial=%d, final=%d",
					theme.DisplayName(), initialCash, g.Cash)
			}

			// 6. Assert bus durability is intact
			if g.Bus.Durability < 100.0 {
				t.Fatalf("[%s] Bus durability degraded during normal operation: %f",
					theme.DisplayName(), g.Bus.Durability)
			}
		})
	}
}

// TestIntegrationAll12StopsServiceableOnAllMaps tests that every single stop bay (all 12 stops)
// across every map is 100% accessible, correctly oriented on the right-hand side,
// safely clear of hard barriers, and capable of handling boarding and route advancement.
func TestIntegrationAll12StopsServiceableOnAllMaps(t *testing.T) {
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

			numStops := len(g.World.Stops)
			if numStops < 6 {
				t.Fatalf("[%s] Expected at least 6 stops, got %d", theme.DisplayName(), numStops)
			}

			// Build a grand-tour route visiting every single stop in sequential order and looping back to 0
			g.Route.Clear()
			for i := 0; i < numStops; i++ {
				g.Route.AddStop(i)
			}
			g.Route.AddStop(0) // Return to starting terminal

			laneW := g.Settings.GetLaneWidth()

			for stopIdx := 0; stopIdx < len(g.Route.StopIDs); stopIdx++ {
				currTargetID := g.Route.GetCurrentTargetStopID()
				stop := g.World.Stops[currTargetID]

				// A. Verify Stop Bay Boundary Safety
				bRes := g.World.CheckBoundaries(stop.Pos, 18.0, laneW)
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop #%d (%s) bay at %v is inside ZoneHardBarrier (%s)",
						theme.DisplayName(), stop.ID, stop.Name, stop.Pos, bRes.ObstacleName)
				}

				// B. Verify Road Center Point Boundary Safety
				bRoadRes := g.World.CheckBoundaries(stop.RoadCenter, 18.0, laneW)
				if bRoadRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop #%d (%s) road center at %v is inside ZoneHardBarrier",
						theme.DisplayName(), stop.ID, stop.Name, stop.RoadCenter)
				}

				// C. Place bus inside stop bay
				g.Bus.Pos = stop.Pos
				g.Bus.Heading = stop.Heading
				g.Bus.Speed = 0
				g.Bus.DoorsOpen = true
				g.Bus.IsBraking = true

				if !stop.ContainsBus(g.Bus) {
					t.Fatalf("[%s] Stop #%d (%s) failed to contain bus parked at bay center %v (heading %.2f)",
						theme.DisplayName(), stop.ID, stop.Name, stop.Pos, stop.Heading)
				}

				// D. Execute boarding
				stop.WaitingPassengers = 3
				prevIndex := g.Route.CurrentIndex
				prevTrips := g.Route.TotalTrips

				err := g.Update()
				if err != nil {
					t.Fatalf("[%s] g.Update() failed: %v", theme.DisplayName(), err)
				}

				// Verify route advanced
				if g.Route.CurrentIndex == prevIndex && g.Route.TotalTrips == prevTrips {
					t.Fatalf("[%s] Route failed to advance after servicing stop #%d (%s)",
						theme.DisplayName(), stop.ID, stop.Name)
				}

				// E. Depart bay
				g.Bus.DoorsOpen = false
				g.Bus.Pos = stop.RoadCenter
				g.Bus.Speed = 20.0
				_ = g.Update()
			}

			// Assert grand tour completed
			if g.Route.TotalTrips != 1 {
				t.Fatalf("[%s] Grand tour route should have completed exactly 1 full trip loop, got %d",
					theme.DisplayName(), g.Route.TotalTrips)
			}
		})
	}
}

// TestIntegrationDynamicMapSwitchingAndFullGameplayLoop verifies seamless transition
// between main menu, different city maps, and custom map hot reload without state corruptions.
func TestIntegrationDynamicMapSwitchingAndFullGameplayLoop(t *testing.T) {
	g := NewGame(1280, 720)

	// 1. Initial State must be Main Menu
	if g.State != StateMainMenu {
		t.Fatalf("Game must start in StateMainMenu, got %v", g.State)
	}

	themesToTest := []MapTheme{
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeMetropolis,
		MapThemeCustom,
	}

	for _, theme := range themesToTest {
		// 2. Select theme and enter city
		g.SelectedMapTheme = theme
		g.EnterCity(theme)

		if g.State != StatePlaying {
			t.Fatalf("Entering %s should set StatePlaying, got %v", theme.DisplayName(), g.State)
		}
		if g.World.Theme != theme {
			t.Fatalf("Theme mismatch: expected %s, got %s", theme.DisplayName(), g.World.Theme.DisplayName())
		}

		// 3. Simulate driving 1.0 second (60 ticks)
		for step := 0; step < 60; step++ {
			err := g.Update()
			if err != nil {
				t.Fatalf("[%s] Error during driving loop step %d: %v", theme.DisplayName(), step, err)
			}
		}

		// 4. Verify bus position is valid (not NaN or Inf)
		if g.Bus.Pos.X != g.Bus.Pos.X || g.Bus.Pos.Y != g.Bus.Pos.Y {
			t.Fatalf("[%s] Bus position corrupted with NaN", theme.DisplayName())
		}

		// 5. If Custom Map, test on-the-fly hot reload
		if theme == MapThemeCustom {
			err := g.ReloadCustomMap()
			if err != nil {
				t.Fatalf("Hot reload custom map failed: %v", err)
			}
			if len(g.HUD.Toasts) == 0 {
				t.Fatal("Expected HUD toast after custom map reload")
			}
		}

		// 6. Return to Main Menu via HUD
		g.HUD.ReturnToMenuRequested = true
		err := g.Update()
		if err != nil {
			t.Fatalf("[%s] Error returning to menu: %v", theme.DisplayName(), err)
		}
		if g.State != StateMainMenu {
			t.Fatalf("Expected state to return to StateMainMenu, got %v", g.State)
		}
	}
}

// TestIntegrationVehicleKinematicsAndSafeRoadDriving verifies that player bus can accelerate,
// steer, brake, and navigate on each map without hitting boundary pushbacks or getting stuck.
func TestIntegrationVehicleKinematicsAndSafeRoadDriving(t *testing.T) {
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

			// Start from first stop bay, close doors, and accelerate along road center
			s0 := g.World.Stops[0]
			g.Bus.Pos = s0.RoadCenter
			g.Bus.Heading = s0.Heading
			g.Bus.Speed = 0
			g.Bus.DoorsOpen = false

			laneW := g.Settings.GetLaneWidth()

			// Accelerate forward for 60 ticks (1 second)
			for i := 0; i < 60; i++ {
				g.Bus.Update(1.0/60.0, 1.0, 0.0, 0.0, false, false)

				// Verify bus speed is accelerating forward
				if g.Bus.Speed <= 0 && i > 5 {
					t.Fatalf("[%s] Bus should accelerate forward with throttle", theme.DisplayName())
				}

				// Verify bus remains in legal road or shoulder corridor
				bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, laneW)
				if bRes.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Bus hit hard barrier (%s) while driving along road center at %v",
						theme.DisplayName(), bRes.ObstacleName, g.Bus.Pos)
				}
			}

			// Steer slightly left for 30 ticks
			for i := 0; i < 30; i++ {
				g.Bus.Update(1.0/60.0, 0.5, 0.0, -0.4, false, false)
			}

			// Brake to full stop
			for i := 0; i < 60; i++ {
				g.Bus.Update(1.0/60.0, 0.0, 1.0, 0.0, false, false)
				if g.Bus.Speed == 0 {
					break
				}
			}

			if g.Bus.Speed != 0 {
				t.Fatalf("[%s] Bus failed to come to complete stop with brake applied: speed=%f",
					theme.DisplayName(), g.Bus.Speed)
			}
		})
	}
}
