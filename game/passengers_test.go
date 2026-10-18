package game

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestBerserkModePassengerBoardingAndDockingCycle ensures that in Berserk mode,
// at various approach speeds, the bus reliably docks, opens doors, boards waiting passengers,
// finishes dwell, closes doors, and departs smoothly.
func TestBerserkModePassengerBoardingAndDockingCycle(t *testing.T) {
	testSpeeds := []float64{25.0, 20.0, 15.0, 8.0}
	for _, approachSpeed := range testSpeeds {
		t.Run(fmt.Sprintf("ApproachSpeed_%.1f", approachSpeed), func(t *testing.T) {
			g := NewGame(1280, 720)
			g.EnterCity(sim.MapThemeMetropolis)
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			g.AutopilotDriver.BerserkMode = true

			// Route from Stop 0 to Stop 2
			g.Route.StopIDs = []int{0, 2, 10, 8, 0}
			g.Route.CurrentIndex = 1
			targetStop := g.World.Stops[2]
			targetStop.WaitingPassengers = 8

			// Place bus approaching Stop 2 with given speed
			g.Bus.Pos = targetStop.Pos.Sub(sim.Vec2{X: math.Cos(targetStop.Heading) * 70, Y: math.Sin(targetStop.Heading) * 70})
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = approachSpeed
			g.Bus.DoorsOpen = false
			g.Bus.Passengers = nil
			g.Bus.PassengerCount = 0

			g.AutopilotDriver.DwellTotal = 0.4
			g.AutopilotDriver.State = autopilot.AutopilotStateCruising

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

				var activeStop *sim.BusStop
				for _, s := range g.World.Stops {
					isAutopilotDockedAtThisStop := g.Settings.AutopilotEnabled && g.AutopilotDriver != nil &&
						g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == s.ID &&
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

				if isDocked || g.AutopilotDriver.State == autopilot.AutopilotStateDocked {
					dockedSeen = true
				}
				if g.Bus.DoorsOpen {
					doorsOpenedSeen = true
				}
				if doorsOpenedSeen && !g.Bus.DoorsOpen && g.Bus.DoorProgress <= 0.05 {
					doorsClosedSeen = true
					if g.AutopilotDriver.State != autopilot.AutopilotStateDocked {
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

func TestPassengerNoLoopingOverFullRoute(t *testing.T) {
	// Full simulated loop test: ensure passengers get off at their target stops
	// and NO passenger ever stays on the bus for a full loop!
	game := NewGame(1280, 720)

	// Ensure Metropolis map and standard loop [0, 2, 10, 8, 0]
	game.SetMapTheme(sim.MapThemeMetropolis)
	route := game.Route
	if len(route.StopIDs) < 2 || route.StopIDs[0] != route.StopIDs[len(route.StopIDs)-1] {
		t.Fatalf("Expected closed loop route for Metropolis")
	}

	// Simulate 3 full trips
	for trip := 0; trip < 3; trip++ {
		for stopSeqIdx := 0; stopSeqIdx < len(route.StopIDs); stopSeqIdx++ {
			currStopID := route.StopIDs[stopSeqIdx]
			stop := game.World.Stops[currStopID]

			// Position bus at this stop
			game.Bus.Pos = stop.Pos
			game.Bus.Heading = stop.Heading
			game.Bus.DoorsOpen = true
			game.boardedThisStop = false

			// Ensure station has waiting passengers
			stop.WaitingPassengers = 8

			// Update driver mode to trigger passenger boarding/alighting
			game.updateDriverMode(0.016)

			// Invariant check: PassengerCount must always equal len(Passengers)
			if game.Bus.PassengerCount != len(game.Bus.Passengers) {
				t.Fatalf("Trip %d Stop %d: PassengerCount (%d) != len(Passengers) (%d)",
					trip, currStopID, game.Bus.PassengerCount, len(game.Bus.Passengers))
			}

			// Invariant check: No passenger on board should have target == currStopID
			// (because all passengers destined for currStopID must have alighted)
			if game.Bus.CountPassengersAlightingAt(currStopID) > 0 {
				t.Fatalf("Trip %d Stop %d: passengers still on board destined for this stop!", trip, currStopID)
			}
		}

		// At the completion of the loop (Stop 0 terminus), all passengers must have alighted
		// Any new passengers boarded at Stop 0 must strictly have destination in [2, 10, 8]
		for _, p := range game.Bus.Passengers {
			if p.TargetStopID == 0 {
				t.Fatalf("Trip %d: Found passenger on board with TargetStopID == 0 right after departing terminal!", trip)
			}
		}
	}
}

func TestRepeatedDockingAtSameStopBoardsPassengers(t *testing.T) {
	g := NewGame(1280, 720)
	g.Bus.ClearPassengers()
	g.Bus.Capacity = 45

	s0 := g.World.Stops[0]
	g.Bus.Pos = s0.Pos
	g.Bus.Heading = s0.Heading
	g.Bus.Speed = 0
	s0.WaitingPassengers = 15

	// 1. First docking: open doors -> passengers board
	g.Bus.DoorsOpen = true
	g.boardedThisStop = false
	g.updateDriverMode(0.016)

	if g.Bus.PassengerCount != 15 {
		t.Fatalf("Expected 15 passengers on board after first docking, got %d", g.Bus.PassengerCount)
	}
	if s0.WaitingPassengers != 0 {
		t.Fatalf("Expected 0 passengers waiting after boarding, got %d", s0.WaitingPassengers)
	}

	// 2. Close doors at the same station
	g.Bus.DoorsOpen = false
	g.updateDriverMode(0.016)

	// More passengers arrive while parked at the station (e.g. 10 waiting)
	s0.WaitingPassengers = 10

	// Reopen doors at the SAME station -> all 10 must board!
	g.Bus.DoorsOpen = true
	g.updateDriverMode(0.016)

	if g.Bus.PassengerCount != 25 {
		t.Fatalf("Expected 25 passengers on board after reopening doors at same stop, got %d", g.Bus.PassengerCount)
	}
	if s0.WaitingPassengers != 0 {
		t.Fatalf("Expected 0 passengers waiting after second boarding, got %d", s0.WaitingPassengers)
	}

	// 3. Keep doors open, new passengers arrive while parked
	s0.WaitingPassengers = 8
	g.updateDriverMode(0.016)

	if g.Bus.PassengerCount != 33 {
		t.Fatalf("Expected 33 passengers on board after continuous open door boarding, got %d", g.Bus.PassengerCount)
	}
	if s0.WaitingPassengers != 0 {
		t.Fatalf("Expected 0 passengers waiting, got %d", s0.WaitingPassengers)
	}
}

func TestOnlyPlannedRouteStopsGeneratePassengersAndDestinationsStrictlyOnRoute(t *testing.T) {
	g := NewGame(1280, 720)

	// Replan route to only include stops [0, 2, 4, 0]
	g.ReplanRoute([]int{0, 2, 4, 0})

	// 1. Verify stops on the route have passengers, while stops NOT on the route have 0 passengers
	routeSet := map[int]bool{0: true, 2: true, 4: true}
	for _, s := range g.World.Stops {
		if routeSet[s.ID] {
			if s.WaitingPassengers <= 0 {
				t.Fatalf("Stop #%d on route expected > 0 waiting passengers, got %d", s.ID, s.WaitingPassengers)
			}
		} else {
			if s.WaitingPassengers != 0 {
				t.Fatalf("Stop #%d NOT on route expected 0 waiting passengers, got %d", s.ID, s.WaitingPassengers)
			}
		}
	}

	// 2. Simulate 100 seconds of world updates (passenger generation)
	for i := 0; i < 200; i++ {
		g.World.Update(0.5, g.Route)
	}

	// Non-route stops must strictly remain at 0 passengers!
	for _, s := range g.World.Stops {
		if !routeSet[s.ID] {
			if s.WaitingPassengers != 0 {
				t.Fatalf("Stop #%d NOT on route must NOT spawn passengers, got %d", s.ID, s.WaitingPassengers)
			}
		}
	}

	// 3. Verify boarding passenger destinations are strictly stops that the route reaches
	s0 := g.World.Stops[0]
	g.Bus.Pos = s0.Pos
	g.Bus.Heading = s0.Heading
	g.Bus.Speed = 0
	g.Bus.ClearPassengers()
	g.Bus.DoorsOpen = true
	g.boardedThisStop = false
	s0.WaitingPassengers = 20

	g.updateDriverMode(0.016)

	if g.Bus.PassengerCount != 20 {
		t.Fatalf("Expected 20 passengers on board, got %d", g.Bus.PassengerCount)
	}
	for i, p := range g.Bus.Passengers {
		if !routeSet[p.TargetStopID] {
			t.Fatalf("Passenger #%d target stop %d is NOT on the planned route [0, 2, 4, 0]!", i, p.TargetStopID)
		}
	}
}

func TestStopBoardingToastMessages(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)

	stop := g.World.Stops[0]
	g.Bus.Pos = stop.Pos
	g.Bus.Heading = stop.Heading
	g.Bus.Speed = 0
	g.Bus.DoorsOpen = true

	// Scenario 1: Stop has 0 waiting passengers, bus is not full (passengerCount = 0).
	// Must NOT say "当前车厢已满载！". Must say "本站无乘客上下车！".
	stop.WaitingPassengers = 0
	g.Bus.ClearPassengers()
	g.HUD.Toasts = nil
	g.boardedThisStop = false
	_ = g.Update()

	if len(g.HUD.Toasts) == 0 {
		t.Fatal("Expected toast to be added")
	}
	lastToast := g.HUD.Toasts[len(g.HUD.Toasts)-1].Text
	if strings.Contains(lastToast, "已满载") {
		t.Fatalf("Bus is empty and no passengers boarded, but toast said full! got: %s", lastToast)
	}
	if !strings.Contains(lastToast, "无乘客") {
		t.Fatalf("Expected toast to mention no passengers, got: %s", lastToast)
	}

	// Scenario 2: 2 passengers alight, but 0 passengers waiting, bus is not full.
	// Must say "下客 2 人，本站无乘客上车！", NOT "当前车厢已满载！".
	stop.WaitingPassengers = 0
	g.Bus.ClearPassengers()
	g.Bus.Passengers = append(g.Bus.Passengers, sim.Passenger{TargetStopID: stop.ID}, sim.Passenger{TargetStopID: stop.ID})
	g.Bus.PassengerCount = len(g.Bus.Passengers)
	g.HUD.Toasts = nil
	g.boardedThisStop = false
	_ = g.Update()

	lastToast = g.HUD.Toasts[len(g.HUD.Toasts)-1].Text
	if strings.Contains(lastToast, "已满载") {
		t.Fatalf("Bus has space and no one boarded, but toast said full! got: %s", lastToast)
	}
	if !strings.Contains(lastToast, "下客 2 人") || !strings.Contains(lastToast, "无乘客上车") {
		t.Fatalf("Expected toast to say alighted 2, no boarding, got: %s", lastToast)
	}

	// Scenario 3: Bus is truly at full capacity, and 4 passengers are waiting.
	// Must say "当前车厢已满载，无法上客！".
	stop.WaitingPassengers = 4
	g.Bus.ClearPassengers()
	for i := 0; i < g.Bus.Capacity; i++ {
		g.Bus.Passengers = append(g.Bus.Passengers, sim.Passenger{TargetStopID: 9999})
	}
	g.Bus.PassengerCount = len(g.Bus.Passengers)
	g.HUD.Toasts = nil
	g.boardedThisStop = false
	_ = g.Update()

	lastToast = g.HUD.Toasts[len(g.HUD.Toasts)-1].Text
	if !strings.Contains(lastToast, "已满载") || !strings.Contains(lastToast, "无法上客") {
		t.Fatalf("Expected toast to say full capacity cannot board, got: %s", lastToast)
	}

	// Scenario 4: Normal boarding (bus has space, 3 passengers waiting).
	// Must say "上客 3 人，营收 +$9！".
	stop.WaitingPassengers = 3
	g.Bus.ClearPassengers()
	g.HUD.Toasts = nil
	g.boardedThisStop = false
	_ = g.Update()

	lastToast = g.HUD.Toasts[len(g.HUD.Toasts)-1].Text
	if !strings.Contains(lastToast, "上客 3 人") || !strings.Contains(lastToast, "营收 +$9") {
		t.Fatalf("Expected toast to say boarded 3 and revenue, got: %s", lastToast)
	}
}
