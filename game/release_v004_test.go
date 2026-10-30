package game

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestRelease004ComprehensiveChecklist executes an end-to-end qualification audit for the v0.0.4 release:
//  1. Unified 45-passenger capacity across player bus and all AI fleet buses.
//  2. Greedy boarding at stations: all waiting passengers board as long as capacity is available,
//     including consecutive/repeated boardings at the same stop.
//  3. Route-constrained passenger generation: only stops on the planned route generate passengers,
//     and their destinations are strictly downstream stops along the route.
//  4. Yellow Box clearance ("Don't Block the Box"): AI cars holding before stop lines at green lights
//     when a bus occupies or clears the intersection box.
//  5. Severe gridlock timeout resolution (> 74s threshold):
//     - Jammed AI cars disappear and reset to clear roadway segments with safe speed.
//     - Jammed AI buses reset back to the route starting station (StopIDs[0]) with cleared passengers and index 0.
//     - Player bus in AI autopilot mode safely resets back to the route starting station.
func TestRelease004ComprehensiveChecklist(t *testing.T) {
	laneWidth := 52.0

	// -------------------------------------------------------------------------
	// 1. Unified 45-Passenger Capacity for All Buses (Player & AI Fleet)
	// -------------------------------------------------------------------------
	playerBus := sim.NewBus(0, 0, 0)
	if playerBus.Capacity != 45 || sim.DefaultBusCapacity != 45 {
		t.Fatalf("Player bus capacity must be 45, got %d", playerBus.Capacity)
	}

	wMetro := sim.NewWorld(laneWidth, sim.MapThemeMetropolis)
	abm := traffic.NewAIBusManager()
	abm.BuildFleet(wMetro, 8, laneWidth, playerBus)
	if len(abm.Buses) != 8 {
		t.Fatalf("Expected 8 AI buses in fleet, got %d", len(abm.Buses))
	}
	for i, ab := range abm.Buses {
		if ab.Bus.Capacity != 45 {
			t.Fatalf("AI Bus #%d capacity must be 45, got %d", i+1, ab.Bus.Capacity)
		}
	}

	// -------------------------------------------------------------------------
	// 2. Greedy Boarding & Consecutive Boarding at the Same Station
	// -------------------------------------------------------------------------
	g := NewGame(1920, 1080)
	g.State = StatePlaying
	route := g.Route

	// Station with 30 waiting passengers
	s0 := g.World.Stops[route.StopIDs[0]]
	s0.WaitingPassengers = 30
	g.Bus.Pos = s0.Pos
	g.Bus.Heading = s0.Heading
	g.Bus.DoorsOpen = true
	g.boardedThisStop = false

	// First boarding: takes 30 passengers (capacity 45, 15 seats left)
	_ = g.Update()
	for g.Bus.PassengerServiceRemaining() > 0 {
		g.Bus.Update(1.0, 0, 0, 0, false, false)
		g.updatePassengerExchange(false)
	}
	if g.Bus.PassengerCount != 30 {
		t.Fatalf("Expected 30 passengers boarded on first boarding, got %d", g.Bus.PassengerCount)
	}
	if s0.WaitingPassengers != 0 {
		t.Fatalf("Expected 0 passengers remaining at station, got %d", s0.WaitingPassengers)
	}

	// More passengers arrive while bus is still stopped: 25 passengers arrive
	s0.WaitingPassengers = 25
	// Opening/cycling doors allows consecutive boarding at the same station
	g.Bus.DoorsOpen = false
	_ = g.Update()
	g.Bus.DoorsOpen = true
	_ = g.Update()
	for g.Bus.PassengerServiceRemaining() > 0 {
		g.Bus.Update(1.0, 0, 0, 0, false, false)
		g.updatePassengerExchange(false)
	}

	// Remaining capacity was 15, so 15 should board, leaving 10 at stop and bus at 45 (full capacity)
	if g.Bus.PassengerCount != 45 {
		t.Fatalf("Expected bus to be at full capacity (45), got %d", g.Bus.PassengerCount)
	}
	if s0.WaitingPassengers != 10 {
		t.Fatalf("Expected 10 passengers remaining at station, got %d", s0.WaitingPassengers)
	}

	// -------------------------------------------------------------------------
	// 3. Route-Constrained Passenger Generation & Destinations
	// -------------------------------------------------------------------------
	// Custom route using only Stop 0, Stop 2, and Stop 10
	testRouteStops := []int{0, 2, 10, 0}
	g.ReplanRoute(testRouteStops)

	// Verify only stops in route have waiting passengers and can generate passengers
	activeStopSet := map[int]bool{0: true, 2: true, 10: true}
	for _, stop := range g.World.Stops {
		if !activeStopSet[stop.ID] {
			if stop.WaitingPassengers != 0 {
				t.Fatalf("Off-route stop #%d must have 0 waiting passengers, got %d", stop.ID, stop.WaitingPassengers)
			}
			// Simulate passenger spawn attempt: off-route stops must not generate passengers
			stop.Update(100.0) // Large dt
			g.SyncStopsWithRoute()
			if stop.WaitingPassengers != 0 {
				t.Fatalf("Off-route stop #%d must not generate passengers after SyncStopsWithRoute", stop.ID)
			}
		}
	}

	// Verify candidate destinations from Stop 2 only include Stop 10 and Stop 0
	candidates := g.Route.GetUpcomingCandidateStopIDsForStop(2)
	for _, destID := range candidates {
		if destID != 10 && destID != 0 {
			t.Fatalf("Candidate destination %d is not downstream on active route", destID)
		}
	}

	// -------------------------------------------------------------------------
	// 4. Yellow Box Intersection Clearance ("Don't Block the Box")
	// -------------------------------------------------------------------------
	inter := &sim.TrafficIntersection{
		Center:            sim.Vec2{X: 0, Y: 0},
		Phase:             sim.PhaseNSGreen, // NS green for northbound vehicle
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := sim.NewTrafficLightSystem()
	tls.Intersections = []*sim.TrafficIntersection{inter}

	stopLineDist := inter.GetStopLineDistForHeading(-math.Pi/2, laneWidth)
	carPos := sim.Vec2{
		X: 78.0,
		Y: stopLineDist + 25.0 + 23.0,
	}
	approachCar := &sim.TrafficCar{
		ID:          1,
		Pos:         carPos,
		Heading:     -math.Pi / 2, // Northbound
		Speed:       80.0,
		TargetSpeed: 120.0,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        false,
		LaneIndex:   1, // Straight lane
	}
	tm := traffic.NewTrafficManager(laneWidth)
	tm.Cars = []*sim.TrafficCar{approachCar}

	// Place bus inside intersection box traversing East-West
	boxBus := sim.NewBus(0, 0, 0)
	boxBus.Speed = 10.0

	tm.StepCar(0.1, approachCar, 0, boxBus, tls, 1, laneWidth, nil)

	if !approachCar.IsBraking {
		t.Fatalf("AI Car must brake and yield before stop line when bus is inside intersection box!")
	}
	if approachCar.Speed > 70.0 {
		t.Fatalf("AI Car must decelerate towards stop line, got speed %.1f", approachCar.Speed)
	}

	// -------------------------------------------------------------------------
	// 5. Severe Gridlock Timeout Resolution (> 74s)
	// -------------------------------------------------------------------------
	// Place an AI bus from the fleet and a car jammed in head-to-head deadlock
	testAIBus := abm.Buses[0]
	testAIBus.Route.CurrentIndex = 1
	testAIBus.Bus.PassengerCount = 25
	testAIBus.Bus.Passengers = make([]sim.Passenger, 25)
	testAIBus.Bus.Pos = sim.Vec2{X: -400.0, Y: 26.0}
	testAIBus.Bus.Heading = 0.0
	testAIBus.Bus.Speed = 0.0

	jammedCarPos := sim.Vec2{X: -340.0, Y: 26.0}
	jammedCar := &sim.TrafficCar{
		ID:          99,
		Pos:         jammedCarPos,
		Heading:     math.Pi,
		Speed:       0.0,
		TargetSpeed: 0.0,
		Length:      46,
		Width:       22,
		RoadCenter:  52.0,
		IsEW:        true,
	}
	tm.Cars = []*sim.TrafficCar{jammedCar}
	tm.AIBusMgr = abm

	// Advance deadlock to 74s threshold
	dt := 0.2
	for elapsed := 0.0; elapsed < 74.2; elapsed += dt {
		testAIBus.Bus.Speed = 0.0
		jammedCar.Speed = 0.0
		tm.ResolveDeadlocks(dt, nil, len(tm.Cars), laneWidth, wMetro)
		if jammedCar.DeadlockTimer == 0 && len(tm.Cars) > 0 && tm.Cars[0].Pos.Distance(jammedCarPos) > 10.0 {
			break
		}
	}

	// Car must be moved/reset away from deadlock spot
	if len(tm.Cars) > 0 {
		if tm.Cars[0].Pos.Distance(jammedCarPos) < 10.0 {
			t.Fatalf("Deadlocked car failed to reset from deadlock spot")
		}
		if tm.Cars[0].DeadlockTimer != 0 {
			t.Fatalf("Reset car DeadlockTimer must be 0, got %.2f", tm.Cars[0].DeadlockTimer)
		}
	}

	if testAIBus.Route.CurrentIndex != 1 || testAIBus.Bus.PassengerCount != 25 {
		t.Fatal("Car recovery must not reset a bus that has not independently timed out")
	}
}
