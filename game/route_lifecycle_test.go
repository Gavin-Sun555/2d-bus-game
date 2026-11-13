package game

import (
	"math"
	"slices"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

func TestRoutePlanningRejectsMoreThanTwoUTurnsIncludingReturn(t *testing.T) {
	g := NewGame(1280, 720)
	g.World = &sim.World{
		Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 4}},
		Stops: []*sim.BusStop{
			{ID: 0, Pos: sim.Vec2{Y: 90}, Heading: 0},
			{ID: 1, Pos: sim.Vec2{Y: -90}, Heading: math.Pi},
		},
	}
	allowed := []int{0, 1}
	overLimit := []int{0, 1, 0, 1}
	if got := sim.RouteUTurnCount(g.World, allowed, 52); got != 2 {
		t.Fatalf("out and back route has %d turns, want 2", got)
	}
	g.Route.StopIDs = slices.Clone(allowed)
	g.ReplanRoute(overLimit)
	if !slices.Equal(g.Route.StopIDs, allowed) {
		t.Fatal("programmatic replan replaced a route with more than two U-turns")
	}
	g.IsPlanMode = true
	g.rememberPlannedRoute(g.Route)
	g.Route.StopIDs = overLimit
	if g.finishPlannedRoute() || !slices.Equal(g.operatingLines()[0].StopIDs, allowed) {
		t.Fatal("planner committed an over-limit draft")
	}
}

func TestReplanningPreservesOtherServicesAndReachablePassengers(t *testing.T) {
	for _, edited := range []int{0, 1} {
		t.Run([]string{"player line", "AI line"}[edited], func(t *testing.T) {
			g := NewGame(1280, 720)
			g.SetTransitLineCount(2)
			original := []int{0, 1, 2, 3, 0}
			other := []int{0, 2, 4, 5, 0}
			stops := [2][]int{}
			stops[edited], stops[1-edited] = original, other
			g.ReplanRoute(stops[0])
			g.Lines[1].StopIDs = stops[1]
			for i := 0; i < 2; i++ {
				g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 1})
			}
			fleet := [2]*traffic.AIBus{}
			for _, bus := range g.AIBusMgr.Buses {
				fleet[bus.LineIndex] = bus
			}
			changed, retained := fleet[edited], fleet[1-edited]
			retained.Bus.Speed = 50
			retained.Route.CurrentIndex = 2
			retained.Route.TotalTrips = 3
			g.trackOperations(1)
			ops := g.Operations
			_, retainedFare := ops.BoardWithDestinations(retained.ID, retained.Bus, retained.Route, 0, []int{2})
			ops.BoardWithDestinations(changed.ID, changed.Bus, changed.Route, 0, []int{3})
			record := ops.Vehicle(retained.ID)
			record.CompletedLaps = 2
			playerPos := g.Bus.Pos

			stop := g.World.Stops[0]
			stop.WaitingPassengers = 5
			stop.WaitingDestinations = []int{1, 2, 3, 4, -1}
			stop.WaitingSince = []float64{1, 2, 3, 4, 5}
			stop.SpawnTimer = .5
			removed := g.World.Stops[1]
			removed.SeedWaitingPassengers(3)
			removed.SpawnTimer = .75
			removed.SpawnInterval = .25
			reordered := g.World.Stops[2]
			reordered.WaitingPassengers = 2
			reordered.WaitingDestinations = []int{3, 4}
			reordered.WaitingSince = []float64{6, 7}
			empty := g.World.Stops[3]
			empty.WaitingPassengers = 0
			empty.SyncWaitingQueue(nil)

			g.PlanTransitLine(edited)
			g.ClearPlannedRoute()
			for _, id := range []int{0, 3, 2, 0} {
				g.planningRoute().AddStop(id)
			}
			// Settings and the statistics panel can refresh during planning.
			g.buildLineFleets(false)
			g.ensureOperations()
			if g.Operations != ops || g.Operations.Vehicles[changed.ID] == nil {
				t.Fatal("draft discarded the running service's statistics")
			}
			g.TogglePlanMode()
			g.ensureOperations()

			if g.Operations != ops || ops.Vehicles[retained.ID] != record || record.CompletedLaps != 2 ||
				ops.Vehicles[changed.ID] != nil || ops.Revenue != retainedFare || ops.Stations[0].Boarded != 1 {
				t.Fatal("replanning reset unrelated statistics or kept old statistics for the edited line")
			}
			for _, bus := range g.AIBusMgr.Buses {
				if bus.LineIndex == edited && (bus == changed || !slices.Equal(bus.Route.StopIDs, []int{0, 3, 2, 0})) {
					t.Fatal("edited service was not dispatched on its new route")
				}
				if bus.LineIndex != edited && (bus != retained || bus.Bus.Speed != 50 || bus.Bus.PassengerCount != 1 ||
					bus.Route.CurrentIndex != 2 || bus.Route.TotalTrips != 3) {
					t.Fatal("replanning disturbed an unrelated bus")
				}
			}
			if edited == 1 && g.Bus.Pos != playerPos {
				t.Fatal("replanning an AI line moved the player")
			}
			if stop.WaitingPassengers != 4 || !slices.Equal(stop.WaitingDestinations, []int{2, 3, 4, -1}) ||
				!slices.Equal(stop.WaitingSince, []float64{2, 3, 4, 5}) || stop.SpawnTimer != .5 {
				t.Fatal("shared stop did not preserve the reachable queue and its timing")
			}
			if reordered.WaitingPassengers != 1 || !slices.Equal(reordered.WaitingDestinations, []int{4}) ||
				!slices.Equal(reordered.WaitingSince, []float64{7}) || empty.WaitingPassengers != 0 {
				t.Fatal("the route reorder reassigned passengers or reseeded an empty stop")
			}
			g.World.Update(1, g.activeLines()...)
			if removed.WaitingPassengers != 0 || len(removed.WaitingDestinations) != 0 || len(removed.WaitingSince) != 0 || removed.SpawnTimer != 0 {
				t.Fatal("unserved stop retained or generated passengers")
			}
		})
	}
}

func TestRestoringOriginalPlanPreservesCursorAcrossSave(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.savePathOverride = t.TempDir() + "/save.json"
	g.Route.CurrentIndex = 2
	g.Route.TotalTrips = 3
	g.trackOperations(1)
	pos := g.Bus.Pos
	original := slices.Clone(g.Route.StopIDs)
	g.TogglePlanMode()
	g.ClearPlannedRoute()
	for _, id := range original {
		g.planningRoute().AddStop(id)
	}
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	ops := g.Operations
	g.TogglePlanMode()
	if g.IsPlanMode || g.Route.CurrentIndex != 2 || g.Route.TotalTrips != 3 ||
		g.Operations != ops || ops.Vehicle(0).Seconds != 1 || g.Bus.Pos != pos {
		t.Fatal("committing the original itinerary reset an active journey")
	}
}

func TestReplanRouteResetsPlayerAndDistributesAIBuses(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)

	laneW := g.Settings.GetLaneWidth()
	g.Settings.AIBusCount = 2
	g.AIBusMgr.BuildFleet(g.World, 2, laneW, g.Bus)

	if len(g.AIBusMgr.Buses) != 2 {
		t.Fatalf("Expected 2 AI buses, got %d", len(g.AIBusMgr.Buses))
	}

	// 1. Move player bus away and load passengers to simulate active driving
	g.Bus.Pos = sim.Vec2{X: 800, Y: 400}
	g.Bus.Heading = 1.2
	g.Bus.Speed = 160.0
	g.Bus.DoorsOpen = false
	g.Bus.BoardPassengers(5, []int{2, 10})
	if g.Bus.PassengerCount != 5 {
		t.Fatalf("Expected 5 passengers, got %d", g.Bus.PassengerCount)
	}

	// 2. Enter Plan Mode and exit WITHOUT modifying route: player vehicle must NOT reset
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true after TogglePlanMode")
	}
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after TogglePlanMode")
	}
	if g.Bus.Pos.Distance(sim.Vec2{X: 800, Y: 400}) > 1e-3 || g.Bus.Speed != 160.0 || g.Bus.PassengerCount != 5 {
		t.Fatalf("Player vehicle state should not reset if route was not modified: pos=%+v speed=%f pax=%d",
			g.Bus.Pos, g.Bus.Speed, g.Bus.PassengerCount)
	}

	// 3. Enter Plan Mode and REPLAN route to start at Stop 3 ([3, 7, 2, 3])
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true")
	}
	g.Route.Clear()
	g.Route.AddStop(3)
	g.Route.AddStop(7)
	g.Route.AddStop(2)
	g.Route.AddStop(3)
	g.planModeRouteModified = true

	// Exit Plan Mode: player returns to Stop 3; AI buses spread along the new route.
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after exiting plan mode")
	}

	stop3 := g.World.Stops[3]
	if g.Bus.Pos.Distance(stop3.FrontBerthPosition()) > 1e-3 {
		t.Fatalf("Player bus position was not reset to first stop: expected %+v, got %+v", stop3.FrontBerthPosition(), g.Bus.Pos)
	}
	if math.Abs(sim.NormalizeAngle(g.Bus.Heading-stop3.Heading)) > 1e-4 {
		t.Fatalf("Player bus heading was not reset: expected %f, got %f", stop3.Heading, g.Bus.Heading)
	}
	if g.Bus.Speed != 0 {
		t.Fatalf("Player bus speed must be 0 after reset, got %f", g.Bus.Speed)
	}
	if g.Bus.PassengerCount != 0 {
		t.Fatalf("Player bus passengers must be cleared for new route, got %d", g.Bus.PassengerCount)
	}
	if !g.Bus.DoorsOpen {
		t.Fatal("Player bus doors should be open at initial station bay")
	}
	if g.Route.GetCurrentTargetStopID() != 3 {
		t.Fatalf("Active target stop must be first stop 3, got %d", g.Route.GetCurrentTargetStopID())
	}

	// Replanning keeps the player at Stop 3 and redistributes the new AI fleet.
	if len(g.AIBusMgr.Buses) != 2 {
		t.Fatalf("Expected 2 AI buses, got %d", len(g.AIBusMgr.Buses))
	}
	for i, ai := range g.AIBusMgr.Buses {
		if ai.Bus.Speed != 0 || ai.Bus.Pos.Distance(g.Bus.Pos) < 120 {
			t.Fatalf("AI bus %d was not safely distributed", i)
		}
		if len(ai.Route.StopIDs) != 4 || ai.Route.StopIDs[0] != 3 || ai.Route.StopIDs[1] != 7 {
			t.Fatalf("AI bus %d itinerary was not updated: %v", i, ai.Route.StopIDs)
		}
		if ai.Route.CurrentIndex <= 0 || ai.Route.CurrentIndex >= len(ai.Route.StopIDs) {
			t.Fatalf("AI bus %d has invalid distributed target %d", i, ai.Route.CurrentIndex)
		}
	}

	// 4. Test empty route validation: cannot exit plan mode if route is empty
	g.TogglePlanMode()
	g.Route.Clear()
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Game should not exit Plan Mode when route is empty")
	}

	// 5. Test programmatic ReplanRoute helper
	g.ReplanRoute([]int{5, 9, 5})
	stop5 := g.World.Stops[5]
	if g.Bus.Pos.Distance(stop5.FrontBerthPosition()) > 1e-3 {
		t.Fatalf("ReplanRoute failed to reset player bus to Stop 5: expected %+v, got %+v", stop5.FrontBerthPosition(), g.Bus.Pos)
	}
	if len(g.AIBusMgr.Buses) > 0 {
		if ai := g.AIBusMgr.Buses[0]; ai.Route.CurrentIndex <= 0 || ai.Bus.Pos.Distance(g.Bus.Pos) < 120 {
			t.Fatal("ReplanRoute failed to distribute the AI fleet")
		}
	}

	// 6. Test HandleModeBtnClick in HUD
	if !g.HUD.HandleModeBtnClick(sim.Vec2{X: 100, Y: 25}) {
		t.Fatal("Expected HandleModeBtnClick to return true for badge coordinates (100, 25)")
	}
	if g.HUD.HandleModeBtnClick(sim.Vec2{X: 500, Y: 25}) {
		t.Fatal("Expected HandleModeBtnClick to return false for outside coordinates (500, 25)")
	}
}

// TestRouteRemoveLastStopAndUndo verifies:
// 1. Route.RemoveLastStop removes the most recently added stop and adjusts CurrentIndex correctly.
// 2. UndoLastPlannedStop properly manages Route, marks planModeRouteModified, and notifies user via HUD.
// 3. ClearPlannedRoute properly clears all stops.
// 4. HUD interactive buttons for Plan Undo and Clear detect clicks correctly.
func TestRouteRemoveLastStopAndUndo(t *testing.T) {
	// 1. Direct Route method testing
	r := sim.NewRoute()
	if _, ok := r.RemoveLastStop(); ok {
		t.Fatal("Expected RemoveLastStop on empty route to return false")
	}

	r.AddStop(0)
	r.AddStop(2)
	r.AddStop(10)
	if len(r.StopIDs) != 3 {
		t.Fatalf("Expected 3 stops, got %d", len(r.StopIDs))
	}

	r.CurrentIndex = 2 // Pointing to stop index 2 (stopID 10)
	removedID, ok := r.RemoveLastStop()
	if !ok || removedID != 10 {
		t.Fatalf("Expected removed stopID 10, got %d (ok=%v)", removedID, ok)
	}
	if len(r.StopIDs) != 2 || r.StopIDs[0] != 0 || r.StopIDs[1] != 2 {
		t.Fatalf("Expected stops [0, 2], got %v", r.StopIDs)
	}
	if r.CurrentIndex != 1 {
		t.Fatalf("Expected CurrentIndex clamped to 1, got %d", r.CurrentIndex)
	}

	// Remove remaining stops down to 0
	r.RemoveLastStop()
	r.RemoveLastStop()
	if len(r.StopIDs) != 0 || r.CurrentIndex != 0 {
		t.Fatalf("Expected 0 stops and CurrentIndex=0, got len=%d, idx=%d", len(r.StopIDs), r.CurrentIndex)
	}

	// 2. Game level UndoLastPlannedStop & ClearPlannedRoute testing
	g := NewGame(1280, 720)
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected to enter Plan Mode")
	}

	g.Route.Clear()
	g.Route.AddStop(1)
	g.Route.AddStop(3)
	g.Route.AddStop(5)
	g.planModeRouteModified = false

	// Undo stop 5
	success := g.UndoLastPlannedStop()
	if !success {
		t.Fatal("Expected UndoLastPlannedStop to succeed")
	}
	if !g.planModeRouteModified {
		t.Fatal("Expected planModeRouteModified to be true after undo")
	}
	if len(g.Route.StopIDs) != 2 || g.Route.StopIDs[1] != 3 {
		t.Fatalf("Expected stops [1, 3], got %v", g.Route.StopIDs)
	}

	// Undo down to empty
	g.UndoLastPlannedStop()
	g.UndoLastPlannedStop()
	if len(g.Route.StopIDs) != 0 {
		t.Fatalf("Expected route to be empty, got %d stops", len(g.Route.StopIDs))
	}

	// Undo on empty route returns false
	if g.UndoLastPlannedStop() {
		t.Fatal("Expected UndoLastPlannedStop on empty route to return false")
	}

	// 3. ClearPlannedRoute testing
	g.Route.AddStop(1)
	g.Route.AddStop(2)
	g.ClearPlannedRoute()
	if len(g.Route.StopIDs) != 0 {
		t.Fatalf("Expected ClearPlannedRoute to empty route, got %d stops", len(g.Route.StopIDs))
	}

	// 4. HUD click handlers testing
	hud := ui.NewHUD()
	hud.PlanUndoBtnBounds = sim.Rect{X: 100, Y: 200, W: 140, H: 24}
	hud.PlanClearBtnBounds = sim.Rect{X: 250, Y: 200, W: 100, H: 24}
	hud.PlanPanelBounds = sim.Rect{X: 50, Y: 150, W: 400, H: 100}

	if !hud.HandlePlanUndoClick(sim.Vec2{X: 150, Y: 210}) {
		t.Fatal("Expected HandlePlanUndoClick to return true for point inside bounds")
	}
	if hud.HandlePlanUndoClick(sim.Vec2{X: 50, Y: 210}) {
		t.Fatal("Expected HandlePlanUndoClick to return false for point outside bounds")
	}
	if !hud.HandlePlanClearClick(sim.Vec2{X: 280, Y: 210}) {
		t.Fatal("Expected HandlePlanClearClick to return true for point inside bounds")
	}
	if !hud.IsInsidePlanPanel(sim.Vec2{X: 120, Y: 180}) {
		t.Fatal("Expected IsInsidePlanPanel to return true for point inside panel")
	}
}
