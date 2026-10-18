package game

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

func TestReplanRouteResetsPlayerAndAIBusesToFirstStop(t *testing.T) {
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

	// Exit Plan Mode: should trigger reset of player bus and AI fleet to first stop (Stop 3)
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after exiting plan mode")
	}

	stop3 := g.World.Stops[3]
	if g.Bus.Pos.Distance(stop3.Pos) > 1e-3 {
		t.Fatalf("Player bus position was not reset to first stop: expected %+v, got %+v", stop3.Pos, g.Bus.Pos)
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

	// Verify AI buses reset and queued behind player bus at Stop 3
	if len(g.AIBusMgr.Buses) != 2 {
		t.Fatalf("Expected 2 AI buses, got %d", len(g.AIBusMgr.Buses))
	}
	cosH := math.Cos(stop3.Heading)
	sinH := math.Sin(stop3.Heading)
	backVec := sim.Vec2{X: -cosH, Y: -sinH}

	for i, ai := range g.AIBusMgr.Buses {
		if math.Abs(sim.NormalizeAngle(ai.Bus.Heading-stop3.Heading)) > 1e-4 {
			t.Fatalf("AI bus %d heading mismatch: expected %f, got %f", i, stop3.Heading, ai.Bus.Heading)
		}
		if ai.Bus.Speed != 0 {
			t.Fatalf("AI bus %d speed must be 0, got %f", i, ai.Bus.Speed)
		}
		expectedPos := stop3.Pos.Add(backVec.Mul(float64(i+1) * 120.0))
		if ai.Bus.Pos.Distance(expectedPos) > 1.0 {
			t.Fatalf("AI bus %d position mismatch: expected %+v, got %+v", i, expectedPos, ai.Bus.Pos)
		}
		if len(ai.Route.StopIDs) != 4 || ai.Route.StopIDs[0] != 3 || ai.Route.StopIDs[1] != 7 {
			t.Fatalf("AI bus %d route was not updated to replanned route: %v", i, ai.Route.StopIDs)
		}
		if ai.Route.CurrentIndex != 0 {
			t.Fatalf("AI bus %d CurrentIndex must be 0, got %d", i, ai.Route.CurrentIndex)
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
	if g.Bus.Pos.Distance(stop5.Pos) > 1e-3 {
		t.Fatalf("ReplanRoute failed to reset player bus to Stop 5: expected %+v, got %+v", stop5.Pos, g.Bus.Pos)
	}
	if len(g.AIBusMgr.Buses) > 0 {
		expectedAI0Pos := stop5.Pos.Add(sim.Vec2{X: -math.Cos(stop5.Heading), Y: -math.Sin(stop5.Heading)}.Mul(120.0))
		if g.AIBusMgr.Buses[0].Bus.Pos.Distance(expectedAI0Pos) > 1.0 {
			t.Fatalf("ReplanRoute failed to reset AI bus behind Stop 5: expected %+v, got %+v",
				expectedAI0Pos, g.AIBusMgr.Buses[0].Bus.Pos)
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
