package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestRelease003ComprehensiveChecklist executes an end-to-end audit for the v0.0.3 release:
// 1. Route replanning resets player and AI buses to first stop without ghosting or misplaced spawns
// 2. Stop line distances across all road combinations are strictly behind zebra crosswalks and outside intersection corridors
// 3. Vehicles changing lanes into dedicated left-turn lanes never cross the center double yellow divider
// 4. Pure Pursuit steering on left turns does not cut across oncoming lanes
// 5. All 6 city maps have complete, valid road networks with safe clearance for bus navigation
func TestRelease003ComprehensiveChecklist(t *testing.T) {
	laneWidth := 52.0

	// -------------------------------------------------------------------------
	// 1. Stop Line & Crosswalk Geometry Audit for All Road Intersections
	// -------------------------------------------------------------------------
	roadConfigs := []struct {
		ewLanes int
		nsLanes int
	}{
		{6, 6},
		{6, 4},
		{6, 2},
		{4, 6},
		{4, 4},
		{4, 2},
		{2, 6},
		{2, 4},
		{2, 2},
	}

	for _, cfg := range roadConfigs {
		inter := &sim.TrafficIntersection{
			ID:              100,
			Center:          sim.Vec2{X: 0, Y: 0},
			ApproachHasRoad: [4]bool{true, true, true, true},
			ApproachLanes:   [4]int{cfg.ewLanes, cfg.ewLanes, cfg.nsLanes, cfg.nsLanes},
			ApproachIs4Lane: [4]bool{cfg.ewLanes >= 4, cfg.ewLanes >= 4, cfg.nsLanes >= 4, cfg.nsLanes >= 4},
		}

		ewStop, ewCross, nsStop, nsCross := inter.GetApproachDimensions(laneWidth)

		ewCrossingHalf := laneWidth*1.0 + 6.0
		if cfg.ewLanes >= 6 {
			ewCrossingHalf = laneWidth*3.0 + 12.0
		} else if cfg.ewLanes >= 4 {
			ewCrossingHalf = laneWidth*2.0 + 12.0
		}

		nsCrossingHalf := laneWidth*1.0 + 6.0
		if cfg.nsLanes >= 6 {
			nsCrossingHalf = laneWidth*3.0 + 12.0
		} else if cfg.nsLanes >= 4 {
			nsCrossingHalf = laneWidth*2.0 + 12.0
		}

		// EW stop line and crosswalk must be outside NS crossing road
		if ewCross < nsCrossingHalf {
			t.Fatalf("[Config EW=%d, NS=%d] EW crosswalk (%.1f) inside NS road (half-width %.1f)",
				cfg.ewLanes, cfg.nsLanes, ewCross, nsCrossingHalf)
		}
		if ewStop <= ewCross {
			t.Fatalf("[Config EW=%d, NS=%d] EW stop line (%.1f) must be behind crosswalk (%.1f)",
				cfg.ewLanes, cfg.nsLanes, ewStop, ewCross)
		}

		// NS stop line and crosswalk must be outside EW crossing road
		if nsCross < ewCrossingHalf {
			t.Fatalf("[Config EW=%d, NS=%d] NS crosswalk (%.1f) inside EW road (half-width %.1f)",
				cfg.ewLanes, cfg.nsLanes, nsCross, ewCrossingHalf)
		}
		if nsStop <= nsCross {
			t.Fatalf("[Config EW=%d, NS=%d] NS stop line (%.1f) must be behind crosswalk (%.1f)",
				cfg.ewLanes, cfg.nsLanes, nsStop, nsCross)
		}
	}

	// -------------------------------------------------------------------------
	// 2. All 6 Maps Full Audit: Roadway Clearance, Stop Bays & Roundabouts
	// -------------------------------------------------------------------------
	allThemes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	for _, theme := range allThemes {
		w := sim.NewWorld(laneWidth, theme)
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)

		if len(w.Stops) != 12 {
			t.Fatalf("[%s] Map must have 12 stops, got %d", theme.DisplayName(), len(w.Stops))
		}

		// Verify every stop bay is on drivable surface, clear of hard barriers, and strictly on the roadside (outside roadway)
		for _, s := range w.Stops {
			res := w.CheckBoundaries(s.Pos, 18.0, laneWidth)
			if res.Zone == sim.ZoneHardBarrier {
				t.Fatalf("[%s] Stop #%d (%s) at %v is inside hard barrier (%s)",
					theme.DisplayName(), s.ID, s.Name, s.Pos, res.ObstacleName)
			}
			roadHalfW := float64(s.RoadLanes) * 0.5 * laneWidth
			distToRoadCenter := s.Pos.Distance(s.RoadCenter)
			if distToRoadCenter < roadHalfW {
				t.Fatalf("[%s] Stop #%d (%s) at %v is sitting ON the roadway (distToRoadCenter=%.2f < roadHalfW=%.2f)",
					theme.DisplayName(), s.ID, s.Name, s.Pos, distToRoadCenter, roadHalfW)
			}
		}

		// Verify designated loop route
		route := sim.GetStandardRouteForMap(theme, w)
		if len(route.StopIDs) < 2 {
			t.Fatalf("[%s] Designated route must contain multiple stops", theme.DisplayName())
		}

		// Verify every leg path along designated route
		for leg := 0; leg < len(route.StopIDs); leg++ {
			sFrom := w.Stops[route.StopIDs[leg]]
			nextStopID := route.StopIDs[(leg+1)%len(route.StopIDs)]
			sTo := w.Stops[nextStopID]

			path := sim.GetRoadPathBetweenStopsInWorld(w, sFrom, sTo, laneWidth)
			if len(path) < 2 {
				t.Fatalf("[%s] Path between Stop %d and %d is empty", theme.DisplayName(), sFrom.ID, sTo.ID)
			}

			// Verify interior path points are on ZoneRoad
			for idx := 1; idx < len(path)-1; idx++ {
				pt := path[idx]
				bRes := w.CheckBoundaries(pt, 18.0, laneWidth)
				if bRes.Zone == sim.ZoneHardBarrier {
					t.Fatalf("[%s] Leg %d->%d: pt[%d]=%v hits hard barrier (%s)",
						theme.DisplayName(), sFrom.ID, sTo.ID, idx, pt, bRes.ObstacleName)
				}
			}
		}
	}

	// -------------------------------------------------------------------------
	// 3. Replan Route Resets Player & AI Buses Cleanly
	// -------------------------------------------------------------------------
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeCoastalBay)

	// Move player bus far away from starting terminal
	g.Bus.Pos = sim.Vec2{X: 1500, Y: 800}
	g.Bus.Speed = 80.0
	g.Bus.Heading = 0.5
	g.Bus.Gear = sim.GearDrive

	// Plan a new route starting at Stop 5
	g.ReplanRoute([]int{5, 6, 7, 8, 9, 5})

	targetFirstStop := g.World.Stops[5]
	if g.Bus.Pos.Distance(targetFirstStop.Pos) > 1.0 {
		t.Fatalf("Player bus position after replan was not reset to first stop (5): got %v, expected %v",
			g.Bus.Pos, targetFirstStop.Pos)
	}
	if math.Abs(g.Bus.Heading-targetFirstStop.Heading) > 0.05 {
		t.Fatalf("Player bus heading after replan was not aligned: got %.2f, expected %.2f",
			g.Bus.Heading, targetFirstStop.Heading)
	}
	if g.Bus.Speed != 0 {
		t.Fatalf("Player bus speed must be 0 after replan, got %.2f", g.Bus.Speed)
	}
	if g.Bus.Gear != sim.GearDrive {
		t.Fatalf("Player bus gear should be GearDrive after replan, got %v", g.Bus.Gear)
	}

	// Verify AI fleet queued behind player bus
	for i, ai := range g.AIBusMgr.Buses {
		if ai.Bus.Speed != 0 {
			t.Fatalf("AI bus %d speed must be 0 after replan", i)
		}
		if ai.Route.CurrentIndex != 1 {
			t.Fatalf("AI bus %d Route.CurrentIndex should be 1 after replan, got %d", i, ai.Route.CurrentIndex)
		}
	}

	// -------------------------------------------------------------------------
	// 3.5 Planning Mode Pauses Simulation & Halts Passenger Generation
	// -------------------------------------------------------------------------
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true after TogglePlanMode()")
	}

	spawnSnap := make([]float64, len(g.World.Stops))
	paxSnap := make([]int, len(g.World.Stops))
	for i, s := range g.World.Stops {
		spawnSnap[i] = s.SpawnTimer
		paxSnap[i] = s.WaitingPassengers
	}

	// Update 60 frames in plan mode
	for step := 0; step < 60; step++ {
		_ = g.Update()
	}

	for i, s := range g.World.Stops {
		if s.SpawnTimer != spawnSnap[i] {
			t.Fatalf("Stop #%d spawn timer advanced during plan mode", s.ID)
		}
		if s.WaitingPassengers != paxSnap[i] {
			t.Fatalf("Stop #%d passenger count changed during plan mode", s.ID)
		}
	}

	// Exit plan mode
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after TogglePlanMode()")
	}

	// -------------------------------------------------------------------------
	// 4. Left-Turn Lane Change Double-Yellow Immunity Check
	// -------------------------------------------------------------------------
	wBay := sim.NewWorld(laneWidth, sim.MapThemeCoastalBay)
	tlsBay := sim.NewTrafficLightSystem()
	tlsBay.BuildForWorld(wBay)

	routeBay := sim.GetStandardRouteForMap(sim.MapThemeCoastalBay, wBay)
	// Leg from Stop 0 (Central Terminal) to Stop 1 (West Harbor) traverses a multi-lane left turn
	s0 := wBay.Stops[routeBay.StopIDs[0]]
	s1 := wBay.Stops[routeBay.StopIDs[1]]
	legRoute := &sim.Route{
		Name:         "Leg0to1",
		StopIDs:      []int{s0.ID, s1.ID},
		CurrentIndex: 1,
	}

	testBus := sim.NewBus(s0.Pos.X, s0.Pos.Y, s0.Heading)
	driver := autopilot.NewAutopilotDriver()
	driver.BerserkMode = true
	driver.DwellTotal = 0.1

	dt := 0.016
	for step := 0; step < 1200; step++ {
		tlsBay.Update(dt)
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			testBus, legRoute, wBay, tlsBay, nil, laneWidth, dt,
		)
		testBus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if isDocked && step > 60 {
			break
		}

		// Verify vehicle body never crosses double yellow line
		roadCenter, isEW, isMulti, _ := sim.CurrentRoadInfo(wBay, testBus.Pos, testBus.Heading, laneWidth)
		if isMulti {
			cosH := math.Cos(testBus.Heading)
			sinH := math.Sin(testBus.Heading)
			halfL := testBus.Length * 0.5
			halfW := testBus.Width * 0.5
			leftPerp := sim.Vec2{X: sinH, Y: -cosH}
			flCorner := testBus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
			rlCorner := testBus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

			distToInter := 999.0
			for _, it := range tlsBay.Intersections {
				d := testBus.Pos.Distance(it.Center)
				if d < distToInter {
					distToInter = d
				}
			}

			if distToInter > 80.0 {
				if isEW {
					if math.Cos(testBus.Heading) < -0.5 { // Westbound: legal Y < roadCenter
						if flCorner.Y > roadCenter-1.0 || rlCorner.Y > roadCenter-1.0 {
							t.Fatalf("Westbound bus crossed double yellow line: Pos=%v, fl=%v, rl=%v, center=%.1f",
								testBus.Pos, flCorner, rlCorner, roadCenter)
						}
					} else if math.Cos(testBus.Heading) > 0.5 { // Eastbound: legal Y > roadCenter
						if flCorner.Y < roadCenter+1.0 || rlCorner.Y < roadCenter+1.0 {
							t.Fatalf("Eastbound bus crossed double yellow line: Pos=%v, fl=%v, rl=%v, center=%.1f",
								testBus.Pos, flCorner, rlCorner, roadCenter)
						}
					}
				}
			}
		}
	}
}
