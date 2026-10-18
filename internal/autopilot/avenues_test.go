package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestSixLaneRoadGeometryAndNavigation validates 6-lane roads (3 lanes per direction / 6 lanes total):
// 1. Identification via IsRoad6Lane and GetRoadLanes across Metropolis, Coastal Bay, River Valley, and Twin Cities
// 2. Geometry width calculation (float64(Lanes)*laneWidth + 24.0)
// 3. Navigation lane offset: 2.5 * laneWidth on 6-lane roads vs 1.5*lw on 4-lane and 0.5*lw on 2-lane
// 4. Smooth multi-lane transitions from outer cruising lane (2.5*lw) to inner turn lane (0.5*lw)
// 5. Berserk overtaking on 6-lane expressways
func TestSixLaneRoadGeometryAndNavigation(t *testing.T) {
	lw := 52.0

	// 1. Map 1 (Metropolis): North Ring Expressway (Y = -1200 * MapGridScale) is 6-lane
	w1 := sim.NewWorld(lw, sim.MapThemeMetropolis)
	if !w1.IsRoad6Lane(-1200*sim.MapGridScale, true) {
		t.Fatalf("Metropolis North Ring (Y=%.0f) must be 6-lane road", -1200*sim.MapGridScale)
	}
	if lanes := w1.GetRoadLanes(-1200*sim.MapGridScale, true); lanes != 6 {
		t.Fatalf("Expected 6 lanes at Y=%.0f in Metropolis, got %d", -1200*sim.MapGridScale, lanes)
	}
	if off := sim.RoadLaneOffset(-1200*sim.MapGridScale, true, lw, w1); math.Abs(off-lw*2.5) > 0.01 {
		t.Fatalf("Expected 6-lane lane offset 2.5*lw (%.1f), got %.2f", lw*2.5, off)
	}

	// 2. Map 2 (Coastal Bay): Central Grand Boulevard (Y = 0) is 6-lane
	w2 := sim.NewWorld(lw, sim.MapThemeCoastalBay)
	if !w2.IsRoad6Lane(0, true) {
		t.Fatalf("Coastal Bay Central Boulevard (Y=0) must be 6-lane road")
	}
	if lanes := w2.GetRoadLanes(0, true); lanes != 6 {
		t.Fatalf("Expected 6 lanes at Y=0 in Coastal Bay, got %d", lanes)
	}
	if off := sim.RoadLaneOffset(0, true, lw, w2); math.Abs(off-lw*2.5) > 0.01 {
		t.Fatalf("Expected 6-lane lane offset 2.5*lw (%.1f), got %.2f", lw*2.5, off)
	}

	// 3. Map 3 (River Valley): High Speed Rail Boulevard (Y = 1200*MapGridScale) and Century Suspension Bridge (X = 1300*MapGridScale) are 6-lane
	w3 := sim.NewWorld(lw, sim.MapThemeRiverValley)
	if !w3.IsRoad6Lane(1200*sim.MapGridScale, true) {
		t.Fatalf("River Valley HSR Boulevard (Y=%.0f) must be 6-lane road", 1200*sim.MapGridScale)
	}
	if !w3.IsRoad6Lane(1300*sim.MapGridScale, false) {
		t.Fatalf("River Valley Century Suspension Bridge (X=%.0f) must be 6-lane road", 1300*sim.MapGridScale)
	}

	// 4. Map 5 (Twin Cities): Central Elevated Expressway (Y = 0) is 6-lane
	w5 := sim.NewWorld(lw, sim.MapThemeTwinCities)
	if !w5.IsRoad6Lane(0, true) {
		t.Fatalf("Twin Cities Central Viaduct Expressway (Y=0) must be 6-lane road")
	}

	// 5. Road width validation:
	// 6-lane: 6*52 + 24 = 336
	// 4-lane: 4*52 + 24 = 232
	// 2-lane: 2*52 + 12 = 116
	r6 := sim.RoadSegment{Lanes: 6, Type: sim.RoadExpressway6Lane}
	r4 := sim.RoadSegment{Lanes: 4, Type: sim.RoadAvenue4Lane}
	r2 := sim.RoadSegment{Lanes: 2, Type: sim.RoadStreet2Lane}
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
	sStart := sim.NewBusStop(401, "Start6Lane", sim.Vec2{X: 800 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, math.Pi, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	sTurn := sim.NewBusStop(402, "TurnToAvenue", sim.Vec2{X: 0, Y: -600 * sim.MapGridScale}, math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	path := sim.ComputeRoadPathBetweenStops(w1, sStart, sTurn, lw)
	if len(path) == 0 {
		t.Fatalf("Failed to compute left-turn path on 6-lane road")
	}

	var hasOuterCruising bool
	var hasInnerLeftTurn bool
	for _, pt := range path {
		// Westbound outer cruising lane is at Y = -1200*MapGridScale - 2.5*lw
		if math.Abs(pt.Y-(-1200*sim.MapGridScale-lw*2.5)) < 2.0 {
			hasOuterCruising = true
		}
		// Inner left turn lane is at Y = -1200*MapGridScale - 0.5*lw
		if math.Abs(pt.Y-(-1200*sim.MapGridScale-lw*0.5)) < 2.0 {
			hasInnerLeftTurn = true
		}
	}
	if !hasOuterCruising {
		t.Fatalf("Expected path on 6-lane road to include outer cruising lane (Y=%.1f)", -1200*sim.MapGridScale-lw*2.5)
	}
	if !hasInnerLeftTurn {
		t.Fatalf("Expected path on 6-lane road to transition to inner left-turn lane (Y=%.1f)", -1200*sim.MapGridScale-lw*0.5)
	}

	// 7. Berserk mode overtaking on 6-lane road:
	wTest := sim.NewWorld(lw, sim.MapThemeMetropolis)
	s6Start := sim.NewBusStop(501, "RingStart", sim.Vec2{X: -1000 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	s6End := sim.NewBusStop(502, "RingEnd", sim.Vec2{X: 1000 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	wTest.Stops = []*sim.BusStop{s6Start, s6End}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: -1200*sim.MapGridScale + lw*2.5} // In outer cruising lane
	bus.Heading = 0
	bus.Speed = 160.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	route := &sim.Route{
		Name:         "Overtake6LaneRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	slowCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -600*sim.MapGridScale + 90, Y: -1200*sim.MapGridScale + lw*2.5},
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

// TestSixLaneStraightIntersectionChoosesClearLane verifies that on a 6-lane road going straight
// through an intersection:
// 1. If outer straight lane has queued cars and inner straight lane is empty, bus selects inner lane.
// 2. Bus maintains its straight lane through the intersection without forcing a merge to the rightmost lane.
// 3. Traffic lights in inner straight lane obey straight-through signals, not left-arrow signals.
func TestSixLaneStraightIntersectionChoosesClearLane(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// North Ring Avenue (Y = -1200*MapGridScale = -3600) is a 6-lane expressway in Metropolis
	roadY := -1200.0 * sim.MapGridScale
	if !w.IsRoad6Lane(roadY, true) {
		t.Fatalf("Expected Y=%.1f to be a 6-lane road", roadY)
	}

	// Both stops on 6-lane road heading West (math.Pi), traversing intersection at (0, roadY)
	s1 := sim.NewBusStop(0, "SixLaneStart", sim.Vec2{X: 1200 * sim.MapGridScale, Y: roadY}, math.Pi, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(1, "SixLaneEnd", sim.Vec2{X: -1200 * sim.MapGridScale, Y: roadY}, math.Pi, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
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

	bus := sim.NewBus(600, outerLaneY, math.Pi)
	bus.Speed = apBerserkCruiseSpeed * 0.9

	// Run initial step to sync path and waypoints
	driver.ComputeControls(bus, route, w, nil, nil, lw, 0.016)

	// Scenario 1: Outer through lane has queued stopped cars approaching intersection at X=100
	// Inner through lane is completely open!
	queueCar1 := VehicleObstacle{
		Pos:     sim.Vec2{X: 320.0, Y: outerLaneY},
		Heading: math.Pi,
		Speed:   0.0, // Stopped in queue
		Length:  40.0,
		Width:   18.0,
	}
	queueCar2 := VehicleObstacle{
		Pos:     sim.Vec2{X: 380.0, Y: outerLaneY},
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
	inter := &sim.TrafficIntersection{
		ID:              0,
		Center:          sim.Vec2{X: 0, Y: roadY},
		IsMajor:         true,
		ApproachHasRoad: [4]bool{true, true, true, true},
		ApproachLanes:   [4]int{6, 6, 6, 6},
		ApproachIs4Lane: [4]bool{true, true, true, true},
		Phase:           sim.PhaseEWLeftGreen, // Left arrow is Green, Through is Red!
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	ewStop, _, _, _ := inter.GetApproachDimensions(lw)
	// Position bus approaching stop line in inner straight lane (Y = innerLaneY)
	// Bumper approaching stop line from the East (bumper is at X = ewStop + 24.0, stop line is at ewStop)
	bus.Pos = sim.Vec2{X: ewStop + 70.0, Y: innerLaneY}
	bus.Heading = math.Pi
	bus.Speed = 120.0

	_, brakeLight, _, _, _, _ := driver.ComputeControls(bus, route, w, tls, nil, lw, 0.016)
	if brakeLight <= 0.0 {
		t.Fatalf("Bus going straight in inner through lane must stop for Red through light even when left-turn signal is Green")
	}

	// Now switch Through signal to Green, Left signal to Red
	inter.Phase = sim.PhaseEWGreen // Through is Green, Left arrow is Red!

	throttleLight, _, _, _, _, _ := driver.ComputeControls(bus, route, w, tls, nil, lw, 0.016)
	if throttleLight <= 0.0 {
		t.Fatalf("Bus going straight in inner through lane must proceed when through signal is Green even when left-turn signal is Red")
	}
}

// TestBerserkModeRedLightStraightLaneSelection6Lane verifies that on a 6-lane road approaching an intersection,
// when going STRAIGHT and approaching a red light / traffic queue, the bus selects whichever through lane
// has fewer queued vehicles (between outer through lane and middle through lane), and NEVER enters the left-turn lane.
func TestBerserkModeRedLightStraightLaneSelection6Lane(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	// Metropolis North Ring (Y = -1200*MapGridScale = -3600) is 6-lane expressway
	roadCoord := -1200.0 * sim.MapGridScale

	// Find intersection at (0, roadCoord)
	var inter *sim.TrafficIntersection
	for _, in := range tls.Intersections {
		if math.Abs(in.Center.X) < 10.0 && math.Abs(in.Center.Y-roadCoord) < 10.0 {
			inter = in
			break
		}
	}
	if inter == nil {
		t.Fatalf("Could not find 6-lane intersection at (0, %f)", roadCoord)
	}
	inter.Phase = sim.PhaseNSGreen // EW through light is Red!

	// Eastbound straight route on 6-lane road:
	// Base path is in outer through lane (offset 0.0, Y = roadCoord + lw*2.5)
	// Middle through lane is offset -lw (Y = roadCoord + lw*1.5)
	// Dedicated left-turn lane is offset -2*lw (Y = roadCoord + lw*0.5)
	s1 := sim.NewBusStop(801, "SixLaneStart", sim.Vec2{X: -600 * sim.MapGridScale, Y: roadCoord + lw*2.5}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(802, "SixLaneEnd", sim.Vec2{X: 600 * sim.MapGridScale, Y: roadCoord + lw*2.5}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "SixLaneQueueSelectionRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	// Scenario A: Outer through lane has 2 queued cars waiting at red light; middle through lane has 0 cars.
	// Bus should select middle through lane (offset -lw).
	busA := sim.NewBus(1, 0, lw)
	busA.Pos = sim.Vec2{X: -350.0, Y: roadCoord + lw*2.5}
	busA.Heading = 0
	busA.Speed = 160.0
	busA.Gear = sim.GearDrive

	driverA := NewAutopilotDriver()
	driverA.BerserkMode = true
	driverA.State = AutopilotStateCruising

	queuedCarsOuter := []VehicleObstacle{
		{Pos: sim.Vec2{X: -140.0, Y: roadCoord + lw*2.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
		{Pos: sim.Vec2{X: -200.0, Y: roadCoord + lw*2.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
	}

	driverA.ComputeControls(busA, route, w, tls, queuedCarsOuter, lw, 0.016)
	if driverA.TargetLaneOffset != -lw {
		t.Fatalf("Scenario A: Bus should select middle through lane (-%.1f) with fewer queued cars, got %.2f",
			lw, driverA.TargetLaneOffset)
	}

	// Scenario B: Middle through lane has 2 queued cars, outer lane has 0 cars.
	// Bus currently in middle through lane (offset -lw) should switch back to outer through lane (offset 0.0).
	busB := sim.NewBus(2, 0, lw)
	busB.Pos = sim.Vec2{X: -350.0, Y: roadCoord + lw*1.5}
	busB.Heading = 0
	busB.Speed = 160.0
	busB.Gear = sim.GearDrive

	driverB := NewAutopilotDriver()
	driverB.BerserkMode = true
	driverB.State = AutopilotStateCruising
	driverB.LaneOffset = -lw
	driverB.TargetLaneOffset = -lw

	queuedCarsMiddle := []VehicleObstacle{
		{Pos: sim.Vec2{X: -140.0, Y: roadCoord + lw*1.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
		{Pos: sim.Vec2{X: -200.0, Y: roadCoord + lw*1.5}, Heading: 0, Speed: 0.0, Length: 48, Width: 22},
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

// TestBerserkMode3LaneOvertakingOn6LaneRoad verifies that in Berserk Mode on 6-lane roads (3 lanes per direction),
// when both the right lane (offset 0.0) and middle lane (offset -lw) have vehicles, and the leftmost lane (offset -2*lw)
// is clear, the bus changes into the leftmost lane to overtake, safely passes, and returns once clear.
func TestBerserkMode3LaneOvertakingOn6LaneRoad(t *testing.T) {
	lw := 52.0
	wTest := sim.NewWorld(lw, sim.MapThemeMetropolis)
	s6Start := sim.NewBusStop(601, "RingEastStart", sim.Vec2{X: -1000 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	s6End := sim.NewBusStop(602, "RingEastEnd", sim.Vec2{X: 1000 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	wTest.Stops = []*sim.BusStop{s6Start, s6End}

	route := &sim.Route{
		Name:         "Overtake3LaneRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	// 1. Bus in outer lane (0.0): car in outer lane (0.0) AND car in middle lane (-lw), left lane clear
	t.Run("OuterAndMiddleBlocked_ChangesToLeftmostLane", func(t *testing.T) {
		bus := sim.NewBus(1, 0, lw)
		bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: -1200*sim.MapGridScale + lw*2.5} // In outer lane (0.0)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = sim.GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		// Car in outer lane (0.0) ahead
		carOuter := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 90, Y: -1200*sim.MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		// Car in middle lane (-lw) ahead
		carMiddle := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 110, Y: -1200*sim.MapGridScale + lw*1.5},
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
		bus := sim.NewBus(1, 0, lw)
		bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: -1200*sim.MapGridScale + lw*1.5} // In middle lane (-lw)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = sim.GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		// Car in middle lane (-lw) ahead
		carMiddle := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 95, Y: -1200*sim.MapGridScale + lw*1.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		// Car in outer lane (0.0) alongside
		carOuter := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 10, Y: -1200*sim.MapGridScale + lw*2.5},
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
		bus := sim.NewBus(1, 0, lw)
		bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: -1200*sim.MapGridScale + lw*0.5} // In leftmost lane (-2*lw)
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = sim.GearDrive

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
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 50, Y: -1200*sim.MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddle := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 60, Y: -1200*sim.MapGridScale + lw*1.5},
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
			Pos:     sim.Vec2{X: -600*sim.MapGridScale - 160, Y: -1200*sim.MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddleBehind := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale - 150, Y: -1200*sim.MapGridScale + lw*1.5},
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
		bus := sim.NewBus(1, 0, lw)
		bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: -1200*sim.MapGridScale + lw*2.5}
		bus.Heading = 0
		bus.Speed = 160.0
		bus.Gear = sim.GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = false // Standard mode!
		driver.State = AutopilotStateCruising

		carOuter := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 90, Y: -1200*sim.MapGridScale + lw*2.5},
			Heading: 0,
			Speed:   50.0,
			Length:  48,
			Width:   22,
		}
		carMiddle := VehicleObstacle{
			Pos:     sim.Vec2{X: -600*sim.MapGridScale + 110, Y: -1200*sim.MapGridScale + lw*1.5},
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
