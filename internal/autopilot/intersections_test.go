package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestAutopilotNavigationNoUTurnNoSpinning verifies Bug 2:
// Autopilot driving along routes on all maps never commands sudden 180° U-turns or spins in circles.
func TestAutopilotNavigationNoUTurnNoSpinning(t *testing.T) {
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
			world := sim.NewWorld(52.0, theme)
			route := sim.GetStandardRouteForMap(theme, world)
			if len(route.StopIDs) < 2 {
				t.Skip("Route has insufficient stops")
			}

			startStop := world.Stops[route.StopIDs[0]]
			bus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
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

				headingDelta := sim.NormalizeAngle(bus.Heading - prevHeading)
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

// TestRightTurnNormalDrivingAndOncomingExemption verifies that right turns proceed on red
// and oncoming vehicles stopped in opposite lanes do not cause unexpected stops (Bug #3).
func TestRightTurnNormalDrivingAndOncomingExemption(t *testing.T) {
	laneW := 52.0
	inter := &sim.TrafficIntersection{
		ID:             0,
		Center:         sim.Vec2{X: 0, Y: 0},
		IsMajor:        true,
		Phase:          sim.PhaseNSGreen, // East-West approach has Red light
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	// 1. Through vehicle approaching red light must stop
	posApproach := sim.Vec2{X: -140, Y: laneW * 1.5}
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
	bus := sim.NewBus(-50, 78, -2.80) // Turning right, nose pointed northwest
	bus.Speed = 60.0
	ad.drivePath = []sim.Vec2{
		{X: -100, Y: 78},
		{X: -78, Y: 78},
		{X: -78, Y: -200},
	}
	ad.cumLen = []float64{0, 22, 300}
	ad.WaypointIdx = 1

	// Oncoming civilian car stopped at North approach opposite lane (X = -130, Y = -50, heading South)
	oncomingCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -130, Y: -50},
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
	stopLineDist := laneW*1.6 + 18.0     // 101.2px
	crosswalkDist := stopLineDist - 25.0 // 76.2px
	inter := &sim.TrafficIntersection{
		ID:             0,
		Center:         sim.Vec2{X: 0, Y: 0},
		IsMajor:        true,
		Phase:          sim.PhaseNSGreen, // East-West approach has Red light
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	// Place bus approaching stop line from West heading East (0)
	// Stop line is at X = -stopLineDist (-101.2). Zebra crosswalk is at X = -crosswalkDist (-76.2).
	bus := sim.NewBus(-200, laneW*1.5, 0)
	bus.Speed = 60.0
	ad := NewAutopilotDriver()
	ad.drivePath = []sim.Vec2{{X: -300, Y: laneW * 1.5}, {X: 300, Y: laneW * 1.5}}
	ad.cumLen = []float64{0, 600}

	// Simulate deceleration and stopping before red light
	dt := 0.016
	for step := 0; step < 240; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := ad.ComputeControls(
			bus,
			&sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 0},
			sim.NewWorld(laneW),
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

// TestMajorIntersectionAutopilotLeftTurnRedLightStopping verifies that when turning left at a major intersection,
// the autopilot strictly respects the dedicated left-turn red signal (even when through traffic has a green light),
// never confuses lane merges for right turns, and comes to a complete, safe stop behind the stop line.
func TestMajorIntersectionAutopilotLeftTurnRedLightStopping(t *testing.T) {
	lw := 52.0
	inter := &sim.TrafficIntersection{
		ID:                0,
		Center:            sim.Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseEWGreen, // Through is GREEN, Left arrow is RED!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
	stopLineDist, _, _, _ := inter.GetApproachDimensions(lw)

	// Verify signals for Eastbound approach:
	thruSig, leftSig := inter.GetSignalForApproach(0)
	if thruSig != sim.SignalGreen {
		t.Fatalf("Expected through signal to be Green during PhaseEWGreen, got %v", thruSig)
	}
	if leftSig != sim.SignalRed {
		t.Fatalf("Expected left signal to be Red during PhaseEWGreen, got %v", leftSig)
	}

	// 1. Direct CheckStopForVehicle test in dedicated left-turn lane (Y = lw*0.5 = 26.0)
	posApproachLeft := sim.Vec2{X: -(stopLineDist + 50.0), Y: lw * 0.5} // bumper ~ -(stopLineDist + 2.0)
	bumperPos := posApproachLeft.Add(sim.Vec2{X: 48, Y: 0})

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
	driver.drivePath = []sim.Vec2{{X: -600, Y: lw * 1.5}, {X: -450, Y: lw / 2}}
	driver.drivePath = append(driver.drivePath, sim.LeftTurnPath(sim.Vec2{}, 0, lw, 190)...)
	driver.drivePath = append(driver.drivePath, sim.Vec2{X: lw / 2, Y: -400})

	driver.cumLen = make([]float64, len(driver.drivePath))
	for i := 1; i < len(driver.drivePath); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + driver.drivePath[i].Distance(driver.drivePath[i-1])
	}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -stopLineDist - 180, Y: lw * 0.5} // Start with enough braking distance before the enlarged stop line
	bus.Heading = 0
	bus.Speed = 140.0
	bus.Gear = sim.GearDrive

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// Simulate the approach and stopped hold:
	dt := 0.016
	for step := 0; step < 600; step++ {
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
	inter.Phase = sim.PhaseEWLeftGreen
	_, leftSigNew := inter.GetSignalForApproach(0)
	if leftSigNew != sim.SignalGreen {
		t.Fatalf("Expected left signal to be Green during PhaseEWLeftGreen, got %v", leftSigNew)
	}

	// Simulate through the green left turn:
	for step := 0; step < 600; step++ {
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

	inter := &sim.TrafficIntersection{
		ID:                0,
		Center:            sim.Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseEWGreen, // Through Green, Left RED
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.drivePath = []sim.Vec2{
		{X: -800, Y: lw * 0.5},
		{X: lw * 0.5, Y: lw * 0.5},
		{X: lw * 0.5, Y: -500},
	}
	driver.cumLen = make([]float64, len(driver.drivePath))
	for i := 1; i < len(driver.drivePath); i++ {
		driver.cumLen[i] = driver.cumLen[i-1] + driver.drivePath[i].Distance(driver.drivePath[i-1])
	}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -400, Y: lw * 0.5}
	bus.Heading = 0
	bus.Speed = 260.0 // Approaching at high speed
	bus.Gear = sim.GearDrive

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

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
	world := sim.NewWorld(laneW, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, world)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(world)

	startStop := world.Stops[route.StopIDs[0]]
	bus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
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
			t.Fatalf("Step %d: Autopilot committed traffic violation: %s, bus pos: %v, speed: %.2f heading %.2f target %v path %.1f remaining %.1f",
				step, notice, bus.Pos, bus.Speed, bus.Heading, driver.LastPursuitPt, driver.LastPathS, driver.LastRemaining)
		}
	}
}

// TestBerserkModeRedLightNoLeftTurnLaneInvasion4Lane verifies that on a 4-lane avenue approaching an intersection,
// when going STRAIGHT and lead cars in the through lane are waiting at a red light,
// the bus NEVER changes lanes into the dedicated left-turn lane.
func TestBerserkModeRedLightNoLeftTurnLaneInvasion4Lane(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// Central Boulevard at Y=0 is a 4-lane avenue with dedicated left-turn lanes at intersections
	inter := &sim.TrafficIntersection{
		ID:                0,
		Center:            sim.Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseNSGreen, // EW through light is Red!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	// Setup Eastbound route going straight through intersection (0, 0)
	s1 := sim.NewBusStop(701, "EWStart", sim.Vec2{X: -600 * sim.MapGridScale, Y: lw * 1.5}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(702, "EWEnd", sim.Vec2{X: 600 * sim.MapGridScale, Y: lw * 1.5}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "RedLightStraightRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := sim.NewBus(1, 0, lw)
	// Position bus 280px before stop line in outer through lane (Y = lw*1.5)
	bus.Pos = sim.Vec2{X: -320.0, Y: lw * 1.5}
	bus.Heading = 0
	bus.Speed = 120.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Stopped car ahead in through lane waiting at red light (X = -150.0, speed = 0.0)
	stoppedCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -150.0, Y: lw * 1.5},
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

// TestOpposingLeftTurnsSimultaneousClearance verifies that when two opposing vehicles simultaneously
// make a left turn at the same intersection (one East->North, one West->South), the compact turning arc
// guarantees that they pass passenger-side to passenger-side with ample safety clearance (>45px centerline,
// >13px body clearance), with zero collisions and zero path crossing.
func TestOpposingLeftTurnsSimultaneousClearance(t *testing.T) {
	lw := 52.0
	w := &sim.World{}
	w.Roads = []sim.RoadSegment{
		{P1: sim.Vec2{X: -1500, Y: 0}, P2: sim.Vec2{X: 1500, Y: 0}, Lanes: 4, Type: sim.RoadAvenue4Lane, HasLines: true},
		{P1: sim.Vec2{X: 0, Y: -1500}, P2: sim.Vec2{X: 0, Y: 1500}, Lanes: 4, Type: sim.RoadAvenue4Lane, HasLines: true},
	}
	w.RebuildIntersections()

	// Intersection at (0, 0)
	inter := &sim.TrafficIntersection{
		ID:                99,
		Center:            sim.Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseEWLeftGreen, // Left GREEN
		GreenDuration:     30.0,
		YellowDuration:    3.0,
		LeftGreenDuration: 30.0,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	// Stop 1: Eastbound on road Y=0 approaching (0,0)
	sEastIn := sim.NewBusStop(0, "EastboundIn", sim.Vec2{X: -600, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sEastIn)
	sNorthOut := sim.NewBusStop(1, "NorthboundOut", sim.Vec2{X: 0, Y: -600}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sNorthOut)
	path1 := sim.ComputeRoadPathBetweenStops(w, sEastIn, sNorthOut, lw)

	// Stop 2: Westbound on road Y=0 approaching (0,0)
	sWestIn := sim.NewBusStop(2, "WestboundIn", sim.Vec2{X: 600, Y: 0}, math.Pi, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sWestIn)
	sSouthOut := sim.NewBusStop(3, "SouthboundOut", sim.Vec2{X: 0, Y: 600}, math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, sSouthOut)
	path2 := sim.ComputeRoadPathBetweenStops(w, sWestIn, sSouthOut, lw)

	bus1 := sim.NewBus(-320.0, 78.0, 0.0)
	bus1.Speed = 120.0
	route1 := &sim.Route{StopIDs: []int{sEastIn.ID, sNorthOut.ID}, CurrentIndex: 1}
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

	bus2 := sim.NewBus(320.0, -78.0, math.Pi)
	bus2.Speed = 120.0
	route2 := &sim.Route{StopIDs: []int{sWestIn.ID, sSouthOut.ID}, CurrentIndex: 1}
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
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// s1: On 6-lane North Ring Expressway (Y = -3600), Westbound
	s1 := sim.NewBusStop(len(w.Stops), "6LaneStart", sim.Vec2{X: 600 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, math.Pi, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, s1)
	// s2: Turning left onto North-South Avenue (X = 0), Southbound towards (0, 0)
	s2 := sim.NewBusStop(len(w.Stops), "4LaneSouth", sim.Vec2{X: 0, Y: -400 * sim.MapGridScale}, math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = append(w.Stops, s2)

	path := sim.ComputeRoadPathBetweenStops(w, s1, s2, lw)

	interCenter := sim.Vec2{X: 0, Y: -1200 * sim.MapGridScale}
	inter := &sim.TrafficIntersection{
		ID:                95,
		Center:            interCenter,
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{6, 6, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseEWLeftGreen, // Left green light for approach
		GreenDuration:     40.0,
		YellowDuration:    3.0,
		LeftGreenDuration: 40.0,
	}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}

	bus := sim.NewBus(s1.Pos.X, s1.Pos.Y, s1.Heading)
	bus.Speed = 280.0 // Approaching at high speed in Berserk mode
	route := &sim.Route{StopIDs: []int{s1.ID, s2.ID}, CurrentIndex: 1}

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
			angDelta := math.Abs(sim.NormalizeAngle(bus.Heading - prevHeading))
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

// TestAIBusLeftTurnNotBlockedByOncomingBusInLeftTurnLane verifies that an AI bus making a left
// turn at an intersection does NOT falsely brake or deadlock due to an oncoming vehicle waiting
// in the dedicated left-turn lane across the road centerline.
func TestAIBusLeftTurnNotBlockedByOncomingBusInLeftTurnLane(t *testing.T) {
	lw := 26.0
	w := sim.NewWorld(lw, sim.MapThemeTwinCities)
	route := sim.GetStandardRouteForMap(sim.MapThemeTwinCities, w)

	// In TwinCities, the major intersection is at X=3900, Y=0.
	// AI#8 is turning left from Northbound avenue (X=3900) into Westbound expressway lanes (Y=-26).
	// Mid-turn, match the west-northwest path tangent. The old +2.7
	// heading pointed its long nose south into the supposedly separated car.
	turningBus := sim.NewBus(3880, -10, math.Atan2(-16, -40))
	turningBus.Speed = 30.0

	driver := NewAutopilotDriver()
	driver.State = AutopilotStateCruising
	// Path tracks left turn curve into westbound lane Y=-26
	driver.drivePath = []sim.Vec2{
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
		Pos:     sim.Vec2{X: 3845, Y: 26},
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

}
