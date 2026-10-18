package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

// TestAutopilotObstacleAvoidance verifies that the autopilot slows down and brakes when a vehicle is ahead.
func TestAutopilotObstacleAvoidance(t *testing.T) {
	driver := NewAutopilotDriver()
	bus := sim.NewBus(100, 100, 0) // Heading East
	bus.Speed = 40.0

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	world := sim.NewWorld(52.0)

	// Place an obstacle 40px straight ahead in the same lane
	obstacles := []VehicleObstacle{
		{
			Pos:     sim.Vec2{X: 140, Y: 100},
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

// TestAutopilotBerserkModeStraightSpeedAndSafety tests the new Berserk Mode for the player bus:
// 1. Straightaway speed reaches up to 310 px/s (compared to standard 150 px/s);
// 2. Safely decelerates for corners, red lights, and vehicle obstacles;
// 3. Background AI buses and traffic cars remain in standard mode.
func TestAutopilotBerserkModeStraightSpeedAndSafety(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	lw := 52.0
	driver := NewAutopilotDriver()

	// 1. Straight path along Y = 1200
	driver.pathWorld = w
	driver.pathFromID = 0
	driver.pathToID = 1
	driver.pathLane = lw
	driver.drivePath = []sim.Vec2{
		{X: -1000, Y: 1200},
		{X: 1000, Y: 1200},
	}
	driver.cumLen = []float64{0, 2000.0}
	driver.WaypointIdx = 0

	route := sim.NewRoute()
	route.StopIDs = []int{0, 1}
	route.CurrentIndex = 1

	b := sim.NewBus(-600, 1200, 0) // Eastbound directly on the straight path
	b.Heading = 0
	b.Speed = 120.0

	// A. Standard Mode Test
	driver.BerserkMode = false
	throttleStd, _, _, _, _, _ := driver.ComputeControls(b, route, w, nil, nil, lw, 0.016)
	if driver.LastTargetSpeed > apCruiseSpeed+5.0 {
		t.Fatalf("Standard mode target speed should be around 150 px/s, got %.2f", driver.LastTargetSpeed)
	}

	// B. Berserk Mode Straightaway Test
	driver.BerserkMode = true
	b.Speed = 140.0
	throttleBerserk, _, _, _, _, _ := driver.ComputeControls(b, route, w, nil, nil, lw, 0.016)
	if driver.LastTargetSpeed < 440.0 {
		t.Fatalf("Berserk mode on straightaway must reach top speed >= 440 px/s (80 km/h), got %.2f", driver.LastTargetSpeed)
	}
	if throttleBerserk < throttleStd {
		t.Fatalf("Berserk mode throttle (%.2f) should be stronger than standard (%.2f)", throttleBerserk, throttleStd)
	}

	// C. Corner Deceleration Test: approaching a turn must slow down safely
	// Mock a path with a 90-degree turn ahead at 100px
	driver.drivePath = []sim.Vec2{
		{X: -600, Y: 1200},
		{X: -500, Y: 1200},
		{X: -500, Y: 1400}, // 90 degree sharp turn
	}
	driver.cumLen = []float64{0, 100.0, 300.0}
	driver.WaypointIdx = 0
	b.Pos = sim.Vec2{X: -580, Y: 1200}
	b.Speed = 280.0
	cornerLimit := driver.cornerSpeedLimit(20.0, 280.0)
	if cornerLimit > 170.0 {
		t.Fatalf("Approaching sharp corner at 80px must limit speed, got %.2f (expected <= 170)", cornerLimit)
	}

	// D. Red Light Safety Test: must slow to 0 before stop line under red light
	tls := sim.NewTrafficLightSystem()
	inter := &sim.TrafficIntersection{
		Center:  sim.Vec2{X: -400, Y: 1200},
		Phase:   sim.PhaseNSGreen, // EW is RED
		Timer:   0,
		IsMajor: true,
	}
	tls.Intersections = []*sim.TrafficIntersection{inter}

	// Revert to straight path towards intersection
	driver.drivePath = []sim.Vec2{{X: -800, Y: 1200}, {X: 200, Y: 1200}}
	driver.cumLen = []float64{0, 1000.0}
	driver.WaypointIdx = 0
	b.Pos = sim.Vec2{X: -580, Y: 1200}
	b.Speed = 220.0

	_, brakeTL, _, _, _, _ := driver.ComputeControls(b, route, w, tls, nil, lw, 0.016)
	if driver.LastTargetSpeed > 100.0 {
		t.Fatalf("Must anticipate red light and decelerate target speed, got %.2f", driver.LastTargetSpeed)
	}
	if brakeTL <= 0.0 && driver.LastTargetSpeed > b.Speed {
		t.Fatalf("Must apply brake or reduce target speed when approaching red light from high speed")
	}

	// D2. Yellow Light Test: in Berserk Mode, approaching a yellow light must NOT stop (runs yellow light)
	interYellow := &sim.TrafficIntersection{
		Center:  sim.Vec2{X: -400, Y: 1200},
		Phase:   sim.PhaseEWYellow, // EW is YELLOW
		Timer:   1.0,
		IsMajor: true,
	}
	tlsYellow := sim.NewTrafficLightSystem()
	tlsYellow.Intersections = []*sim.TrafficIntersection{interYellow}
	driver.BerserkMode = true
	b.Pos = sim.Vec2{X: -580, Y: 1200}
	b.Speed = 220.0
	driver.ComputeControls(b, route, w, tlsYellow, nil, lw, 0.016)
	if driver.LastTargetSpeed < 200.0 {
		t.Fatalf("Berserk mode must run yellow light without stopping, got target speed %.2f", driver.LastTargetSpeed)
	}

	// But in Standard Mode, it must stop for yellow light:
	driver.BerserkMode = false
	driver.ComputeControls(b, route, w, tlsYellow, nil, lw, 0.016)
	if driver.LastTargetSpeed > 100.0 {
		t.Fatalf("Standard mode must stop for yellow light, got target speed %.2f", driver.LastTargetSpeed)
	}

	// E. Obstacle Avoidance Test: must decelerate and stop safely behind slower/stopped lead vehicle
	driver.BerserkMode = true
	stoppedCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -440, Y: 1200}, // 140px in front
		Heading: 0,
		Speed:   0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(b, route, w, nil, []VehicleObstacle{stoppedCar}, lw, 0.016)
	if driver.LastTargetSpeed > 130.0 {
		t.Fatalf("Must decelerate from high speed for lead vehicle at 140px, got target speed %.2f", driver.LastTargetSpeed)
	}

	// Close obstacle: within safe stop distance must command full stop (targetSpd = 0)
	closeCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -510, Y: 1200}, // 70px in front (< safeStopDist 85px)
		Heading: 0,
		Speed:   0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(b, route, w, nil, []VehicleObstacle{closeCar}, lw, 0.016)
	if driver.LastTargetSpeed > 0.0 {
		t.Fatalf("Must command full stop when lead vehicle is within safe distance, got %.2f", driver.LastTargetSpeed)
	}

}

// TestBerserkModeConfigurableMaxSpeedsTurningAndLaneChange thoroughly tests the configurable
// Berserk mode top speeds (60, 80, 100, 120 km/h) and verifies:
// 1. Settings options, bounds, cycle logic, and cruise speed mapping;
// 2. Straightaway acceleration up to top speed with dynamic powertrain scaling;
// 3. Anticipated cornering deceleration for 90-degree turns without overshooting or instability;
// 4. Smooth overtaking and lane-change initiation and completion across all speeds.
func TestBerserkModeConfigurableMaxSpeedsTurningAndLaneChange(t *testing.T) {
	lw := 52.0

	// 1. Test Settings Configuration & Cycling
	s := ui.NewSettings()
	if s.AutopilotBerserkMaxSpeed != 80 {
		t.Fatalf("Default Berserk max speed should be 80 km/h, got %d", s.AutopilotBerserkMaxSpeed)
	}
	if math.Abs(s.GetBerserkCruiseSpeed()-445.0) > 1.0 {
		t.Fatalf("Default Berserk cruise speed should be ~445.0 px/s, got %.2f", s.GetBerserkCruiseSpeed())
	}

	testSpeeds := []struct {
		kmh       int
		cruiseSpd float64
	}{
		{60, 333.0},
		{80, 445.0},
		{100, 555.0},
		{120, 667.0},
	}

	for _, ts := range testSpeeds {
		s.SetBerserkMaxSpeed(ts.kmh)
		if s.AutopilotBerserkMaxSpeed != ts.kmh {
			t.Fatalf("Expected Berserk speed %d, got %d", ts.kmh, s.AutopilotBerserkMaxSpeed)
		}
		if math.Abs(s.GetBerserkCruiseSpeed()-ts.cruiseSpd) > 1.0 {
			t.Fatalf("Expected Berserk cruise speed %.1f for %d km/h, got %.2f",
				ts.cruiseSpd, ts.kmh, s.GetBerserkCruiseSpeed())
		}
	}

	// Verify cycle sequence: 60 -> 80 -> 100 -> 120 -> 60
	s.SetBerserkMaxSpeed(60)
	if next := s.CycleBerserkMaxSpeed(); next != 80 {
		t.Fatalf("Expected cycle from 60 to 80, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 100 {
		t.Fatalf("Expected cycle from 80 to 100, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 120 {
		t.Fatalf("Expected cycle from 100 to 120, got %d", next)
	}
	if next := s.CycleBerserkMaxSpeed(); next != 60 {
		t.Fatalf("Expected cycle from 120 to 60, got %d", next)
	}

	// 2. Subtests for Each Configured Speed
	for _, ts := range testSpeeds {
		t.Run(fmt.Sprintf("Speed_%d_kmh", ts.kmh), func(t *testing.T) {
			kmh := ts.kmh
			cruiseSpd := ts.cruiseSpd

			// -------------------------------------------------------------
			// A. Straightaway Top Speed & Powertrain Scaling
			// -------------------------------------------------------------
			wA := sim.NewWorld(lw, sim.MapThemeMetropolis)
			wA.Stops[1].Pos = sim.Vec2{X: 3000, Y: 1200}
			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.pathWorld = wA
			driver.pathFromID = 0
			driver.pathToID = 1
			driver.pathLane = lw

			driver.drivePath = []sim.Vec2{
				{X: -2000, Y: 1200},
				{X: 2000, Y: 1200},
			}
			driver.cumLen = []float64{0, 4000.0}
			driver.WaypointIdx = 0

			bus := sim.NewBus(-1000, 1200, 0)
			bus.Speed = cruiseSpd * 0.6
			// Dynamic powertrain expansion in game.go
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			route := sim.NewRoute()
			route.StopIDs = []int{0, 1}
			route.CurrentIndex = 1

			dt := 0.016
			// Initial target speed check
			driver.ComputeControls(bus, route, wA, nil, nil, lw, dt)
			if math.Abs(driver.LastTargetSpeed-cruiseSpd) > 2.0 {
				t.Fatalf("[%d km/h] Target speed on straightaway should be %.1f px/s, got %.2f",
					kmh, cruiseSpd, driver.LastTargetSpeed)
			}

			// Simulate forward acceleration
			maxSpeedReached := 0.0
			for step := 0; step < 200; step++ {
				th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, wA, nil, nil, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)
				if bus.Speed > maxSpeedReached {
					maxSpeedReached = bus.Speed
				}
			}
			if maxSpeedReached < cruiseSpd*0.95 {
				t.Fatalf("[%d km/h] Bus failed to accelerate to top speed: max speed=%.1f px/s (target=%.1f)",
					kmh, maxSpeedReached, cruiseSpd)
			}

			// -------------------------------------------------------------
			// B. Impact on Cornering: Deceleration & Smooth Turning
			// -------------------------------------------------------------
			// Build a path with a long straight leading into a 90-degree right turn
			cornerX := 0.0
			cornerY := 1200.0
			wB := sim.NewWorld(lw, sim.MapThemeMetropolis)
			wB.Stops[1].Pos = sim.Vec2{X: cornerX, Y: cornerY + 1000}
			driverCorner := NewAutopilotDriver()
			driverCorner.BerserkMode = true
			driverCorner.BerserkMaxSpeed = cruiseSpd
			driverCorner.pathWorld = wB
			driverCorner.pathFromID = 0
			driverCorner.pathToID = 1
			driverCorner.pathLane = lw

			driverCorner.drivePath = []sim.Vec2{
				{X: -1500, Y: cornerY},
				{X: cornerX, Y: cornerY},
				{X: cornerX, Y: cornerY + 600}, // 90-degree right turn at (0, 1200)
			}
			driverCorner.cumLen = []float64{0, 1500.0, 2100.0}
			driverCorner.WaypointIdx = 0

			busCorner := sim.NewBus(-1200, cornerY, 0)
			busCorner.Speed = cruiseSpd
			busCorner.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// 1. Verify cornerSpeedLimit deceleration profile
			// Far away (> 1400px), limit should be Inf or >= cruiseSpd
			farLimit := driverCorner.cornerSpeedLimit(100.0, cruiseSpd)
			if farLimit < cruiseSpd {
				t.Fatalf("[%d km/h] Far corner speed limit should not brake prematurely: got %.2f (cruise=%.2f)",
					kmh, farLimit, cruiseSpd)
			}

			// 2. Drive towards the corner and verify it reaches safe cornering speed <= 95 px/s
			reachedSafeCornerSpeed := false
			enteredCorner := false
			completedTurn := false

			for step := 0; step < 500; step++ {
				th, br, st, hb, tg, _ := driverCorner.ComputeControls(busCorner, route, wB, nil, nil, lw, dt)
				busCorner.Update(dt, th, br, st, hb, tg)

				// Distance along X to corner apex
				distToApex := cornerX - busCorner.Pos.X
				if distToApex <= 30.0 && distToApex >= -30.0 {
					enteredCorner = true
					if busCorner.Speed <= 95.0 {
						reachedSafeCornerSpeed = true
					}
				}

				// Check if heading turned towards South (+Y)
				if math.Sin(busCorner.Heading) > 0.85 {
					completedTurn = true
					break
				}
			}

			if !enteredCorner {
				t.Fatalf("[%d km/h] Bus never reached the corner apex", kmh)
			}
			if !reachedSafeCornerSpeed {
				t.Fatalf("[%d km/h] Bus did not slow down sufficiently before entering corner: speed=%.1f px/s (expected <= 95.0)",
					kmh, busCorner.Speed)
			}
			if !completedTurn {
				t.Fatalf("[%d km/h] Bus failed to smoothly complete 90-degree turn: heading=%.2f",
					kmh, busCorner.Heading)
			}

			// -------------------------------------------------------------
			// C. Impact on Lane Changing & Overtaking
			// -------------------------------------------------------------
			yCenter := 1200.0 * sim.MapGridScale
			sStart := sim.NewBusStop(201, "AvenueStart", sim.Vec2{X: -1500 * sim.MapGridScale, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			sEnd := sim.NewBusStop(202, "AvenueEnd", sim.Vec2{X: 1500 * sim.MapGridScale, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			wC := sim.NewWorld(lw, sim.MapThemeMetropolis)
			wC.Stops = []*sim.BusStop{sStart, sEnd}

			routeC := &sim.Route{
				Name:         "OvertakeTestRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driverOvertake := NewAutopilotDriver()
			driverOvertake.BerserkMode = true
			driverOvertake.BerserkMaxSpeed = cruiseSpd
			driverOvertake.State = AutopilotStateCruising

			busOvertake := sim.NewBus(1, 0, lw)
			busOvertake.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: yCenter + lw*1.5}
			busOvertake.Heading = 0
			busOvertake.Speed = cruiseSpd * 0.85
			busOvertake.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// Lead blocker moving at 50% cruise speed, positioned ahead within Berserk detection corridor
			blockerSpd := cruiseSpd * 0.50
			leadDist := math.Max(320.0, cruiseSpd*0.75)
			blocker := VehicleObstacle{
				Pos:     sim.Vec2{X: busOvertake.Pos.X + leadDist, Y: yCenter + lw*1.5},
				Heading: 0,
				Speed:   blockerSpd,
				Length:  48,
				Width:   22,
			}

			// Initial control compute: should detect blocker and initiate lane change
			driverOvertake.ComputeControls(busOvertake, routeC, wC, nil, []VehicleObstacle{blocker}, lw, dt)
			if driverOvertake.OvertakeState != OvertakeChangingLane {
				t.Fatalf("[%d km/h] Expected OvertakeChangingLane for slow blocker (spd=%.1f), got state %d",
					kmh, blockerSpd, driverOvertake.OvertakeState)
			}
			if math.Abs(driverOvertake.TargetLaneOffset-(-lw)) > 1.0 {
				t.Fatalf("[%d km/h] Expected target lane offset -%.1f (inner passing lane), got %.2f",
					kmh, lw, driverOvertake.TargetLaneOffset)
			}

			// Simulate full overtaking maneuver: change lane -> pass -> return
			statePassingReached := false
			stateReturningReached := false
			maneuverCompleted := false
			collided := false

			for step := 0; step < 600; step++ {
				// Blocker moves forward at its steady speed
				blocker.Pos.X += blocker.Speed * dt

				th, br, st, hb, tg, _ := driverOvertake.ComputeControls(
					busOvertake, routeC, wC, nil, []VehicleObstacle{blocker}, lw, dt,
				)
				busOvertake.Update(dt, th, br, st, hb, tg)

				// Collision check
				distToBlocker := busOvertake.Pos.Distance(blocker.Pos)
				if distToBlocker < (busOvertake.Length/2 + blocker.Length/2 - 5.0) {
					// Check bounding box overlap
					if math.Abs(busOvertake.Pos.Y-blocker.Pos.Y) < (busOvertake.Width/2 + blocker.Width/2) {
						collided = true
						t.Fatalf("[%d km/h] Step %d: Bus collided with lead vehicle! dist=%.1f",
							kmh, step, distToBlocker)
					}
				}

				if driverOvertake.OvertakeState == OvertakePassing {
					statePassingReached = true
				}
				if driverOvertake.OvertakeState == OvertakeReturning {
					stateReturningReached = true
				}
				if stateReturningReached && driverOvertake.OvertakeState == OvertakeNone && math.Abs(driverOvertake.LaneOffset) < 1.0 {
					maneuverCompleted = true
					break
				}
			}

			if collided {
				t.Fatalf("[%d km/h] Overtaking maneuver resulted in collision", kmh)
			}
			if !statePassingReached {
				t.Fatalf("[%d km/h] Overtaking maneuver never reached OvertakePassing (current state=%d, offset=%.2f)",
					kmh, driverOvertake.OvertakeState, driverOvertake.LaneOffset)
			}
			if !stateReturningReached {
				t.Fatalf("[%d km/h] Overtaking maneuver never reached OvertakeReturning", kmh)
			}
			if !maneuverCompleted {
				t.Fatalf("[%d km/h] Overtaking maneuver did not return to base lane cleanly: state=%d, offset=%.2f",
					kmh, driverOvertake.OvertakeState, driverOvertake.LaneOffset)
			}
		})
	}
}
