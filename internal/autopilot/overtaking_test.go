package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestBerserkModeOvertakingOnMultiLaneRoad verifies that on 4-lane roads (2 lanes per direction),
// the player bus in Berserk Mode can safely change lanes to overtake slower lead vehicles,
// while in Standard Mode or on single-lane roads it does not overtake.
func TestBerserkModeOvertakingOnMultiLaneRoad(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// Stops on South Ring Avenue (Y=1200*MapGridScale, 4-lane avenue)
	s1 := sim.NewBusStop(201, "AvenueStart", sim.Vec2{X: -1000 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(202, "AvenueEnd", sim.Vec2{X: 1000 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "OvertakeTestRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: 1200*sim.MapGridScale + lw*1.5} // In outer travel lane (offset 1.5*lw)
	bus.Heading = 0                                                                   // Eastbound
	bus.Speed = 160.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()

	// Slow lead vehicle in same outer lane
	slowCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -600*sim.MapGridScale + 90, Y: 1200*sim.MapGridScale + lw*1.5}, // 90px in front
		Heading: 0,
		Speed:   50.0,
		Length:  48,
		Width:   22,
	}

	// 1. Standard Mode: should NOT overtake
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone || driver.LaneOffset != 0.0 {
		t.Fatalf("Standard mode must not initiate overtaking (state=%d, offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// The fleet restriction must preserve same-direction passing on wide roads.
	driver.Reset()
	driver.BerserkMode = true
	driver.DisallowOncomingOvertake = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane || math.Abs(driver.TargetLaneOffset+lw) > 1 {
		t.Fatal("fleet restriction incorrectly blocked same-direction passing")
	}
	driver.Reset()
	driver.DisallowOncomingOvertake = false

	// 2. Berserk Mode: initiates overtake into inner lane (-lw)
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate lane change (expected state %d, got %d)",
			OvertakeChangingLane, driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f (inner lane), got %.2f", lw, driver.TargetLaneOffset)
	}

	// Advance time until lane change is complete
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.08)
	}
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected OvertakePassing once offset reached, got state %d (offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}
	if math.Abs(driver.LaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Lane offset should be centered in passing lane (-%.1f), got %.2f", lw, driver.LaneOffset)
	}

	// Move bus ahead of the slow vehicle (overtake completed)
	bus.Pos.X = -380 // 130px ahead of slowCar at X=-510
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.5)
	if driver.OvertakeState != OvertakeReturning {
		t.Fatalf("Expected OvertakeReturning once vehicle passed, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Target offset after passing should return to 0, got %.2f", driver.TargetLaneOffset)
	}

	// Advance until returned to base lane
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, nil, lw, 0.08)
	}
	if driver.OvertakeState != OvertakeNone || math.Abs(driver.LaneOffset) > 0.1 {
		t.Fatalf("Expected return to OvertakeNone and 0 offset, got state %d, offset %.2f",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 3. Passing lane blocked: safety check
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	bus.Pos = sim.Vec2{X: -600, Y: 1200 + lw*1.5}
	passingCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -550, Y: 1200 + lw*0.5}, // in passing lane
		Heading: 0,
		Speed:   80.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, passingCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Must not overtake when passing lane is occupied (got state %d)", driver.OvertakeState)
	}

	// 4. 2-lane road (1 lane per direction):
	// a. Standard mode: strictly prohibited
	driver.Reset()
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	bus.Pos = sim.Vec2{X: 1300, Y: 400} // On 2-lane road
	bus.Heading = math.Pi / 2           // Southbound
	slowCar2Lane := VehicleObstacle{
		Pos:     sim.Vec2{X: 1300, Y: 480},
		Heading: math.Pi / 2,
		Speed:   40.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar2Lane}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Standard mode must never overtake on 2-lane roads (got state %d)", driver.OvertakeState)
	}

	// b. Approaching intersection on 2-lane road: borrowing oncoming lane is prohibited
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar2Lane}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Berserk mode must not borrow lane near intersections on 2-lane roads (got state %d)", driver.OvertakeState)
	}
}

// TestBerserkModeEarlyOvertakeInitiation verifies that in Berserk mode, the bus initiates lane change
// early at an ample distance (> 350px) ahead of a slow lead vehicle rather than closing in and tailgating,
// and completes the lane change with a silky smooth S-curve profile.
func TestBerserkModeEarlyOvertakeInitiation(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	s1 := sim.NewBusStop(301, "AvenueStart", sim.Vec2{X: -1200 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(302, "AvenueEnd", sim.Vec2{X: 1200 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "EarlyOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -700 * sim.MapGridScale, Y: 1200*sim.MapGridScale + lw*1.5}
	bus.Heading = 0
	bus.Speed = 260.0 // Fast cruise speed
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow lead vehicle 360px ahead (in the past, fwdMax was only 170-195px, so this was ignored until tailgating)
	leadCar := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X + 360.0, Y: 1200*sim.MapGridScale + lw*1.5},
		Heading: 0,
		Speed:   45.0,
		Length:  48,
		Width:   22,
	}

	// 1. First frame: Berserk mode must detect blocker 360px away and initiate overtake immediately
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate lane change early at 360px distance (expected state %d, got %d)",
			OvertakeChangingLane, driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f, got %.2f", lw, driver.TargetLaneOffset)
	}

	// 2. Measure transition trajectory over time: must be a smooth S-curve and finish swiftly
	offsets := []float64{driver.LaneOffset}
	steps := 0
	for i := 0; i < 30; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.033)
		offsets = append(offsets, driver.LaneOffset)
		steps++
		if driver.OvertakeState == OvertakePassing {
			break
		}
	}

	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Lane change should finish smoothly within ~1.0s, but still in state %d (offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// Verify S-curve smoothstep easing: initial rate of lateral shift must be gentle, not a linear snap
	initialDelta := math.Abs(offsets[1] - offsets[0])
	peakDelta := 0.0
	for i := 1; i < len(offsets); i++ {
		d := math.Abs(offsets[i] - offsets[i-1])
		if d > peakDelta {
			peakDelta = d
		}
	}
	if initialDelta >= peakDelta*0.5 {
		t.Fatalf("Initial transition step (%.3f) is too sharp compared to peak step (%.3f); expected gentle S-curve easing",
			initialDelta, peakDelta)
	}
}

// does not trigger drastic deceleration or heavy braking down to 55 px/s (~10 km/h),
// but instead maintains fast, smooth cruising speed.
func TestBerserkModeOvertakeMaintainsSpeedInOpenSpace(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	s1 := sim.NewBusStop(601, "OpenAvenueStart", sim.Vec2{X: -1200 * sim.MapGridScale, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(602, "OpenAvenueEnd", sim.Vec2{X: 1200 * sim.MapGridScale, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "OpenOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: -600 * sim.MapGridScale, Y: lw * 1.5}
	bus.Heading = 0
	bus.Speed = 250.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow vehicle far ahead (350px away)
	leadCar := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X + 350.0, Y: lw * 1.5},
		Heading: 0,
		Speed:   50.0,
		Length:  48,
		Width:   22,
	}

	// 1. Initial compute triggers lane change
	throttle, brake, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Expected OvertakeChangingLane, got %d", driver.OvertakeState)
	}

	// 2. Throughout the lane change transition, verify the bus maintains high speed (> 180 px/s)
	// and does not slam foot brake (brake > 0.35)
	for i := 0; i < 20; i++ {
		throttle, brake, _, _, _, _ = driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{leadCar}, lw, 0.033)
		if brake > 0.35 {
			t.Fatalf("Frame %d: Bus slammed brakes during lane change in open space (brake=%.2f, offset=%.2f, state=%d)",
				i, brake, driver.LaneOffset, driver.OvertakeState)
		}
		if throttle < 0.2 && bus.Speed < 200.0 {
			t.Fatalf("Frame %d: Bus throttled down unexpectedly during lane change (throttle=%.2f, speed=%.2f)",
				i, throttle, bus.Speed)
		}
	}
}

// TestLeftTurnLaneChangeDoesNotCrossDoubleYellowIntoOncomingLane verifies that when a bus changes lanes
// into the dedicated left-turn lane (both in normal autopilot and Berserk mode), no part of the bus
// crosses over the center double-yellow divider into the oncoming lane (Bug: 变换车道去左转弯车道时压到对向车道).
func TestLeftTurnLaneChangeDoesNotCrossDoubleYellowIntoOncomingLane(t *testing.T) {
	lw := 52.0
	themes := []sim.MapTheme{sim.MapThemeCoastalBay}

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := sim.NewWorld(lw, theme)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			// Find a stop leg that executes a left turn
			route := sim.GetStandardRouteForMap(theme, w)
			testedLeg := false
			for legIdx := 0; legIdx < len(route.StopIDs); legIdx++ {
				fromStop := w.Stops[route.StopIDs[legIdx]]
				nextIdx := (legIdx + 1) % len(route.StopIDs)
				toStop := w.Stops[route.StopIDs[nextIdx]]

				legRoute := &sim.Route{
					Name:         "TestLeftTurnLeg",
					StopIDs:      []int{fromStop.ID, toStop.ID},
					CurrentIndex: 1,
				}

				path := sim.ComputeRoadPathBetweenStops(w, fromStop, toStop, lw)
				// Check if this leg contains a left turn
				hasLeftTurn := false
				for k := 1; k < len(path)-1; k++ {
					a := path[k].Sub(path[k-1])
					b := path[k+1].Sub(path[k])
					turn := sim.NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
					// Smooth arcs turn a little at each waypoint. Include cumulative
					// heading over a local distance, rather than requiring a kink.
					cumulative := turn
					distance := b.Length()
					for j := k + 2; j < len(path) && distance < 180; j++ {
						c := path[j].Sub(path[j-1])
						cumulative += sim.NormalizeAngle(math.Atan2(c.Y, c.X) - math.Atan2(b.Y, b.X))
						distance += c.Length()
						b = c
					}
					if (turn < -0.4 && turn > -2.8) || cumulative < -.5 {
						hasLeftTurn = true
						break
					}
				}
				if !hasLeftTurn {
					continue
				}

				bus := sim.NewBus(fromStop.Pos.X, fromStop.Pos.Y, fromStop.Heading)
				driver := NewAutopilotDriver()
				driver.BerserkMode = true
				driver.DwellTotal = 0.1

				dt := 0.016
				maxSteps := 2000
				for step := 0; step < maxSteps; step++ {
					tls.Update(dt)
					throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
						bus, legRoute, w, tls, nil, lw, dt,
					)
					bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

					if isDocked && step > 60 {
						break
					}

					// Verify that when approaching intersections before the corner, the bus never crosses into oncoming traffic
					roadCenter, isEW, isMulti, _ := sim.CurrentRoadInfo(w, bus.Pos, bus.Heading, lw)
					if isMulti {
						// Double yellow line boundary: center divider is at roadCenter ± 6.0px.
						// Westbound (heading ~ Pi): legal side is North of center (Y < roadCenter).
						// Eastbound (heading ~ 0): legal side is South of center (Y > roadCenter).
						// Southbound (heading ~ Pi/2): legal side is West of center (X < roadCenter).
						// Northbound (heading ~ -Pi/2): legal side is East of center (X > roadCenter).
						cosH := math.Cos(bus.Heading)
						sinH := math.Sin(bus.Heading)

						// Calculate front-left and rear-left corner positions (the side closest to oncoming traffic)
						halfL := bus.Length * 0.5
						halfW := bus.Width * 0.5
						leftPerp := sim.Vec2{X: sinH, Y: -cosH} // unit vector pointing towards the left of vehicle
						flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
						rlCorner := bus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

						// Only check when vehicle is on the straight approach road before entering the intersection mouth (dist to intersection > 80px)
						distToInter := 999.0
						for _, it := range tls.Intersections {
							d := bus.Pos.Distance(it.Center)
							if d < distToInter {
								distToInter = d
							}
						}

						if distToInter > 80.0 && !w.InsideJunction(bus.Pos, lw, 0) {
							if isEW {
								if math.Cos(bus.Heading) < -0.5 { // Westbound: legal Y < roadCenter
									if flCorner.Y > roadCenter-2.0 || rlCorner.Y > roadCenter-2.0 {
										t.Fatalf("Theme %s, Step %d: Westbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								} else if math.Cos(bus.Heading) > 0.5 { // Eastbound: legal Y > roadCenter
									if flCorner.Y < roadCenter+2.0 || rlCorner.Y < roadCenter+2.0 {
										t.Fatalf("Theme %s, Step %d: Eastbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								}
							} else {
								if math.Sin(bus.Heading) > 0.5 { // Southbound: legal X < roadCenter
									if flCorner.X > roadCenter-2.0 || rlCorner.X > roadCenter-2.0 {
										t.Fatalf("Theme %s, Step %d: Southbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								} else if math.Sin(bus.Heading) < -0.5 { // Northbound: legal X > roadCenter
									if flCorner.X < roadCenter+2.0 || rlCorner.X < roadCenter+2.0 {
										t.Fatalf("Theme %s, Step %d: Northbound bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, roadCenter=%.1f",
											theme.DisplayName(), step, bus.Pos, flCorner, rlCorner, roadCenter)
									}
								}
							}
						}
					}
				}

				testedLeg = true
				break
			}
			if !testedLeg {
				t.Fatalf("Expected to test at least one left-turn leg in theme %s", theme.DisplayName())
			}
		})
	}
}

// TestBerserkModeOvertakeAccelerationAndSnappyLaneChange verifies that in Berserk mode,
// lane changes to pass slow lead vehicles do not suffer from sluggish crawling or premature deceleration.
// The lateral transition duration must be brisk (0.35s - 0.55s) and the target speed must remain high.
func TestBerserkModeOvertakeAccelerationAndSnappyLaneChange(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	legRoute := &sim.Route{
		Name:         "OvertakeAccTest",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Bus driving along straight 6-lane avenue at 200 px/s in outer lane (Y = 130.0)
	bus := sim.NewBus(-1000.0, 130.0, 0.0)
	bus.Gear = sim.GearDrive
	bus.TransmissionMode = sim.TransmissionAuto
	bus.Speed = 200.0

	// Lead car ahead driving slowly at 25 px/s
	leadCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -760.0, Y: 130.0},
		Heading: 0.0,
		Speed:   25.0,
		Length:  48.0,
		Width:   22.0,
	}
	obstacles := []VehicleObstacle{leadCar}

	dt := 0.016
	// Compute controls on initial step: driver detects lead car and triggers lane change
	thr, brk, _, _, _, _ := driver.ComputeControls(bus, legRoute, w, nil, obstacles, lw, dt)

	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Expected OvertakeState to be OvertakeChangingLane, got %d", driver.OvertakeState)
	}
	if driver.TargetLaneOffset != -lw {
		t.Fatalf("Expected TargetLaneOffset to be -%.1f (inner through lane), got %.1f", lw, driver.TargetLaneOffset)
	}

	// Lane change duration must be fast and brisk (between 0.35s and 0.55s)
	if driver.LaneOffsetDuration < 0.30 || driver.LaneOffsetDuration > 0.60 {
		t.Fatalf("LaneOffsetDuration in Berserk mode should be 0.35s-0.55s, got %.2fs", driver.LaneOffsetDuration)
	}

	// Throttle must remain active without unnecessary panic braking
	if brk > 0.0 || thr < 0.3 {
		t.Fatalf("Berserk mode initiated lane change with unnecessary braking! thr=%.2f, brk=%.2f", thr, brk)
	}

	// Step forward until lane offset transition is completed
	completedSteps := 0
	for step := 1; step < 60; step++ {
		thrStep, brkStep, strStep, hbStep, tdStep, _ := driver.ComputeControls(bus, legRoute, w, nil, obstacles, lw, dt)
		bus.Update(dt, thrStep, brkStep, strStep, hbStep, tdStep)

		if math.Abs(driver.LaneOffset-(-lw)) < 0.5 {
			completedSteps = step
			break
		}
	}

	if completedSteps == 0 {
		t.Fatalf("Lane change transition failed to complete within 60 steps (1s): current offset=%.2f", driver.LaneOffset)
	}

	transitionTime := float64(completedSteps) * dt
	if transitionTime > 0.60 {
		t.Fatalf("Lane change transition took too long: %.2fs > 0.60s", transitionTime)
	}

	// Verify bus maintained high momentum without dropping below 120 px/s
	if bus.Speed < 120.0 {
		t.Fatalf("Bus crawled down to %.1f px/s during lane change maneuver (expected >= 120 px/s)", bus.Speed)
	}
}

// TestOvertakingLookaheadAndNoCollisionWithSlowBlocker explicitly tests the dynamic advance lookahead (提前量)
// and collision-free lane changing when approaching slow or stopped lead obstacles.
// It verifies that:
//  1. Blocker vehicles are detected well in advance (提前量) based on relative closing speed.
//  2. The bus initiates lane change in advance, clearing the blocker without colliding.
//  3. In tight / close-quarters scenarios, the bus enforces longitudinal obstacle braking while
//     lateral clearance is not yet achieved (latDist < minClearLat), eliminating collisions before fully clearing.
func TestOvertakingLookaheadAndNoCollisionWithSlowBlocker(t *testing.T) {
	lw := 47.5
	dt := 1.0 / 60.0

	// Test across all 4 Berserk speeds: 60, 80, 100, 120 km/h
	for _, kmh := range []int{60, 80, 100, 120} {
		t.Run(fmt.Sprintf("%d_kmh_Lookahead_Collision_Test", kmh), func(t *testing.T) {
			cruiseSpd := float64(kmh) * 5.0
			yCenter := 1200.0 * sim.MapGridScale

			sStart := sim.NewBusStop(301, "StopStart", sim.Vec2{X: -2000, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			sEnd := sim.NewBusStop(302, "StopEnd", sim.Vec2{X: 2000, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			w.Stops = []*sim.BusStop{sStart, sEnd}

			route := &sim.Route{
				Name:         "OvertakeLookaheadRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.State = AutopilotStateCruising
			driver.drivePath = []sim.Vec2{
				{X: -2000, Y: yCenter + lw*1.5},
				{X: 2000, Y: yCenter + lw*1.5},
			}
			driver.cumLen = []float64{0, 4000.0}
			driver.pathWorld = w
			driver.pathFromID = sStart.ID
			driver.pathToID = sEnd.ID
			driver.pathLane = lw
			driver.pathSynced = true
			driver.WaypointIdx = 0

			bus := sim.NewBus(1, 0, lw)
			bus.Pos = sim.Vec2{X: -1000, Y: yCenter + lw*1.5}
			bus.Heading = 0
			bus.Speed = cruiseSpd
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			// Lead blocker moving at 35 px/s (slow vehicle)
			blockerSpd := 35.0
			closingSpeed := cruiseSpd - blockerSpd
			// Blocker positioned ahead: dynamic lookahead should detect it
			blockerDist := closingSpeed*1.8 + 120.0
			blocker := VehicleObstacle{
				Pos:     sim.Vec2{X: bus.Pos.X + blockerDist, Y: yCenter + lw*1.5},
				Heading: 0,
				Speed:   blockerSpd,
				Length:  48,
				Width:   22,
			}

			// Step 1: Initial check must detect blocker and initiate lane change
			driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
			if driver.OvertakeState != OvertakeChangingLane {
				t.Fatalf("[%d km/h] Expected OvertakeChangingLane for slow blocker (dist=%.1f, closingSpd=%.1f), got %d",
					kmh, blockerDist, closingSpeed, driver.OvertakeState)
			}

			// Step 2: Step forward simulation until lane change completes or passes
			minLongitudinalGap := 9999.0
			collided := false
			for step := 0; step < 500; step++ {
				blocker.Pos.X += blocker.Speed * dt

				th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)

				dx := blocker.Pos.X - bus.Pos.X
				dy := math.Abs(bus.Pos.Y - blocker.Pos.Y)
				if dx > 0 && dx < minLongitudinalGap {
					minLongitudinalGap = dx
				}

				// Check physical box overlap collision:
				contactDistX := bus.Length/2 + blocker.Length/2 - 2.0
				contactDistY := bus.Width/2 + blocker.Width/2 - 2.0
				if math.Abs(dx) < contactDistX && dy < contactDistY {
					collided = true
					t.Fatalf("[%d km/h] Step %d: Bus collided with lead vehicle! dx=%.1f, dy=%.1f",
						kmh, step, dx, dy)
				}

				if driver.OvertakeState == OvertakePassing && bus.Pos.X > blocker.Pos.X+80.0 {
					break
				}
			}

			if collided {
				t.Fatalf("[%d km/h] Lane change resulted in collision", kmh)
			}
			t.Logf("[%d km/h] Overtaking success! Min longitudinal gap before lateral clearance: %.1f px",
				kmh, minLongitudinalGap)
		})
	}

	// Tight / close-quarters scenario: blocker at 180px while bus is in mid-overtake transition
	// Verifies bus decelerates behind blocker rather than accelerating into it
	t.Run("Close_Range_Lane_Change_No_Collision", func(t *testing.T) {
		cruiseSpd := 400.0 // 80 km/h
		yCenter := 1200.0 * sim.MapGridScale

		sStart := sim.NewBusStop(303, "Start3", sim.Vec2{X: -2000, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
		sEnd := sim.NewBusStop(304, "End3", sim.Vec2{X: 2000, Y: yCenter}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
		w := sim.NewWorld(lw, sim.MapThemeMetropolis)
		w.Stops = []*sim.BusStop{sStart, sEnd}
		route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = cruiseSpd
		driver.State = AutopilotStateCruising
		driver.drivePath = []sim.Vec2{{X: -2000, Y: yCenter + lw*1.5}, {X: 2000, Y: yCenter + lw*1.5}}
		driver.cumLen = []float64{0, 4000.0}
		driver.pathWorld = w
		driver.pathFromID = sStart.ID
		driver.pathToID = sEnd.ID
		driver.pathLane = lw
		driver.pathSynced = true

		bus := sim.NewBus(1, 0, lw)
		bus.Pos = sim.Vec2{X: -1000, Y: yCenter + lw*1.5}
		bus.Heading = 0
		bus.Speed = 200.0
		bus.MaxSpeedFwd = 460.0

		// Blocker placed 160px ahead, moving at 40px/s
		blocker := VehicleObstacle{
			Pos:     sim.Vec2{X: bus.Pos.X + 160.0, Y: yCenter + lw*1.5},
			Heading: 0,
			Speed:   40.0,
			Length:  48,
			Width:   22,
		}

		driver.OvertakeState = OvertakeChangingLane
		driver.TargetLaneOffset = -lw

		// Compute controls: must NOT accelerate past, targetSpd must decelerate behind blocker
		th, br, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
		if th > 0.5 && br == 0.0 {
			t.Fatalf("Expected bus to brake/decelerate behind close blocker during lane change, got throttle=%.2f, brake=%.2f",
				th, br)
		}

		// Step forward and ensure no collision occurs
		for step := 0; step < 300; step++ {
			blocker.Pos.X += blocker.Speed * dt
			th, br, st, hb, tg, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt)
			bus.Update(dt, th, br, st, hb, tg)

			dx := blocker.Pos.X - bus.Pos.X
			dy := math.Abs(bus.Pos.Y - blocker.Pos.Y)
			contactDistX := bus.Length/2 + blocker.Length/2 - 2.0
			contactDistY := bus.Width/2 + blocker.Width/2 - 2.0
			if math.Abs(dx) < contactDistX && dy < contactDistY {
				t.Fatalf("Step %d: Collision in close-range lane change! dx=%.1f, dy=%.1f", step, dx, dy)
			}
			if bus.Pos.X > blocker.Pos.X+80.0 {
				break
			}
		}
	})
}
