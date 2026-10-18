package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestBerserkModeBorrowLaneOvertakingOn2LaneRoad tests 借道超车 in Berserk Mode on 2-way 2-lane roads:
// 1. Standard mode: cannot borrow oncoming lane
// 2. Berserk mode with oncoming vehicle: blocked from borrowing lane
// 3. Berserk mode with oncoming lane clear: borrows oncoming lane (-lw) to overtake
// 4. Smooth lateral transition into oncoming lane
// 5. Passing slow car and safely returning to base lane (offset 0.0)
// 6. Active abort: if an oncoming car appears while in oncoming lane, aborts back to base lane
// 7. Verification across all Berserk top speed settings (60, 80, 100, 120 km/h)
func TestBerserkModeBorrowLaneOvertakingOn2LaneRoad(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// Stops along 2-lane road "北二路" (Y = -600 * MapGridScale, Eastbound)
	// Road segment: P1: {-2600*MapGridScale, -600*MapGridScale}, P2: {2600*MapGridScale, -600*MapGridScale}, Lanes: 2, Type: RoadStreet2Lane
	// Travel lane (Eastbound) is at Y = roadY + lw*0.5
	// Oncoming lane (Westbound) is at Y = roadY - lw*0.5
	roadY := -600 * sim.MapGridScale
	s1 := sim.NewBusStop(401, "2LaneStart", sim.Vec2{X: -1000 * sim.MapGridScale, Y: roadY}, 0, 2, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(402, "2LaneEnd", sim.Vec2{X: 1000 * sim.MapGridScale, Y: roadY}, 0, 2, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "2LaneBorrowRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	startX := -600 * sim.MapGridScale
	bus := sim.NewBus(1, 0, lw)
	bus.Pos = sim.Vec2{X: startX, Y: roadY + lw*0.5} // Base travel lane
	bus.Heading = 0                                  // Eastbound
	bus.Speed = 160.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()

	// Slow blocker in our travel lane ahead
	slowCar := VehicleObstacle{
		Pos:     sim.Vec2{X: startX + 90, Y: roadY + lw*0.5}, // 90px ahead
		Heading: 0,
		Speed:   40.0,
		Length:  48,
		Width:   22,
	}

	// 1. Standard Mode: should NOT borrow oncoming lane
	driver.BerserkMode = false
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone || driver.LaneOffset != 0.0 {
		t.Fatalf("Standard mode must not borrow oncoming lane (state=%d, offset=%.2f)",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 2. Berserk Mode with oncoming vehicle in oncoming lane: MUST NOT borrow lane
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	oncomingCar := VehicleObstacle{
		Pos:     sim.Vec2{X: startX + 400, Y: roadY - lw*0.5}, // Ahead in oncoming lane (400px ahead)
		Heading: math.Pi,                                      // Heading Westbound (oncoming!)
		Speed:   80.0,
		Length:  48,
		Width:   22,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, oncomingCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeNone {
		t.Fatalf("Must not borrow lane when oncoming lane has oncoming car (got state %d)", driver.OvertakeState)
	}

	// 3. Berserk Mode with oncoming lane CLEAR: initiates 借道超车
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakeChangingLane {
		t.Fatalf("Berserk mode must initiate 借道超车 when oncoming lane is clear (got state %d)", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Target lane offset should be -%.1f (oncoming lane), got %.2f", lw, driver.TargetLaneOffset)
	}

	// 4. Progress into oncoming lane
	for i := 0; i < 20; i++ {
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.05)
	}
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected OvertakePassing once arrived in oncoming lane, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.LaneOffset-(-lw)) > 1.0 {
		t.Fatalf("Expected lane offset -%.1f in oncoming lane, got %.2f", lw, driver.LaneOffset)
	}

	// 5. Still alongside slowCar: should stay in passing lane
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
	if driver.OvertakeState != OvertakePassing {
		t.Fatalf("Must remain in OvertakePassing while alongside blocker, got %d", driver.OvertakeState)
	}

	// 6. Passed slowCar: safely return to base lane
	bus.Pos.X = startX + 260 // 170px ahead of slowCar at startX+90 (beyond minSafeBehind ~147px)
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.5)
	if driver.OvertakeState != OvertakeReturning {
		t.Fatalf("Expected OvertakeReturning once vehicle passed, got %d", driver.OvertakeState)
	}
	if math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Target offset after passing should return to 0, got %.2f", driver.TargetLaneOffset)
	}

	// 7. Advance until returned to base lane
	for i := 0; i < 25; i++ {
		driver.ComputeControls(bus, route, w, nil, nil, lw, 0.08)
	}
	if driver.OvertakeState != OvertakeNone || math.Abs(driver.LaneOffset) > 0.1 {
		t.Fatalf("Expected return to OvertakeNone and 0 offset, got state %d, offset %.2f",
			driver.OvertakeState, driver.LaneOffset)
	}

	// 8. Active abort: if oncoming car appears while in oncoming lane, aborts to base lane
	driver.Reset()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	bus.Pos = sim.Vec2{X: startX, Y: roadY - lw*0.5} // Physically in oncoming lane
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.OvertakeState = OvertakePassing
	imminentOncoming := VehicleObstacle{
		Pos:     sim.Vec2{X: startX + 300, Y: roadY - lw*0.5}, // 300px ahead in oncoming lane
		Heading: math.Pi,                                      // Oncoming
		Speed:   90.0,
		Length:  48,
		Width:   22,
	}
	// Base lane is clear to return
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{imminentOncoming}, lw, 0.016)
	if driver.OvertakeState != OvertakeReturning || math.Abs(driver.TargetLaneOffset) > 0.1 {
		t.Fatalf("Must abort back to base lane when oncoming car appears (state=%d, target=%.2f)",
			driver.OvertakeState, driver.TargetLaneOffset)
	}

	// 9. Verify across all 4 Berserk top speed settings (60, 80, 100, 120 km/h)
	speedConfigs := []struct {
		kmh       int
		cruiseSpd float64
	}{
		{60, 333.0},
		{80, 445.0},
		{100, 555.0},
		{120, 667.0},
	}
	for _, sc := range speedConfigs {
		driver.Reset()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = sc.cruiseSpd
		driver.State = AutopilotStateCruising
		bus.Pos = sim.Vec2{X: startX, Y: roadY + lw*0.5}
		bus.Speed = sc.cruiseSpd

		// With oncoming car: cannot borrow lane
		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar, oncomingCar}, lw, 0.016)
		if driver.OvertakeState != OvertakeNone {
			t.Fatalf("MaxSpeed %d: must not borrow lane with oncoming traffic (got %d)", sc.kmh, driver.OvertakeState)
		}

		// When oncoming lane is clear: can borrow lane
		driver.Reset()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = sc.cruiseSpd
		driver.State = AutopilotStateCruising
		bus.Pos = sim.Vec2{X: startX, Y: roadY + lw*0.5}
		bus.Speed = sc.cruiseSpd

		driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{slowCar}, lw, 0.016)
		if driver.OvertakeState != OvertakeChangingLane {
			t.Fatalf("MaxSpeed %d: expected OvertakeChangingLane when oncoming lane clear, got %d", sc.kmh, driver.OvertakeState)
		}
		if math.Abs(driver.TargetLaneOffset-(-lw)) > 1.0 {
			t.Fatalf("MaxSpeed %d: expected target offset -%.1f, got %.2f", sc.kmh, lw, driver.TargetLaneOffset)
		}
	}
}

// TestAutopilotRightTurnInto2LaneRoad verifies that when a bus makes a 90-degree right turn
// into a 2-lane road (1 lane in each direction, 52px wide), the vehicle's turning radius is
// sufficiently tight such that neither the bus center nor its front-left overhang crosses the
// road centerline into the oncoming traffic lane.
func TestAutopilotRightTurnInto2LaneRoad(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// In Metropolis:
	// Eastbound on 北二路 (Y = -1800.0, lane center Y = -1774.0)
	// Right turn onto 东环路 (X = 3900.0, 2-lane road, Southbound lane center X = 3874.0, Oncoming Northbound X > 3900.0).
	// Road centerline is X = 3900.0.
	legRoute := &sim.Route{
		Name:         "TestRightTurnRoute",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	bus := sim.NewBus(3600.0, -1774.0, 0.0) // 300px before intersection at (3900, -1800)
	bus.Gear = sim.GearDrive
	bus.TransmissionMode = sim.TransmissionAuto
	bus.Speed = 50.0

	dt := 0.016
	maxFLX := -1e9
	for step := 0; step < 600; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, legRoute, w, nil, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Calculate front-left corner coordinates of the bus envelope
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := sim.Vec2{X: sinH, Y: -cosH}
		flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		if bus.Pos.X > 3800.0 {
			if flCorner.X > maxFLX {
				maxFLX = flCorner.X
			}
		}

		// Once bus has completed right turn and traveled downstream along X = 3874
		if bus.Pos.Y > -1400.0 {
			break
		}
	}

	roadCenterlineX := 3900.0
	if maxFLX > roadCenterlineX {
		t.Fatalf("Bus right turn intruded into oncoming traffic lane! Max front-left X=%.2f, centerline=%.2f (intrusion=%.2fpx)",
			maxFLX, roadCenterlineX, maxFLX-roadCenterlineX)
	}

	// Verify comfortable safety margin inside its own lane
	safetyMargin := roadCenterlineX - maxFLX
	if safetyMargin < 2.0 {
		t.Errorf("Safety margin to centerline too small: %.2fpx", safetyMargin)
	}
}

// TestAutopilotBerserkRightTurnInto2LaneDoesNotOvertakeIntoOppositeShoulder verifies that when the bus
// executes a right turn into a 2-way 2-lane road with a slow vehicle downstream, it does NOT initiate
// borrow-lane overtaking during the turn or before straightening out (which would swing into the opposite shoulder).
// Once straightened out in the travel lane, it initiates borrow-lane overtaking, passes the slow vehicle,
// and returns safely without ever touching the opposite shoulder.
func TestAutopilotBerserkRightTurnInto2LaneDoesNotOvertakeIntoOppositeShoulder(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	legRoute := &sim.Route{
		Name:         "TestRightTurnRoute",
		StopIDs:      []int{0, 2},
		CurrentIndex: 1,
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Bus starts 300px before right turn intersection at (3900, -1800)
	bus := sim.NewBus(3600.0, -1774.0, 0.0)
	bus.Gear = sim.GearDrive
	bus.TransmissionMode = sim.TransmissionAuto
	bus.Speed = 60.0

	// Downstream slow vehicle on 2-lane road (Southbound lane center X = 3874.0)
	slowCar := VehicleObstacle{
		Pos:     sim.Vec2{X: 3874.0, Y: -1450.0},
		Heading: math.Pi / 2,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	oppShoulderX := 3900.0 + lw // 3952.0 (East shoulder of 2-lane road)
	maxFLX := -1e9
	overtakeInitiatedAfterStraight := false

	for step := 0; step < 800; step++ {
		// Update slow vehicle
		slowCar.Pos.Y += slowCar.Speed * dt

		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, legRoute, w, nil, []VehicleObstacle{slowCar}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// 1. Verify physical boundaries: bus must never be on shoulder or crash barrier
		bRes := w.CheckBoundaries(bus.Pos, bus.Width/2, lw)
		if bRes.Zone == sim.ZoneShoulder || bRes.Zone == sim.ZoneHardBarrier || bus.OnShoulder {
			t.Fatalf("Step %d: Bus drove onto shoulder/barrier! Pos=%v, Zone=%v, OnShoulder=%v, State=%v, LaneOffset=%.2f",
				step, bus.Pos, bRes.Zone, bus.OnShoulder, driver.OvertakeState, driver.LaneOffset)
		}

		// 2. Track front-left and rear-left corner coordinates on the 2-lane road
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := sim.Vec2{X: sinH, Y: -cosH}

		flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
		rlCorner := bus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		if bus.Pos.Y > -1750.0 && bus.Pos.X > 3800.0 {
			if flCorner.X > maxFLX {
				maxFLX = flCorner.X
			}
			if rlCorner.X > maxFLX {
				maxFLX = rlCorner.X
			}
			// Strict opposite shoulder check: bus body must NEVER cross or touch opposite shoulder (3952.0)
			if flCorner.X >= oppShoulderX || rlCorner.X >= oppShoulderX {
				t.Fatalf("Step %d: Bus corner penetrated opposite shoulder! maxFLX=%.2f, oppShoulderX=%.2f, busPos=%v, Heading=%.2f, State=%v",
					step, maxFLX, oppShoulderX, bus.Pos, bus.Heading, driver.OvertakeState)
			}
		}

		// 3. During the right turn itself (before vehicle has cleared the intersection turn at Y = -1650):
		// Autopilot must NOT initiate borrow-lane overtaking!
		if bus.Pos.Y < -1650.0 {
			if driver.OvertakeState != OvertakeNone {
				t.Fatalf("Step %d: Initiated overtaking prematurely during right turn! State=%v, Heading=%.2f, Pos=%v",
					step, driver.OvertakeState, bus.Heading, bus.Pos)
			}
		} else {
			// Once straightened out in the Southbound lane, borrow-lane overtake should initiate
			if driver.OvertakeState == OvertakeChangingLane || driver.OvertakeState == OvertakePassing {
				overtakeInitiatedAfterStraight = true
			}
		}

		// Finish once passed slow vehicle
		if bus.Pos.Y > slowCar.Pos.Y+120.0 {
			break
		}
	}

	if !overtakeInitiatedAfterStraight {
		t.Fatalf("Expected borrow-lane overtake to initiate after completing the right turn and straightening out")
	}
}
