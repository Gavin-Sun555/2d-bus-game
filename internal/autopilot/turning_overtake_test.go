package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestBerserkModeOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode, when overtaking a vehicle
// on a multi-lane road where the upcoming intersection requires a left turn:
// 1. After passing or while approaching, the bus does NOT return to the outer right lane.
// 2. The bus seamlessly continues forward directly into the dedicated left-turn lane.
// 3. The bus NEVER overshoots across the double yellow line into the oncoming lane during or after the merge taper.
func TestBerserkModeOvertakingIntoUpcomingLeftTurn(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	w.Roundabouts = nil

	// Eastbound along Y = 0 (Metropolis Central Boulevard, 4-lane avenue), turning North (Left) onto X = 0 (North-South avenue).
	// Legal travel is at Y > 0 (outer lane = 1.5*lw = 78, left-turn lane = 0.5*lw = 26).
	// Centerline is at Y = 0.0; oncoming Westbound traffic is at Y < 0.
	sStart := sim.NewBusStop(0, "StartEast", sim.Vec2{X: -600 * sim.MapGridScale, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0) // X = -1800
	sEnd := sim.NewBusStop(1, "EndNorth", sim.Vec2{X: 0, Y: -600 * sim.MapGridScale}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{sStart, sEnd}

	route := &sim.Route{
		Name:         "LeftTurnOvertakeRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1,
	}

	bus := sim.NewBus(-1400, lw*1.5, 0) // in outer lane (Y = 78), 1400px before X=0
	bus.Speed = 160.0
	bus.Gear = sim.GearDrive

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising

	// Slow vehicle in outer lane ahead
	slowCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -1050, Y: lw * 1.5},
		Heading: 0,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	tls := &sim.TrafficLightSystem{}
	inter := &sim.TrafficIntersection{
		ID:                0,
		Center:            sim.Vec2{X: 0, Y: 0},
		IsMajor:           true,
		ApproachHasRoad:   [4]bool{true, true, true, true},
		ApproachLanes:     [4]int{4, 4, 4, 4},
		ApproachIs4Lane:   [4]bool{true, true, true, true},
		Phase:             sim.PhaseEWLeftGreen, // Left green
		GreenDuration:     15.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 15.0,
	}
	tls.Intersections = []*sim.TrafficIntersection{inter}

	// Step through simulation
	dt := 0.016
	passedObstacle := false

	for step := 0; step < 400; step++ {
		slowCar.Pos.X += slowCar.Speed * dt
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, tls, []VehicleObstacle{slowCar}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Calculate front-left and rear-left corners (closest to centerline)
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		halfL := bus.Length * 0.5
		halfW := bus.Width * 0.5
		leftPerp := sim.Vec2{X: sinH, Y: -cosH}
		flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
		rlCorner := bus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

		// Double yellow line check: while approaching intersection before the mouth (>80px),
		// the bus must NEVER cross into oncoming traffic (legal side is Y > 0).
		distToInter := math.Abs(bus.Pos.X)
		if distToInter > 80.0 && !w.InsideJunction(bus.Pos, lw, 0) {
			if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
				t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v, LaneOffset=%.2f, TargetLaneOffset=%.2f",
					step, bus.Pos, flCorner, rlCorner, driver.LaneOffset, driver.TargetLaneOffset)
			}
		}

		// Once bus has passed the obstacle (bus.Pos.X > slowCar.Pos.X + 60):
		if bus.Pos.X > slowCar.Pos.X+60.0 {
			passedObstacle = true
			// Bus must NOT return to outer lane (Y must not veer back towards 78)
			if driver.OvertakeState == OvertakeReturning && driver.TargetLaneOffset == 0.0 && bus.Pos.Y > lw*1.2 {
				t.Fatalf("Step %d: Bus attempted to return to outer lane despite upcoming left turn! Pos=%v", step, bus.Pos)
			}
		}

		// Stop when bus reaches intersection mouth
		if bus.Pos.X >= -70.0 {
			break
		}
	}

	if !passedObstacle {
		t.Fatalf("Bus failed to overtake slowCar before intersection")
	}

	// Verify that the bus positioned safely in the dedicated left-turn lane (Y near 0.5*lw = 26)
	if bus.Pos.Y > lw*1.0 {
		t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f (expected near %.1f)", bus.Pos.Y, lw*0.5)
	}
}

// TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode on a 6-lane road:
// When overtaking or cruising where the upcoming intersection requires a left turn:
// TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn verifies that in Berserk Mode on a 6-lane road:
// When overtaking or cruising where the upcoming intersection requires a left turn:
// 1. The bus does NOT swerve into oncoming traffic (Y < 0).
// 2. The bus does NOT perform an S-curve (swerving right toward outer lanes then swerving left).
// 3. The bus smoothly transitions into the dedicated left-turn lane (Y ~ 0.5*lw = 26).
func TestBerserkMode6LaneOvertakingIntoUpcomingLeftTurn(t *testing.T) {
	lw := 52.0

	t.Run("OvertakingTrafficIntoUpcomingLeftTurn", func(t *testing.T) {
		w := sim.NewWorld(lw, sim.MapThemeCoastalBay)
		w.Roundabouts = nil

		sStart := sim.NewBusStop(0, "StartEast6Lane", sim.Vec2{X: -600 * sim.MapGridScale, Y: 0}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0) // X = -1800
		sEnd := sim.NewBusStop(1, "EndNorth", sim.Vec2{X: 0, Y: -600 * sim.MapGridScale}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
		w.Stops = []*sim.BusStop{sStart, sEnd}

		route := &sim.Route{
			Name:         "LeftTurn6LaneOvertakeRoute",
			StopIDs:      []int{0, 1},
			CurrentIndex: 1,
		}

		bus := sim.NewBus(-1400, lw*2.5, 0) // in outer lane (Y = 130), 1400px before X=0
		bus.Speed = 220.0
		bus.Gear = sim.GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising

		slowOuter := VehicleObstacle{
			Pos:     sim.Vec2{X: -950, Y: lw * 2.5},
			Heading: 0,
			Speed:   35.0,
			Length:  48,
			Width:   22,
		}
		slowMid := VehicleObstacle{
			Pos:     sim.Vec2{X: -900, Y: lw * 1.5},
			Heading: 0,
			Speed:   35.0,
			Length:  48,
			Width:   22,
		}
		obstacles := []VehicleObstacle{slowOuter, slowMid}

		tls := &sim.TrafficLightSystem{}
		inter := &sim.TrafficIntersection{
			ID:                0,
			Center:            sim.Vec2{X: 0, Y: 0},
			IsMajor:           true,
			ApproachHasRoad:   [4]bool{true, true, true, true},
			ApproachLanes:     [4]int{6, 4, 6, 4},
			ApproachIs4Lane:   [4]bool{true, true, true, true},
			Phase:             sim.PhaseEWLeftGreen,
			GreenDuration:     15.0,
			YellowDuration:    2.5,
			LeftGreenDuration: 15.0,
		}
		tls.Intersections = []*sim.TrafficIntersection{inter}

		dt := 0.016
		passedObstacle := false
		enteredLeftTurnCorridor := false

		for step := 0; step < 500; step++ {
			for i := range obstacles {
				obstacles[i].Pos.X += obstacles[i].Speed * dt
			}
			throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
				bus, route, w, tls, obstacles, lw, dt,
			)
			bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

			cosH := math.Cos(bus.Heading)
			sinH := math.Sin(bus.Heading)
			halfL := bus.Length * 0.5
			halfW := bus.Width * 0.5
			leftPerp := sim.Vec2{X: sinH, Y: -cosH}
			flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
			rlCorner := bus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

			distToInter := math.Abs(bus.Pos.X)
			if distToInter > 80.0 && !w.InsideJunction(bus.Pos, lw, 0) {
				if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
					t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v",
						step, bus.Pos, flCorner, rlCorner)
				}
			}

			if distToInter <= 600.0 && bus.Pos.Y <= lw*1.15 {
				enteredLeftTurnCorridor = true
			}
			if enteredLeftTurnCorridor && distToInter > 80.0 {
				if bus.Pos.Y > lw*1.35 {
					t.Fatalf("Step %d: Bus swerved back right (S-curve) after entering left-turn corridor! Pos=%v",
						step, bus.Pos)
				}
			}

			if bus.Pos.X > obstacles[1].Pos.X+60.0 {
				passedObstacle = true
			}
			if bus.Pos.X >= -70.0 {
				break
			}
		}

		if !passedObstacle {
			t.Fatalf("Bus failed to overtake slow vehicles before intersection: pos%v speed%.1f offset%.1f target%.1f", bus.Pos, bus.Speed, driver.LaneOffset, driver.TargetLaneOffset)
		}
		if bus.Pos.Y > lw*1.0 {
			t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f", bus.Pos.Y)
		}
	})

	t.Run("MiddleLaneApproachSmoothTransitionNoSCurve", func(t *testing.T) {
		w := sim.NewWorld(lw, sim.MapThemeCoastalBay)
		w.Roundabouts = nil

		sStart := sim.NewBusStop(0, "StartEast6Lane", sim.Vec2{X: -600, Y: 0}, 0, 6, sim.RGBA(0, 0, 0, 0), lw, 0)
		sEnd := sim.NewBusStop(1, "EndNorth", sim.Vec2{X: 0, Y: -600}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
		w.Stops = []*sim.BusStop{sStart, sEnd}

		route := &sim.Route{
			Name:         "LeftTurn6LaneRoute",
			StopIDs:      []int{0, 1},
			CurrentIndex: 1,
		}

		bus := sim.NewBus(-500, lw*1.5, 0) // in middle lane (Y = 78), 500px before X=0
		bus.Speed = 220.0
		bus.Gear = sim.GearDrive

		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.State = AutopilotStateCruising
		driver.LaneOffset = -lw
		driver.TargetLaneOffset = -lw
		driver.lastTargetLaneOffset = -lw

		tls := &sim.TrafficLightSystem{}
		inter := &sim.TrafficIntersection{
			ID:                0,
			Center:            sim.Vec2{X: 0, Y: 0},
			IsMajor:           true,
			ApproachHasRoad:   [4]bool{true, true, true, true},
			ApproachLanes:     [4]int{6, 4, 6, 4},
			ApproachIs4Lane:   [4]bool{true, true, true, true},
			Phase:             sim.PhaseEWLeftGreen,
			GreenDuration:     15.0,
			YellowDuration:    2.5,
			LeftGreenDuration: 15.0,
		}
		tls.Intersections = []*sim.TrafficIntersection{inter}

		dt := 0.016
		enteredLeftTurnCorridor := false

		for step := 0; step < 500; step++ {
			throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
				bus, route, w, tls, nil, lw, dt,
			)
			bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

			cosH := math.Cos(bus.Heading)
			sinH := math.Sin(bus.Heading)
			halfL := bus.Length * 0.5
			halfW := bus.Width * 0.5
			leftPerp := sim.Vec2{X: sinH, Y: -cosH}
			flCorner := bus.Pos.Add(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))
			rlCorner := bus.Pos.Sub(sim.Vec2{X: cosH * halfL, Y: sinH * halfL}).Add(leftPerp.Mul(halfW))

			distToInter := math.Abs(bus.Pos.X)
			if distToInter > 80.0 && !w.InsideJunction(bus.Pos, lw, 0) {
				if flCorner.Y < 0.0 || rlCorner.Y < 0.0 {
					t.Fatalf("Step %d: Bus crossed double yellow line into oncoming lane! Pos=%v, fl=%v, rl=%v",
						step, bus.Pos, flCorner, rlCorner)
				}
			}

			if bus.Pos.Y <= lw*1.15 {
				enteredLeftTurnCorridor = true
			}
			if enteredLeftTurnCorridor && distToInter > 80.0 {
				if bus.Pos.Y > lw*1.35 {
					t.Fatalf("Step %d: Bus swerved back right (S-curve) after entering left-turn corridor! Pos=%v",
						step, bus.Pos)
				}
			}

			if bus.Pos.X >= -70.0 {
				break
			}
		}

		if bus.Pos.Y > lw*1.0 {
			t.Fatalf("Bus failed to enter left-turn lane before intersection mouth, got Y=%.2f", bus.Pos.Y)
		}
	})
}
