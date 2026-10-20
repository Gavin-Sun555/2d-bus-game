package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestRoundaboutBusDrivesInOuterLane simulates an autopilot bus driving through the roundabout
// and verifies that the bus stays centered in the outer lane without straddling the middle divider line.
func TestRoundaboutBusDrivesInOuterLane(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	midR := (rApron + rOuter) * 0.5 // 188.0px
	_ = midR

	// Spawn approaching roundabout on West entrance avenue (at rb.Center.X - 550px)
	bus := sim.NewBus(rb.Center.X-550, 78.0, 0)
	bus.Speed = 60.0
	ad := NewAutopilotDriver()

	r := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	dt := 0.016
	inRoundaboutCount := 0
	minDistToCenter := math.MaxFloat64
	maxDistToCenter := 0.0

	for step := 0; step < 800; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := ad.ComputeControls(
			bus, r, w, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		d := bus.Pos.Distance(rb.Center)

		// Check while bus is traversing within the circular carriageway
		if d < rb.Radius-15.0 && d > rApron+10.0 {
			inRoundaboutCount++
			if d < minDistToCenter {
				minDistToCenter = d
			}
			if d > maxDistToCenter {
				maxDistToCenter = d
			}

			// Bus center must remain strictly within the outer lane (midR=188 to rOuter=240)
			if d < midR+5.0 || d > rOuter-5.0 {
				t.Fatalf("Step %d: Bus center (dist=%.2f) strayed outside outer lane [%.2f, %.2f]! Pos=(%.1f, %.1f)",
					step, d, midR, rOuter, bus.Pos.X, bus.Pos.Y)
			}

			// Bus inner edge must not cross the circular dashed divider line into the inner lane
			busInnerEdgeDist := d - bus.Width*0.5
			if busInnerEdgeDist < midR {
				t.Fatalf("Step %d: Bus inner edge (dist=%.2f) crossed dashed divider (%.2f)! Pos=(%.1f, %.1f)",
					step, busInnerEdgeDist, midR, bus.Pos.X, bus.Pos.Y)
			}
		}

		if bus.Pos.X > rb.Center.X+rb.Radius+100 {
			break
		}
	}

	if inRoundaboutCount < 30 {
		t.Fatalf("Bus did not traverse roundabout enough steps, inRoundaboutCount=%d", inRoundaboutCount)
	}

	// Traversal bounds must remain strictly inside outer lane
	if minDistToCenter < midR+10.0 {
		t.Errorf("Minimum distance (%.2f) too close to dashed lane line (%.2f)", minDistToCenter, midR)
	}
	if maxDistToCenter > rOuter-10.0 {
		t.Errorf("Maximum distance (%.2f) too close to outer curb (%.2f)", maxDistToCenter, rOuter)
	}
}

// TestRoundaboutNoDeadlockWithYieldingCar tests that a bus circulating inside a roundabout
// does not stop or deadlock with a civilian car waiting at the entrance branch yield line.
func TestRoundaboutNoDeadlockWithYieldingCar(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	s0 := w.Stops[0]
	s1 := w.Stops[1]
	_ = s0
	_ = s1
	rb := w.Roundabouts[0]

	// Place bus inside roundabout on the East side (radius 214px), heading North (-Pi/2)
	bus := sim.NewBus(rb.Center.X+214.0, rb.Center.Y-20.0, -math.Pi/2)
	bus.Speed = 60.0

	// Place civilian car waiting at the East entrance yield line outside the roundabout
	yieldingCar := VehicleObstacle{
		Pos:     sim.Vec2{X: rb.Center.X + 258.0, Y: rb.Center.Y - 40.0},
		Heading: math.Pi, // Facing West into roundabout
		Speed:   0.0,
		Length:  36.0,
		Width:   18.0,
	}

	r := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	driver := NewAutopilotDriver()

	dt := 0.016
	stuckSteps := 0

	for step := 0; step < 120; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, r, w, nil, []VehicleObstacle{yieldingCar}, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if bus.Speed < 1.0 {
			stuckSteps++
		}
	}

	if stuckSteps > 5 {
		t.Fatalf("Bus got stuck in deadlock for %d steps with vehicle waiting outside roundabout! Final pos=(%.1f, %.1f), speed=%.2f",
			stuckSteps, bus.Pos.X, bus.Pos.Y, bus.Speed)
	}

	// Bus should have successfully circulated North past the East entrance
	if bus.Pos.Y > rb.Center.Y-40.0 {
		t.Fatalf("Bus failed to progress past East entrance: Pos.Y=%.1f", bus.Pos.Y)
	}
}

// TestRoundaboutAutopilotBrakesForEncroachingCar verifies that when an autopilot bus
// is circulating inside a roundabout, it properly brakes and stops before a car that is
// encroaching at the entrance mouth, rather than ignoring it or ramming into it.
func TestRoundaboutAutopilotBrakesForEncroachingCar(t *testing.T) {
	w := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	r := &sim.Route{StopIDs: []int{1, 2}, CurrentIndex: 1}
	driver := NewAutopilotDriver()

	// Place bus circulating inside roundabout approaching East entrance (X=214, Y=+30), heading North (-Pi/2)
	bus := sim.NewBus(rb.Center.X+214.0, rb.Center.Y+30.0, -math.Pi/2)
	bus.Speed = 40.0

	// Place a car encroaching at the East entrance mouth ahead of the bus (X=225, Y=-55)
	encroachingCar := VehicleObstacle{
		Pos:     sim.Vec2{X: rb.Center.X + 225.0, Y: rb.Center.Y - 55.0},
		Heading: math.Pi, // Facing West
		Speed:   0.0,
		Length:  48.0,
		Width:   22.0,
	}

	dt := 0.016
	// Advance 80 frames: bus should recognize obstacle and brake to a stop before ramming
	for frame := 0; frame < 80; frame++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, r, w, nil, []VehicleObstacle{encroachingCar}, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)
	}

	if bus.Speed > 5.0 {
		t.Fatalf("Autopilot bus failed to brake for encroaching car! Speed = %.2f", bus.Speed)
	}
	distToCar := bus.Pos.Distance(encroachingCar.Pos)
	if distToCar < (bus.Length/2 + encroachingCar.Length/2) {
		t.Fatalf("Autopilot bus collided with encroaching car! Distance = %.2f px", distToCar)
	}
}

// TestAutopilotBerserkRoundaboutFollowsSlowCarWithoutPassing verifies that in Berserk mode,
// a bus stays in its outer circulation lane behind a slow vehicle rather than
// creating a passing maneuver that must be undone before the exit.
func TestAutopilotBerserkRoundaboutFollowsSlowCarWithoutPassing(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.PrepareLeg(w, fromStop, toStop, lw)

	// Find the roundabout
	var rb *sim.Roundabout
	for i := range w.Roundabouts {
		if w.Roundabouts[i].Center.Distance(sim.Vec2{X: 0, Y: 0}) < 10.0 {
			rb = &w.Roundabouts[i]
			break
		}
	}
	if rb == nil {
		t.Fatalf("Civic roundabout not found in Metropolis")
	}

	// Find a waypoint in the roundabout along the bottom curve (near South road, Y > 0)
	var wpIdx int = -1
	for idx, pt := range driver.drivePath {
		if pt.Distance(rb.Center) <= rb.Radius && pt.Y > 100.0 && pt.X < -20.0 {
			wpIdx = idx
			break
		}
	}
	if wpIdx < 0 {
		t.Fatalf("Could not locate bottom waypoint inside roundabout")
	}

	startPt := driver.drivePath[wpIdx]
	nextPt := driver.drivePath[wpIdx+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := sim.NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 60.0
	driver.WaypointIdx = wpIdx

	// Place a slower blocker ahead in the outer lane of the roundabout
	blockerWP := driver.drivePath[wpIdx+3]
	blocker := VehicleObstacle{
		Pos:     blockerWP,
		Heading: heading,
		Speed:   35.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{blocker}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if driver.TargetLaneOffset < -.01 || driver.LaneOffset < -.01 {
			t.Fatalf("step%d: bus left its outer lane to pass slow traffic", step)
		}
		// 1. TargetLaneOffset must NEVER be positive in a roundabout
		if driver.TargetLaneOffset > 0.1 {
			t.Fatalf("Step %d: TargetLaneOffset became positive (%.2f) in roundabout, shifting outwards into exit!",
				step, driver.TargetLaneOffset)
		}
		if driver.LaneOffset > 0.1 {
			t.Fatalf("Step %d: LaneOffset became positive (%.2f) in roundabout!", step, driver.LaneOffset)
		}

		// 2. Bus position must strictly remain inside the roundabout, not exiting south
		distToCenter := bus.Pos.Distance(rb.Center)
		if distToCenter > rb.Radius+15.0 {
			t.Fatalf("Step %d: Bus exited roundabout! Pos=%v, Dist=%.2f (Radius=%.2f)",
				step, bus.Pos, distToCenter, rb.Radius)
		}

		// Invariant: Bus must not drive into south avenue (Y > 240)
		if bus.Pos.Y > rb.Radius+15.0 {
			t.Fatalf("Step %d: Bus drove into south exit avenue! Pos=%v", step, bus.Pos)
		}
	}

	if math.Abs(driver.TargetLaneOffset) > .01 || math.Abs(driver.LaneOffset) > .01 || driver.OvertakeState != OvertakeNone {
		t.Fatalf("slow traffic caused an unnecessary inner-lane pass: %.2f/%.2f state%d", driver.LaneOffset, driver.TargetLaneOffset, driver.OvertakeState)
	}
}

// TestAutopilotOvertakingPastQueueBeforeRoundaboutDrivesForward tests the screenshot scenario:
// On an approach to a roundabout on a multi-lane road, when outer lane 1 has a queue of stopped vehicles,
// a player bus in passing lane 0 must NOT steer into the queue and brake to 0 km/h;
// it must keep lane 0, yield only if the entry sweep is occupied, and resume
// through the roundabout when the queue clears.
func TestAutopilotOvertakingPastQueueBeforeRoundaboutDrivesForward(t *testing.T) {
	w := sim.NewWorld(0) // Metropolis
	lw := 40.0
	route := sim.NewRoute()
	route.AddStop(0)       // Central Terminal (X: -650, Y: 0)
	route.AddStop(2)       // Civic Plaza (X: 1300, Y: -300)
	route.CurrentIndex = 1 // Traveling towards Stop 2!

	fromStop := w.Stops[0]
	toStop := w.Stops[2]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.State = AutopilotStateCruising
	driver.PrepareLeg(w, fromStop, toStop, lw)

	startIdx := 2
	driver.WaypointIdx = startIdx
	busPos := sim.Vec2{X: -320.0, Y: 20.0} // In Lane 0 (inner passing lane)
	bus := sim.NewBus(busPos.X, busPos.Y, 0.0)
	bus.Speed = 60.0

	// Set initial passing lane state
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.OvertakeState = OvertakePassing

	// Place 6 stopped vehicles in Lane 1 (outer lane at Y = 60) queueing before the roundabout
	obstacles := []VehicleObstacle{
		{Pos: sim.Vec2{X: -420, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19},
		{Pos: sim.Vec2{X: -380, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19},
		{Pos: sim.Vec2{X: -340, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19},
		{Pos: sim.Vec2{X: -300, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19}, // Alongside bus
		{Pos: sim.Vec2{X: -260, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19}, // Lead vehicle in queue
		{Pos: sim.Vec2{X: -225, Y: 60}, Heading: 0, Speed: 0, Length: 36, Width: 19}, // At yield entrance
	}

	dt := 0.016
	initialX := bus.Pos.X
	for step := 0; step < 400; step++ {
		// The entry-mouth queue moves off after two seconds. The bus may yield
		// while its swept entry path is blocked, then must resume without freezing.
		if step == 125 {
			obstacles = nil
		}
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, nil, obstacles, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// 1. After the entry-mouth queue clears, the bus must resume driving.
		if step > 160 && bus.Speed < 15.0 {
			t.Fatalf("Step %d: Bus stalled or braked to 0 km/h (Speed=%.2f, Brake=%.2f)! TargetSpd=%.2f, LaneOffset=%.2f, TargetLaneOffset=%.2f, Pos=%v, Heading=%.2f",
				step, bus.Speed, brake, driver.LastTargetSpeed, driver.LaneOffset, driver.TargetLaneOffset, bus.Pos, bus.Heading)
		}

		// 2. Bus must NOT steer sharply into the stopped queue on the right (steer should not be hard right into lane 1)
		// Note: entering the roundabout naturally curves right (steer > 0) to follow the rotary circle;
		// Only apply the straight-lane bound before the circular roadway;
		// inside the ring the inner lane necessarily curves toward increasing Y.
		if bus.Pos.X < -w.Roundabouts[0].Radius && bus.Pos.Y > 35.0 {
			t.Fatalf("Step %d: Bus deviated into lane 1! Pos=%v, Steer=%.2f, TargetPt=%v, LaneOffset=%.2f, TargetLaneOffset=%.2f",
				step, bus.Pos, steer, driver.LastPursuitPt, driver.LaneOffset, driver.TargetLaneOffset)
		}

		for _, obstacle := range obstacles {
			if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}); hit {
				t.Fatalf("collision with queue at step %d: bus=%v heading=%v obstacle=%v", step, bus.Pos, bus.Heading, obstacle.Pos)
			}
		}

		// 3. LaneOffset must remain in passing lane (negative) while passing alongside queue
		if bus.Pos.X < -210.0 && driver.TargetLaneOffset > -15.0 {
			t.Fatalf("Step %d: TargetLaneOffset shifted into queued lane (%.2f)!", step, driver.TargetLaneOffset)
		}
	}

	// 4. Bus must have advanced forward significantly past the queue
	if bus.Pos.X <= initialX+150.0 {
		t.Fatalf("Bus failed to drive forward past queue! InitialX=%.2f, FinalX=%.2f", initialX, bus.Pos.X)
	}
}
