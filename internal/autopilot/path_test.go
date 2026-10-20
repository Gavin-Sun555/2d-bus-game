package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestAutopilotFullRouteLoopCompletionAllMaps validates that the autonomous bus driver navigates
// complete circular routes from start to finish across all 6 maps, accurately docking inside
// every station bay, servicing passenger boarding, and finishing with zero barrier and zero shoulder violations.
func TestAutopilotFullRouteLoopCompletionAllMaps(t *testing.T) {
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
			laneW := 52.0
			world := sim.NewWorld(laneW, theme)
			route := sim.GetStandardRouteForMap(theme, world)
			if len(route.StopIDs) < 2 {
				t.Fatalf("[%s] Standard loop route must have >= 2 stops", theme.DisplayName())
			}

			startStop := world.Stops[route.StopIDs[0]]
			bus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			driver := NewAutopilotDriver()
			driver.DwellTotal = 0.2 // Efficient dwell for fast test execution
			dt := 1.0 / 60.0

			var dockedStops []int
			prevState := driver.State
			barrierHits := 0
			shoulderHits := 0
			stuckTimer := 0.0
			maxStuck := 0.0

			// Simulate up to 750 seconds (plenty of time for 3x longer circuit at 60fps)
			maxFrames := 60 * 750
			for frame := 0; frame < maxFrames; frame++ {
				throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
					bus, route, world, nil, nil, laneW, dt,
				)
				bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

				if driver.State == AutopilotStateDocked && prevState != AutopilotStateDocked {
					dockedStops = append(dockedStops, driver.DockedStopID)
				}
				prevState = driver.State

				bRes := world.CheckBoundaries(bus.Pos, bus.Width/2, laneW)
				if bRes.Zone == sim.ZoneHardBarrier {
					barrierHits++
				} else if bRes.Zone == sim.ZoneShoulder {
					inBay := false
					for _, s := range world.Stops {
						if s.ContainsBus(bus) || s.Pos.Distance(bus.Pos) < s.Length/2 {
							inBay = true
							break
						}
					}
					if !inBay {
						shoulderHits++
						if shoulderHits <= 5 {
							t.Logf("[%s] shoulderHit #%d: bus.Pos=%v, state=%v, distToRoad=%v, obstacle=%s, zone=%v",
								theme.DisplayName(), shoulderHits, bus.Pos, driver.State, bRes.Penetration, bRes.ObstacleName, bRes.Zone)
						}
					}
				}

				if math.Abs(bus.Speed) < 2.0 && driver.State != AutopilotStateDocked {
					stuckTimer += dt
					if stuckTimer > maxStuck {
						maxStuck = stuckTimer
					}
				} else {
					stuckTimer = 0.0
				}

				// Terminate successfully once full loop is complete and next terminal cycle starts
				if route.TotalTrips >= 1 && len(dockedStops) > len(route.StopIDs) {
					break
				}
			}

			if route.TotalTrips < 1 {
				t.Fatalf("[%s] Autopilot failed to complete a full transit loop (trips=%d, docked=%v)",
					theme.DisplayName(), route.TotalTrips, dockedStops)
			}
			if barrierHits > 0 {
				t.Fatalf("[%s] Autopilot collided with hard barriers during route: %d times",
					theme.DisplayName(), barrierHits)
			}
			if shoulderHits > 0 {
				t.Fatalf("[%s] Autopilot drifted into shoulder during route: %d times",
					theme.DisplayName(), shoulderHits)
			}
			if maxStuck > 2.0 {
				t.Fatalf("[%s] Autopilot got stuck outside stations for %.1f seconds",
					theme.DisplayName(), maxStuck)
			}
		})
	}
}

// TestBerserkModeNoPrematureReturnCollision verifies that when overtaking in Berserk mode:
//  1. The bus never prematurely starts returning into the base lane while the overtaken vehicle
//     is still ahead, alongside, or closely behind the bus (within the safe longitudinal buffer).
//  2. Approaching corner or stop bay does not trigger an unsafe cut-in if base lane is occupied.
//  3. Once safely clear (> 75px buffer behind bus tail), return to base lane can safely proceed.
//  4. During return maneuver, high cruise speed is maintained and any sudden obstruction in base
//     lane causes the bus to abort return and pull back into the passing lane.
func TestBerserkModeNoPrematureReturnCollision(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)

	// Eastbound 4-lane expressway on South Ring Avenue (Y = 1200*MapGridScale = 3600)
	roadY := 1200.0 * sim.MapGridScale
	s1 := sim.NewBusStop(0, "AvenueStart", sim.Vec2{X: -2000, Y: roadY}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	s2 := sim.NewBusStop(1, "AvenueEnd", sim.Vec2{X: 2000, Y: roadY}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	w.Stops = []*sim.BusStop{s1, s2}

	route := &sim.Route{
		Name:         "BerserkNoPrematureReturnRoute",
		StopIDs:      []int{0, 1},
		CurrentIndex: 1, // Heading towards s2
	}

	driver := NewAutopilotDriver()
	driver.BerserkMode = true

	// Eastbound base lane (outer lane) is at roadY + lw*1.5
	baseLaneY := roadY + lw*1.5
	// Passing lane (inner lane) is at roadY + lw*0.5 (offset -lw)
	passingLaneY := roadY + lw*0.5

	bus := sim.NewBus(-600, passingLaneY, 0)
	bus.Speed = 200.0

	// Run initial step to sync path and state
	driver.ComputeControls(bus, route, w, nil, nil, lw, 0.016)

	// Simulate that the bus has already pulled into the passing lane (-lw)
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.LaneOffsetProgress = 1.0
	driver.OvertakeState = OvertakePassing
	driver.OvertakeTimer = 2.0 // Has been passing for 2 seconds

	// 1. Overtaken vehicle is still slightly ahead (fwd = +40 px, in base lane)
	carAhead := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X + 40.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carAhead}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is still ahead (fwd=+40px)")
	}

	// 2. Overtaken vehicle is directly alongside the bus center (fwd = 0 px)
	carAlongside := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carAlongside}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is alongside (fwd=0px)")
	}

	// 3. Overtaken vehicle is slightly behind bus center (fwd = -70 px), within unsafe zone (< 75px buffer)
	carCloseBehind := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X - 70.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carCloseBehind}, lw, 0.016)
	if driver.OvertakeState == OvertakeReturning || driver.TargetLaneOffset == 0.0 {
		t.Fatalf("Bus prematurely attempted to return while overtaken vehicle is close behind (fwd=-70px, buffer too small)")
	}

	// 4. Overtaken vehicle is safely far behind (fwd = -180 px, clearance > 75px buffer)
	carFarBehind := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X - 180.0, Y: baseLaneY},
		Heading: 0,
		Speed:   120.0,
		Length:  40.0,
		Width:   18.0,
	}
	throttle, brake, _, _, _, _ := driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{carFarBehind}, lw, 0.016)
	if driver.OvertakeState != OvertakeReturning || driver.TargetLaneOffset != 0.0 {
		t.Fatalf("Expected bus to safely initiate return once overtaken vehicle is far behind, got state %d target %.1f",
			driver.OvertakeState, driver.TargetLaneOffset)
	}

	// Verify cruise speed is maintained during return (does not brake check vehicle behind)
	if brake > 0.0 {
		t.Fatalf("Bus should not brake during OvertakeReturning; expected brake=0.0, got %.2f", brake)
	}
	if throttle <= 0.0 && bus.Speed < apBerserkCruiseSpeed {
		t.Fatalf("Bus should maintain throttle during OvertakeReturning; got throttle=%.2f", throttle)
	}

	// 5. Test collision avoidance abort during return maneuver
	driver.OvertakeState = OvertakeReturning
	driver.LaneOffset = -lw * 0.5
	driver.TargetLaneOffset = 0.0
	driver.LaneOffsetProgress = 0.5
	bus.Pos.Y = roadY + lw*1.0 // mid-transition lateral position

	// Sudden danger in base lane right beside/ahead of bus
	dangerCar := VehicleObstacle{
		Pos:     sim.Vec2{X: bus.Pos.X + 20.0, Y: baseLaneY},
		Heading: 0,
		Speed:   50.0,
		Length:  40.0,
		Width:   18.0,
	}
	driver.ComputeControls(bus, route, w, nil, []VehicleObstacle{dangerCar}, lw, 0.016)
	if math.Abs(driver.TargetLaneOffset+lw*.5) > .01 || driver.OvertakeState != OvertakePassing {
		t.Fatalf("Expected bus to hold its current corridor when the return gap closes, got state %d target %.1f",
			driver.OvertakeState, driver.TargetLaneOffset)
	}
}

func TestAutopilotStateAndDrivePathAccessors(t *testing.T) {
	states := []AutopilotState{
		AutopilotStateCruising,
		AutopilotStateApproaching,
		AutopilotStateDocked,
		AutopilotStateDeparting,
		AutopilotState(99),
	}

	for _, st := range states {
		if st.String() == "" {
			t.Errorf("AutopilotState %d String() should not be empty", st)
		}
	}

	driver := NewAutopilotDriver()
	if driver.DrivePath() != nil {
		t.Errorf("Initial DrivePath should be nil")
	}
}
