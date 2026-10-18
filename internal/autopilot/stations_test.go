package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestAutopilotStationLaunchDoorsOpen verifies Bug 1:
// If the bus is stopped at a station with doors open and autopilot is activated,
// the autopilot automatically commands doors to close and starts driving towards the next stop.
func TestAutopilotStationLaunchDoorsOpen(t *testing.T) {
	world := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, world)
	route.CurrentIndex = 0 // Starts at Stop 0, target is Stop 2

	stop0 := world.Stops[route.StopIDs[0]]
	bus := sim.NewBus(stop0.Pos.X, stop0.Pos.Y, stop0.Heading)
	bus.DoorsOpen = true
	bus.DoorProgress = 1.0 // Fully open
	bus.Speed = 0

	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.5 // Brief dwell
	dt := 0.05

	launched := false
	for step := 0; step < 150; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, world, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 && bus.Speed > 10.0 {
			launched = true
			break
		}
	}

	if bus.DoorsOpen {
		t.Fatalf("Bus doors should have closed automatically when launching, but are still open")
	}
	if !launched {
		t.Fatalf("Bus failed to start driving from the station after doors closed; final speed = %.2f", bus.Speed)
	}
}

// TestAutopilotStationLaunchDoorsClosed verifies that enabling autopilot when parked at a station
// with doors closed performs passenger boarding (dwells) and then launches towards the target.
func TestAutopilotStationLaunchDoorsClosed(t *testing.T) {
	world := sim.NewWorld(52.0, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, world)
	route.CurrentIndex = 0

	stop0 := world.Stops[route.StopIDs[0]]
	bus := sim.NewBus(stop0.Pos.X, stop0.Pos.Y, stop0.Heading)
	bus.DoorsOpen = false
	bus.DoorProgress = 0.0
	bus.Speed = 0

	driver := NewAutopilotDriver()
	driver.DwellTotal = 0.3
	dt := 0.05

	doorsOpened := false
	doorsClosedAfterDwell := false
	launched := false

	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, world, nil, nil, 52.0, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if bus.DoorsOpen {
			doorsOpened = true
		}
		if doorsOpened && !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			doorsClosedAfterDwell = true
		}
		if doorsClosedAfterDwell && bus.Speed > 10.0 {
			launched = true
			break
		}
	}

	if !doorsOpened {
		t.Fatalf("Autopilot should open doors for boarding when activating at a station")
	}
	if !doorsClosedAfterDwell {
		t.Fatalf("Autopilot should finish dwell and close doors")
	}
	if !launched {
		t.Fatalf("Bus failed to launch after dwelling at station; speed = %.2f", bus.Speed)
	}
}

// TestBerserkModeStationDockingWithWaitingPassengers verifies that when passengers are waiting
// at a station, the bus in Berserk mode (at 60, 80, 100, 120 km/h) reliably decelerates,
// pulls into the stop bay, and docks successfully without overshooting.
func TestBerserkModeStationDockingWithWaitingPassengers(t *testing.T) {
	lw := 47.5
	dt := 1.0 / 60.0

	for _, kmh := range []int{60, 80, 100, 120} {
		t.Run(fmt.Sprintf("%d_kmh_Station_Docking_Test", kmh), func(t *testing.T) {
			cruiseSpd := float64(kmh) * 5.0
			yRoad := 800.0

			// Stop 0 at -1500, Stop 1 at 0
			// Bay center at Y = yRoad + lw*1.5 + bayOffset (~yRoad + 115)
			s0 := sim.NewBusStop(0, "DepartStop", sim.Vec2{X: -1500, Y: yRoad + lw*1.5 + 44.0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			s1 := sim.NewBusStop(1, "ArriveStop", sim.Vec2{X: 0, Y: yRoad + lw*1.5 + 44.0}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
			s1.WaitingPassengers = 6 // Station has waiting passengers!

			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			w.Stops = []*sim.BusStop{s0, s1}

			route := &sim.Route{
				Name:         "StationDockingRoute",
				StopIDs:      []int{0, 1},
				CurrentIndex: 1,
			}

			driver := NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = cruiseSpd
			driver.State = AutopilotStateCruising
			driver.drivePath = []sim.Vec2{
				{X: -1500, Y: yRoad + lw*1.5},
				{X: 0, Y: yRoad + lw*1.5},
				s1.Pos,
			}
			driver.cumLen = []float64{0, 1500.0, 1500.0 + s1.Pos.Distance(sim.Vec2{X: 0, Y: yRoad + lw*1.5})}

			bus := sim.NewBus(1, 0, lw)
			bus.Pos = sim.Vec2{X: -600, Y: yRoad + lw*1.5}
			bus.Heading = 0
			bus.Speed = cruiseSpd
			bus.MaxSpeedFwd = math.Max(460.0, cruiseSpd+20.0)

			docked := false
			for step := 0; step < 800; step++ {
				th, br, st, hb, tg, isDocked := driver.ComputeControls(bus, route, w, nil, nil, lw, dt)
				bus.Update(dt, th, br, st, hb, tg)

				if isDocked || driver.State == AutopilotStateDocked {
					docked = true
					break
				}

				// If bus overshoots far past the stop without docking, fail early
				if bus.Pos.X > s1.Pos.X+120.0 {
					t.Fatalf("[%d km/h] Bus blew past the station without docking! bus.Pos=%v, stop.Pos=%v, speed=%.1f",
						kmh, bus.Pos, s1.Pos, bus.Speed)
				}
			}

			if !docked {
				t.Fatalf("[%d km/h] Bus never docked at the station with waiting passengers", kmh)
			}
			t.Logf("[%d km/h] Station docking successful! Final bus speed=%.1f, pos=%v, stop=%v",
				kmh, bus.Speed, bus.Pos, s1.Pos)
		})
	}
}

// TestAutopilotBerserkStationApproachOvertakesIfRunwayPermits verifies that when there is sufficient runway,
// the Berserk autopilot overtakes the lead vehicle ahead, pulls cleanly in front, returns to offset 0.0,
// and docks at the station bay without colliding.
func TestAutopilotBerserkStationApproachOvertakesIfRunwayPermits(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.PrepareLeg(w, fromStop, toStop, lw)

	// Pick a straight section with plenty of runway before the stop (~650px)
	startWp := 55 // Waypoint on the long straight Southbound approach to Stop 2 (~685px runway)

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := sim.NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 70.0
	driver.WaypointIdx = startWp

	// Lead bus 100px ahead in the same base lane
	leadPt := startPt.Add(sim.Vec2{X: math.Cos(heading) * 100.0, Y: math.Sin(heading) * 100.0})
	leadBus := VehicleObstacle{
		Pos:     leadPt,
		Heading: heading,
		Speed:   20.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	overtookSuccessfully := false
	docked := false

	for step := 0; step < 2400; step++ {
		// Move lead bus forward slowly, slowing down if user bus has cleanly pulled in front
		if bus.Pos.Y > leadBus.Pos.Y+75.0 && leadBus.Speed > 0.0 {
			leadBus.Speed = math.Max(0.0, leadBus.Speed-25.0*dt)
		}
		leadBus.Pos = leadBus.Pos.Add(sim.Vec2{X: math.Cos(leadBus.Heading) * leadBus.Speed * dt, Y: math.Sin(leadBus.Heading) * leadBus.Speed * dt})

		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Check for physical body collision using Separating Axis Theorem (SAT)
		dCol := leadBus.Pos.Sub(bus.Pos)
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		fwdCol := math.Abs(dCol.X*cosH + dCol.Y*sinH)
		latCol := math.Abs(-dCol.X*sinH + dCol.Y*cosH)

		cosL := math.Cos(leadBus.Heading)
		sinL := math.Sin(leadBus.Heading)
		fwdLead := math.Abs(dCol.X*cosL + dCol.Y*sinL)
		latLead := math.Abs(-dCol.X*sinL + dCol.Y*cosL)

		halfL := bus.Length/2 + leadBus.Length/2
		halfW := bus.Width/2 + leadBus.Width/2

		if (fwdCol < halfL && latCol < halfW) && (fwdLead < halfL && latLead < halfW) {
			t.Fatalf("Step %d: Physical collision detected between bus and lead bus! fwdCol=%.2f, latCol=%.2f, fwdLead=%.2f, latLead=%.2f",
				step, fwdCol, latCol, fwdLead, latLead)
		}

		if driver.OvertakeState == OvertakePassing || driver.OvertakeState == OvertakeReturning {
			overtookSuccessfully = true
		}

		if isDocked || driver.State == AutopilotStateDocked {
			docked = true
			break
		}
	}

	if !overtookSuccessfully {
		t.Fatalf("Expected autopilot to initiate and execute overtaking when runway permitted")
	}
	if !docked && bus.Pos.Distance(toStop.Pos) > 40.0 {
		t.Fatalf("Bus did not reach destination stop bay cleanly: busPos=%v, stopPos=%v, dist=%.2f, speed=%.2f, docked=%v, state=%v",
			bus.Pos, toStop.Pos, bus.Pos.Distance(toStop.Pos), bus.Speed, docked, driver.State)
	}
}

// TestAutopilotBerserkStationApproachWaitsBehindWhenRunwayInsufficient verifies that when approaching
// a station stop without sufficient runway (e.g. lead vehicle slow near stop), the Berserk autopilot
// does NOT initiate an overtake and instead safely queues behind the lead vehicle.
func TestAutopilotBerserkStationApproachWaitsBehindWhenRunwayInsufficient(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.PrepareLeg(w, fromStop, toStop, lw)

	// Pick a start point only ~280px before the stop (insufficient runway to complete an overtake)
	targetWp := len(driver.drivePath) - 1
	var startWp int = -1
	totalPathLen := driver.cumLen[targetWp]
	for i := 0; i < targetWp; i++ {
		rem := totalPathLen - driver.cumLen[i]
		if rem <= 280.0 {
			startWp = i
			break
		}
	}
	if startWp < 0 {
		t.Fatalf("Could not find start waypoint with ~280px runway")
	}

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	bus := sim.NewBus(startPt.X, startPt.Y, heading)
	bus.Speed = 45.0
	driver.WaypointIdx = startWp

	// Lead bus stopped or crawling 110px ahead near the station bay
	leadPt := startPt.Add(sim.Vec2{X: math.Cos(heading) * 110.0, Y: math.Sin(heading) * 110.0})
	leadBus := VehicleObstacle{
		Pos:     leadPt,
		Heading: heading,
		Speed:   0.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	for step := 0; step < 250; step++ {
		throttle, brake, steer, handbrake, toggleDoors, _ := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Must NOT switch to passing lane when runway is insufficient
		if driver.OvertakeState == OvertakeChangingLane || driver.TargetLaneOffset < -10.0 {
			t.Fatalf("Step %d: Autopilot initiated overtaking without sufficient runway! State=%v, TargetOffset=%.2f",
				step, driver.OvertakeState, driver.TargetLaneOffset)
		}

		// Check for collision
		dist := bus.Pos.Distance(leadBus.Pos)
		minClearDist := bus.Length/2 + leadBus.Length/2 + 4.0
		if dist < minClearDist {
			t.Fatalf("Step %d: Collision detected! Dist=%.2f, MinClear=%.2f", step, dist, minClearDist)
		}
	}

	// Bus must have stopped safely behind lead vehicle
	if bus.Speed > 2.0 {
		t.Fatalf("Expected bus to stop safely behind lead bus, got speed=%.2f", bus.Speed)
	}
}

// TestAutopilotBerserkStationApproachWaitsAlongsideAndEntersBayAfterLeadLeaves verifies that when
// the bus is approaching in the adjacent passing lane while the curb bay is occupied by a lead bus,
// the autopilot maintains its lane, does not steer into the lead bus, slows/stops alongside, and
// once the lead bus departs, smoothly enters the bay and docks.
func TestAutopilotBerserkStationApproachWaitsAlongsideAndEntersBayAfterLeadLeaves(t *testing.T) {
	lw := 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)
	route.CurrentIndex = 1 // Target is Stop 2 (toStop)

	fromStop := w.Stops[route.StopIDs[0]]
	toStop := w.Stops[route.StopIDs[1]]

	driver := NewAutopilotDriver()
	driver.BerserkMode = true
	driver.PrepareLeg(w, fromStop, toStop, lw)

	targetWp := len(driver.drivePath) - 1
	var startWp int = -1
	totalPathLen := driver.cumLen[targetWp]
	for i := 0; i < targetWp; i++ {
		rem := totalPathLen - driver.cumLen[i]
		if rem <= 240.0 {
			startWp = i
			break
		}
	}
	if startWp < 0 {
		t.Fatalf("Could not find start waypoint with ~220px runway")
	}

	startPt := driver.drivePath[startWp]
	nextPt := driver.drivePath[startWp+1]
	heading := math.Atan2(nextPt.Y-startPt.Y, nextPt.X-startPt.X)

	// Bus starts in adjacent passing lane (-lw offset)
	hDir := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	rightDir := sim.Vec2{X: -hDir.Y, Y: hDir.X}
	busPos := startPt.Add(rightDir.Mul(-lw))

	bus := sim.NewBus(busPos.X, busPos.Y, heading)
	bus.Speed = 40.0
	driver.WaypointIdx = startWp
	driver.LaneOffset = -lw
	driver.TargetLaneOffset = -lw
	driver.OvertakeState = OvertakePassing

	// Lead AI bus stopped at the destination stop bay
	leadBus := VehicleObstacle{
		Pos:     toStop.Pos,
		Heading: toStop.Heading,
		Speed:   0.0,
		Length:  48,
		Width:   22,
	}

	dt := 0.016
	// Phase 1: Wait alongside the lead bus for 200 steps
	for step := 0; step < 200; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, []VehicleObstacle{leadBus}, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Must not have docked in the middle of the road while in adjacent lane
		if isDocked || driver.State == AutopilotStateDocked {
			t.Fatalf("Step %d: Falsely docked in adjacent lane!", step)
		}

		// Must not cut into the lead bus (TargetLaneOffset must stay negative / waiting)
		if driver.TargetLaneOffset > -15.0 {
			t.Fatalf("Step %d: Swerved into occupied bay while lead bus was present! TargetLaneOffset=%.2f",
				step, driver.TargetLaneOffset)
		}

		// Must not collide with lead bus
		dist := bus.Pos.Distance(leadBus.Pos)
		minClearDist := bus.Length/2 + leadBus.Length/2 + 2.0
		if dist < minClearDist {
			t.Fatalf("Step %d: Collision with lead bus in bay! Dist=%.2f, MinClear=%.2f",
				step, dist, minClearDist)
		}
	}

	// Bus should be stopped or nearly stopped alongside the bay
	if bus.Speed > 5.0 {
		t.Fatalf("Expected bus to wait alongside at low speed/stopped, got Speed=%.2f", bus.Speed)
	}

	// Phase 2: Lead bus departs (clear obstacles)
	docked := false
	for step := 0; step < 300; step++ {
		throttle, brake, steer, handbrake, toggleDoors, isDocked := driver.ComputeControls(
			bus, route, w, nil, nil, lw, dt,
		)
		bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		if isDocked || driver.State == AutopilotStateDocked {
			docked = true
			break
		}
	}

	if !docked {
		t.Fatalf("After lead bus departed, bus failed to enter the bay and dock cleanly! BusPos=%v, StopPos=%v, LaneOffset=%.2f",
			bus.Pos, toStop.Pos, driver.LaneOffset)
	}
}

// TestAutopilotStopBayIgnoresOpposingVehicles verifies that in autopilot functions,
// vehicles in oncoming traffic do not cause false bay-occupancy or block returning to base lane.
func TestAutopilotStopBayIgnoresOpposingVehicles(t *testing.T) {
	lw := 52.0
	stop := sim.NewBusStop(10, "TestStop", sim.Vec2{X: 500, Y: 300}, 0, 4, sim.RGBA(0, 0, 0, 0), lw, 0)
	bus := sim.NewBus(400, 280, 0)
	cosH, sinH := 1.0, 0.0

	// An oncoming vehicle passing near the stop bay
	oppObs := VehicleObstacle{
		Pos:     stop.Pos.Add(sim.Vec2{X: 5.0, Y: 5.0}),
		Heading: math.Pi, // Oncoming direction
		Speed:   30.0,
		Length:  50.0,
		Width:   25.0,
	}

	// isBaseLaneClearToReturn should NOT be blocked by this oncoming vehicle
	clear := isBaseLaneClearToReturn(bus, []VehicleObstacle{oppObs}, cosH, sinH, 0.0, nil, 0, stop)
	if !clear {
		t.Fatalf("isBaseLaneClearToReturn should return true when nearby obstacle is oncoming vehicle")
	}
}
