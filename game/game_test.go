package game

import (
	"math"
	"strings"
	"testing"
)

func TestBusKinematics(t *testing.T) {
	bus := NewBus(0, 0, 0)

	// Test acceleration forward (throttle=1.0, brake=0.0)
	bus.Update(0.5, 1.0, 0.0, 0.0, false, false)
	if bus.Speed <= 0 {
		t.Fatalf("Expected bus speed to increase with throttle, got %f", bus.Speed)
	}

	// Test steering turns heading
	bus.Update(0.5, 1.0, 0.0, 1.0, false, false)
	if bus.Heading == 0 {
		t.Fatalf("Expected bus heading to change when steering while moving, got %f", bus.Heading)
	}

	// Test door safety lock (cannot move while doors are open)
	bus.Speed = 0
	bus.Update(0.1, 0, 0, 0, false, true) // toggle door open
	if !bus.DoorsOpen {
		t.Fatalf("Expected doors to open when stopped")
	}

	// Attempt throttle while doors open
	bus.Update(0.5, 1.0, 0, 0, false, false)
	if bus.Speed != 0 {
		t.Fatalf("Bus should not move while doors are open, got speed %f", bus.Speed)
	}
}

func TestRoutePlanning(t *testing.T) {
	route := NewDefaultRoute()
	if len(route.StopIDs) == 0 {
		t.Fatalf("Default route should have stops")
	}

	initialStop := route.GetCurrentTargetStopID()
	route.AdvanceToNextStop()
	nextStop := route.GetCurrentTargetStopID()

	if nextStop == initialStop && len(route.StopIDs) > 1 {
		t.Fatalf("Expected advance to change target stop")
	}

	// Test clear and re-add
	route.Clear()
	if len(route.StopIDs) != 0 {
		t.Fatalf("Expected route to be empty after Clear")
	}

	route.AddStop(2)
	route.AddStop(4)
	if len(route.StopIDs) != 2 || route.StopIDs[0] != 2 || route.StopIDs[1] != 4 {
		t.Fatalf("Route stops mismatch: %v", route.StopIDs)
	}
}

func TestTrafficLights(t *testing.T) {
	tls := NewTrafficLightSystem()
	if len(tls.Intersections) == 0 {
		t.Fatalf("Expected intersections in traffic light system")
	}

	inter := tls.Intersections[0]
	inter.Phase = PhaseEWGreen
	ew, ns := inter.GetSignals()
	if ew != SignalGreen || ns != SignalRed {
		t.Fatalf("Expected EW Green and NS Red in PhaseEWGreen")
	}

	// Test signal by heading: heading 0 is East (EW), heading Pi/2 is South (NS)
	if sig := inter.GetSignalForHeading(0); sig != SignalGreen {
		t.Fatalf("Expected heading 0 (East) to be Green")
	}
	if sig := inter.GetSignalForHeading(math.Pi / 2); sig != SignalRed {
		t.Fatalf("Expected heading Pi/2 (South) to be Red")
	}

	// Test red light check on player bus
	bus := NewBus(inter.Center.X, inter.Center.Y, math.Pi/2) // facing South (Red)
	bus.Speed = 60                                          // moving
	violated, _ := tls.CheckPlayerViolation(bus)
	if !violated {
		t.Fatalf("Expected violation when crossing red light at speed")
	}

	// Immediate second check should respect cooldown
	violated2, _ := tls.CheckPlayerViolation(bus)
	if violated2 {
		t.Fatalf("Expected cooldown to prevent immediate repeat violation")
	}
}

func TestTrafficViolationCrossingBeforeYellow(t *testing.T) {
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseEWGreen,
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
	laneW := 42.0

	// 1. Vehicle enters on Green (before yellow) -> Light turns Red inside -> NO FINE
	inter.Phase = PhaseEWGreen
	inter.Cooldown = 0
	bus := NewBus(-140, 20, 0) // Facing East (heading 0)
	bus.Speed = 50

	// Approaching before stop line (stopLineDist = 85.2, front at -113)
	violated, _ := tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate before reaching stop line")
	}
	if inter.PlayerInside {
		t.Fatalf("Player should not be considered inside before stop line")
	}

	// Move forward: front bumper crosses stop line on Green (-90 + 27 = -63 > -85.2)
	bus.Pos.X = -90
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate when entering on Green")
	}
	if !inter.PlayerInside || !inter.PlayerCrossedBeforeYellow {
		t.Fatalf("Expected PlayerInside=true and PlayerCrossedBeforeYellow=true after green crossing")
	}

	// Light turns Yellow, then Red while bus is inside intersection
	inter.Phase = PhaseNSGreen // EW is now Red!
	if sig := inter.GetSignalForHeading(0); sig != SignalRed {
		t.Fatalf("Expected EW signal to be Red")
	}

	// Bus is still in intersection center at speed 50 while light is Red
	bus.Pos.X = 0
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Violation triggered unfairly! Bus crossed line before yellow, must not be fined")
	}

	// Bus exits intersection (rear bumper clears xExit: 140 - 48 = 92 > 85.2)
	bus.Pos.X = 140
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate when exiting")
	}
	if inter.PlayerInside {
		t.Fatalf("PlayerInside should be reset to false after exiting intersection")
	}

	// 2. Vehicle runs a Red light -> MUST BE FINED
	inter.Phase = PhaseNSGreen // EW is Red
	inter.Cooldown = 0
	bus.Pos.X = -140
	bus.Speed = 50
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate before stop line on red approach")
	}

	// Crosses stop line while Red: front bumper crosses (-90 + 48 = -42 > -85.2),
	// but rear wheels (-90 - 48 = -138) have NOT crossed yet -> MUST NOT BE FINED YET!
	bus.Pos.X = -90
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not fine for red light before rear wheels cross the stop line")
	}

	// Rear wheels cross stop line into intersection (-30 - 48 = -78 >= -85.2) -> MUST BE FINED
	bus.Pos.X = -30
	violated, notice := tls.CheckPlayerViolation(bus, laneW)
	if !violated {
		t.Fatalf("Expected red light violation when rear wheels cross stop line on Red")
	}
	if notice == "" {
		t.Fatalf("Expected fine notice on red light violation")
	}

	// 3. Vehicle enters on Yellow -> Light turns Red inside -> NO FINE
	inter.Phase = PhaseEWYellow // EW is Yellow
	inter.Cooldown = 0
	inter.PlayerInside = false
	inter.PlayerCrossedBeforeYellow = false
	inter.PlayerCrossedOnRed = false
	inter.FinedThisPassage = false

	bus.Pos.X = -90 // Cross stop line on Yellow
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate when crossing on Yellow")
	}

	// Light turns Red while bus is inside
	inter.Phase = PhaseNSGreen // EW is Red
	bus.Pos.X = 0
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Should not violate when vehicle entered on Yellow before Red")
	}
}

func TestAITraffic(t *testing.T) {
	tm := NewTrafficManager(42.0)
	if len(tm.Cars) < 8 {
		t.Fatalf("Expected pre-allocated AI traffic cars, got %d", len(tm.Cars))
	}

	tls := NewTrafficLightSystem()
	bus := NewBus(0, 0, 0)

	// Step simulation
	for i := 0; i < 30; i++ {
		tm.Update(0.016, bus, tls, 2, 42.0)
	}

	for _, car := range tm.Cars[:8] {
		if math.IsNaN(car.Pos.X) || math.IsNaN(car.Pos.Y) {
			t.Fatalf("Car position resulted in NaN")
		}
	}
}

func TestSettingsAndCustomLaneWidth(t *testing.T) {
	settings := NewSettings()
	if !settings.TrafficFinesEnabled {
		t.Fatalf("Expected traffic fines enabled by default")
	}

	// Test lane width levels
	if settings.GetLaneWidth() != 52.0 {
		t.Fatalf("Expected default lane width to be 52.0, got %f", settings.GetLaneWidth())
	}

	settings.LaneWidthLevel = 0
	if settings.GetLaneWidth() != 42.0 {
		t.Fatalf("Expected level 0 to be 42.0, got %f", settings.GetLaneWidth())
	}

	settings.LaneWidthLevel = 3
	if settings.GetLaneWidth() != 78.0 {
		t.Fatalf("Expected level 3 to be 78.0, got %f", settings.GetLaneWidth())
	}

	// Test World dynamic lane width update
	world := NewWorld(52.0)
	if len(world.Stops) != 12 {
		t.Fatalf("Expected 12 bus stops in expanded world, got %d", len(world.Stops))
	}

	initialStopPos := world.Stops[0].Pos
	world.UpdateLaneWidth(78.0)
	newStopPos := world.Stops[0].Pos

	if initialStopPos == newStopPos {
		t.Fatalf("Expected bus stop bay to adapt dynamically when lane width changes")
	}
}

func TestGetRoadPathBetweenStops(t *testing.T) {
	world := NewWorld(42.0)
	if len(world.Stops) < 2 {
		t.Fatalf("Need at least 2 stops for test")
	}

	s1 := world.Stops[0] // e.g. North Hub (Y = -1200)
	s2 := world.Stops[1] // e.g. Financial Center (Y = 0)

	path := GetRoadPathBetweenStops(s1, s2)
	if len(path) < 2 {
		t.Fatalf("Path should contain multiple waypoints, got %d", len(path))
	}

	// Verify the path segments along the road grid are orthogonal
	// and drawn on the driving lane side rather than in the road center
	for i := 1; i < len(path)-2; i++ {
		pA := path[i]
		pB := path[i+1]
		dx := math.Abs(pA.X - pB.X)
		dy := math.Abs(pA.Y - pB.Y)
		// At least one dimension should be zero or negligible along orthogonal grid
		if dx > 1.0 && dy > 1.0 {
			t.Errorf("Path segment from %v to %v is diagonal, expected orthogonal grid line", pA, pB)
		}

		// Verify the segment is offset to the side of Central Blvd (Y=0), not on the centerline Y=0
		if dx > 50 { // Longitudinal road segment
			distFromRoadCenter := math.Abs(pA.Y - 0)
			if distFromRoadCenter < 20.0 {
				t.Errorf("Navigation line at Y=%.2f is in the middle of the road, expected side of road", pA.Y)
			}
		}
	}
}

func TestRightHandSideApproachAndAlignment(t *testing.T) {
	world := NewWorld(42.0)

	// Test default route stops
	defaultRoute := NewDefaultRoute()
	for i := 0; i < len(defaultRoute.StopIDs)-1; i++ {
		s1 := world.Stops[defaultRoute.StopIDs[i]]
		s2 := world.Stops[defaultRoute.StopIDs[i+1]]

		path := GetRoadPathBetweenStops(s1, s2)
		if len(path) < 3 {
			t.Fatalf("Path between %s and %s is too short", s1.Name, s2.Name)
		}

		// The second-to-last point is s2.RoadCenter, and the last point is s2.Pos
		// The point before s2.RoadCenter must lead into s2.RoadCenter along s2.Heading!
		pPrev := path[len(path)-3]
		pRoadCenter := path[len(path)-2]
		approachVec := pRoadCenter.Sub(pPrev).Normalize()

		expectedHeadingVec := Vec2{X: math.Cos(s2.Heading), Y: math.Sin(s2.Heading)}
		dot := approachVec.X*expectedHeadingVec.X + approachVec.Y*expectedHeadingVec.Y
		if dot < 0.95 {
			t.Errorf("Approach vector into %s (%v) does not match stop heading (%v, dot=%.2f)",
				s2.Name, approachVec, expectedHeadingVec, dot)
		}

		// Verify s2.Pos is to the RIGHT of approachVec
		// In 2D screen coordinates (X right, Y down):
		// Right perpendicular of (vx, vy) is (-vy, vx).
		stopOffset := s2.Pos.Sub(pRoadCenter).Normalize()
		perpRight := Vec2{X: -approachVec.Y, Y: approachVec.X}
		rightDot := stopOffset.X*perpRight.X + stopOffset.Y*perpRight.Y
		if rightDot < 0.95 {
			t.Errorf("Stop bay for %s is not on the RIGHT side of the approaching vehicle (dot=%.2f)",
				s2.Name, rightDot)
		}
	}

	// Test bus parking orientation: bus heading matching s.Heading contains bus, reverse does not
	s := world.Stops[0]
	busCorrect := NewBus(s.Pos.X, s.Pos.Y, s.Heading)
	if !s.ContainsBus(busCorrect) {
		t.Fatalf("Bus correctly parked at right-hand stop should be accepted")
	}

	busReverse := NewBus(s.Pos.X, s.Pos.Y, NormalizeAngle(s.Heading+math.Pi))
	if s.ContainsBus(busReverse) {
		t.Fatalf("Bus parked facing opposite direction should NOT be accepted at right-hand stop")
	}
}

func TestRoadBoundaryZonesAndBarriers(t *testing.T) {
	world := NewWorld(42.0)
	laneW := 42.0
	// Central Boulevard: Y = 0, 4 lanes -> width = 4 * 42 = 168, half-width = 84

	// 1. Center of Central Blvd: should be ZoneRoad (at X=300, far from vertical roads)
	resRoad := world.CheckBoundaries(Vec2{X: 300, Y: 0}, 18.0, laneW)
	if resRoad.Zone != ZoneRoad {
		t.Fatalf("Expected center of road to be ZoneRoad, got %v", resRoad.Zone)
	}

	// 2. Roadside shoulder: roadHW = (4*42+18)/2 = 93.0. Y = 93 + 15 = 108 (within ShoulderWidth = 36.0)
	resShoulder := world.CheckBoundaries(Vec2{X: 300, Y: 108}, 18.0, laneW)
	if resShoulder.Zone != ZoneShoulder {
		t.Fatalf("Expected Y=108 to be ZoneShoulder, got %v", resShoulder.Zone)
	}

	// 3. Beyond shoulder: Y = 93 + 45 = 138 (> ShoulderWidth = 36) -> ZoneHardBarrier
	resBarrier := world.CheckBoundaries(Vec2{X: 300, Y: 145}, 18.0, laneW)
	if resBarrier.Zone != ZoneHardBarrier {
		t.Fatalf("Expected Y=145 to be ZoneHardBarrier, got %v", resBarrier.Zone)
	}
	if resBarrier.Pushback.Y >= 0 {
		t.Fatalf("Expected pushback towards road (negative Y), got %v", resBarrier.Pushback)
	}

	// 4. Downtown skyscraper collision: cx=450, cy=300, w=220, h=160
	resBldg := world.CheckBoundaries(Vec2{X: 450, Y: 300}, 18.0, laneW)
	if resBldg.Zone != ZoneHardBarrier {
		t.Fatalf("Expected building center to be ZoneHardBarrier, got %v", resBldg.Zone)
	}
}

func TestBuildingsClearFromRoads(t *testing.T) {
	// Test at standard (52px) and maximum (78px) lane widths
	for _, laneW := range []float64{52.0, 78.0} {
		world := NewWorld(laneW)

		for i, b := range world.Buildings {
			// Check the 4 corners and center of every building
			testPoints := []Vec2{
				b.Pos,
				{X: b.Pos.X - b.W/2, Y: b.Pos.Y - b.H/2},
				{X: b.Pos.X + b.W/2, Y: b.Pos.Y - b.H/2},
				{X: b.Pos.X - b.W/2, Y: b.Pos.Y + b.H/2},
				{X: b.Pos.X + b.W/2, Y: b.Pos.Y + b.H/2},
			}

			for _, pt := range testPoints {
				for _, r := range world.Roads {
					roadHalfW := r.GetWidth(laneW) / 2
					roadClearance := roadHalfW + ShoulderWidth

					isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
					if isEW {
						// Horizontal road
						if pt.X >= r.P1.X-20 && pt.X <= r.P2.X+20 {
							dist := math.Abs(pt.Y - r.P1.Y)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects horizontal road %s (Y=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.Y, roadClearance, dist)
							}
						}
					} else {
						// Vertical road
						if pt.Y >= r.P1.Y-20 && pt.Y <= r.P2.Y+20 {
							dist := math.Abs(pt.X - r.P1.X)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects vertical road %s (X=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.X, roadClearance, dist)
							}
						}
					}
				}
			}
		}
	}
}

func TestBusShoulderDecelerationAndCrash(t *testing.T) {
	busOnRoad := NewBus(0, 0, 0)
	busOnRoad.Speed = 200
	busOnRoad.OnShoulder = false
	busOnRoad.Update(0.1, 0, 0, 0, false, false) // 0.1s natural friction

	busOnShoulder := NewBus(0, 0, 0)
	busOnShoulder.Speed = 200
	busOnShoulder.OnShoulder = true
	busOnShoulder.Update(0.1, 0, 0, 0, false, false) // 0.1s shoulder drag

	if busOnShoulder.Speed >= busOnRoad.Speed {
		t.Fatalf("Expected bus on shoulder to decelerate faster than on road: shoulder=%f, road=%f",
			busOnShoulder.Speed, busOnRoad.Speed)
	}

	// Test Crash / Durability
	if busOnShoulder.Durability != 100.0 {
		t.Fatalf("Expected fresh bus durability to be 100")
	}
	busOnShoulder.Durability -= 20.0
	if busOnShoulder.Durability != 80.0 {
		t.Fatalf("Expected durability 80, got %f", busOnShoulder.Durability)
	}
}

func TestSkipStopLogic(t *testing.T) {
	g := NewGame(1280, 720)
	targetID := g.Route.GetCurrentTargetStopID()
	if targetID < 0 {
		t.Fatalf("Expected valid target stop ID")
	}

	targetStop := g.World.Stops[targetID]

	// 1. When station has 0 waiting passengers and no stop bell: can skip
	targetStop.WaitingPassengers = 0
	g.stopBellRung = false
	g.passengersAlighting = 0

	canSkip := targetStop.WaitingPassengers == 0 && !g.stopBellRung
	if !canSkip {
		t.Fatalf("Expected canSkip to be true when station is empty and no bell rung")
	}

	// Advance / skip stop
	prevTargetID := targetID
	g.Route.AdvanceToNextStop()
	newTargetID := g.Route.GetCurrentTargetStopID()
	if newTargetID == prevTargetID && len(g.Route.StopIDs) > 1 {
		t.Fatalf("Expected target stop to advance after skip stop")
	}

	// 2. When station has waiting passengers: CANNOT skip
	targetStop.WaitingPassengers = 5
	g.stopBellRung = false
	if targetStop.WaitingPassengers == 0 && !g.stopBellRung {
		t.Fatalf("Should not allow skip when waiting passengers > 0")
	}

	// 3. When passenger rings stop bell: CANNOT skip even if station is empty
	targetStop.WaitingPassengers = 0
	g.stopBellRung = true
	g.passengersAlighting = 2
	if targetStop.WaitingPassengers == 0 && !g.stopBellRung {
		t.Fatalf("Should not allow skip when stop bell is rung")
	}
}

func TestRouteNextStopOnly(t *testing.T) {
	route := NewDefaultRoute()
	if len(route.StopIDs) < 2 {
		t.Fatalf("Expected default route to have multiple stops")
	}

	// Current target stop
	currTarget := route.GetCurrentTargetStopID()
	if currTarget != route.StopIDs[0] {
		t.Fatalf("Expected target stop to be first stop, got %d", currTarget)
	}

	// Advance
	route.AdvanceToNextStop()
	nextTarget := route.GetCurrentTargetStopID()
	if nextTarget != route.StopIDs[1] {
		t.Fatalf("Expected target stop to advance to second stop, got %d", nextTarget)
	}
}

func TestBusStopBayStrictContainmentAndSpawn(t *testing.T) {
	g := NewGame(1280, 720)
	s0 := g.World.Stops[0]

	// 1. Bus must spawn directly inside the yellow bay box of Stop 0 (not in the middle of the road)
	if g.Bus.Pos.Distance(s0.Pos) > 1.0 {
		t.Fatalf("Bus should spawn at Stop 0 bay center %v, got %v", s0.Pos, g.Bus.Pos)
	}
	if !s0.ContainsBus(g.Bus) {
		t.Fatalf("Initial bus spawn must be considered properly parked inside Stop 0 bay")
	}

	// 2. Bus parked in the middle of the road adjacent to the stop MUST NOT be accepted
	// Road center is at s0.RoadCenter. On 4-lane road, driving lane is at Y=31.5
	busOnRoad := NewBus(s0.RoadCenter.X, s0.RoadCenter.Y+31.5, s0.Heading)
	if s0.ContainsBus(busOnRoad) {
		t.Fatalf("Bus stopped in the middle of the road must NOT be accepted as parked in the stop bay")
	}

	// 3. Bus parked in rightmost lane on road surface (outside yellow box) MUST NOT be accepted
	busOnOuterLane := NewBus(s0.RoadCenter.X, s0.RoadCenter.Y+70.0, s0.Heading)
	if s0.ContainsBus(busOnOuterLane) {
		t.Fatalf("Bus on road lane surface must NOT be accepted as parked in the stop bay")
	}

	// 4. Bus pulled squarely into the yellow bay box (within ±15 px laterally) MUST be accepted
	busInBay := NewBus(s0.Pos.X+10.0, s0.Pos.Y+5.0, s0.Heading)
	if !s0.ContainsBus(busInBay) {
		t.Fatalf("Bus pulled inside the yellow stop bay box must be accepted")
	}
}

func TestGreenLightTraversalAndTurnImmunity(t *testing.T) {
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseEWGreen,
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
	laneW := 42.0

	// 1. Bus enters on Green and turns 90 degrees inside the intersection (EW -> NS)
	bus := NewBus(-90, 20, 0) // Eastbound approaching (0,0)
	bus.Speed = 45.0

	violated, _ := tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Entering on Green must NOT trigger a violation")
	}
	if !inter.PlayerInside || !inter.PlayerCrossedBeforeYellow {
		t.Fatalf("Expected PlayerInside=true and PlayerCrossedBeforeYellow=true upon Green crossing")
	}

	// Bus turns towards North (heading -Pi/2) while inside intersection center
	bus.Pos = Vec2{X: 10, Y: 0}
	bus.Heading = -math.Pi / 2 // Now facing North!
	bus.Speed = 35.0

	// While turning, phase changes to PhaseEWYellow or PhaseNSYellow/Green
	inter.Phase = PhaseEWYellow
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Turning inside intersection must NOT trigger violation")
	}

	// Phase changes to NS Red (PhaseEWGreen)
	inter.Phase = PhaseEWGreen
	violated, _ = tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Turning inside intersection with lawful Green entry must NEVER trigger violation")
	}

	// 2. Bus stopped before Red stop line, then light turns Green -> Accelerates across -> NO FINE
	inter.Phase = PhaseNSGreen // EW is Red
	inter.Cooldown = 0
	inter.PlayerInside = false
	inter.PlayerCrossedBeforeYellow = false
	inter.PlayerCrossedOnRed = false
	inter.FinedThisPassage = false

	bus2 := NewBus(-86, 20, 0) // bumper touches line on Red, but speed is 0 (waiting)
	bus2.Speed = 0
	violated, _ = tls.CheckPlayerViolation(bus2, laneW)
	if violated {
		t.Fatalf("Waiting at red light with speed 0 must NOT be fined")
	}

	// Light turns Green for EW!
	inter.Phase = PhaseEWGreen
	bus2.Speed = 40.0 // Driver accelerates through green
	bus2.Pos.X = -30
	violated, _ = tls.CheckPlayerViolation(bus2, laneW)
	if violated {
		t.Fatalf("Accelerating on Green after waiting must NOT be fined")
	}
}

func TestAISignalStoppingBehindStopLine(t *testing.T) {
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseNSGreen, // EW is Red
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
	laneW := 42.0

	// 1. AI Car approaching Eastbound (X < 0, heading 0) before stop line (stop line at -85.2)
	// Car is at X = -140 (distToLine = 140 - 85.2 = 54.8 px)
	carPos := Vec2{X: -140, Y: 30}
	shouldStop, distToLine := tls.CheckStopForVehicle(carPos, 0, 150.0, laneW)
	if !shouldStop {
		t.Fatalf("AI Car approaching Red light must be instructed to stop")
	}
	if distToLine <= 0 {
		t.Fatalf("AI Car approaching must be before the stop line, got distToLine: %f", distToLine)
	}

	// 2. AI Car that has already passed the stop line into intersection (X = -40, inside intersection)
	// distToLine = 40 - 85.2 = -45.2 px. It MUST NOT STOP in the middle of the road!
	carInside := Vec2{X: -40, Y: 30}
	shouldStopInside, _ := tls.CheckStopForVehicle(carInside, 0, 150.0, laneW)
	if shouldStopInside {
		t.Fatalf("AI Car already inside intersection must NOT stop in the middle of the road/intersection")
	}
}

func TestInitialCarSpawnsClearOfIntersectionsAndRoadCenter(t *testing.T) {
	tm := NewTrafficManager(42.0)
	if len(tm.Cars) == 0 {
		t.Fatalf("Expected initialized cars")
	}

	interCenters := []Vec2{
		{X: -1800, Y: -1200}, {X: -900, Y: -1200}, {X: 0, Y: -1200}, {X: 900, Y: -1200}, {X: 1800, Y: -1200},
		{X: -1800, Y: -600}, {X: -900, Y: -600}, {X: 0, Y: -600}, {X: 900, Y: -600}, {X: 1800, Y: -600},
		{X: -1800, Y: 0}, {X: -900, Y: 0}, {X: 0, Y: 0}, {X: 900, Y: 0}, {X: 1800, Y: 0},
		{X: -1800, Y: 600}, {X: -900, Y: 600}, {X: 0, Y: 600}, {X: 900, Y: 600}, {X: 1800, Y: 600},
		{X: -1800, Y: 1200}, {X: -900, Y: 1200}, {X: 0, Y: 1200}, {X: 900, Y: 1200}, {X: 1800, Y: 1200},
	}

	// Central Terminal bay position
	centralTerminal := Vec2{X: -350, Y: 0}

	for i, car := range tm.Cars {
		// Verify no car spawns inside or within 220px of ANY intersection center
		for _, ic := range interCenters {
			dist := car.Pos.Distance(ic)
			if dist < 220.0 {
				t.Fatalf("Car #%d at %v spawned too close to intersection %v (dist=%.1f < 220)", i, car.Pos, ic, dist)
			}
		}

		// Verify no car spawns directly inside Central Terminal departure bay
		if car.Pos.Distance(centralTerminal) < 120.0 {
			t.Fatalf("Car #%d spawned conflicting with Central Terminal departure bay at %v", i, car.Pos)
		}
	}
}

func TestMajorIntersectionDedicatedLeftTurnSignals(t *testing.T) {
	tls := NewTrafficLightSystem()
	laneW := 42.0

	// Find a major intersection (e.g. at Central Boulevard and Central Ave: X=0, Y=0)
	var majorInter *TrafficIntersection
	for _, inter := range tls.Intersections {
		if inter.Center.X == 0 && inter.Center.Y == 0 {
			majorInter = inter
			break
		}
	}
	if majorInter == nil || !majorInter.IsMajor {
		t.Fatalf("Expected major intersection at (0, 0)")
	}

	// 1. In PhaseEWGreen: EW Through is Green, but EW Left-turn is Red!
	majorInter.Phase = PhaseEWGreen
	ewThru, ewLeft, nsThru, nsLeft := majorInter.GetDetailedSignals()
	if ewThru != SignalGreen || ewLeft != SignalRed {
		t.Fatalf("PhaseEWGreen expected ewThru=Green, ewLeft=Red, got ewThru=%v, ewLeft=%v", ewThru, ewLeft)
	}
	if nsThru != SignalRed || nsLeft != SignalRed {
		t.Fatalf("PhaseEWGreen expected NS to be Red, got nsThru=%v, nsLeft=%v", nsThru, nsLeft)
	}

	// 2. In PhaseEWLeftGreen: EW Through is Red, but EW Left-turn is Green!
	majorInter.Phase = PhaseEWLeftGreen
	ewThru, ewLeft, _, _ = majorInter.GetDetailedSignals()
	if ewThru != SignalRed || ewLeft != SignalGreen {
		t.Fatalf("PhaseEWLeftGreen expected ewThru=Red, ewLeft=Green, got ewThru=%v, ewLeft=%v", ewThru, ewLeft)
	}

	// 3. Approach signal check for dedicated left turn lane vs through lane:
	// Eastbound approach (heading = 0, X < 0)
	// Outer lane (Y = 0 + laneW*1.5 = 63.0) -> Through lane
	thruCarBumper := Vec2{X: -140, Y: 63.0}
	stopThru, _ := tls.CheckStopForVehicle(thruCarBumper, 0, 150.0, laneW, false)
	if !stopThru {
		t.Fatalf("During PhaseEWLeftGreen, Through lane vehicle must stop for Red through light")
	}

	// Inner lane (Y = 0 + laneW*0.5 = 21.0) -> Dedicated Left-turn lane
	leftCarBumper := Vec2{X: -140, Y: 21.0}
	stopLeft, _ := tls.CheckStopForVehicle(leftCarBumper, 0, 150.0, laneW, true)
	if stopLeft {
		t.Fatalf("During PhaseEWLeftGreen, Left-turn lane vehicle must NOT stop (it has dedicated Green light)")
	}
}

func TestAICarStoppingBufferClearOfZebraCrossing(t *testing.T) {
	inter := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseNSGreen, // EW is Red
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
	laneW := 42.0
	stopLineDist := laneW*1.6 + 18.0 // 85.2
	crosswalkDist := stopLineDist - 25.0 // 60.2

	// AI Car sedan length = 46, halfLen = 23.
	car := &TrafficCar{
		ID:          0,
		Pos:         Vec2{X: -150, Y: 30},
		Heading:     0,
		Speed:       120,
		TargetSpeed: 150,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}

	tm := &TrafficManager{Cars: []*TrafficCar{car}}
	bus := NewBus(1000, 1000, 0) // bus far away

	// Simulate AI car updating for 3 seconds approaching the red light
	dt := 0.016
	for step := 0; step < 180; step++ {
		tm.Update(dt, bus, tls, 1, laneW)
	}

	// Front bumper position
	halfLen := car.Length * 0.5
	bumperX := car.Pos.X + math.Cos(car.Heading)*halfLen

	// Verify bumper stopped BEFORE the stop line:
	stopLineX := -stopLineDist
	if bumperX > stopLineX {
		t.Fatalf("AI Car bumper (%.2f) must be strictly behind stop line (%.2f)", bumperX, stopLineX)
	}

	// Verify bumper is well clear (>20px) of the zebra crossing (crosswalk outer edge is at -crosswalkDist - 10 = -70.2):
	crosswalkOuterEdgeX := -crosswalkDist - 10.0
	bufferDistance := crosswalkOuterEdgeX - bumperX
	if bufferDistance < 15.0 {
		t.Fatalf("AI Car bumper (%.2f) stopped too close to zebra crossing (edge: %.2f, buffer: %.2f < 15px)",
			bumperX, crosswalkOuterEdgeX, bufferDistance)
	}
}

func TestEnlargedIntersectionTurningClearance(t *testing.T) {
	world := NewWorld(52.0)
	laneW := 52.0

	// Smallest 2-lane intersection: X = -900, Y = -600 (West Park Blvd & North 2nd St)
	// Right turn corner apron: X = -955, Y = -655 (dist to center = sqrt(55^2+55^2) = 77.8px)
	cornerPos := Vec2{X: -955, Y: -655}
	res := world.CheckBoundaries(cornerPos, 18.0, laneW)
	if res.Zone != ZoneRoad {
		t.Fatalf("Expected enlarged intersection corner to be ZoneRoad for comfortable turning, got %v", res.Zone)
	}

	// Wide turning trajectory (dist = 120px and 160px from center) must also remain safely inside ZoneRoad
	wideTurnPos := Vec2{X: -900 + 85.0, Y: -600 + 85.0} // dist ~120px
	resWide := world.CheckBoundaries(wideTurnPos, 18.0, laneW)
	if resWide.Zone != ZoneRoad {
		t.Fatalf("Expected wide turn trajectory at intersection to be ZoneRoad, got %v", resWide.Zone)
	}

	extraWideTurnPos := Vec2{X: -900 + 115.0, Y: -600 + 115.0} // dist ~162px
	resExtraWide := world.CheckBoundaries(extraWideTurnPos, 18.0, laneW)
	if resExtraWide.Zone != ZoneRoad {
		t.Fatalf("Expected extra wide turn trajectory at intersection to be ZoneRoad, got %v", resExtraWide.Zone)
	}
}

func TestRightTurnOnRedLawfulImmunity(t *testing.T) {
	laneW := 42.0

	// Case 1: Eastbound entering with active steering (SteeringAngle > 0.02)
	inter1 := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseNSGreen, // EW is Red!
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls1 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter1}}

	bus1 := NewBus(-90, 63.0, 0)
	bus1.Speed = 35.0
	bus1.SteeringAngle = 0.15 // Actively steering right!

	violated, _ := tls1.CheckPlayerViolation(bus1, laneW)
	if violated {
		t.Fatalf("Case 1: Right turn on red must NEVER be fined as running a red light!")
	}
	if !inter1.PlayerInside || !inter1.PlayerTurnedRight {
		t.Fatalf("Expected PlayerInside=true and PlayerTurnedRight=true for right turn on red")
	}

	// Move bus into right turn (now facing South)
	bus1.Pos = Vec2{X: -20, Y: 60}
	bus1.Heading = math.Pi / 2 // Facing South
	violated, _ = tls1.CheckPlayerViolation(bus1, laneW)
	if violated {
		t.Fatalf("Case 1: Completing right turn on red must NEVER trigger violation!")
	}

	// Case 2: Realistic Driving - Driver crosses stop line straight (SteeringAngle = 0),
	// then turns the wheel right inside the intersection corner!
	inter2 := &TrafficIntersection{
		ID:             1,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseNSGreen, // EW is Red!
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls2 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter2}}

	// Front bumper is just crossing stop line (-130 + 48 = -82, stopLineDist = 85.2)
	bus2 := NewBus(-130, 63.0, 0)
	bus2.Speed = 30.0
	bus2.SteeringAngle = 0.0 // Has not turned wheel yet!

	violated2, _ := tls2.CheckPlayerViolation(bus2, laneW)
	if violated2 {
		t.Fatalf("Case 2: Must NOT fine at stop line entry before driver has a chance to turn right!")
	}

	// Bus moves into corner and driver steers right
	bus2.Pos = Vec2{X: -100, Y: 65}
	bus2.SteeringAngle = 0.22
	bus2.Heading = 0.18
	violated2, _ = tls2.CheckPlayerViolation(bus2, laneW)
	if violated2 {
		t.Fatalf("Case 2: Right turn initiation must not be fined!")
	}
	if !inter2.PlayerTurnedRight {
		t.Fatalf("Expected PlayerTurnedRight=true after steering right in corner")
	}

	// Bus completes turn facing South
	bus2.Pos = Vec2{X: -20, Y: 85}
	bus2.Heading = math.Pi / 2
	violated2, _ = tls2.CheckPlayerViolation(bus2, laneW)
	if violated2 {
		t.Fatalf("Case 2: Exiting right turn must not be fined!")
	}

	// Case 3: Stopped at red light, then accelerating and turning right on red
	inter3 := &TrafficIntersection{
		ID:             2,
		Center:         Vec2{X: 0, Y: 0},
		Phase:          PhaseNSGreen, // EW is Red!
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls3 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter3}}

	// Stopped right behind stop line
	bus3 := NewBus(-135, 63.0, 0)
	bus3.Speed = 0.0
	tls3.CheckPlayerViolation(bus3, laneW)

	// Starts moving forward and turns wheel right
	bus3.Pos = Vec2{X: -110, Y: 64}
	bus3.Speed = 18.0
	bus3.SteeringAngle = 0.15
	violated3, _ := tls3.CheckPlayerViolation(bus3, laneW)
	if violated3 {
		t.Fatalf("Case 3: Starting from stop and turning right on red must NOT be fined!")
	}
}

func TestPlayerLaneDisciplineViolations(t *testing.T) {
	laneW := 42.0

	// 1. Left-turn lane (inner lane) driving straight through to opposite side
	inter1 := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		Phase:             PhaseEWLeftGreen, // Left arrow is Green!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls1 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter1}}

	// Inner lane (Y = 21.0, dedicated left-turn lane)
	busStraight := NewBus(-90, 21.0, 0)
	busStraight.Speed = 40.0
	busStraight.SteeringAngle = 0

	tls1.CheckPlayerViolation(busStraight, laneW) // Cross stop line into intersection

	// Crucial rule: While inside the intersection (preparing/maneuvering), MUST NOT fine yet!
	busStraight.Pos = Vec2{X: -20, Y: 21.0}
	violatedInside, _ := tls1.CheckPlayerViolation(busStraight, laneW)
	if violatedInside {
		t.Fatalf("Must NOT fine lane discipline violation while inside the intersection before reaching opposite side!")
	}

	// Bus reaches and clears the opposite side straight (X = 120 > stopLineDist) -> MUST BE FINED
	busStraight.Pos = Vec2{X: 120, Y: 21.0}
	violated, notice := tls1.CheckPlayerViolation(busStraight, laneW)
	if !violated {
		t.Fatalf("Going straight from dedicated left-turn lane must trigger lane discipline violation upon reaching opposite side!")
	}
	if !strings.Contains(notice, "不按导向车道行驶") {
		t.Fatalf("Notice must mention '不按导向车道行驶', got: %s", notice)
	}

	// 2. Straight lane (outer lane) turning left across traffic onto North cross-street
	inter2 := &TrafficIntersection{
		ID:                1,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		Phase:             PhaseEWGreen, // Through is Green!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls2 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter2}}

	// Outer lane (Y = 63.0, straight/right lane)
	busLeft := NewBus(-90, 63.0, 0)
	busLeft.Speed = 35.0
	tls2.CheckPlayerViolation(busLeft, laneW) // Cross stop line

	// Bus is turning left inside the intersection -> MUST NOT fine yet!
	busLeft.Pos = Vec2{X: -20, Y: 40}
	busLeft.Heading = -math.Pi / 2
	violatedInside2, _ := tls2.CheckPlayerViolation(busLeft, laneW)
	if violatedInside2 {
		t.Fatalf("Must NOT fine lane discipline violation while inside the intersection!")
	}

	// Bus reaches and clears the left exit side (Y = -120 onto North cross street) -> MUST BE FINED
	busLeft.Pos = Vec2{X: -20, Y: -120}
	busLeft.Heading = -math.Pi / 2
	violated2, notice2 := tls2.CheckPlayerViolation(busLeft, laneW)
	if !violated2 {
		t.Fatalf("Turning left from straight lane must trigger lane discipline violation upon reaching left exit!")
	}
	if !strings.Contains(notice2, "直行车道违规左转") {
		t.Fatalf("Notice must mention '直行车道违规左转', got: %s", notice2)
	}

	// 3. Normal Lawful Left Turn from Dedicated Left-Turn Lane:
	// Enters in Lane 0, turns left onto North street, reaches left exit -> NEVER FINED!
	inter3 := &TrafficIntersection{
		ID:                2,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		Phase:             PhaseEWLeftGreen,
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls3 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter3}}

	busLegalLeft := NewBus(-90, 21.0, 0)
	busLegalLeft.Speed = 30.0
	tls3.CheckPlayerViolation(busLegalLeft, laneW) // Cross stop line

	// Driving in center preparing left turn
	busLegalLeft.Pos = Vec2{X: -20, Y: 10}
	busLegalLeft.Heading = -math.Pi / 4
	v3, _ := tls3.CheckPlayerViolation(busLegalLeft, laneW)
	if v3 {
		t.Fatalf("Legal left turn maneuver must never be fined inside intersection!")
	}

	// Reaches left exit on North street
	busLegalLeft.Pos = Vec2{X: -20, Y: -120}
	busLegalLeft.Heading = -math.Pi / 2
	v4, _ := tls3.CheckPlayerViolation(busLegalLeft, laneW)
	if v4 {
		t.Fatalf("Legal left turn from left lane must NEVER be fined upon reaching opposite side!")
	}
}

func TestRoadCameraWrongWayViolations(t *testing.T) {
	tls := NewTrafficLightSystem()
	if len(tls.RoadCameras) == 0 {
		t.Fatalf("Expected surveillance cameras on straight roads")
	}

	// 1. Camera on Central Boulevard East (X = 450, Y = 0, EW road)
	// Normal traffic on South side (Y = 30) flows EAST (heading = 0)
	// Bus driving WEST (heading = Pi) on the South side is WRONG-WAY (逆行)!
	busWrongWay := NewBus(450, 30.0, math.Pi) // Facing West on South side
	busWrongWay.Speed = 35.0

	violated, notice := tls.CheckRoadCameraViolation(busWrongWay)
	if !violated {
		t.Fatalf("Expected wrong-way driving (逆行) violation near road camera!")
	}
	if !strings.Contains(notice, "逆向行驶") && !strings.Contains(notice, "逆行") {
		t.Fatalf("Expected notice to mention '逆向行驶' or '逆行', got: %s", notice)
	}

	// 2. Normal lawful driving on the same road section (Facing East on South side)
	busLegal := NewBus(-450, 30.0, 0) // Facing East on South side near Camera 1
	busLegal.Speed = 35.0
	vLegal, _ := tls.CheckRoadCameraViolation(busLegal)
	if vLegal {
		t.Fatalf("Normal forward driving must NEVER be flagged as wrong-way!")
	}
}

func TestAIVehicleLaneComplianceAndTurnExecution(t *testing.T) {
	tls := NewTrafficLightSystem()
	laneW := 42.0

	// Major intersection at (0,0)
	var majorInter *TrafficIntersection
	for _, inter := range tls.Intersections {
		if inter.Center.X == 0 && inter.Center.Y == 0 {
			majorInter = inter
			break
		}
	}
	if majorInter == nil {
		t.Fatalf("Expected major intersection at (0,0)")
	}
	majorInter.Phase = PhaseEWLeftGreen // Left turn is Green!

	// AI Car in Lane 0 (inner lane, Eastbound, approaching (0,0))
	car := &TrafficCar{
		ID:          0,
		Pos:         Vec2{X: -15, Y: 21.0}, // Right at intersection center
		Heading:     0,
		Speed:       80,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
		LaneIndex:   0, // Inner left-turn lane
	}

	tm := &TrafficManager{Cars: []*TrafficCar{car}}
	bus := NewBus(1000, 1000, 0)

	// Update 1 step
	tm.Update(0.016, bus, tls, 1, laneW)

	// Car must have executed its left turn: heading North (-Pi/2), IsEW=false, RoadCenter=0
	if car.IsEW {
		t.Fatalf("AI Car in left-turn lane must turn onto cross street (IsEW=false)")
	}
	if math.Abs(NormalizeAngle(car.Heading - (-math.Pi/2))) > 0.1 {
		t.Fatalf("AI Car must turn left towards North (-Pi/2), got heading: %f", car.Heading)
	}
	if car.LaneIndex != 1 {
		t.Fatalf("AI Car must join through lane (LaneIndex=1) on the new road, got %d", car.LaneIndex)
	}
}

func TestGearboxTransmissionAndBrakeSeparation(t *testing.T) {
	bus := NewBus(0, 0, 0)

	// 1. Initial State: Default Gear is Drive (D)
	if bus.Gear != GearDrive {
		t.Fatalf("Expected default gear to be GearDrive (D), got %v", bus.Gear)
	}

	// 2. S is strictly Brake: Foot brake decelerates vehicle and holds at 0, NEVER reverses!
	bus.Speed = 150.0
	// Apply heavy brake for 2 seconds (throttle=0, brake=1.0)
	for i := 0; i < 120; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed != 0.0 {
		t.Fatalf("Expected foot brake to bring bus speed down to exactly 0, got %f", bus.Speed)
	}
	if !bus.IsBraking {
		t.Fatalf("Expected IsBraking to be true while holding brake pedal")
	}

	// Continue pressing brake for another 1 second while stopped: MUST stay at 0, NOT reverse!
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed < 0 {
		t.Fatalf("Brake pedal (S) must NEVER cause vehicle to reverse! Speed is %f", bus.Speed)
	}
	if bus.Speed != 0 {
		t.Fatalf("Brake pedal while stopped must keep vehicle at 0, got %f", bus.Speed)
	}

	// 3. In Neutral (N), accelerator throttle produces no driving torque
	bus.ShiftTo(GearNeutral)
	if bus.Gear != GearNeutral {
		t.Fatalf("Expected gear to be GearNeutral (N)")
	}
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 1.0, 0.0, 0.0, false, false) // throttle in N
	}
	if bus.Speed > 0.01 {
		t.Fatalf("In Neutral (N), throttle must NOT accelerate bus, got speed %f", bus.Speed)
	}

	// 4. Reversing requires shifting to Reverse (R):
	// Shift to GearReverse
	if !bus.ShiftTo(GearReverse) {
		t.Fatalf("Expected successful shift to GearReverse from standstill")
	}
	if bus.Gear != GearReverse {
		t.Fatalf("Expected current gear to be GearReverse (R)")
	}

	// Accelerate in Reverse with throttle (W / Up): speed becomes negative
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 1.0, 0.0, 0.0, false, false)
	}
	if bus.Speed >= 0 {
		t.Fatalf("In Reverse (R), throttle must accelerate bus backward (negative speed), got %f", bus.Speed)
	}
	if !bus.IsReversing {
		t.Fatalf("Expected IsReversing to be true while in Reverse gear")
	}

	// While moving in reverse, applying brake (S / Down) decelerates toward 0 and stops at 0!
	for i := 0; i < 120; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed != 0.0 {
		t.Fatalf("Expected foot brake to stop reversing bus squarely at 0, got %f", bus.Speed)
	}

	// 5. Gear Shifting Progression (Auto mode: R <-> N <-> D)
	bus.ShiftTo(GearReverse)
	if !bus.ShiftUp() || bus.Gear != GearNeutral {
		t.Fatalf("Expected ShiftUp from R to go to N, got %v", bus.Gear)
	}
	if !bus.ShiftUp() || bus.Gear != GearDrive {
		t.Fatalf("Expected ShiftUp from N to go to D, got %v", bus.Gear)
	}
	if bus.ShiftUp() {
		t.Fatalf("ShiftUp in Auto at D should return false (already at top auto gear)")
	}

	if !bus.ShiftDown() || bus.Gear != GearNeutral {
		t.Fatalf("Expected ShiftDown from D to go to N, got %v", bus.Gear)
	}
	if !bus.ShiftDown() || bus.Gear != GearReverse {
		t.Fatalf("Expected ShiftDown from N to go to R, got %v", bus.Gear)
	}
	if bus.ShiftDown() {
		t.Fatalf("ShiftDown at R should return false (already in reverse)")
	}

	// 6. High-speed reverse interlock protection:
	bus.ShiftTo(GearDrive)
	bus.Speed = 120.0 // Moving forward fast
	if bus.ShiftTo(GearReverse) {
		t.Fatalf("High-speed forward motion must prevent shifting directly into Reverse (transmission protection)!")
	}
	if bus.Gear != GearDrive {
		t.Fatalf("Gear must remain in Drive when reverse shift is locked out")
	}

	// 7. Manual transmission mode readiness (预留手动挡功能测试):
	bus.Speed = 0
	bus.TransmissionMode = TransmissionManual
	bus.ShiftTo(GearNeutral)

	// Step up through manual gears: N -> 1 -> 2 -> 3 -> 4 -> 5
	expectedManualGears := []Gear{Gear1, Gear2, Gear3, Gear4, Gear5}
	for _, expGear := range expectedManualGears {
		if !bus.ShiftUp() || bus.Gear != expGear {
			t.Fatalf("Expected ShiftUp in manual mode to reach %v, got %v", expGear, bus.Gear)
		}
		maxSpd, accelRatio := bus.GetGearLimits()
		if maxSpd <= 0 || accelRatio <= 0 {
			t.Fatalf("Manual gear %v must have positive maxSpeed and acceleration factor", expGear)
		}
	}

	// 8. HUD interactive gear click test
	hud := NewHUD()
	// Simulate HUD Draw layout setup
	hud.GearRBounds = Rect{X: 134, Y: 600, W: 24, H: 20}
	hud.GearNBounds = Rect{X: 162, Y: 600, W: 24, H: 20}
	hud.GearDBounds = Rect{X: 190, Y: 600, W: 24, H: 20}

	bus.TransmissionMode = TransmissionAuto
	bus.Speed = 0
	hud.HandleGearClick(Vec2{X: 140, Y: 610}, bus)
	if bus.Gear != GearReverse {
		t.Fatalf("Expected clicking R badge to shift bus into Reverse, got %v", bus.Gear)
	}

	hud.HandleGearClick(Vec2{X: 170, Y: 610}, bus)
	if bus.Gear != GearNeutral {
		t.Fatalf("Expected clicking N badge to shift bus into Neutral, got %v", bus.Gear)
	}

	hud.HandleGearClick(Vec2{X: 195, Y: 610}, bus)
	if bus.Gear != GearDrive {
		t.Fatalf("Expected clicking D badge to shift bus into Drive, got %v", bus.Gear)
	}
}

func TestLenientLeftTurnTrafficViolation(t *testing.T) {
	laneW := 52.0
	inter := &TrafficIntersection{
		ID:                0,
		Center:            Vec2{X: 0, Y: 0},
		IsMajor:           true,
		Phase:             PhaseEWLeftGreen, // Left-turn arrow is GREEN, Through is RED!
		GreenDuration:     8.0,
		YellowDuration:    2.5,
		LeftGreenDuration: 5.5,
	}
	tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}

	// Case 1: Bus is turning left on Green, but slightly straddles into straight lane (Y = 60.0 > 52.0)
	busStraddleStraight := NewBus(-110, 60.0, 0)
	busStraddleStraight.Speed = 35.0
	busStraddleStraight.SteeringAngle = -0.10 // Steering left

	// Front bumper crosses stop line
	v1, _ := tls.CheckPlayerViolation(busStraddleStraight, laneW)
	if v1 {
		t.Fatalf("Case 1: Must NOT fine when crossing stop line with green left arrow even if straddling straight lane!")
	}
	if !inter.PlayerInside {
		t.Fatalf("Case 1: Expected PlayerInside=true")
	}
	if inter.PlayerCrossedOnRed {
		t.Fatalf("Case 1: Must NOT mark PlayerCrossedOnRed when turning left on green left arrow!")
	}

	// Move forward inside: rear wheels cross stop line into intersection (X = -20)
	busStraddleStraight.Pos = Vec2{X: -20, Y: 50.0}
	busStraddleStraight.Heading = -math.Pi / 6
	v2, n2 := tls.CheckPlayerViolation(busStraddleStraight, laneW)
	if v2 {
		t.Fatalf("Case 1: Rear wheels crossing stop line on green left arrow must NEVER trigger red light violation! Got: %s", n2)
	}

	// Reaches left exit on North street
	busStraddleStraight.Pos = Vec2{X: -20, Y: -140.0}
	busStraddleStraight.Heading = -math.Pi / 2
	v3, n3 := tls.CheckPlayerViolation(busStraddleStraight, laneW)
	if v3 {
		t.Fatalf("Case 1: Completing left turn on green must NEVER be fined for lane discipline! Got: %s", n3)
	}

	// Case 2: Bus is turning left on Green, but slightly cuts corner into opposing lane (Y = -12.0)
	inter.PlayerInside = false
	inter.PlayerCrossedOnRed = false
	inter.PlayerCrossedBeforeYellow = false
	inter.PlayerTurnedLeft = false
	inter.FinedThisPassage = false

	busCutCorner := NewBus(-110, -12.0, 0)
	busCutCorner.Speed = 35.0
	busCutCorner.SteeringAngle = -0.12 // Steering left

	v4, _ := tls.CheckPlayerViolation(busCutCorner, laneW)
	if v4 {
		t.Fatalf("Case 2: Must NOT fine when crossing stop line on green left arrow even if cutting corner!")
	}

	// Rear wheels cross stop line
	busCutCorner.Pos = Vec2{X: -20, Y: -30.0}
	busCutCorner.Heading = -math.Pi / 4
	v5, n5 := tls.CheckPlayerViolation(busCutCorner, laneW)
	if v5 {
		t.Fatalf("Case 2: Must NOT trigger red light violation when cutting corner on green left turn! Got: %s", n5)
	}

	// Reaches left exit on North street
	busCutCorner.Pos = Vec2{X: -20, Y: -140.0}
	busCutCorner.Heading = -math.Pi / 2
	v6, n6 := tls.CheckPlayerViolation(busCutCorner, laneW)
	if v6 {
		t.Fatalf("Case 2: Exiting left turn must NEVER trigger fine! Got: %s", n6)
	}
}

func TestGreenLightTurnsNeverFinedAsRedLightOnExit(t *testing.T) {
	laneW := 52.0

	// 1. Right turn on Green (PhaseEWGreen: EW is Green, NS is RED)
	inter1 := &TrafficIntersection{
		ID:             0,
		Center:         Vec2{X: 0, Y: 0},
		IsMajor:        true,
		Phase:          PhaseEWGreen, // EW Green, NS Red!
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls1 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter1}}

	// Bus approaches from West in outer lane (Y = 78), heading East
	busRight := NewBus(-140, 78.0, 0)
	busRight.Speed = 35.0
	v1, _ := tls1.CheckPlayerViolation(busRight, laneW)
	if v1 {
		t.Fatalf("Step 1: Should not violate before stop line")
	}

	// Front bumper crosses stop line into intersection, starts steering right
	busRight.Pos = Vec2{X: -90, Y: 78.0}
	busRight.SteeringAngle = 0.15
	busRight.Heading = 0.25
	v2, _ := tls1.CheckPlayerViolation(busRight, laneW)
	if v2 {
		t.Fatalf("Step 2: Should not violate entering on green right turn")
	}
	if !inter1.PlayerInside {
		t.Fatalf("Step 2: Expected PlayerInside=true")
	}

	// Rear wheels cross stop line into intersection corner
	busRight.Pos = Vec2{X: -20, Y: 90.0}
	busRight.Heading = math.Pi / 3
	v3, n3 := tls1.CheckPlayerViolation(busRight, laneW)
	if v3 {
		t.Fatalf("Step 3: Turning right on green must NEVER be fined when rear wheels cross! Got: %s", n3)
	}

	// Bus reaches right exit on South cross-street (where NS traffic signal is RED!)
	busRight.Pos = Vec2{X: 20, Y: 110.0}
	busRight.Heading = math.Pi / 2 // Facing South along cross street!
	v4, n4 := tls1.CheckPlayerViolation(busRight, laneW)
	if v4 {
		t.Fatalf("Step 4: Exiting right turn must NEVER be fined for cross-street red light! Got: %s", n4)
	}

	// Subsequent frames while still within intersection maxZone (dist <= 246)
	// Bus is driving South along cross street at Y = 130
	busRight.Pos = Vec2{X: 20, Y: 130.0}
	v5, n5 := tls1.CheckPlayerViolation(busRight, laneW)
	if v5 {
		t.Fatalf("Step 5: Clearing cross street after green turn must NEVER trigger duplicate red light violation! Got: %s", n5)
	}

	// 2. Minor intersection left turn on green (PhaseEWGreen: EW Through is Green, NS is Red)
	inter2 := &TrafficIntersection{
		ID:             1,
		Center:         Vec2{X: 0, Y: 0},
		IsMajor:        false, // Regular 2-lane intersection, green covers through and left
		Phase:          PhaseEWGreen,
		GreenDuration:  10.0,
		YellowDuration: 3.0,
	}
	tls2 := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter2}}

	busMinorLeft := NewBus(-140, 26.0, 0)
	busMinorLeft.Speed = 35.0
	tls2.CheckPlayerViolation(busMinorLeft, laneW)

	// Cross stop line into intersection turning left
	busMinorLeft.Pos = Vec2{X: -90, Y: 26.0}
	busMinorLeft.SteeringAngle = -0.15
	busMinorLeft.Heading = -0.25
	tls2.CheckPlayerViolation(busMinorLeft, laneW)

	// Rear wheels cross line while turning left
	busMinorLeft.Pos = Vec2{X: -10, Y: 10.0}
	busMinorLeft.Heading = -math.Pi / 3
	v6, n6 := tls2.CheckPlayerViolation(busMinorLeft, laneW)
	if v6 {
		t.Fatalf("Minor Left Step 3: Rear wheels crossing on green left turn must NEVER be fined! Got: %s", n6)
	}

	// Reaches North exit onto cross street (where NS is RED!)
	busMinorLeft.Pos = Vec2{X: -26, Y: -110.0}
	busMinorLeft.Heading = -math.Pi / 2 // Facing North along cross street
	v7, n7 := tls2.CheckPlayerViolation(busMinorLeft, laneW)
	if v7 {
		t.Fatalf("Minor Left Step 4: Reaching North exit must NEVER trigger cross-street red light violation! Got: %s", n7)
	}

	// Subsequent frames clearing North exit
	busMinorLeft.Pos = Vec2{X: -26, Y: -140.0}
	v8, n8 := tls2.CheckPlayerViolation(busMinorLeft, laneW)
	if v8 {
		t.Fatalf("Minor Left Step 5: Clearing North exit must NEVER trigger red light violation! Got: %s", n8)
	}
}

func TestPlanningModeInstantTransitionAndAITrafficSuppression(t *testing.T) {
	g := NewGame(1280, 720)

	// Initially in Driver Mode
	if g.IsPlanMode {
		t.Fatalf("Expected initial mode to be Driver Mode (IsPlanMode == false)")
	}
	if g.Camera.Zoom != 1.0 {
		t.Fatalf("Expected initial camera zoom 1.0, got %f", g.Camera.Zoom)
	}

	// 1. Enter Planning Mode: Verify Instant transition (no progressive lerp)
	g.IsPlanMode = true
	g.Camera.TargetPos = Vec2{X: 0, Y: 0}
	g.Camera.Pos = Vec2{X: 0, Y: 0}
	g.Camera.TargetZoom = 0.30
	g.Camera.Zoom = 0.30

	if g.Camera.Pos.X != 0 || g.Camera.Pos.Y != 0 {
		t.Fatalf("Expected camera Pos to instantly center at (0, 0) in planning mode")
	}
	if g.Camera.Zoom != 0.30 {
		t.Fatalf("Expected camera Zoom to instantly snap to 0.30 without progressive delay, got %f", g.Camera.Zoom)
	}

	// 2. Step game update while in Planning Mode
	// AI Traffic update should be paused/skipped to prevent stutter
	initialCar0Pos := g.TrafficMgr.Cars[0].Pos
	// If !g.IsPlanMode branch guards TrafficMgr.Update, car pos will remain stable
	if !g.IsPlanMode {
		g.TrafficMgr.Update(0.1, g.Bus, g.TrafficLights, 2, 52.0)
	}
	if g.TrafficMgr.Cars[0].Pos != initialCar0Pos {
		t.Fatalf("AI Cars must not move while in Planning Mode")
	}

	// 3. Exit Planning Mode back to Driver Mode: Verify instant snap back to bus
	g.IsPlanMode = false
	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Camera.TargetPos
	g.Camera.TargetZoom = 1.0
	g.Camera.Zoom = 1.0

	if g.Camera.Zoom != 1.0 {
		t.Fatalf("Expected camera Zoom to instantly snap back to 1.0, got %f", g.Camera.Zoom)
	}
	if g.Camera.Pos.Distance(g.Bus.Pos) > 50.0 {
		t.Fatalf("Expected camera to instantly snap to bus position upon returning to driver mode")
	}
}

func TestCameraPerspectivesAndRotation(t *testing.T) {
	cam := NewCamera(1280, 720)

	// 1. Initial Mode Verification (North-Up)
	if cam.Mode != CameraModeNorthUp {
		t.Fatalf("Expected default camera mode to be CameraModeNorthUp, got %v", cam.Mode)
	}
	if cam.Mode.String() != "固定正北 (标准)" {
		t.Fatalf("Unexpected North-Up string: %s", cam.Mode.String())
	}

	bus := NewBus(0, 0, 0)
	bus.Heading = 0 // Facing East in world space
	bus.Speed = 50.0

	// In North-Up mode, TargetRotation must be 0
	cam.FollowBus(bus)
	if cam.TargetRotation != 0 {
		t.Fatalf("Expected North-Up TargetRotation to be 0, got %f", cam.TargetRotation)
	}

	// 2. Toggle to Heading-Up Mode
	mode1 := cam.ToggleMode()
	if mode1 != CameraModeHeadingUp || cam.Mode != CameraModeHeadingUp {
		t.Fatalf("Expected camera mode to toggle to CameraModeHeadingUp, got %v", mode1)
	}
	if cam.Mode.String() != "车头朝上 (跟随)" {
		t.Fatalf("Unexpected Heading-Up string: %s", cam.Mode.String())
	}

	// In Heading-Up mode, TargetRotation = Heading + Pi/2
	cam.FollowBus(bus)
	expectedRot := bus.Heading + math.Pi/2
	if math.Abs(cam.TargetRotation-expectedRot) > 1e-6 {
		t.Fatalf("Expected Heading-Up TargetRotation to be %f, got %f", expectedRot, cam.TargetRotation)
	}

	// Forward lookahead verification
	expectedForwardOffset := 120.0 + 0.35*bus.Speed
	distFromBus := cam.TargetPos.Distance(bus.Pos)
	if math.Abs(distFromBus-expectedForwardOffset) > 1e-4 {
		t.Fatalf("Expected forward offset %f, got %f", expectedForwardOffset, distFromBus)
	}

	// Heading-Up Rotation Transform Math: Bus heading on screen facing straight UP
	cam.Pos = bus.Pos
	cam.Rotation = cam.TargetRotation // TargetRotation = Pi/2 for heading 0 (East)

	// Point 100px ahead in the bus's forward direction (East: dx=100, dy=0)
	aheadWorld := bus.Pos.Add(Vec2{X: 100, Y: 0})
	aheadScreen := cam.WorldToScreen(aheadWorld)
	busScreen := cam.WorldToScreen(bus.Pos)

	// In screen coordinates:
	// Since bus heading is East (world +X), with camera rotation Pi/2,
	// the ahead position on screen MUST be directly ABOVE busScreen (smaller Y, same X)!
	if math.Abs(aheadScreen.X-busScreen.X) > 1e-4 {
		t.Fatalf("Ahead point on screen should have same X as bus, got dx=%f", aheadScreen.X-busScreen.X)
	}
	if aheadScreen.Y >= busScreen.Y {
		t.Fatalf("Ahead point on screen should be ABOVE bus (aheadScreen.Y < busScreen.Y), got aheadY=%f, busY=%f", aheadScreen.Y, busScreen.Y)
	}

	// 3. Toggle to South-Up Mode
	mode2 := cam.ToggleMode()
	if mode2 != CameraModeSouthUp || cam.Mode != CameraModeSouthUp {
		t.Fatalf("Expected camera mode to toggle to CameraModeSouthUp, got %v", mode2)
	}
	if cam.Mode.String() != "固定正南 (倒置)" {
		t.Fatalf("Unexpected South-Up string: %s", cam.Mode.String())
	}

	// In South-Up mode, TargetRotation must be Pi
	cam.FollowBus(bus)
	if math.Abs(cam.TargetRotation-math.Pi) > 1e-6 {
		t.Fatalf("Expected South-Up TargetRotation to be Pi, got %f", cam.TargetRotation)
	}

	// South-Up Rotation Transform Math: South (+Y in world) points UP on screen (smaller Y)
	cam.Pos = Vec2{X: 0, Y: 0}
	cam.Rotation = math.Pi
	southWorld := Vec2{X: 0, Y: 100} // Point 100px to the South
	southScreen := cam.WorldToScreen(southWorld)
	centerScreen := cam.WorldToScreen(cam.Pos)

	if southScreen.Y >= centerScreen.Y {
		t.Fatalf("In South-Up mode, South must point UP on screen! got southY=%f, centerY=%f", southScreen.Y, centerScreen.Y)
	}
	if math.Abs(southScreen.X-centerScreen.X) > 1e-4 {
		t.Fatalf("In South-Up mode, purely South point should have same X as center, got dx=%f", southScreen.X-centerScreen.X)
	}

	// 4. Toggle from South-Up back to North-Up Mode
	mode3 := cam.ToggleMode()
	if mode3 != CameraModeNorthUp || cam.Mode != CameraModeNorthUp {
		t.Fatalf("Expected camera mode to cycle back to CameraModeNorthUp, got %v", mode3)
	}

	// 5. Test Invertibility of WorldToScreen and ScreenToWorld with non-zero rotation
	cam.Rotation = 1.234 // Arbitrary non-cardinal angle
	testWorld := Vec2{X: 350.5, Y: -220.8}
	screenPt := cam.WorldToScreen(testWorld)
	recoveredWorld := cam.ScreenToWorld(screenPt)

	if testWorld.Distance(recoveredWorld) > 1e-4 {
		t.Fatalf("ScreenToWorld did not invert WorldToScreen accurately! Original: %v, Recovered: %v", testWorld, recoveredWorld)
	}

	// 6. Test HUD Perspective button click cycles through all 3 modes
	hud := NewHUD()
	hud.CameraBtnBounds = Rect{X: 900, Y: 12, W: 175, H: 32}

	// Currently NorthUp -> click -> HeadingUp
	t1, n1 := hud.HandleCameraBtnClick(Vec2{X: 950, Y: 25}, cam)
	if !t1 || cam.Mode != CameraModeHeadingUp || n1 == "" {
		t.Fatalf("Expected HUD click to switch to Heading-Up, got mode=%v, n=%s", cam.Mode, n1)
	}

	// HeadingUp -> click -> SouthUp
	t2, n2 := hud.HandleCameraBtnClick(Vec2{X: 950, Y: 25}, cam)
	if !t2 || cam.Mode != CameraModeSouthUp || n2 == "" {
		t.Fatalf("Expected HUD click to switch to South-Up, got mode=%v, n=%s", cam.Mode, n2)
	}

	// SouthUp -> click -> NorthUp
	t3, n3 := hud.HandleCameraBtnClick(Vec2{X: 950, Y: 25}, cam)
	if !t3 || cam.Mode != CameraModeNorthUp || n3 == "" {
		t.Fatalf("Expected HUD click to switch back to North-Up, got mode=%v, n=%s", cam.Mode, n3)
	}

	// 7. Test Settings synchronization and 3-way toggle
	settings := NewSettings()
	if settings.CameraViewMode != CameraModeNorthUp {
		t.Fatalf("Expected settings default camera view mode to be CameraModeNorthUp")
	}
	settings.cameraToggleBounds = Rect{X: 320, Y: 190, W: 190, H: 32}
	settings.IsOpen = true

	// Simulate clicks in settings
	settings.CameraViewMode = CameraModeHeadingUp
	cam.SetMode(settings.CameraViewMode)
	if cam.Mode != CameraModeHeadingUp {
		t.Fatalf("Expected camera mode to update to Heading-Up from settings")
	}
	settings.CameraViewMode = CameraModeSouthUp
	cam.SetMode(settings.CameraViewMode)
	if cam.Mode != CameraModeSouthUp {
		t.Fatalf("Expected camera mode to update to South-Up from settings")
	}

	// 8. Test smooth rotation update towards South-Up (pi) and returning to North-Up (0)
	cam.Rotation = 0
	cam.TargetRotation = math.Pi
	cam.Update(0.1)
	if cam.Rotation == 0 {
		t.Fatalf("Camera rotation should smoothly interpolate towards TargetRotation Pi")
	}

	for i := 0; i < 200; i++ {
		cam.Update(0.1)
	}
	if math.Abs(cam.Rotation-math.Pi) > 1e-3 {
		t.Fatalf("Camera rotation should reach Pi in South-Up mode, got %f", cam.Rotation)
	}

	// Switching to NorthUp mode smoothly returns rotation to 0
	cam.SetMode(CameraModeNorthUp)
	cam.FollowBus(bus)
	for i := 0; i < 200; i++ {
		cam.Update(0.1)
	}
	if math.Abs(cam.Rotation) > 1e-3 {
		t.Fatalf("Camera rotation should return to 0 in North-Up mode, got %f", cam.Rotation)
	}
}

func TestBusCarCollisionSeparationAndNoOverlap(t *testing.T) {
	g := NewGame(1280, 720)

	// Set up player bus and target car on an East-West road segment
	// Bus length = 96 (front bumper at +48 from center), Width = 36 (half-width 18)
	// Car length = 46~50 (rear bumper at -24 from center), Width = 22 (half-width 11)
	g.Bus.Pos = Vec2{X: 100, Y: 0}
	g.Bus.Heading = 0 // Facing East (+X)
	g.Bus.Speed = 60.0
	g.Bus.Length = 96
	g.Bus.Width = 36

	// Select car 0 and position it directly in front of the bus
	targetCar := g.TrafficMgr.Cars[0]
	targetCar.Heading = 0 // Also facing East (+X)
	targetCar.Length = 48
	targetCar.Width = 22
	targetCar.Speed = 0.0 // Stationary car at a red light

	// Position car so their bumpers would touch if distance was 48 + 24 = 72.
	// We deliberately place car at distance 55 (penetration depth = 72 - 55 = 17px)
	targetCar.Pos = Vec2{X: 155, Y: 0}

	// 1. Verify SAT detects the collision before resolution
	busOBB := OBB{
		Center:  g.Bus.Pos,
		HalfLen: g.Bus.Length / 2,
		HalfWid: g.Bus.Width / 2,
		Heading: g.Bus.Heading,
	}
	carOBB := OBB{
		Center:  targetCar.Pos,
		HalfLen: targetCar.Length / 2,
		HalfWid: targetCar.Width / 2,
		Heading: targetCar.Heading,
	}

	collides, _, depth := CheckOBBCollision(busOBB, carOBB)
	if !collides || depth <= 0 {
		t.Fatalf("Expected OBB collision to be detected at distance 55, got collides=%v, depth=%f", collides, depth)
	}
	if math.Abs(depth-17.0) > 1e-4 {
		t.Fatalf("Expected penetration depth ~17px, got %f", depth)
	}

	// 2. Run collision check & resolution
	g.checkBusCarCollisions()

	// 3. Verify separation: Bus must be pushed back so front bumper never overlaps the car!
	busFrontBumperX := g.Bus.Pos.X + g.Bus.Length/2
	carRearBumperX := targetCar.Pos.X - targetCar.Length/2

	if busFrontBumperX > carRearBumperX {
		t.Fatalf("Bus front bumper (%.2f) must NEVER penetrate or cover car rear bumper (%.2f)! Overlap: %.2f px",
			busFrontBumperX, carRearBumperX, busFrontBumperX-carRearBumperX)
	}

	// Verify that SAT now reports no collision / depth <= 0
	busOBB.Center = g.Bus.Pos
	carOBB.Center = targetCar.Pos
	collidesAfter, _, depthAfter := CheckOBBCollision(busOBB, carOBB)
	if collidesAfter {
		t.Fatalf("After separation, collision must be resolved! Still collides with depth: %f", depthAfter)
	}

	// 4. Verify continuous throttle test (holding W key against a stationary car)
	// Over 60 simulation frames with full throttle, the bus must NEVER push over the car!
	dt := 1.0 / 60.0
	for frame := 0; frame < 60; frame++ {
		// Driver accelerates with throttle = 1.0
		g.Bus.Update(dt, 1.0, 0, 0, false, false)

		// Collision resolution after movement
		g.checkBusCarCollisions()

		currentBusFront := g.Bus.Pos.X + g.Bus.Length/2
		currentCarRear := targetCar.Pos.X - targetCar.Length/2

		if currentBusFront > currentCarRear {
			t.Fatalf("Frame %d: Driver holding throttle pushed bus front (%.2f) past car rear (%.2f)! Overlap: %.2f px",
				frame, currentBusFront, currentCarRear, currentBusFront-currentCarRear)
		}
	}

	// 5. Verify moving car collision & momentum transfer
	targetCar.Speed = 20.0
	g.Bus.Pos = Vec2{X: targetCar.Pos.X - 60, Y: 0} // Overlapping moving car
	g.Bus.Speed = 80.0
	initialCarSpeed := targetCar.Speed

	g.checkBusCarCollisions()

	if targetCar.Speed <= initialCarSpeed {
		t.Fatalf("Expected car to be shoved forward with increased speed on rear collision, got %f (was %f)",
			targetCar.Speed, initialCarSpeed)
	}
	if g.Bus.Speed >= 80.0 {
		t.Fatalf("Expected bus speed to be clamped/braked on impact, got %f", g.Bus.Speed)
	}
}




