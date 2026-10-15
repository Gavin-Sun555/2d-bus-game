package game

import (
	"math"
	"os"
	"path/filepath"
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
	bus.Speed = 60                                           // moving
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

	// Test lane width is fixed to 52.0 (宽敞市区 Comfortable Wide) as default
	if settings.GetLaneWidth() != 52.0 {
		t.Fatalf("Expected default fixed lane width to be 52.0, got %f", settings.GetLaneWidth())
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
	// Verify the path segments along the road grid are orthogonal outside roundabouts,
	// curve smoothly around roundabouts, and are drawn on the driving lane side rather than in the road center
	for i := 1; i < len(path)-2; i++ {
		pA := path[i]
		pB := path[i+1]
		dx := math.Abs(pA.X - pB.X)
		dy := math.Abs(pA.Y - pB.Y)

		isRoundaboutSeg := false
		for _, rb := range world.Roundabouts {
			if pA.Distance(rb.Center) <= rb.Radius+30 || pB.Distance(rb.Center) <= rb.Radius+30 {
				isRoundaboutSeg = true
				if pA.Distance(rb.Center) < rb.InnerRadius-5.0 {
					t.Errorf("Path waypoint %v penetrates roundabout inner island (dist: %.1f < %.1f)",
						pA, pA.Distance(rb.Center), rb.InnerRadius)
				}
				break
			}
		}

		// Segments along road grid should be orthogonal or gentle lane-change tapers (outside roundabouts)
		isLaneTaper := (dx > 1.0 && dy > 1.0 && (dx <= 120.0 || dy <= 120.0))
		if !isRoundaboutSeg && !isLaneTaper && dx > 1.0 && dy > 1.0 {
			t.Errorf("Path segment from %v to %v is diagonal, expected orthogonal grid line", pA, pB)
		}

		// Verify the segment is offset to the side of Central Blvd (Y=0), not on the centerline Y=0
		if !isRoundaboutSeg && dx > 50 { // Longitudinal road segment
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
					isNS := math.Abs(r.P2.X-r.P1.X) < 1.0
					if isEW {
						// Horizontal road
						minX := math.Min(r.P1.X, r.P2.X)
						maxX := math.Max(r.P1.X, r.P2.X)
						if pt.X >= minX-20 && pt.X <= maxX+20 {
							dist := math.Abs(pt.Y - r.P1.Y)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects horizontal road %s (Y=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.Y, roadClearance, dist)
							}
						}
					} else if isNS {
						// Vertical road
						minY := math.Min(r.P1.Y, r.P2.Y)
						maxY := math.Max(r.P1.Y, r.P2.Y)
						if pt.Y >= minY-20 && pt.Y <= maxY+20 {
							dist := math.Abs(pt.X - r.P1.X)
							if dist <= roadClearance {
								t.Fatalf("Building #%d at %v intersects vertical road %s (X=%.0f, clearance=%.1f, dist=%.1f)",
									i, b.Pos, r.Name, r.P1.X, roadClearance, dist)
							}
						}
					} else {
						// Curved or diagonal road segment
						d, _ := distToSegment(pt, r.P1, r.P2)
						if d <= roadClearance {
							t.Fatalf("Building #%d at %v intersects curved road %s (clearance=%.1f, dist=%.1f)",
								i, b.Pos, r.Name, roadClearance, d)
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

	isOccupied := g.IsStopOccupiedByOtherVehicle(targetStop)
	canSkip := !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)
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

	// 2. When station has waiting passengers and is NOT occupied: CANNOT skip
	targetStop.WaitingPassengers = 5
	g.stopBellRung = false
	isOccupied = false
	canSkip = !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)
	if canSkip {
		t.Fatalf("Should not allow skip when waiting passengers > 0 and station is not occupied")
	}

	// 3. When station has waiting passengers BUT is OCCUPIED by another bus, AND no stop bell rung: CAN SKIP!
	// (当前方车站有车在靠站的时候，且车上无人下车，可直接飞站)
	targetStop.WaitingPassengers = 5
	g.stopBellRung = false
	isOccupied = true
	canSkip = !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)
	if !canSkip {
		t.Fatalf("Should allow skip when station is occupied by another vehicle and no stop bell rung on our bus")
	}

	// 4. When station is OCCUPIED, but our bus has passengers alighting (bell rung): CANNOT skip!
	// (有乘客按铃下车，必须进站停靠下客)
	targetStop.WaitingPassengers = 5
	g.stopBellRung = true
	g.passengersAlighting = 2
	isOccupied = true
	canSkip = !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)
	if canSkip {
		t.Fatalf("Should not allow skip when stop bell is rung, even if station is occupied")
	}

	// 5. When passenger rings stop bell even if station is empty: CANNOT skip
	targetStop.WaitingPassengers = 0
	g.stopBellRung = true
	g.passengersAlighting = 1
	isOccupied = false
	canSkip = !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)
	if canSkip {
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

	interCenters := make([]Vec2, 25)
	rawInters := []Vec2{
		{X: -2600, Y: -1200}, {X: -1300, Y: -1200}, {X: 0, Y: -1200}, {X: 1300, Y: -1200}, {X: 2600, Y: -1200},
		{X: -2600, Y: -600}, {X: -1300, Y: -600}, {X: 0, Y: -600}, {X: 1300, Y: -600}, {X: 2600, Y: -600},
		{X: -2600, Y: 0}, {X: -1300, Y: 0}, {X: 0, Y: 0}, {X: 1300, Y: 0}, {X: 2600, Y: 0},
		{X: -2600, Y: 600}, {X: -1300, Y: 600}, {X: 0, Y: 600}, {X: 1300, Y: 600}, {X: 2600, Y: 600},
		{X: -2600, Y: 1200}, {X: -1300, Y: 1200}, {X: 0, Y: 1200}, {X: 1300, Y: 1200}, {X: 2600, Y: 1200},
	}
	for idx, ic := range rawInters {
		interCenters[idx] = ic.Mul(MapGridScale)
	}

	// Central Terminal bay position
	centralTerminal := Vec2{X: -650 * MapGridScale, Y: 0}

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
	stopLineDist := laneW*1.6 + 18.0     // 85.2
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

	// Smallest 2-lane intersection: X = -1300 * MapGridScale, Y = -600 * MapGridScale
	icX := -1300.0 * MapGridScale
	icY := -600.0 * MapGridScale
	cornerPos := Vec2{X: icX - 55.0, Y: icY - 55.0}
	res := world.CheckBoundaries(cornerPos, 18.0, laneW)
	if res.Zone != ZoneRoad {
		t.Fatalf("Expected enlarged intersection corner to be ZoneRoad for comfortable turning, got %v", res.Zone)
	}

	// Wide turning trajectory (dist = 120px and 160px from center) must also remain safely inside ZoneRoad
	wideTurnPos := Vec2{X: icX + 85.0, Y: icY + 85.0} // dist ~120px
	resWide := world.CheckBoundaries(wideTurnPos, 18.0, laneW)
	if resWide.Zone != ZoneRoad {
		t.Fatalf("Expected wide turn trajectory at intersection to be ZoneRoad, got %v", resWide.Zone)
	}

	extraWideTurnPos := Vec2{X: icX + 115.0, Y: icY + 115.0} // dist ~162px
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

	// 1. Camera on Central Boulevard East (X = 1950 * MapGridScale, Y = 0, EW road)
	// Normal traffic on South side (Y = 30) flows EAST (heading = 0)
	// Bus driving WEST (heading = Pi) on the South side is WRONG-WAY (逆行)!
	busWrongWay := NewBus(1950*MapGridScale, 30.0, math.Pi) // Facing West on South side
	busWrongWay.Speed = 35.0

	violated, notice := tls.CheckRoadCameraViolation(busWrongWay)
	if !violated {
		t.Fatalf("Expected wrong-way driving (逆行) violation near road camera!")
	}
	if !strings.Contains(notice, "逆向行驶") && !strings.Contains(notice, "逆行") {
		t.Fatalf("Expected notice to mention '逆向行驶' or '逆行', got: %s", notice)
	}

	// 2. Normal lawful driving on the same road section (Facing East on South side)
	busLegal := NewBus(-1950*MapGridScale, 30.0, 0) // Facing East on South side near Camera 1
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
	if math.Abs(NormalizeAngle(car.Heading-(-math.Pi/2))) > 0.1 {
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

func TestAICarsAvoidStoppedBusAndNoUnfairFine(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.TrafficFinesEnabled = true

	// 1. Social vehicle smoothly stopping behind stationary bus in normal traffic flow
	// Bus stopped in eastbound lane (Y=63) on EW avenue (road center Y=0)
	g.Bus.Pos = Vec2{X: 500, Y: 63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 0
	g.Bus.Length = 96.0
	g.Bus.Width = 36.0

	if IsBusFacingWrongWay(g.Bus) {
		t.Fatalf("Bus on South side (Y=63) facing East must NOT be considered wrong-way")
	}

	approachingCar := &TrafficCar{
		Pos:         Vec2{X: 250, Y: 63.0}, // 250px away
		Heading:     0,
		Speed:       120.0,
		TargetSpeed: 120.0,
		Length:      48.0,
		Width:       22.0,
		IsEW:        true,
		RoadCenter:  0,
		LaneIndex:   1,
	}
	g.TrafficMgr.Cars = []*TrafficCar{approachingCar}

	dt := 1.0 / 60.0
	// Simulate 4.0 seconds (240 frames) of traffic AI
	for frame := 0; frame < 240; frame++ {
		g.TrafficMgr.updateSingleCar(dt, approachingCar, 0, g.Bus, g.TrafficLights, 1, 42.0, g.World)
	}

	carFrontBumperX := approachingCar.Pos.X + approachingCar.Length/2
	busRearBumperX := g.Bus.Pos.X - g.Bus.Length/2
	bumperGap := busRearBumperX - carFrontBumperX

	if approachingCar.Speed > 0.1 {
		t.Fatalf("AI car must completely stop behind stationary bus, but has speed: %.2f", approachingCar.Speed)
	}
	if bumperGap < 20.0 {
		t.Fatalf("AI car must maintain safe buffer gap (>= 20px) behind stopped bus, but got gap: %.2f px", bumperGap)
	}
	if carFrontBumperX > busRearBumperX {
		t.Fatalf("AI car must NEVER ram into stationary bus! Penetration: %.2f px", carFrontBumperX-busRearBumperX)
	}

	// 2. Stationary bus struck by social vehicle must NOT be fined (Zero Fine Protection)
	initialCash := g.Cash
	g.Bus.CrashCooldown = 0
	g.Bus.Pos = Vec2{X: 500, Y: 63.0}
	g.Bus.Speed = 0

	// Force car into collision contact from behind
	approachingCar.Pos = Vec2{X: 435, Y: 63.0} // overlapping
	approachingCar.Speed = 40.0
	g.checkBusCarCollisions()

	if g.Cash != initialCash {
		t.Fatalf("Stationary bus struck by social vehicle must NOT be fined! Expected cash %d, got %d",
			initialCash, g.Cash)
	}
	if approachingCar.Speed > 0 {
		t.Fatalf("Social vehicle striking bus from behind must have speed zeroed, got %f", approachingCar.Speed)
	}

	// 3. Bus driving wrong-way (逆行) and colliding with car MUST be penalized
	// Bus facing East on North side of road (Y = -63) -> WRONG WAY!
	g.Bus.Pos = Vec2{X: 500, Y: -63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 30.0
	g.Bus.CrashCooldown = 0

	if !IsBusFacingWrongWay(g.Bus) {
		t.Fatalf("Bus facing East on North side (Y=-63) MUST be identified as wrong-way (逆行)")
	}

	oncomingCar := &TrafficCar{
		Pos:     Vec2{X: 540, Y: -63.0},
		Heading: math.Pi,
		Speed:   40.0,
		Length:  48.0,
		Width:   22.0,
		IsEW:    true,
	}
	g.TrafficMgr.Cars = []*TrafficCar{oncomingCar}
	cashBeforeWrongWay := g.Cash

	g.checkBusCarCollisions()

	if g.Cash != cashBeforeWrongWay-50 {
		t.Fatalf("Wrong-way collision must deduct $50 fine! Expected %d, got %d",
			cashBeforeWrongWay-50, g.Cash)
	}

	// 4. Bus actively front-ramming stationary car at speed MUST be penalized
	g.Bus.Pos = Vec2{X: 200, Y: 63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 45.0 // High speed front impact
	g.Bus.CrashCooldown = 0

	victimCar := &TrafficCar{
		Pos:     Vec2{X: 260, Y: 63.0},
		Heading: 0,
		Speed:   0.0,
		Length:  48.0,
		Width:   22.0,
		IsEW:    true,
	}
	g.TrafficMgr.Cars = []*TrafficCar{victimCar}
	cashBeforeFrontRam := g.Cash

	g.checkBusCarCollisions()

	if g.Cash != cashBeforeFrontRam-50 {
		t.Fatalf("Active front-ramming by bus must deduct $50 fine! Expected %d, got %d",
			cashBeforeFrontRam-50, g.Cash)
	}
}

func TestSingleLaneRoadRightTurnNoViolation(t *testing.T) {
	tls := NewTrafficLightSystem()
	laneW := 42.0

	// 1. Single-lane North-South street (X = 1300, 东市民大道, 2-lane street) crossing 4-lane avenue (Y = 0, 中央主干大道)
	// Approach from South towards North (Heading = -Pi/2), turning RIGHT onto Eastbound avenue (Heading = 0)
	bus1 := NewBus(1300, 110.0, -math.Pi/2)
	bus1.Speed = 35.0
	bus1.SteeringAngle = 0.05 // steering right

	// Entry before stop line
	violated, notice := tls.CheckPlayerViolation(bus1, laneW)
	if violated {
		t.Fatalf("Entering right turn from single-lane road must not violate: %s", notice)
	}

	// Inside the intersection turning right
	bus1.Pos = Vec2{X: 1315.0, Y: 20.0}
	bus1.Heading = -math.Pi / 4
	bus1.SteeringAngle = 0.15
	violated, notice = tls.CheckPlayerViolation(bus1, laneW)
	if violated {
		t.Fatalf("Maneuvering right turn inside intersection must not violate: %s", notice)
	}

	// Clearing the right exit onto Eastbound avenue (Heading = 0, X = 1380 > stopLineDist)
	bus1.Pos = Vec2{X: 1390.0, Y: 21.0}
	bus1.Heading = 0
	bus1.SteeringAngle = 0
	violated, notice = tls.CheckPlayerViolation(bus1, laneW)
	if violated {
		t.Fatalf("Clearing right turn on single-lane road must NEVER be fined as lane discipline violation! Got: %s", notice)
	}

	// 2. Approach from North (X = 1300, Y = -110 heading South Pi/2), turning RIGHT onto Westbound avenue (Heading = Pi)
	bus2 := NewBus(1300, -110.0, math.Pi/2)
	bus2.Speed = 30.0
	bus2.SteeringAngle = 0.08

	tls.CheckPlayerViolation(bus2, laneW) // entry
	bus2.Pos = Vec2{X: 1285.0, Y: -20.0}
	bus2.Heading = 3 * math.Pi / 4
	tls.CheckPlayerViolation(bus2, laneW) // inside

	// Clearing west exit
	bus2.Pos = Vec2{X: 1210.0, Y: -21.0}
	bus2.Heading = math.Pi
	violated2, notice2 := tls.CheckPlayerViolation(bus2, laneW)
	if violated2 {
		t.Fatalf("Clearing right turn from North approach on single-lane road must NOT violate! Got: %s", notice2)
	}

	// 3. East-West single-lane street (Y = -600, 北二路, 2-lane street) crossing 4-lane avenue (X = 0, 城市中轴大道)
	// Approach from West (X = -110, Y = -600, heading East 0), turning RIGHT onto Southbound avenue (Heading = Pi/2)
	bus3 := NewBus(-110.0, -600.0, 0)
	bus3.Speed = 32.0
	bus3.SteeringAngle = 0.06

	tls.CheckPlayerViolation(bus3, laneW) // entry
	bus3.Pos = Vec2{X: -20.0, Y: -585.0}
	bus3.Heading = math.Pi / 4
	tls.CheckPlayerViolation(bus3, laneW) // inside

	// Clearing south exit
	bus3.Pos = Vec2{X: -21.0, Y: -510.0}
	bus3.Heading = math.Pi / 2
	violated3, notice3 := tls.CheckPlayerViolation(bus3, laneW)
	if violated3 {
		t.Fatalf("Clearing right turn from EW 2-lane street must NOT violate! Got: %s", notice3)
	}

	// 4. Minor-to-minor 2-lane intersection (X = 1300, Y = 600)
	// Both streets are single-lane in each direction!
	bus4 := NewBus(1300, 710.0, -math.Pi/2)
	bus4.Speed = 28.0
	tls.CheckPlayerViolation(bus4, laneW) // entry
	bus4.Pos = Vec2{X: 1390.0, Y: 620.0}
	bus4.Heading = 0
	violated4, notice4 := tls.CheckPlayerViolation(bus4, laneW)
	if violated4 {
		t.Fatalf("Turning right at minor 2-lane intersection must NOT violate! Got: %s", notice4)
	}
}

func TestSingleLaneRoadPermissiveTurnsNoLaneViolation(t *testing.T) {
	tls := NewTrafficLightSystem()
	laneW := 42.0

	// Find intersection at (1300 * MapGridScale, 0)
	itX := 1300.0 * MapGridScale
	var inter *TrafficIntersection
	for _, it := range tls.Intersections {
		if math.Abs(it.Center.X-itX) < 5.0 && math.Abs(it.Center.Y) < 5.0 {
			inter = it
			break
		}
	}
	if inter == nil {
		t.Fatalf("Intersection at (%.0f, 0) not found", itX)
	}

	// Set NS phase to Green
	inter.Phase = PhaseNSGreen
	inter.Timer = 1.0

	// 1. Permissive Left Turn on Green from single-lane street (X = itX, Northbound towards -Pi/2)
	// Turns LEFT onto Westbound Y = 0 (Heading = Pi)
	busLeft := NewBus(itX, 110.0, -math.Pi/2)
	busLeft.Speed = 30.0
	busLeft.SteeringAngle = -0.05 // steering left

	// Entry
	tls.CheckPlayerViolation(busLeft, laneW)
	// Inside turning left
	busLeft.Pos = Vec2{X: itX - 10.0, Y: 10.0}
	busLeft.Heading = -3 * math.Pi / 4
	tls.CheckPlayerViolation(busLeft, laneW)
	// Exit onto Westbound avenue (Heading = Pi, X = itX - 90 < itX - stopLineDist)
	busLeft.Pos = Vec2{X: itX - 90.0, Y: -21.0}
	busLeft.Heading = math.Pi
	violated, notice := tls.CheckPlayerViolation(busLeft, laneW)
	if violated {
		t.Fatalf("Permissive left turn on green from single-lane road must NEVER be fined! Got: %s", notice)
	}

	// 2. Straight through on Green from single-lane street (X = 1300, Northbound)
	inter.Phase = PhaseNSGreen
	inter.Timer = 2.0
	busThru := NewBus(1300, 110.0, -math.Pi/2)
	busThru.Speed = 40.0
	tls.CheckPlayerViolation(busThru, laneW) // entry
	busThru.Pos = Vec2{X: 1300, Y: -110.0}   // opposite exit
	violatedThru, noticeThru := tls.CheckPlayerViolation(busThru, laneW)
	if violatedThru {
		t.Fatalf("Driving straight on green from single-lane road must NEVER be fined as lane discipline! Got: %s", noticeThru)
	}
}

func TestMultiLaneAvenueRightTurnFromOuterLaneNoViolation(t *testing.T) {
	tls := NewTrafficLightSystem()
	laneW := 42.0

	// 4-lane avenue intersection (0, 0)
	// Bus in Lane 1 (outer straight/right lane, Y = 63.0) heading East (0)
	bus := NewBus(-110.0, 63.0, 0)
	bus.Speed = 35.0
	bus.SteeringAngle = 0.08 // steering right

	// Entry
	tls.CheckPlayerViolation(bus, laneW)
	// Inside turning right
	bus.Pos = Vec2{X: 15.0, Y: 60.0}
	bus.Heading = math.Pi / 4
	tls.CheckPlayerViolation(bus, laneW)
	// Exit onto Southbound avenue (Heading = Pi/2, Y = 110)
	bus.Pos = Vec2{X: -21.0, Y: 110.0}
	bus.Heading = math.Pi / 2
	violated, notice := tls.CheckPlayerViolation(bus, laneW)
	if violated {
		t.Fatalf("Turning right from Lane 1 on 4-lane avenue must NEVER be fined as lane discipline! Got: %s", notice)
	}
}

func TestBlockWidthExtendedAndBusStopsDistancedFromIntersections(t *testing.T) {
	world := NewWorld(42.0)
	tls := NewTrafficLightSystem()

	// 1. Verify block width is extended to 1300px across all East-West intervals
	expectedXs := []float64{-2600, -1300, 0, 1300, 2600}
	for i := 0; i < len(expectedXs)-1; i++ {
		width := expectedXs[i+1] - expectedXs[i]
		if math.Abs(width-1300.0) > 1.0 {
			t.Fatalf("Expected block width of 1300px, got %.1f between X=%.1f and X=%.1f", width, expectedXs[i], expectedXs[i+1])
		}
	}

	// Verify all 25 intersections reflect the extended 1300px block width
	if len(tls.Intersections) != 25 {
		t.Fatalf("Expected 25 intersections, got %d", len(tls.Intersections))
	}
	foundInter := false
	for _, it := range tls.Intersections {
		if math.Abs(it.Center.X-2600*MapGridScale) < 5.0 && math.Abs(it.Center.Y) < 5.0 {
			foundInter = true
			break
		}
	}
	if !foundInter {
		t.Fatalf("Expected outer boulevard intersection at X=%.0f, Y=0", 2600*MapGridScale)
	}

	// 2. Verify all bus stops are located away from intersections
	// Bus stops on 4-lane avenues must have >= 500px runway to the next intersection stop line
	// so the bus has plenty of room to change lanes.
	stopLineBuffer := 42.0*1.6 + 18.0 // approx 85.2px
	for _, stop := range world.Stops {
		isEW := math.Abs(math.Cos(stop.Heading)) > math.Abs(math.Sin(stop.Heading))
		minDistToCenter := math.MaxFloat64
		distToUpcomingStopLine := math.MaxFloat64

		for _, it := range tls.Intersections {
			dCenter := stop.RoadCenter.Distance(it.Center)
			if dCenter < minDistToCenter {
				minDistToCenter = dCenter
			}

			// Along direction of travel:
			if isEW {
				cosH := math.Cos(stop.Heading)
				// If heading East (+X), next intersection has it.Center.X > stop.RoadCenter.X
				if cosH > 0 && it.Center.X > stop.RoadCenter.X && math.Abs(it.Center.Y-stop.RoadCenter.Y) < 10.0 {
					dLine := (it.Center.X - stop.RoadCenter.X) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				} else if cosH < 0 && it.Center.X < stop.RoadCenter.X && math.Abs(it.Center.Y-stop.RoadCenter.Y) < 10.0 {
					dLine := (stop.RoadCenter.X - it.Center.X) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				}
			} else {
				sinH := math.Sin(stop.Heading)
				if sinH > 0 && it.Center.Y > stop.RoadCenter.Y && math.Abs(it.Center.X-stop.RoadCenter.X) < 10.0 {
					dLine := (it.Center.Y - stop.RoadCenter.Y) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				} else if sinH < 0 && it.Center.Y < stop.RoadCenter.Y && math.Abs(it.Center.X-stop.RoadCenter.X) < 10.0 {
					dLine := (stop.RoadCenter.Y - it.Center.Y) - stopLineBuffer
					if dLine < distToUpcomingStopLine {
						distToUpcomingStopLine = dLine
					}
				}
			}
		}

		// Distance to any intersection center should be at least 280px
		if minDistToCenter < 280.0 {
			t.Fatalf("Stop %d (%s) at %v is too close to an intersection center (minDist=%.1f < 280px)",
				stop.ID, stop.Name, stop.Pos, minDistToCenter)
		}

		// On 4-lane avenues, distance to upcoming stop line must be >= 500px for comfortable lane change
		if isEW && distToUpcomingStopLine < 500.0 {
			t.Fatalf("Stop %d (%s) at %v has insufficient lane-change runway to next stop line (dist=%.1f < 500px)",
				stop.ID, stop.Name, stop.Pos, distToUpcomingStopLine)
		}
	}
}

func TestPlanModePerformanceOptimizations(t *testing.T) {
	world := NewWorld(42.0)
	cam := NewCamera(1280, 720)
	cam.Pos = Vec2{X: 0, Y: 0}
	cam.Zoom = 0.22

	// 1. Verify Camera Frustum Culling
	// Center is visible
	if !cam.IsPointVisible(Vec2{X: 0, Y: 0}, 50) {
		t.Fatal("Origin point must be visible on camera centered at origin")
	}
	// Far off-screen point (X=10000) must NOT be visible
	if cam.IsPointVisible(Vec2{X: 10000, Y: 10000}, 50) {
		t.Fatal("Far away point (10000, 10000) must be culled by camera frustum")
	}
	// Far off-screen bounding box must NOT be visible
	if cam.IsBoxVisible(8000, 8000, 9000, 9000) {
		t.Fatal("Far away box (8000..9000) must be culled by camera frustum")
	}

	// 2. Verify Dijkstra Route Path Cache Memoization
	s1 := world.Stops[0]
	s2 := world.Stops[1]
	p1 := GetRoadPathBetweenStops(s1, s2, 42.0)
	if len(p1) < 2 {
		t.Fatalf("Path between stop 0 and 1 must contain at least 2 points, got %d", len(p1))
	}

	// Second call must return the cached slice (same length and points) instantly
	p2 := GetRoadPathBetweenStops(s1, s2, 42.0)
	if len(p2) != len(p1) {
		t.Fatalf("Cached path length mismatch: expected %d, got %d", len(p1), len(p2))
	}
	for i := range p1 {
		if p1[i] != p2[i] {
			t.Fatalf("Point %d mismatch in cached path: %v vs %v", i, p1[i], p2[i])
		}
	}

	// 3. Verify Route Description Cache
	route := NewRoute()
	route.AddStop(0)
	route.AddStop(1)
	desc1 := route.GetRouteDescription(world)
	if desc1 == "" {
		t.Fatal("Route description must not be empty")
	}
	// Calling again should return cached description string
	desc2 := route.GetRouteDescription(world)
	if desc1 != desc2 {
		t.Fatalf("Cached description mismatch: %q vs %q", desc1, desc2)
	}

	// Mutating route should invalidate cache and update correctly
	route.AddStop(2)
	desc3 := route.GetRouteDescription(world)
	if desc3 == desc1 {
		t.Fatal("Description cache should invalidate when a new stop is added")
	}

	// Clear should reset cache
	route.Clear()
	descEmpty := route.GetRouteDescription(world)
	if descEmpty != "未规划站点 (点击站点图标添加)" {
		t.Fatalf("Unexpected empty route description: %q", descEmpty)
	}
}

func TestMapThemesRoundaboutsAndBridges(t *testing.T) {
	// 1. Test MapTheme enum, cycling across all 5 distinct maps and names
	m := MapThemeMetropolis
	if m.DisplayName() != "现代都会 (Metropolis)" || m.ShortName() != "现代都会" {
		t.Fatalf("Unexpected display/short name for Metropolis: %s, %s", m.DisplayName(), m.ShortName())
	}
	c := m.Next()
	if c != MapThemeCoastalBay || c.ShortName() != "海滨港湾" {
		t.Fatalf("Expected next theme to be Coastal Bay, got %v (%s)", c, c.ShortName())
	}
	rv := c.Next()
	if rv != MapThemeRiverValley || rv.ShortName() != "溪谷江城" {
		t.Fatalf("Expected next theme to be River Valley, got %v (%s)", rv, rv.ShortName())
	}
	l := rv.Next()
	if l != MapThemeEmeraldLake || l.ShortName() != "湖畔水乡" {
		t.Fatalf("Expected next theme to be Emerald Lake, got %v (%s)", l, l.ShortName())
	}
	tc := l.Next()
	if tc != MapThemeTwinCities || tc.ShortName() != "高新双城" {
		t.Fatalf("Expected next theme to be Twin Cities, got %v (%s)", tc, tc.ShortName())
	}
	cust := tc.Next()
	if cust != MapThemeCustom || cust.ShortName() != "自定义" {
		t.Fatalf("Expected next theme to be Custom Map, got %v (%s)", cust, cust.ShortName())
	}
	if cust.Next() != MapThemeMetropolis {
		t.Fatalf("Expected cycle back to Metropolis, got %v", cust.Next())
	}

	laneW := 42.0

	// 2. Test Metropolis Map (5x5 grid with canal twin bridges)
	wm := NewWorld(laneW, MapThemeMetropolis)
	if wm.Theme != MapThemeMetropolis {
		t.Fatalf("World theme mismatch: expected Metropolis, got %v", wm.Theme)
	}
	if len(wm.Roundabouts) == 0 {
		t.Fatal("Metropolis map should have at least 1 roundabout")
	}
	if len(wm.Bridges) < 2 {
		t.Fatalf("Metropolis map should have twin canal bridges, got %d", len(wm.Bridges))
	}
	if len(wm.Waters) == 0 {
		t.Fatal("Metropolis map should have water bodies (canal)")
	}
	if len(wm.Stops) != 12 {
		t.Fatalf("Metropolis map should have 12 stops, got %d", len(wm.Stops))
	}

	// 3. Test Coastal Bay Map (coastal boulevard & 1200m sea suspension bridge)
	wc := NewWorld(laneW, MapThemeCoastalBay)
	if wc.Theme != MapThemeCoastalBay {
		t.Fatalf("World theme mismatch: expected Coastal Bay, got %v", wc.Theme)
	}
	if len(wc.Roundabouts) == 0 {
		t.Fatal("Coastal Bay map should have at least 1 roundabout")
	}
	if len(wc.Bridges) == 0 {
		t.Fatal("Coastal Bay map should have ocean bridge")
	}
	hasBeach := false
	for _, wb := range wc.Waters {
		if wb.HasBeach {
			hasBeach = true
			break
		}
	}
	if !hasBeach {
		t.Fatal("Coastal Bay map should have water body with sandy beach")
	}

	// 4. Test River Valley Map (Yangtze River gorge & twin cable-stayed bridges)
	wrv := NewWorld(laneW, MapThemeRiverValley)
	if wrv.Theme != MapThemeRiverValley {
		t.Fatalf("World theme mismatch: expected River Valley, got %v", wrv.Theme)
	}
	if len(wrv.Bridges) < 2 {
		t.Fatalf("River Valley map should have twin grand bridges, got %d", len(wrv.Bridges))
	}
	if len(wrv.Waters) == 0 {
		t.Fatal("River Valley map should feature Yangtze water body")
	}
	if len(wrv.Stops) < 6 {
		t.Fatalf("River Valley map should have stops, got %d", len(wrv.Stops))
	}

	// 5. Test Emerald Lake Map (giant emerald lake, lake island & 4 stone arch bridges)
	wl := NewWorld(laneW, MapThemeEmeraldLake)
	if wl.Theme != MapThemeEmeraldLake {
		t.Fatalf("World theme mismatch: expected Emerald Lake, got %v", wl.Theme)
	}
	if len(wl.Roundabouts) == 0 {
		t.Fatal("Emerald Lake map should have lake island roundabout")
	}
	if len(wl.Bridges) < 2 {
		t.Fatalf("Emerald Lake map should have stone arch bridges, got %d", len(wl.Bridges))
	}
	hasStoneArch := false
	for _, br := range wl.Bridges {
		if br.Style() == BridgeStyleStoneArch {
			hasStoneArch = true
			break
		}
	}
	if !hasStoneArch {
		t.Fatal("Emerald Lake map should feature stone arch bridges")
	}

	// 6. Test Twin Cities Map (Central expressway viaduct overpass & grade-separated interchange)
	wtc := NewWorld(laneW, MapThemeTwinCities)
	if wtc.Theme != MapThemeTwinCities {
		t.Fatalf("World theme mismatch: expected Twin Cities, got %v", wtc.Theme)
	}
	hasOverpass := false
	for _, br := range wtc.Bridges {
		if br.IsOverpass && br.Style() == BridgeStyleOverpass {
			hasOverpass = true
			break
		}
	}
	if !hasOverpass {
		t.Fatal("Twin Cities map must feature a grade-separated highway overpass viaduct")
	}
	if len(wtc.Stops) < 6 {
		t.Fatalf("Twin Cities map should have stops, got %d", len(wtc.Stops))
	}

	// 7. Test Custom Map
	wcm := NewWorld(laneW, MapThemeCustom)
	if wcm.Theme != MapThemeCustom {
		t.Fatalf("World theme mismatch: expected Custom, got %v", wcm.Theme)
	}
	if len(wcm.Stops) < 6 {
		t.Fatalf("Custom map should have at least 6 stops, got %d", len(wcm.Stops))
	}
	if len(wcm.Roads) < 8 {
		t.Fatalf("Custom map should have roads, got %d", len(wcm.Roads))
	}

	// 8. Test Roundabout roadway boundary detection vs center island barrier
	rb := wm.Roundabouts[0]
	roadwayRadius := (rb.InnerRadius + rb.Radius) / 2.0
	posOnRing := Vec2{X: rb.Center.X + roadwayRadius, Y: rb.Center.Y}
	resRing := wm.CheckBoundaries(posOnRing, 1.5, laneW)
	if resRing.Zone == ZoneHardBarrier {
		t.Fatalf("Bus on roundabout driving ring should not be in barrier! Res: %+v", resRing)
	}

	posInIsland := rb.Center
	resIsland := wm.CheckBoundaries(posInIsland, 1.5, laneW)
	if resIsland.Zone != ZoneHardBarrier {
		t.Fatalf("Bus inside central island should hit barrier/boundary! Res: %+v", resIsland)
	}

	// 8. Test Bridge bounds (bus on bridge roadway is safe)
	br := wm.Bridges[0]
	posOnBridge := br.Pos
	resBridge := wm.CheckBoundaries(posOnBridge, 1.5, laneW)
	if resBridge.Zone == ZoneHardBarrier {
		t.Fatalf("Bus on bridge roadway should be safe! Res: %+v", resBridge)
	}

	// 9. Test Overpass Viaduct bounds on Map 5
	overpassBr := wtc.Bridges[0]
	resOverpass := wtc.CheckBoundaries(overpassBr.Pos, 1.5, laneW)
	if resOverpass.Zone == ZoneHardBarrier {
		t.Fatalf("Bus on highway overpass viaduct should be safe! Res: %+v", resOverpass)
	}

	// 10. Test Theme switching & Main Menu State
	g := NewGame(1280, 720)
	if g.State != StateMainMenu {
		t.Fatalf("Expected new game to start in StateMainMenu, got %v", g.State)
	}

	// Enter City
	g.EnterCity(MapThemeCoastalBay)
	if g.State != StatePlaying {
		t.Fatalf("Expected state to be StatePlaying after EnterCity, got %v", g.State)
	}
	if g.World.Theme != MapThemeCoastalBay || g.Settings.MapTheme != MapThemeCoastalBay {
		t.Fatalf("Game map theme switch to Coastal Bay failed")
	}

	// Enter River Valley
	g.EnterCity(MapThemeRiverValley)
	if g.World.Theme != MapThemeRiverValley || g.Settings.MapTheme != MapThemeRiverValley {
		t.Fatalf("Game map theme switch to River Valley failed")
	}

	// Enter Twin Cities
	g.EnterCity(MapThemeTwinCities)
	if g.World.Theme != MapThemeTwinCities || g.Settings.MapTheme != MapThemeTwinCities {
		t.Fatalf("Game map theme switch to Twin Cities failed")
	}

	// 11. Test HUD Menu button click returning to Main Menu
	hud := g.HUD
	hud.MenuBtnBounds = Rect{X: 10, Y: 10, W: 120, H: 30}
	hud.MapThemeBtnBounds = hud.MenuBtnBounds
	clicked := hud.HandleMenuBtnClick(Vec2{X: 20, Y: 20})
	if !clicked || !hud.ReturnToMenuRequested {
		t.Fatal("Expected HUD Menu button click to request return to menu")
	}

	// Update game should transition state back to StateMainMenu
	err := g.Update()
	if err != nil {
		t.Fatalf("Unexpected error from g.Update(): %v", err)
	}
	if g.State != StateMainMenu {
		t.Fatalf("Expected game state to return to StateMainMenu, got %v", g.State)
	}
}

func TestFiveDistinctCityMapsAndMainMenu(t *testing.T) {
	laneW := 52.0

	// 1. Verify 5 completely distinct maps with unique road topologies, stops, and geography
	type mapSpec struct {
		theme            MapTheme
		expectedMinRoads int
		expectedMinStops int
		expectedBridges  int
		expectedWaters   int
	}

	specs := []mapSpec{
		{theme: MapThemeMetropolis, expectedMinRoads: 10, expectedMinStops: 12, expectedBridges: 3, expectedWaters: 1},
		{theme: MapThemeCoastalBay, expectedMinRoads: 9, expectedMinStops: 8, expectedBridges: 1, expectedWaters: 1},
		{theme: MapThemeRiverValley, expectedMinRoads: 12, expectedMinStops: 8, expectedBridges: 2, expectedWaters: 1},
		{theme: MapThemeEmeraldLake, expectedMinRoads: 10, expectedMinStops: 8, expectedBridges: 4, expectedWaters: 1},
		{theme: MapThemeTwinCities, expectedMinRoads: 11, expectedMinStops: 8, expectedBridges: 1, expectedWaters: 1},
	}

	for _, sp := range specs {
		w := NewWorld(laneW, sp.theme)
		if w.Theme != sp.theme {
			t.Fatalf("Theme mismatch for %v", sp.theme)
		}
		if len(w.Roads) < sp.expectedMinRoads {
			t.Fatalf("Map %s has insufficient roads: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Roads), sp.expectedMinRoads)
		}
		if len(w.Stops) < sp.expectedMinStops {
			t.Fatalf("Map %s has insufficient stops: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Stops), sp.expectedMinStops)
		}
		if len(w.Bridges) != sp.expectedBridges {
			t.Fatalf("Map %s bridge count mismatch: %d (expected %d)", sp.theme.DisplayName(), len(w.Bridges), sp.expectedBridges)
		}
		if len(w.Waters) < sp.expectedWaters {
			t.Fatalf("Map %s water bodies mismatch: %d (expected >= %d)", sp.theme.DisplayName(), len(w.Waters), sp.expectedWaters)
		}
		if len(w.Intersections) == 0 {
			t.Fatalf("Map %s should dynamically compute intersections", sp.theme.DisplayName())
		}

		// Verify every road segment in the map is strictly orthogonal and clean (no unnatural jagged piecewise lines)
		for _, r := range w.Roads {
			isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
			isNS := math.Abs(r.P1.X-r.P2.X) < 1.0
			if !isEW && !isNS {
				t.Fatalf("Map %s has non-orthogonal unnatural road: %s (%+v -> %+v)", sp.theme.DisplayName(), r.Name, r.P1, r.P2)
			}
		}
	}

	// 2. Verify Map 5 Grade-Separated Highway Overpass Viaduct
	wTwin := NewWorld(laneW, MapThemeTwinCities)
	foundOverpass := false
	for _, br := range wTwin.Bridges {
		if br.IsOverpass && br.Style() == BridgeStyleOverpass {
			foundOverpass = true
			if br.Length < 300 || br.Width < 336 {
				t.Fatalf("Overpass dimensions too small to cover 6-lane highway: length %f, width %f", br.Length, br.Width)
			}
		}
	}
	if !foundOverpass {
		t.Fatal("Map 5 must include a grade-separated highway overpass viaduct")
	}
	// Verify that the overpass at (0, 0) is recognized and does not generate an at-grade intersection
	if !wTwin.IsOverpassAt(Vec2{X: 0, Y: 0}) {
		t.Fatal("Expected IsOverpassAt(0, 0) to be true for Map 5")
	}
	for _, ipt := range wTwin.Intersections {
		if ipt.Distance(Vec2{X: 0, Y: 0}) < 10 {
			t.Fatalf("Map 5 must not generate an at-grade intersection on top of the overpass at (0, 0)")
		}
	}
	for _, riv := range wTwin.Rivers {
		if math.Abs(riv.Y) < 10 {
			t.Fatalf("Map 5 must not have a horizontal river conflict at Y=0")
		}
	}

	// 3. Main Menu System State Flow
	g := NewGame(1280, 720)
	if g.State != StateMainMenu {
		t.Fatalf("Game must boot directly into Main Menu State")
	}

	// Simulate selecting Map 4 (Emerald Lake) in Main Menu
	g.SelectedMapTheme = MapThemeEmeraldLake
	g.EnterCity(g.SelectedMapTheme)
	if g.State != StatePlaying {
		t.Fatalf("Entering city must switch game state to StatePlaying")
	}
	if g.World.Theme != MapThemeEmeraldLake {
		t.Fatalf("World theme must be Emerald Lake")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus must spawn inside Station Bay 0")
	}
	if g.Bus.Speed != 0 || !g.Bus.DoorsOpen {
		t.Fatalf("Bus at starting bay must be stopped with doors open")
	}

	// Simulate returning to Main Menu via in-game Settings
	g.Settings.IsOpen = true
	g.Settings.ReturnToMenuRequested = true
	err := g.Update()
	if err != nil {
		t.Fatalf("Update failed: %v", err)
	}
	if g.State != StateMainMenu {
		t.Fatalf("Settings ReturnToMenuRequested must return to StateMainMenu")
	}

	// Simulate selecting Map 5 (Twin Cities) and entering
	g.SelectedMapTheme = MapThemeTwinCities
	g.EnterCity(g.SelectedMapTheme)
	if g.State != StatePlaying || g.World.Theme != MapThemeTwinCities {
		t.Fatalf("Entering Twin Cities map failed")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus must reposition to Stop 0 of the new city")
	}

	// Simulate returning to Main Menu via HUD top-bar button
	g.HUD.ReturnToMenuRequested = true
	err = g.Update()
	if err != nil {
		t.Fatalf("Update failed: %v", err)
	}
	if g.State != StateMainMenu {
		t.Fatalf("HUD ReturnToMenuRequested must return to StateMainMenu")
	}
}

func TestCustomMapLoadingValidationAndReloading(t *testing.T) {
	// 1. Verify default preset map data integrity
	defaultMap := GetDefaultCustomMapData()
	if defaultMap == nil {
		t.Fatal("GetDefaultCustomMapData returned nil")
	}
	if err := ValidateCustomMapData(defaultMap); err != nil {
		t.Fatalf("Default custom map failed validation: %v", err)
	}
	if len(defaultMap.Stops) != 12 {
		t.Fatalf("Default custom map should have 12 stops, got %d", len(defaultMap.Stops))
	}
	if len(defaultMap.Roads) < 10 {
		t.Fatalf("Default custom map should have >= 10 roads, got %d", len(defaultMap.Roads))
	}
	if len(defaultMap.Roundabouts) == 0 {
		t.Fatal("Default custom map should have at least 1 roundabout")
	}
	if len(defaultMap.Bridges) == 0 {
		t.Fatal("Default custom map should have at least 1 bridge")
	}

	// 2. Verify all presets pass validation
	presets := GetCustomMapPresets()
	if len(presets) < 3 {
		t.Fatalf("Expected at least 3 custom map presets, got %d", len(presets))
	}
	for i, preset := range presets {
		if err := ValidateCustomMapData(preset); err != nil {
			t.Fatalf("Preset #%d (%s) failed validation: %v", i, preset.Name, err)
		}
		if len(preset.Stops) < 6 {
			t.Fatalf("Preset #%d (%s) should have >= 6 stops, got %d", i, preset.Name, len(preset.Stops))
		}
	}

	// 3. Verify validation error rejection
	// 3a. Empty name
	invalidMap := *defaultMap
	invalidMap.Name = ""
	if err := ValidateCustomMapData(&invalidMap); err == nil {
		t.Fatal("Expected error for empty map name")
	}

	// 3b. Insufficient roads
	invalidMap = *defaultMap
	invalidMap.Roads = nil
	if err := ValidateCustomMapData(&invalidMap); err == nil {
		t.Fatal("Expected error for missing roads")
	}

	// 3c. Non-orthogonal road
	invalidMap = *defaultMap
	invalidMap.Roads = []CustomRoadData{
		{P1: [2]float64{0, 0}, P2: [2]float64{500, 0}, Lanes: 2, Name: "Road 1"},
		{P1: [2]float64{0, 0}, P2: [2]float64{500, 500}, Lanes: 2, Name: "Diagonal Road"},
	}
	if err := ValidateCustomMapData(&invalidMap); err == nil || !strings.Contains(err.Error(), "orthogonal") {
		t.Fatalf("Expected non-orthogonal error, got %v", err)
	}

	// 3d. Duplicate stop ID
	invalidMap = *defaultMap
	invalidMap.Stops = []CustomStopData{
		{ID: 1, Name: "Stop A", Pos: [2]float64{0, 0}, Lanes: 2},
		{ID: 1, Name: "Stop B", Pos: [2]float64{100, 0}, Lanes: 2},
	}
	if err := ValidateCustomMapData(&invalidMap); err == nil || !strings.Contains(err.Error(), "duplicate") {
		t.Fatalf("Expected duplicate stop ID error, got %v", err)
	}

	// 4. Test file serialization roundtrip
	tmpDir, err := os.MkdirTemp("", "custom_map_test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	tmpFile := filepath.Join(tmpDir, "custom_map.json")
	if err := SaveCustomMapToFile(tmpFile, defaultMap); err != nil {
		t.Fatalf("Failed to save custom map to file: %v", err)
	}

	loadedMap, err := LoadCustomMapFromFile(tmpFile)
	if err != nil {
		t.Fatalf("Failed to load custom map from file: %v", err)
	}
	if loadedMap.Name != defaultMap.Name {
		t.Fatalf("Loaded map name mismatch: expected %s, got %s", defaultMap.Name, loadedMap.Name)
	}
	if len(loadedMap.Roads) != len(defaultMap.Roads) {
		t.Fatalf("Loaded road count mismatch: expected %d, got %d", len(defaultMap.Roads), len(loadedMap.Roads))
	}
	if len(loadedMap.Stops) != len(defaultMap.Stops) {
		t.Fatalf("Loaded stop count mismatch: expected %d, got %d", len(defaultMap.Stops), len(loadedMap.Stops))
	}

	// 5. Test World instantiation with custom map
	w := NewWorld(48.0, MapThemeCustom)
	if w.Theme != MapThemeCustom {
		t.Fatalf("Expected world theme Custom, got %v", w.Theme)
	}
	if len(w.Stops) == 0 {
		t.Fatal("Custom world should have populated stops")
	}
	if len(w.Intersections) == 0 {
		t.Fatal("Custom world should have computed road intersections")
	}

	// 6. Test In-Game Custom Map Lifecycle & Reload
	g := NewGame(1280, 720)
	g.SelectedMapTheme = MapThemeCustom
	g.EnterCity(g.SelectedMapTheme)
	if g.World.Theme != MapThemeCustom {
		t.Fatalf("Expected Game to be on Custom Map theme")
	}
	if g.Bus.Pos.Distance(g.World.Stops[0].Pos) > 1.0 {
		t.Fatalf("Bus should spawn at stop 0 in custom map")
	}

	// Hot reload custom map
	err = g.ReloadCustomMap()
	if err != nil {
		t.Fatalf("ReloadCustomMap failed: %v", err)
	}
	if len(g.HUD.Toasts) == 0 {
		t.Fatal("ReloadCustomMap should issue HUD toast feedback")
	}
}

func TestAllMapsBusStopsDistancedFromIntersections(t *testing.T) {
	laneW := 52.0
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, theme := range themes {
		w := NewWorld(laneW, theme)
		if len(w.Stops) == 0 {
			t.Fatalf("Map theme %s has no stops", theme.DisplayName())
		}
		if len(w.Intersections) == 0 {
			t.Fatalf("Map theme %s has no intersections", theme.DisplayName())
		}

		for _, stop := range w.Stops {
			minDist := math.MaxFloat64
			var nearestInter Vec2
			for _, inter := range w.Intersections {
				d := stop.RoadCenter.Distance(inter)
				if d < minDist {
					minDist = d
					nearestInter = inter
				}
			}

			// Bus stop road center must be at least 240px away from any intersection
			if minDist < 240.0 {
				t.Fatalf("Map %s Stop '%s' (id=%d) at %v is too close to intersection at %v (dist=%.1f < 240px)",
					theme.DisplayName(), stop.Name, stop.ID, stop.RoadCenter, nearestInter, minDist)
			}
		}
	}
}

func TestCustomMapStopTooCloseToIntersectionValidation(t *testing.T) {
	customMap := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(customMap); err != nil {
		t.Fatalf("Default custom map failed validation: %v", err)
	}

	// Deliberately place a stop right on an intersection (0, 0)
	invalidMap := *customMap
	invalidStops := make([]CustomStopData, len(customMap.Stops))
	copy(invalidStops, customMap.Stops)
	invalidStops[0].Pos = [2]float64{0, 0}
	invalidMap.Stops = invalidStops

	err := ValidateCustomMapData(&invalidMap)
	if err == nil {
		t.Fatalf("Expected validation error for stop at intersection (0, 0), got nil")
	}
	if !strings.Contains(err.Error(), "距离路口过近") {
		t.Fatalf("Expected error message about being too close to intersection, got: %v", err)
	}
}

func TestRoundaboutsHaveNoTrafficLights(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeEmeraldLake,
		MapThemeCustom,
	}

	for _, theme := range themes {
		w := NewWorld(52.0, theme)
		if len(w.Roundabouts) == 0 {
			t.Fatalf("Theme %s was expected to have at least one roundabout", theme.DisplayName())
		}

		tls := NewTrafficLightSystem()
		tls.BuildForWorld(w)

		for _, rb := range w.Roundabouts {
			// 1. Verify w.IsAtRoundabout returns true for roundabout center and perimeter
			if !w.IsAtRoundabout(rb.Center) {
				t.Fatalf("Theme %s: w.IsAtRoundabout(%v) must be true", theme.DisplayName(), rb.Center)
			}
			perimeterPt := rb.Center.Add(Vec2{X: rb.Radius - 10.0, Y: 0})
			if !w.IsAtRoundabout(perimeterPt) {
				t.Fatalf("Theme %s: w.IsAtRoundabout perimeter point (%v) must be true", theme.DisplayName(), perimeterPt)
			}

			// 2. Verify tls.Intersections does NOT contain any intersection inside the roundabout
			for _, inter := range tls.Intersections {
				if inter.Center.Distance(rb.Center) < rb.Radius {
					t.Fatalf("Theme %s: Roundabout at %v must NOT have a traffic light intersection in tls (found inter at %v)",
						theme.DisplayName(), rb.Center, inter.Center)
				}
			}

			// 3. Verify vehicle driving in or through roundabout is NEVER fined for running a red light
			testPositions := []Vec2{
				rb.Center,
				rb.Center.Add(Vec2{X: (rb.InnerRadius + rb.Radius) * 0.5, Y: 0}),
				rb.Center.Add(Vec2{X: 0, Y: (rb.InnerRadius + rb.Radius) * 0.5}),
				rb.Center.Add(Vec2{X: -rb.Radius + 10.0, Y: 0}),
				rb.Center.Add(Vec2{X: rb.Radius - 10.0, Y: 0}),
			}

			for _, pos := range testPositions {
				bus := NewBus(pos.X, pos.Y, 0)
				bus.Speed = 65.0 // driving at full speed
				violated, notice := tls.CheckPlayerViolation(bus, 52.0)
				if violated {
					t.Fatalf("Theme %s: Bus driving in roundabout at %v must NEVER be fined for red light: %s",
						theme.DisplayName(), pos, notice)
				}

				// 4. Verify AI vehicles are never told to stop for red lights at roundabout
				shouldStop, _ := tls.CheckStopForVehicle(pos, 0, 100.0, 52.0, false)
				if shouldStop {
					t.Fatalf("Theme %s: AI vehicle at %v approaching roundabout must NOT be instructed to stop for red light",
						theme.DisplayName(), pos)
				}
			}
		}
	}
}

// TestNoUnreasonablyCloseParallelRoadsOnAnyMap verifies that across all maps,
// no two parallel two-way roads are unreasonably close to each other (must maintain >= 400px spacing).
func TestNoUnreasonablyCloseParallelRoadsOnAnyMap(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	const minParallelDistance = 400.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(52.0, theme)
			for i := 0; i < len(w.Roads); i++ {
				for j := i + 1; j < len(w.Roads); j++ {
					r1 := w.Roads[i]
					r2 := w.Roads[j]

					isR1EW := math.Abs(r1.P1.Y-r1.P2.Y) < 1.0
					isR2EW := math.Abs(r2.P1.Y-r2.P2.Y) < 1.0
					isR1NS := math.Abs(r1.P1.X-r1.P2.X) < 1.0
					isR2NS := math.Abs(r2.P1.X-r2.P2.X) < 1.0

					// Check parallel EW roads
					if isR1EW && isR2EW {
						y1 := (r1.P1.Y + r1.P2.Y) / 2.0
						y2 := (r2.P1.Y + r2.P2.Y) / 2.0
						dist := math.Abs(y1 - y2)

						minX1 := math.Min(r1.P1.X, r1.P2.X)
						maxX1 := math.Max(r1.P1.X, r1.P2.X)
						minX2 := math.Min(r2.P1.X, r2.P2.X)
						maxX2 := math.Max(r2.P1.X, r2.P2.X)
						overlap := math.Min(maxX1, maxX2) - math.Max(minX1, minX2)

						if overlap > 30.0 && dist > 1.0 && dist < minParallelDistance {
							t.Fatalf("[%s] Parallel EW roads '%s' (Y=%.1f) and '%s' (Y=%.1f) are too close: %.1f px < %.0f px",
								theme.DisplayName(), r1.Name, y1, r2.Name, y2, dist, minParallelDistance)
						}
					}

					// Check parallel NS roads
					if isR1NS && isR2NS {
						x1 := (r1.P1.X + r1.P2.X) / 2.0
						x2 := (r2.P1.X + r2.P2.X) / 2.0
						dist := math.Abs(x1 - x2)

						minY1 := math.Min(r1.P1.Y, r1.P2.Y)
						maxY1 := math.Max(r1.P1.Y, r1.P2.Y)
						minY2 := math.Min(r2.P1.Y, r2.P2.Y)
						maxY2 := math.Max(r2.P1.Y, r2.P2.Y)
						overlap := math.Min(maxY1, maxY2) - math.Max(minY1, minY2)

						if overlap > 30.0 && dist > 1.0 && dist < minParallelDistance {
							t.Fatalf("[%s] Parallel NS roads '%s' (X=%.1f) and '%s' (X=%.1f) are too close: %.1f px < %.0f px",
								theme.DisplayName(), r1.Name, x1, r2.Name, x2, dist, minParallelDistance)
						}
					}
				}
			}
		})
	}
}

// TestCustomMapParallelRoadSpacingValidation verifies that ValidateCustomMapData
// rejects maps containing parallel roads with distance < 400px and accepts well-spaced maps.
func TestCustomMapParallelRoadSpacingValidation(t *testing.T) {
	// Base valid map
	base := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(base); err != nil {
		t.Fatalf("Default custom map must pass validation: %v", err)
	}

	// 1. Add parallel EW road 200px away (too close)
	badEW := GetDefaultCustomMapData()
	badEW.Roads = append(badEW.Roads, CustomRoadData{
		P1:    [2]float64{-1000, -1000},
		P2:    [2]float64{1000, -1000},
		Lanes: 2,
		Name:  "平行过近加塞路 (200px away from Y=-1200)",
	})
	if err := ValidateCustomMapData(badEW); err == nil {
		t.Fatalf("Expected validation error for parallel EW roads 200px apart, got nil")
	}

	// 2. Add parallel NS road 300px away (too close)
	badNS := GetDefaultCustomMapData()
	badNS.Roads = append(badNS.Roads, CustomRoadData{
		P1:    [2]float64{300, -1000},
		P2:    [2]float64{300, 1000},
		Lanes: 2,
		Name:  "垂直平行过近加塞路 (300px away from X=0)",
	})
	if err := ValidateCustomMapData(badNS); err == nil {
		t.Fatalf("Expected validation error for parallel NS roads 300px apart, got nil")
	}

	// 3. Add parallel EW road 500px away (well spaced)
	goodEW := GetDefaultCustomMapData()
	goodEW.Roads = append(goodEW.Roads, CustomRoadData{
		P1:    [2]float64{-1000, -1700},
		P2:    [2]float64{1000, -1700},
		Lanes: 2,
		Name:  "安全间距平行外环路 (500px away from Y=-1200)",
	})
	if err := ValidateCustomMapData(goodEW); err != nil {
		t.Fatalf("Expected valid custom map with 500px parallel spacing, got error: %v", err)
	}
}

// TestBuiltinMapsBusStopsSafeDistanceFromIntersections ensures that all bus stops in built-in maps
// maintain a safe, realistic distance (>= 250px) from all intersection centers and traffic roundabouts,
// preventing bus stops from crowding junctions or obstructing turns.
// (Per requirement: Custom maps MapThemeCustom are explicitly excluded from this constraint).
func TestBuiltinMapsBusStopsSafeDistanceFromIntersections(t *testing.T) {
	builtinThemes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
	}

	const minSafeDistance = 250.0 // Minimum clearance required from intersection center

	for _, theme := range builtinThemes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(42.0, theme)
			if len(w.Stops) == 0 {
				t.Fatalf("[%s] Map must have bus stops defined", theme.DisplayName())
			}
			if len(w.Intersections) == 0 {
				t.Fatalf("[%s] Map must have intersections computed", theme.DisplayName())
			}

			for _, stop := range w.Stops {
				// 1. Check distance to all intersections
				for _, inter := range w.Intersections {
					dist := stop.Pos.Distance(inter)
					if dist < minSafeDistance {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is too close to intersection at (%.1f, %.1f): distance %.1fpx < required %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y, inter.X, inter.Y, dist, minSafeDistance)
					}

					// Also check road center clearance
					rcDist := stop.RoadCenter.Distance(inter)
					if rcDist < minSafeDistance {
						t.Fatalf("[%s] Bus stop #%d '%s' RoadCenter at (%.1f, %.1f) is too close to intersection at (%.1f, %.1f): distance %.1fpx < required %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.RoadCenter.X, stop.RoadCenter.Y, inter.X, inter.Y, rcDist, minSafeDistance)
					}
				}

				// 2. Check clearance from roundabouts
				for _, rb := range w.Roundabouts {
					distToCenter := stop.Pos.Distance(rb.Center)
					minRBClearance := rb.Radius + 40.0
					if distToCenter < minRBClearance {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is inside roundabout '%s' (radius %.1f): dist %.1fpx < %.1fpx",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y, rb.Name, rb.Radius, distToCenter, minRBClearance)
					}
					if w.IsAtRoundabout(stop.Pos) {
						t.Fatalf("[%s] Bus stop #%d '%s' at (%.1f, %.1f) is flagged inside roundabout by IsAtRoundabout",
							theme.DisplayName(), stop.ID, stop.Name, stop.Pos.X, stop.Pos.Y)
					}
				}
			}
		})
	}
}

// TestNavigationToNextStopOnAllMaps tests that the navigation system to the next stop
// functions properly across all built-in maps and custom maps.
// It verifies:
// 1. Route target stop identification and advancement.
// 2. Generation of valid multi-point road paths along driving lanes.
// 3. Traversal strictly along valid roads without jumping across water or missing bridges.
// 4. Approach heading aligns with the target stop's docking direction.
// 5. Lookahead navigation arrow pointing correctly along road corridors.
func TestNavigationToNextStopOnAllMaps(t *testing.T) {
	allThemes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, theme := range allThemes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := NewWorld(42.0, theme)
			if len(w.Stops) < 2 {
				t.Fatalf("[%s] Map should have at least 2 stops for navigation test", theme.DisplayName())
			}

			// Test with multiple consecutive stop sequences
			route := NewRoute()
			numStopsToTest := 6
			if numStopsToTest > len(w.Stops) {
				numStopsToTest = len(w.Stops)
			}
			for i := 0; i < numStopsToTest; i++ {
				route.AddStop(i)
			}
			// Close the loop
			route.AddStop(0)

			// In transit loop: bus starts departing from Stop 0 towards Stop 1
			route.CurrentIndex = 1
			initialTrips := route.TotalTrips

			for leg := 0; leg < len(route.StopIDs)-1; leg++ {
				targetID := route.GetCurrentTargetStopID()
				if targetID < 0 || targetID >= len(w.Stops) {
					t.Fatalf("[%s] Leg %d: Invalid target stop ID %d", theme.DisplayName(), leg, targetID)
				}
				targetStop := w.Stops[targetID]

				prevIdx := (route.CurrentIndex - 1 + len(route.StopIDs)) % len(route.StopIDs)
				prevStopID := route.StopIDs[prevIdx]
				prevStop := w.Stops[prevStopID]

				// 1. Calculate road path between stops
				path := GetRoadPathBetweenStopsInWorld(w, prevStop, targetStop, 42.0)
				if len(path) < 2 {
					t.Fatalf("[%s] Path between Stop %d ('%s') and Stop %d ('%s') has fewer than 2 waypoints: len=%d",
						theme.DisplayName(), prevStop.ID, prevStop.Name, targetStop.ID, targetStop.Name, len(path))
				}

				// 2. Validate waypoint coordinates and continuity
				for i := 0; i < len(path)-1; i++ {
					p1 := path[i]
					p2 := path[i+1]
					segLen := p1.Distance(p2)
					if segLen < 1.0 {
						t.Fatalf("[%s] Consecutive waypoints %d and %d are degenerate: dist=%.2f",
							theme.DisplayName(), i, i+1, segLen)
					}
				}

				// 3. In River Valley, verify path does NOT cross the river Y=0 without using a bridge
				if theme == MapThemeRiverValley {
					for i := 0; i < len(path)-1; i++ {
						p1 := path[i]
						p2 := path[i+1]
						// Crossing river Y=0
						if (p1.Y < -50 && p2.Y > 50) || (p1.Y > 50 && p2.Y < -50) {
							// Must be along one of the two bridge X coordinates (-1300*MapGridScale or 1300*MapGridScale)
							midX := (p1.X + p2.X) * 0.5
							isBridge1 := math.Abs(midX-(-1300*MapGridScale)) < 180
							isBridge2 := math.Abs(midX-1300*MapGridScale) < 180
							if !isBridge1 && !isBridge2 {
								t.Fatalf("[%s] Path crossed the river without using a bridge: from (%.1f, %.1f) to (%.1f, %.1f) at X=%.1f",
									theme.DisplayName(), p1.X, p1.Y, p2.X, p2.Y, midX)
							}
						}
					}
				}

				// 4. In Coastal Bay, verify path does NOT jump across the open bay (between mainland X <= 0 and harbor X >= 1800*MapGridScale)
				if theme == MapThemeCoastalBay {
					for i := 0; i < len(path)-1; i++ {
						p1 := path[i]
						p2 := path[i+1]
						// Crossing open water between mainland and offshore harbor
						if math.Abs(p1.Y-p2.Y) < 10 {
							minX := math.Min(p1.X, p2.X)
							maxX := math.Max(p1.X, p2.X)
							if minX < 1600*MapGridScale && maxX > 250*MapGridScale && (maxX-minX) > 100 {
								// Any segment traversing this open bay must be Causeway Bridge (Y=0) or South Cape Boardwalk (Y=1200*MapGridScale)
								y := (p1.Y + p2.Y) * 0.5
								isCauseway := math.Abs(y) < 140
								isSouthCape := math.Abs(y-1200*MapGridScale) < 140
								if !isCauseway && !isSouthCape {
									t.Fatalf("[%s] Path traversed offshore open water without a bridge/causeway: (%.1f, %.1f) to (%.1f, %.1f)",
										theme.DisplayName(), p1.X, p1.Y, p2.X, p2.Y)
								}
							}
						}
					}
				}

				// 5. Verify final arrival vector enters stop smoothly along stop heading
				if len(path) >= 2 {
					lastSeg := path[len(path)-1].Sub(path[len(path)-2])
					if lastSeg.Length() > 2 {
						stopDir := Vec2{X: math.Cos(targetStop.Heading), Y: math.Sin(targetStop.Heading)}
						dot := lastSeg.Normalize().Dot(stopDir)
						if dot < -0.1 {
							t.Fatalf("[%s] Final approach vector opposes stop heading: dot=%.2f (heading=%.2f)",
								theme.DisplayName(), dot, targetStop.Heading)
						}
					}
				}

				// 6. Test forward lookahead navigation arrow
				bus := NewBus(prevStop.Pos.X, prevStop.Pos.Y, prevStop.Heading)
				lookaheadFound := false
				for _, pt := range path {
					if bus.Pos.Distance(pt) > 70 {
						lookaheadFound = true
						break
					}
				}
				if len(path) >= 3 && !lookaheadFound {
					t.Fatalf("[%s] Multi-segment path failed to find forward lookahead waypoint", theme.DisplayName())
				}

				// 7. Advance route to next stop
				route.AdvanceToNextStop()
			}

			// Ensure loop completes
			if route.TotalTrips != initialTrips+1 {
				t.Fatalf("[%s] Expected route to complete a full transit loop (trips=%d, expected=%d)",
					theme.DisplayName(), route.TotalTrips, initialTrips+1)
			}
		})
	}
}

func TestRoundaboutCarriagewayLaneWidth(t *testing.T) {
	// Verify all built-in maps with roundabouts have ample carriageway lane width (>= 100px)
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeEmeraldLake,
	}

	for _, theme := range themes {
		w := NewWorld(42.0, theme)
		if len(w.Roundabouts) == 0 {
			continue
		}
		for idx, rb := range w.Roundabouts {
			carriagewayWidth := rb.Radius - rb.InnerRadius
			if carriagewayWidth < 100.0 {
				t.Errorf("[%s] Roundabout #%d (%s) carriageway width is too narrow: %.1f px (must be >= 100.0 px)",
					theme.DisplayName(), idx, rb.Name, carriagewayWidth)
			}
			if rb.InnerRadius < 80.0 {
				t.Errorf("[%s] Roundabout #%d (%s) inner island is too small: %.1f px (must be >= 80.0 px)",
					theme.DisplayName(), idx, rb.Name, rb.InnerRadius)
			}
		}
	}

	// Also verify custom map validation rejects narrow roundabout carriageways (< 80px)
	narrowCustomMap := CustomMapData{
		Name:  "Test Narrow Roundabout",
		Style: "lake",
		Roads: []CustomRoadData{
			{P1: [2]float64{-1000, 0}, P2: [2]float64{1000, 0}, Lanes: 2},
		},
		Stops: []CustomStopData{
			{ID: 0, Name: "A", Pos: [2]float64{-600, 40}, Heading: 0},
			{ID: 1, Name: "B", Pos: [2]float64{600, 40}, Heading: 0},
		},
		Roundabouts: []CustomRoundaboutData{
			{Name: "Narrow Island", Center: [2]float64{0, 0}, Radius: 150.0, InnerRadius: 85.0, Type: "civic"}, // width = 65 < 80
		},
	}
	err := ValidateCustomMapData(&narrowCustomMap)
	if err == nil {
		t.Fatalf("Expected ValidateCustomMapData to reject narrow roundabout carriageway (width = 65px < 80px), but validation passed")
	}
}

func TestTrafficAICirculatesRoundaboutWithoutCrossingCenterIsland(t *testing.T) {
	// Build Emerald Lake map which features the 湖心仙岛古风环岛
	w := NewWorld(42.0, MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	innerR := rb.InnerRadius
	laneW := 42.0

	// Test approaches from East, West, North, South
	approaches := []struct {
		name         string
		startPos     Vec2
		heading      float64
		isEW         bool
		roadCenter   float64
		expectedExit float64
	}{
		{
			name:         "Eastbound (from West)",
			startPos:     Vec2{X: center.X - 350, Y: center.Y + laneW*0.75},
			heading:      0,
			isEW:         true,
			roadCenter:   center.Y,
			expectedExit: 0,
		},
		{
			name:         "Westbound (from East)",
			startPos:     Vec2{X: center.X + 350, Y: center.Y - laneW*0.75},
			heading:      math.Pi,
			isEW:         true,
			roadCenter:   center.Y,
			expectedExit: math.Pi,
		},
		{
			name:         "Southbound (from North)",
			startPos:     Vec2{X: center.X + laneW*0.75, Y: center.Y - 350},
			heading:      math.Pi / 2,
			isEW:         false,
			roadCenter:   center.X,
			expectedExit: math.Pi / 2,
		},
		{
			name:         "Northbound (from South)",
			startPos:     Vec2{X: center.X - laneW*0.75, Y: center.Y + 350},
			heading:      -math.Pi / 2,
			isEW:         false,
			roadCenter:   center.X,
			expectedExit: -math.Pi / 2,
		},
	}

	bus := NewBus(5000, 5000, 0) // bus placed far away
	dt := 1.0 / 60.0

	for _, app := range approaches {
		t.Run(app.name, func(t *testing.T) {
			car := &TrafficCar{
				ID:          0,
				Pos:         app.startPos,
				Heading:     app.heading,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  app.roadCenter,
				IsEW:        app.isEW,
				LaneIndex:   1,
			}
			tm := &TrafficManager{Cars: []*TrafficCar{car}}

			enteredRoundabout := false
			minDistToCenter := math.MaxFloat64

			// Simulate 350 frames (~5.8 seconds), enough to enter, circulate, and exit
			for frame := 0; frame < 350; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				dist := car.Pos.Distance(center)
				if dist < minDistToCenter {
					minDistToCenter = dist
				}

				if car.InRoundabout {
					enteredRoundabout = true
					// Vehicle must NEVER penetrate the inner island of the roundabout
					if dist < innerR {
						t.Fatalf("[%s] AI Car penetrated central island! Car Pos: %v, Center: %v, Dist: %.2f < InnerRadius: %.2f",
							app.name, car.Pos, center, dist, innerR)
					}
					// Vehicle must maintain buffer distance (>= 15px) outside the inner island
					if dist < innerR+15.0 {
						t.Fatalf("[%s] AI Car too close to central island curb: dist=%.2f, innerR=%.2f",
							app.name, dist, innerR)
					}

					// Verify vehicle heading aligns to circle tangent during steady circulation
					// (permitting smooth gradual steering transition during entry and exit)
					if car.RoundaboutTraveled >= 0.5 && car.RoundaboutTraveled <= 2.0 {
						expectedTangentHeading := NormalizeAngle(car.RoundaboutAngle - math.Pi/2)
						headingDiff := math.Abs(NormalizeAngle(car.Heading - expectedTangentHeading))
						if headingDiff > 0.15 {
							t.Fatalf("[%s] AI Car heading %.2f differs from rotary tangent %.2f (diff=%.3f)",
								app.name, car.Heading, expectedTangentHeading, headingDiff)
						}
					}
				}
			}

			if !enteredRoundabout {
				t.Fatalf("[%s] AI Car should have entered and circulated around the roundabout", app.name)
			}
			if minDistToCenter < innerR {
				t.Fatalf("[%s] AI Car minimum distance to center %.2f is inside inner island %.2f",
					app.name, minDistToCenter, innerR)
			}
		})
	}
}

func TestRoundaboutRouteLineCurvatureAndCenterClearance(t *testing.T) {
	// In Emerald Lake map, stops 1 and 2 are on opposite sides of the roundabout
	w := NewWorld(42.0, MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	innerR := rb.InnerRadius

	if len(w.Stops) < 2 {
		t.Fatalf("Emerald lake map must have at least 2 stops")
	}

	// Compute path between consecutive stops
	for i := 0; i < len(w.Stops)-1; i++ {
		s1 := w.Stops[i]
		s2 := w.Stops[i+1]
		path := computeRoadPathBetweenStops(w, s1, s2, 42.0)
		if len(path) == 0 {
			continue
		}

		// Verify every waypoint in the navigation route maintains clearance from central island
		for idx, pt := range path {
			dist := pt.Distance(center)
			if dist < innerR-5.0 {
				t.Fatalf("Route waypoint #%d (%v) between [%s] and [%s] cuts through central island (dist=%.2f < innerR=%.2f)",
					idx, pt, s1.Name, s2.Name, dist, innerR)
			}
		}
	}
}

// TestTrafficAIRoundaboutSmoothnessNoTeleportation strictly monitors the car's frame-to-frame displacement
// and heading rate to guarantee 100% absence of teleportation, snapping, or flickering ("闪现")
// during entry, circular travel, and exit transitions.
func TestTrafficAIRoundaboutSmoothnessNoTeleportation(t *testing.T) {
	w := NewWorld(42.0, MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	laneW := 42.0
	bus := NewBus(5000, 5000, 0)
	dt := 1.0 / 60.0

	// Test Eastbound and Westbound approaches across the small roundabout
	testCases := []struct {
		name     string
		startPos Vec2
		heading  float64
		isEW     bool
		roadC    float64
	}{
		{"Eastbound", Vec2{X: center.X - 320, Y: center.Y + laneW*0.75}, 0, true, center.Y},
		{"Westbound", Vec2{X: center.X + 320, Y: center.Y - laneW*0.75}, math.Pi, true, center.Y},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			car := &TrafficCar{
				ID:          0,
				Pos:         tc.startPos,
				Heading:     tc.heading,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  tc.roadC,
				IsEW:        tc.isEW,
				LaneIndex:   1,
			}
			tm := &TrafficManager{Cars: []*TrafficCar{car}}

			prevPos := car.Pos
			prevHeading := car.Heading
			maxStepDisplacement := 0.0
			maxHeadingChange := 0.0

			for frame := 0; frame < 360; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				stepDisplacement := car.Pos.Distance(prevPos)
				headingChange := math.Abs(NormalizeAngle(car.Heading - prevHeading))

				if stepDisplacement > maxStepDisplacement {
					maxStepDisplacement = stepDisplacement
				}
				if headingChange > maxHeadingChange {
					maxHeadingChange = headingChange
				}

				// At speed ~140px/s, nominal frame displacement is 140 * (1/60) = 2.33 px.
				// Any frame-to-frame jump exceeding 8.0 px represents an unphysical snap/teleport ("闪现").
				if stepDisplacement > 8.0 {
					t.Fatalf("[%s] Frame %d: Teleportation detected! Single-frame displacement = %.2f px > 8.0 px limit (prev: %v, curr: %v, inRoundabout: %v)",
						tc.name, frame, stepDisplacement, prevPos, car.Pos, car.InRoundabout)
				}

				// Heading must not instantaneously snap (e.g. 70+ degrees in a single frame)
				if headingChange > 0.25 {
					t.Fatalf("[%s] Frame %d: Instant heading snap detected! delta=%.3f rad (%.1f deg)",
						tc.name, frame, headingChange, headingChange*180.0/math.Pi)
				}

				prevPos = car.Pos
				prevHeading = car.Heading
			}

			// Validate maximum displacement across the entire 6-second journey was smooth and continuous
			if maxStepDisplacement > 6.0 {
				t.Errorf("[%s] Expected silky smooth displacement (<6.0px/frame), got max=%.2f px",
					tc.name, maxStepDisplacement)
			}
		})
	}
}

// TestBusStopApproachAndBayBoundaryClearance verifies that the white box area in the user screenshot
// (the approach deceleration taper leading to Coastal Bay Stop #10 深水港智能集装箱总站)
// as well as all bus bays across all maps provide clean ZoneRoad clearance without hitting ZoneHardBarrier.
func TestBusStopApproachAndBayBoundaryClearance(t *testing.T) {
	laneW := 42.0

	// 1. Specific test for Coastal Bay Stop #10 (深水港智能集装箱总站)
	wc := NewWorld(laneW, MapThemeCoastalBay)
	var stop10 *BusStop
	for _, s := range wc.Stops {
		if s.ID == 10 {
			stop10 = s
			break
		}
	}
	if stop10 == nil {
		t.Fatalf("Coastal Bay stop #10 not found")
	}

	// Verify approach coordinates along stop10 travel heading
	cosH := math.Cos(stop10.Heading)
	sinH := math.Sin(stop10.Heading)
	whiteBoxCoords := []Vec2{
		stop10.Pos.Add(Vec2{X: -cosH * 120, Y: -sinH * 120}),
		stop10.Pos.Add(Vec2{X: -cosH * 70, Y: -sinH * 70}),
		stop10.Pos.Add(Vec2{X: -cosH * 30, Y: -sinH * 30}),
		stop10.Pos,
	}
	for _, pt := range whiteBoxCoords {
		res := wc.CheckBoundaries(pt, 18.0, laneW)
		if res.Zone == ZoneHardBarrier {
			t.Fatalf("Coastal Bay stop #10 approach point %v hit hard barrier: %s (pen=%.1f)",
				pt, res.ObstacleName, res.Penetration)
		}
		if res.Zone != ZoneRoad {
			t.Fatalf("Coastal Bay stop #10 approach point %v expected ZoneRoad, got %v", pt, res.Zone)
		}
	}

	// Verify that physical shelter structure on sidewalk DOES properly detect collision
	shelterHit := wc.CheckBoundaries(stop10.ShelterPos, 18.0, laneW)
	if shelterHit.Zone != ZoneHardBarrier {
		t.Fatalf("Expected physical shelter structure at %v to trigger ZoneHardBarrier, got %v",
			stop10.ShelterPos, shelterHit.Zone)
	}

	// 2. Comprehensive check across all themes: each stop bay and approach corridor must be free of hard barriers
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
	}
	for _, theme := range themes {
		w := NewWorld(laneW, theme)
		for _, s := range w.Stops {
			cosH := math.Cos(s.Heading)
			sinH := math.Sin(s.Heading)
			perpX := -sinH
			perpY := cosH

			// Test bay center
			bRes := w.CheckBoundaries(s.Pos, 18.0, laneW)
			if bRes.Zone == ZoneHardBarrier {
				t.Fatalf("[%s] Stop #%d '%s' bay center %v hit hard barrier: %s",
					theme.DisplayName(), s.ID, s.Name, s.Pos, bRes.ObstacleName)
			}

			// Test points along approach taper: 120px and 60px before stop bay
			for _, offset := range []float64{-120.0, -60.0, 0.0, 60.0} {
				// Point halfway into bay
				testPt := s.Pos.Add(Vec2{X: cosH * offset, Y: sinH * offset})
				res := w.CheckBoundaries(testPt, 18.0, laneW)
				if res.Zone == ZoneHardBarrier {
					t.Fatalf("[%s] Stop #%d '%s' corridor offset %.0f at %v hit hard barrier: %s",
						theme.DisplayName(), s.ID, s.Name, offset, testPt, res.ObstacleName)
				}
			}

			// Outer edge of bay (s.Width * 0.45 towards curb)
			bayEdgePt := s.Pos.Add(Vec2{X: perpX * (s.Width * 0.45), Y: perpY * (s.Width * 0.45)})
			edgeRes := w.CheckBoundaries(bayEdgePt, 18.0, laneW)
			if edgeRes.Zone == ZoneHardBarrier {
				t.Fatalf("[%s] Stop #%d '%s' outer bay edge at %v hit hard barrier: %s",
					theme.DisplayName(), s.ID, s.Name, bayEdgePt, edgeRes.ObstacleName)
			}
		}
	}
}

// TestTrafficAIRoundaboutPureCircularArcNoDiagonalCuts verifies that AI vehicles follow a true circular arc
// inside the roundabout at all times, without expanding outward diagonally or cutting into corner obstacles.
func TestTrafficAIRoundaboutPureCircularArcNoDiagonalCuts(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeEmeraldLake,
	}
	laneW := 42.0
	dt := 1.0 / 60.0
	bus := NewBus(5000, 5000, 0)

	for _, theme := range themes {
		w := NewWorld(laneW, theme)
		if len(w.Roundabouts) == 0 {
			continue
		}
		rb := w.Roundabouts[0]
		center := rb.Center

		t.Run(theme.DisplayName(), func(t *testing.T) {
			// Test approach from West (Eastbound)
			car := &TrafficCar{
				ID:          0,
				Pos:         Vec2{X: center.X - 350, Y: center.Y + laneW*0.75},
				Heading:     0,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  center.Y,
				IsEW:        true,
				LaneIndex:   1,
			}
			tm := &TrafficManager{Cars: []*TrafficCar{car}}

			entered := false
			exited := false

			for frame := 0; frame < 400; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				if car.InRoundabout {
					entered = true
					dist := car.Pos.Distance(center)

					// 1. Vehicle must strictly maintain its circulating radius within a tight band during circulation (no diagonal curb expansion)
					if car.RoundaboutTraveled >= 0.40 {
						if math.Abs(dist-car.RoundaboutTargetR) > 8.0 {
							t.Fatalf("[%s] Frame %d: Car radius deviated from circular target: dist=%.2f, targetR=%.2f, diff=%.2f > 8px",
								theme.DisplayName(), frame, dist, car.RoundaboutTargetR, math.Abs(dist-car.RoundaboutTargetR))
						}
					}

					// 2. Car distance must stay strictly between inner island and outer curb with safe buffer
					if dist < rb.InnerRadius+15.0 {
						t.Fatalf("[%s] Frame %d: Car too close to central island: dist=%.2f, innerR=%.2f",
							theme.DisplayName(), frame, dist, rb.InnerRadius)
					}
					if car.RoundaboutTraveled >= 0.40 && dist > rb.Radius-15.0 {
						t.Fatalf("[%s] Frame %d: Car drifted into outer curb obstacle zone: dist=%.2f, outerR=%.2f",
							theme.DisplayName(), frame, dist, rb.Radius)
					}

					// 3. Boundary check must confirm vehicle is squarely on ZoneRoad at all times
					bRes := w.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone != ZoneRoad {
						t.Fatalf("[%s] Frame %d: Car at %v not on ZoneRoad (got %v, obstacle: %s)",
							theme.DisplayName(), frame, car.Pos, bRes.Zone, bRes.ObstacleName)
					}

					// 4. During steady rotary travel, heading must align with circle tangent
					if car.RoundaboutTraveled >= 0.5 && car.RoundaboutTraveled <= 2.0 {
						expectedTangent := NormalizeAngle(car.RoundaboutAngle - math.Pi/2)
						headingErr := math.Abs(NormalizeAngle(car.Heading - expectedTangent))
						if headingErr > 0.12 {
							t.Fatalf("[%s] Frame %d: Car heading %.2f deviated from circle tangent %.2f by %.3f rad",
								theme.DisplayName(), frame, car.Heading, expectedTangent, headingErr)
						}
					}
				} else if entered {
					exited = true
					// Once exited, verify car is safely on road
					bRes := w.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone == ZoneHardBarrier {
						t.Fatalf("[%s] Frame %d: Car hit hard barrier upon exiting roundabout: %v, obstacle: %s",
							theme.DisplayName(), frame, car.Pos, bRes.ObstacleName)
					}
				}
			}

			if !entered {
				t.Fatalf("[%s] AI Car failed to enter roundabout", theme.DisplayName())
			}
			if !exited {
				t.Fatalf("[%s] AI Car failed to exit roundabout", theme.DisplayName())
			}
		})
	}
}

func TestNonPlayerVehiclesDeadlockTimeoutDespawn(t *testing.T) {
	// 1. Two AI civilian cars jammed head-to-head
	tm := &TrafficManager{
		Cars: make([]*TrafficCar, 0),
	}
	car1 := &TrafficCar{
		ID:          1,
		Pos:         Vec2{X: 100, Y: 0},
		Heading:     0,
		Speed:       0,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}
	car2 := &TrafficCar{
		ID:          2,
		Pos:         Vec2{X: 150, Y: 0},
		Heading:     math.Pi,
		Speed:       0,
		TargetSpeed: 100,
		Length:      46,
		Width:       22,
		RoadCenter:  0,
		IsEW:        true,
	}
	tm.Cars = append(tm.Cars, car1, car2)

	playerBus := NewBus(5000, 5000, 0) // Far away player bus

	// Simulate 10 seconds of motionless deadlock (less than 38s traffic light cycle threshold)
	dt := 0.5
	for elapsed := 0.0; elapsed < 10.0; elapsed += dt {
		tm.resolveDeadlockedCars(dt, playerBus, len(tm.Cars), 52.0, nil)
	}

	if len(tm.Cars) != 2 {
		t.Fatalf("Expected 2 cars still present after only 10s deadlock, got %d", len(tm.Cars))
	}
	if car1.DeadlockTimer < 9.5 || car1.DeadlockTimer > 10.5 {
		t.Fatalf("Expected car1 deadlock timer ~10s, got %f", car1.DeadlockTimer)
	}

	// Moving resets deadlock timer
	car1.Speed = 50.0
	tm.resolveDeadlockedCars(dt, playerBus, len(tm.Cars), 52.0, nil)
	if car1.DeadlockTimer != 0 {
		t.Fatalf("Expected moving car to reset deadlock timer to 0, got %f", car1.DeadlockTimer)
	}
	car1.Speed = 0

	// Now simulate until deadlock timer exceeds DeadlockDurationThreshold (38.0s)
	// (Simulate 39s more)
	for elapsed := 0.0; elapsed < 39.0; elapsed += dt {
		tm.resolveDeadlockedCars(dt, playerBus, len(tm.Cars), 52.0, nil)
		if len(tm.Cars) < 2 {
			break
		}
	}

	if len(tm.Cars) != 1 {
		t.Fatalf("Expected one car to directly disappear after > 38s deadlock, remaining: %d", len(tm.Cars))
	}

	// 2. Verify AI car stopped behind player bus NEVER despawns
	soloCar := tm.Cars[0]
	soloCar.Pos = Vec2{X: 100, Y: 0}
	soloCar.Heading = 0
	soloCar.Speed = 0
	soloCar.DeadlockTimer = 0

	// Place player bus right in front of soloCar
	playerBus.Pos = Vec2{X: 150, Y: 0}
	playerBus.Heading = 0
	playerBus.Speed = 0

	for elapsed := 0.0; elapsed < 45.0; elapsed += dt {
		tm.resolveDeadlockedCars(dt, playerBus, len(tm.Cars), 52.0, nil)
	}

	if len(tm.Cars) != 1 {
		t.Fatalf("Car should not disappear when stopped because of the player bus!")
	}
	if soloCar.DeadlockTimer != 0 {
		t.Fatalf("Deadlock timer should remain 0 when blocked by player bus, got %f", soloCar.DeadlockTimer)
	}

	// 3. Two AI buses jammed head-to-head
	w := NewWorld(52.0, MapThemeMetropolis)
	abm := NewAIBusManager()
	abm.SetLineCount(2, w, 52.0)

	if len(abm.Buses) >= 2 {
		bus1 := abm.Buses[0]
		bus2 := abm.Buses[1]
		bus1.Bus.Pos = Vec2{X: 200, Y: 0}
		bus1.Bus.Heading = 0
		bus1.Bus.Speed = 0
		bus2.Bus.Pos = Vec2{X: 270, Y: 0}
		bus2.Bus.Heading = math.Pi
		bus2.Bus.Speed = 0

		oldPos1 := bus1.Bus.Pos

		// Simulate 40s (> DeadlockDurationThreshold)
		for elapsed := 0.0; elapsed < 40.0; elapsed += dt {
			abm.Update(dt, playerBus, w, nil, tm, 52.0, 2)
		}

		// At least one AI bus must have repositioned to its next route stop to clear the jam
		if bus1.Bus.Pos == oldPos1 && bus2.Bus.Pos.Distance(oldPos1) < 100.0 {
			t.Fatalf("Expected AI buses to resolve deadlock after > 38s by advancing stop")
		}
	}
}

func TestPassengerDestinationAssignment(t *testing.T) {
	// 1. Metropolis closed loop: [0, 2, 10, 8, 0]
	route := NewDefaultRoute()

	// Stop 0 (Terminal at start of loop): candidates must strictly be [2, 10, 8], NOT 0
	c0 := route.GetUpcomingCandidateStopIDs(0)
	if len(c0) != 3 {
		t.Fatalf("Expected 3 candidates for Stop 0, got %v", c0)
	}
	for _, id := range c0 {
		if id == 0 {
			t.Fatalf("Stop 0 must NEVER be a candidate destination for passengers boarding at Stop 0! Got %v", c0)
		}
	}

	// Stop 2 (index 1): candidates must strictly be [10, 8, 0], NOT 2
	c1 := route.GetUpcomingCandidateStopIDs(1)
	if len(c1) != 3 {
		t.Fatalf("Expected 3 candidates for Stop 2, got %v", c1)
	}
	for _, id := range c1 {
		if id == 2 {
			t.Fatalf("Stop 2 must NEVER be a candidate destination for passengers boarding at Stop 2! Got %v", c1)
		}
	}

	// Stop 10 (index 2): candidates must be [8, 0]
	c2 := route.GetUpcomingCandidateStopIDs(2)
	if len(c2) != 2 || c2[0] != 8 || c2[1] != 0 {
		t.Fatalf("Expected candidates [8, 0] for Stop 10, got %v", c2)
	}

	// Stop 8 (index 3): candidates must be [0]
	c3 := route.GetUpcomingCandidateStopIDs(3)
	if len(c3) != 1 || c3[0] != 0 {
		t.Fatalf("Expected candidates [0] for Stop 8, got %v", c3)
	}

	// Stop 0 at terminus (index 4): candidates for next run must be [2, 10, 8], NOT 0
	c4 := route.GetUpcomingCandidateStopIDs(4)
	if len(c4) != 3 {
		t.Fatalf("Expected 3 candidates for terminal boarding, got %v", c4)
	}
	for _, id := range c4 {
		if id == 0 {
			t.Fatalf("Stop 0 must NEVER be candidate destination for terminal boarding, got %v", c4)
		}
	}

	// Also check GetUpcomingCandidateStopIDsForStop
	cFor0 := route.GetUpcomingCandidateStopIDsForStop(0)
	for _, id := range cFor0 {
		if id == 0 {
			t.Fatalf("GetUpcomingCandidateStopIDsForStop(0) must not contain 0, got %v", cFor0)
		}
	}
}

func TestPassengerBoardingAndAlighting(t *testing.T) {
	bus := NewBus(0, 0, 0)
	if bus.PassengerCount != 0 || len(bus.Passengers) != 0 {
		t.Fatalf("New bus must have 0 passengers")
	}

	// Board 10 passengers with destinations in [2, 10, 8]
	candidates := []int{2, 10, 8}
	boarded := bus.BoardPassengers(10, candidates)
	if boarded != 10 || bus.PassengerCount != 10 || len(bus.Passengers) != 10 {
		t.Fatalf("Expected 10 boarded passengers, got %d (count=%d)", boarded, bus.PassengerCount)
	}

	// Verify all passengers have a valid destination in candidates
	for _, p := range bus.Passengers {
		if p.TargetStopID != 2 && p.TargetStopID != 10 && p.TargetStopID != 8 {
			t.Fatalf("Invalid passenger destination: %d", p.TargetStopID)
		}
	}

	// Alight passengers destined for Stop 2
	expectedAlighting2 := bus.CountPassengersAlightingAt(2)
	removed2 := bus.RemovePassengersAlightingAt(2)
	if removed2 != expectedAlighting2 {
		t.Fatalf("Removed count %d != counted alighting %d", removed2, expectedAlighting2)
	}
	if bus.PassengerCount != 10-removed2 || len(bus.Passengers) != 10-removed2 {
		t.Fatalf("PassengerCount mismatch after alighting: %d vs %d", bus.PassengerCount, 10-removed2)
	}
	if bus.CountPassengersAlightingAt(2) != 0 {
		t.Fatalf("Expected 0 passengers remaining for Stop 2")
	}

	// Clear passengers
	remaining := bus.PassengerCount
	cleared := bus.ClearPassengers()
	if cleared != remaining || bus.PassengerCount != 0 || len(bus.Passengers) != 0 {
		t.Fatalf("ClearPassengers failed: cleared %d, count %d", cleared, bus.PassengerCount)
	}
}

func TestPassengerNoLoopingOverFullRoute(t *testing.T) {
	// Full simulated loop test: ensure passengers get off at their target stops
	// and NO passenger ever stays on the bus for a full loop!
	game := NewGame(1280, 720)

	// Ensure Metropolis map and standard loop [0, 2, 10, 8, 0]
	game.SetMapTheme(MapThemeMetropolis)
	route := game.Route
	if len(route.StopIDs) < 2 || route.StopIDs[0] != route.StopIDs[len(route.StopIDs)-1] {
		t.Fatalf("Expected closed loop route for Metropolis")
	}

	// Simulate 3 full trips
	for trip := 0; trip < 3; trip++ {
		for stopSeqIdx := 0; stopSeqIdx < len(route.StopIDs); stopSeqIdx++ {
			currStopID := route.StopIDs[stopSeqIdx]
			stop := game.World.Stops[currStopID]

			// Position bus at this stop
			game.Bus.Pos = stop.Pos
			game.Bus.Heading = stop.Heading
			game.Bus.DoorsOpen = true
			game.boardedThisStop = false

			// Ensure station has waiting passengers
			stop.WaitingPassengers = 8

			// Update driver mode to trigger passenger boarding/alighting
			game.updateDriverMode(0.016)

			// Invariant check: PassengerCount must always equal len(Passengers)
			if game.Bus.PassengerCount != len(game.Bus.Passengers) {
				t.Fatalf("Trip %d Stop %d: PassengerCount (%d) != len(Passengers) (%d)",
					trip, currStopID, game.Bus.PassengerCount, len(game.Bus.Passengers))
			}

			// Invariant check: No passenger on board should have target == currStopID
			// (because all passengers destined for currStopID must have alighted)
			if game.Bus.CountPassengersAlightingAt(currStopID) > 0 {
				t.Fatalf("Trip %d Stop %d: passengers still on board destined for this stop!", trip, currStopID)
			}
		}

		// At the completion of the loop (Stop 0 terminus), all passengers must have alighted
		// Any new passengers boarded at Stop 0 must strictly have destination in [2, 10, 8]
		for _, p := range game.Bus.Passengers {
			if p.TargetStopID == 0 {
				t.Fatalf("Trip %d: Found passenger on board with TargetStopID == 0 right after departing terminal!", trip)
			}
		}
	}
}

func TestAIBusDestinationBasedExchange(t *testing.T) {
	world := NewWorld(52.0, MapThemeMetropolis)
	abm := NewAIBusManager()
	abm.SetLineCount(1, world, 52.0)

	if len(abm.Buses) < 1 {
		t.Fatalf("Expected at least 1 AI bus")
	}

	ab := abm.Buses[0]
	// AI bus route is [0, 2, 10, 8, 0]
	ab.Route.CurrentIndex = 0
	targetStopID := ab.Route.GetCurrentTargetStopID() // Stop 0
	st := world.Stops[targetStopID]
	st.WaitingPassengers = 10

	// Dock bus and open doors
	ab.Bus.DoorsOpen = true
	ab.LastExchangedStopID = -1

	// Simulate exchange directly
	candidates := ab.Route.GetUpcomingCandidateStopIDs(ab.Route.CurrentIndex)
	if len(candidates) == 0 {
		t.Fatalf("Expected candidates for AI bus")
	}
	for _, id := range candidates {
		if id == targetStopID {
			t.Fatalf("AI bus candidate should not include current stop ID %d", targetStopID)
		}
	}

	boarded := ab.Bus.BoardPassengers(5, candidates)
	if boarded != 5 || ab.Bus.PassengerCount != 5 {
		t.Fatalf("Expected 5 boarded on AI bus, got %d", boarded)
	}

	// When docking at next stop, passengers targeting it alight
	nextStopID := candidates[0]
	countForNext := ab.Bus.CountPassengersAlightingAt(nextStopID)
	removed := ab.Bus.RemovePassengersAlightingAt(nextStopID)
	if removed != countForNext {
		t.Fatalf("AI bus alighting count mismatch: %d vs %d", removed, countForNext)
	}
}

func TestCleanStationName(t *testing.T) {
	stop := &BusStop{
		Name: "中央枢纽 (Central Terminal)",
	}

	CurrentLanguage = LanguageZH
	if clean := stop.CleanName(); clean != "中央枢纽" {
		t.Fatalf("Expected Chinese CleanName to be '中央枢纽', got %q", clean)
	}

	CurrentLanguage = LanguageEN
	if clean := stop.CleanName(); clean != "Central Terminal" {
		t.Fatalf("Expected English CleanName to be 'Central Terminal', got %q", clean)
	}

	// Reset language back to Chinese
	CurrentLanguage = LanguageZH
}

func TestFormattedRouteLinesNoOverflow(t *testing.T) {
	world := NewWorld(52.0, MapThemeMetropolis)

	// 1. Default 5-stop route
	route := NewDefaultRoute()
	lines := route.GetFormattedRouteLines(world, 1100.0, 12.0)
	if len(lines) == 0 {
		t.Fatalf("Expected at least 1 formatted route line")
	}
	for i, l := range lines {
		w, _ := MeasureText(l, 12.0)
		if w > 1100.0 {
			t.Fatalf("Line %d exceeds maxWidth 1100.0: w=%.1f text=%q", i, w, l)
		}
	}

	// 2. Constrained width test: ensure wrapping onto multiple lines
	narrowLines := route.GetFormattedRouteLines(world, 350.0, 12.0)
	if len(narrowLines) < 2 {
		t.Fatalf("Expected route to wrap onto multiple lines under 350px width, got %d lines: %v", len(narrowLines), narrowLines)
	}
	for i, l := range narrowLines {
		w, _ := MeasureText(l, 12.0)
		if w > 350.0 {
			t.Fatalf("Narrow Line %d exceeds maxWidth 350.0: w=%.1f text=%q", i, w, l)
		}
	}

	// 3. Extremely long route (30 stops): must wrap and truncate safely without overflowing maxWidth
	longRoute := NewRoute()
	for i := 0; i < 30; i++ {
		longRoute.AddStop(i % len(world.Stops))
	}
	longLines := longRoute.GetFormattedRouteLines(world, 1000.0, 12.0)
	if len(longLines) > 3 {
		t.Fatalf("Expected at most 3 wrapped lines for long route, got %d", len(longLines))
	}
	for i, l := range longLines {
		w, _ := MeasureText(l, 12.0)
		if w > 1000.0 {
			t.Fatalf("Long route Line %d exceeds maxWidth 1000.0: w=%.1f text=%q", i, w, l)
		}
	}
}

func TestReplanRouteResetsPlayerAndAIBusesToFirstStop(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)

	laneW := g.Settings.GetLaneWidth()
	g.Settings.AIBusCount = 2
	g.AIBusMgr.BuildFleet(g.World, 2, laneW, g.Bus)

	if len(g.AIBusMgr.Buses) != 2 {
		t.Fatalf("Expected 2 AI buses, got %d", len(g.AIBusMgr.Buses))
	}

	// 1. Move player bus away and load passengers to simulate active driving
	g.Bus.Pos = Vec2{X: 800, Y: 400}
	g.Bus.Heading = 1.2
	g.Bus.Speed = 160.0
	g.Bus.DoorsOpen = false
	g.Bus.BoardPassengers(5, []int{2, 10})
	if g.Bus.PassengerCount != 5 {
		t.Fatalf("Expected 5 passengers, got %d", g.Bus.PassengerCount)
	}

	// 2. Enter Plan Mode and exit WITHOUT modifying route: player vehicle must NOT reset
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true after TogglePlanMode")
	}
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after TogglePlanMode")
	}
	if g.Bus.Pos.Distance(Vec2{X: 800, Y: 400}) > 1e-3 || g.Bus.Speed != 160.0 || g.Bus.PassengerCount != 5 {
		t.Fatalf("Player vehicle state should not reset if route was not modified: pos=%+v speed=%f pax=%d",
			g.Bus.Pos, g.Bus.Speed, g.Bus.PassengerCount)
	}

	// 3. Enter Plan Mode and REPLAN route to start at Stop 3 ([3, 7, 2, 3])
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true")
	}
	g.Route.Clear()
	g.Route.AddStop(3)
	g.Route.AddStop(7)
	g.Route.AddStop(2)
	g.Route.AddStop(3)
	g.planModeRouteModified = true

	// Exit Plan Mode: should trigger reset of player bus and AI fleet to first stop (Stop 3)
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after exiting plan mode")
	}

	stop3 := g.World.Stops[3]
	if g.Bus.Pos.Distance(stop3.Pos) > 1e-3 {
		t.Fatalf("Player bus position was not reset to first stop: expected %+v, got %+v", stop3.Pos, g.Bus.Pos)
	}
	if math.Abs(NormalizeAngle(g.Bus.Heading-stop3.Heading)) > 1e-4 {
		t.Fatalf("Player bus heading was not reset: expected %f, got %f", stop3.Heading, g.Bus.Heading)
	}
	if g.Bus.Speed != 0 {
		t.Fatalf("Player bus speed must be 0 after reset, got %f", g.Bus.Speed)
	}
	if g.Bus.PassengerCount != 0 {
		t.Fatalf("Player bus passengers must be cleared for new route, got %d", g.Bus.PassengerCount)
	}
	if !g.Bus.DoorsOpen {
		t.Fatal("Player bus doors should be open at initial station bay")
	}
	if g.Route.GetCurrentTargetStopID() != 3 {
		t.Fatalf("Active target stop must be first stop 3, got %d", g.Route.GetCurrentTargetStopID())
	}

	// Verify AI buses reset and queued behind player bus at Stop 3
	if len(g.AIBusMgr.Buses) != 2 {
		t.Fatalf("Expected 2 AI buses, got %d", len(g.AIBusMgr.Buses))
	}
	cosH := math.Cos(stop3.Heading)
	sinH := math.Sin(stop3.Heading)
	backVec := Vec2{X: -cosH, Y: -sinH}

	for i, ai := range g.AIBusMgr.Buses {
		if math.Abs(NormalizeAngle(ai.Bus.Heading-stop3.Heading)) > 1e-4 {
			t.Fatalf("AI bus %d heading mismatch: expected %f, got %f", i, stop3.Heading, ai.Bus.Heading)
		}
		if ai.Bus.Speed != 0 {
			t.Fatalf("AI bus %d speed must be 0, got %f", i, ai.Bus.Speed)
		}
		expectedPos := stop3.Pos.Add(backVec.Mul(float64(i+1) * 120.0))
		if ai.Bus.Pos.Distance(expectedPos) > 1.0 {
			t.Fatalf("AI bus %d position mismatch: expected %+v, got %+v", i, expectedPos, ai.Bus.Pos)
		}
		if len(ai.Route.StopIDs) != 4 || ai.Route.StopIDs[0] != 3 || ai.Route.StopIDs[1] != 7 {
			t.Fatalf("AI bus %d route was not updated to replanned route: %v", i, ai.Route.StopIDs)
		}
		if ai.Route.CurrentIndex != 0 {
			t.Fatalf("AI bus %d CurrentIndex must be 0, got %d", i, ai.Route.CurrentIndex)
		}
	}

	// 4. Test empty route validation: cannot exit plan mode if route is empty
	g.TogglePlanMode()
	g.Route.Clear()
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Game should not exit Plan Mode when route is empty")
	}

	// 5. Test programmatic ReplanRoute helper
	g.ReplanRoute([]int{5, 9, 5})
	stop5 := g.World.Stops[5]
	if g.Bus.Pos.Distance(stop5.Pos) > 1e-3 {
		t.Fatalf("ReplanRoute failed to reset player bus to Stop 5: expected %+v, got %+v", stop5.Pos, g.Bus.Pos)
	}
	if len(g.AIBusMgr.Buses) > 0 {
		expectedAI0Pos := stop5.Pos.Add(Vec2{X: -math.Cos(stop5.Heading), Y: -math.Sin(stop5.Heading)}.Mul(120.0))
		if g.AIBusMgr.Buses[0].Bus.Pos.Distance(expectedAI0Pos) > 1.0 {
			t.Fatalf("ReplanRoute failed to reset AI bus behind Stop 5: expected %+v, got %+v",
				expectedAI0Pos, g.AIBusMgr.Buses[0].Bus.Pos)
		}
	}

	// 6. Test HandleModeBtnClick in HUD
	if !g.HUD.HandleModeBtnClick(Vec2{X: 100, Y: 25}) {
		t.Fatal("Expected HandleModeBtnClick to return true for badge coordinates (100, 25)")
	}
	if g.HUD.HandleModeBtnClick(Vec2{X: 500, Y: 25}) {
		t.Fatal("Expected HandleModeBtnClick to return false for outside coordinates (500, 25)")
	}
}

// TestSingleLaneMeetsThreeLaneIntersectionStopLinePosition verifies that when a single-lane road
// (1 lane each way = 2-lane road) meets a 3-lane road (3 lanes each way = 6-lane road),
// the stop line and traffic lights of the single-lane road are positioned outside the 6-lane road
// rather than in the middle of the road.
func TestSingleLaneMeetsThreeLaneIntersectionStopLinePosition(t *testing.T) {
	laneWidth := 52.0
	// 6-lane East-West road meets 2-lane North-South road at (0, 0)
	inter := &TrafficIntersection{
		ID:              0,
		Center:          Vec2{X: 0, Y: 0},
		ApproachHasRoad: [4]bool{true, true, true, true},
		ApproachLanes:   [4]int{6, 6, 2, 2},
		ApproachIs4Lane: [4]bool{true, true, false, false},
		Phase:           PhaseEWGreen,
	}

	ewStop, ewCross, nsStop, nsCross := inter.GetApproachDimensions(laneWidth)
	_ = ewCross

	// EW 6-lane road has half-width 3 * 52 + 12 = 168.0
	ewRoadHalfWidth := laneWidth*3.0 + 12.0
	// NS 2-lane road has half-width 1 * 52 + 6 = 58.0
	nsRoadHalfWidth := laneWidth*1.0 + 6.0

	// The NS approaches (single lane in each direction) must have their stop line and crosswalk
	// placed completely outside the EW 6-lane road (i.e. > 168.0px from intersection center)
	if nsStop <= ewRoadHalfWidth {
		t.Fatalf("NS approach stop line (%.2f) must be outside the EW 6-lane roadway (half-width %.2f), not in the middle of the road", nsStop, ewRoadHalfWidth)
	}
	if nsCross <= ewRoadHalfWidth {
		t.Fatalf("NS approach crosswalk (%.2f) must be outside the EW 6-lane roadway (half-width %.2f)", nsCross, ewRoadHalfWidth)
	}
	// Stop line must be behind crosswalk
	if nsStop <= nsCross {
		t.Fatalf("NS stop line (%.2f) must be behind crosswalk (%.2f)", nsStop, nsCross)
	}

	// EW approach (the 6-lane road crossing the 2-lane road) stop line must be outside the NS 2-lane road (58.0px)
	if ewStop <= nsRoadHalfWidth {
		t.Fatalf("EW approach stop line (%.2f) must be outside NS 2-lane roadway (half-width %.2f)", ewStop, nsRoadHalfWidth)
	}

	// Check heading-specific stop line calculation
	northboundHeading := -math.Pi / 2 // Heading North (South approach)
	nsStopForHeading := inter.GetStopLineDistForHeading(northboundHeading, laneWidth)
	if nsStopForHeading != nsStop {
		t.Fatalf("Expected northbound heading to use NS stop line %.2f, got %.2f", nsStop, nsStopForHeading)
	}

	eastboundHeading := 0.0 // Heading East (West approach)
	ewStopForHeading := inter.GetStopLineDistForHeading(eastboundHeading, laneWidth)
	if ewStopForHeading != ewStop {
		t.Fatalf("Expected eastbound heading to use EW stop line %.2f, got %.2f", ewStop, ewStopForHeading)
	}

	// Now verify with real CoastalBay world map at intersection (-1950, 0)
	w := NewWorld(laneWidth, MapThemeCoastalBay)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)

	var bayInter *TrafficIntersection
	for _, it := range tls.Intersections {
		if math.Abs(it.Center.X-(-1950.0)) < 10.0 && math.Abs(it.Center.Y-0.0) < 10.0 {
			bayInter = it
			break
		}
	}
	if bayInter == nil {
		t.Fatal("Coastal Bay intersection at (-1950, 0) not found")
	}

	bEWStop, bEWCross, bNSStop, bNSCross := bayInter.GetApproachDimensions(laneWidth)
	if bNSStop <= 168.0 {
		t.Fatalf("Coastal Bay intersection (-1950, 0) NS stop line (%.2f) is inside the 6-lane avenue (half-width 168.0)!", bNSStop)
	}
	if bNSCross <= 168.0 {
		t.Fatalf("Coastal Bay intersection (-1950, 0) NS crosswalk (%.2f) is inside the 6-lane avenue (half-width 168.0)!", bNSCross)
	}

	// Verify vehicle stopping on red:
	// A bus traveling North towards the intersection on the 2-lane road
	bayInter.Phase = PhaseEWGreen // NS is RED!
	busBumperPos := Vec2{X: -1950.0 + 26.0, Y: 250.0} // approaching from South, bumper at Y = 250
	shouldStop, distToLine := tls.CheckStopForVehicle(busBumperPos, -math.Pi/2, 100.0, laneWidth)
	if !shouldStop {
		t.Fatal("Bus approaching red light on 2-lane road must stop")
	}
	// Distance to line should be relative to bNSStop (e.g. 250 - 211 = 39.0)
	expectedDist := 250.0 - bNSStop
	if math.Abs(distToLine-expectedDist) > 1.0 {
		t.Fatalf("Expected distance to stop line around %.2f, got %.2f", expectedDist, distToLine)
	}
	_ = bEWStop
	_ = bEWCross
}

// TestCoastalBaySouthCapeCausewayNoShoulderDriving verifies that along the South Cape Causeway
// (南湾海角栈道, 2 lanes) leading to Stop 11 (Cape Lighthouse), the path waypoints
// are strictly centered in the travel lane (ZoneRoad) and never stray onto the roadside shoulder (ZoneShoulder).
func TestCoastalBaySouthCapeCausewayNoShoulderDriving(t *testing.T) {
	laneW := 52.0
	w := NewWorld(laneW, MapThemeCoastalBay)

	stop11 := w.Stops[11]
	if stop11.ID != 11 {
		t.Fatalf("Expected stop 11, got %d", stop11.ID)
	}

	// Verify all paths leading to Stop 11 have 0 shoulder points
	for i, sFrom := range w.Stops {
		if i == 11 {
			continue
		}
		path := GetRoadPathBetweenStopsInWorld(w, sFrom, stop11, laneW)
		for idx, pt := range path {
			if idx == 0 || idx == len(path)-1 {
				continue // skip starting / ending stop bay
			}
			bRes := w.CheckBoundaries(pt, 18.0, laneW)
			if bRes.Zone != ZoneRoad {
				t.Fatalf("From Stop %d (%s) to Stop 11: pt[%d]=(%.1f, %.1f) is in %v, not ZoneRoad! (Bus driving on shoulder)",
					i, sFrom.Name, idx, pt.X, pt.Y, bRes.Zone)
			}
		}
	}

	// Verify all paths leaving Stop 11 have 0 shoulder points
	for j, sTo := range w.Stops {
		if j == 11 {
			continue
		}
		path := GetRoadPathBetweenStopsInWorld(w, stop11, sTo, laneW)
		for idx, pt := range path {
			if idx == 0 || idx == len(path)-1 {
				continue
			}
			bRes := w.CheckBoundaries(pt, 18.0, laneW)
			if bRes.Zone != ZoneRoad {
				t.Fatalf("From Stop 11 to Stop %d (%s): pt[%d]=(%.1f, %.1f) is in %v, not ZoneRoad! (Bus driving on shoulder)",
					j, sTo.Name, idx, pt.X, pt.Y, bRes.Zone)
			}
		}
	}
}

// TestSixLaneCrossingFourLaneStopLineBehindCrosswalk verifies that at an intersection where a 6-lane road
// and a 4-lane road meet, the stop line on all approaches is positioned strictly behind the zebra crosswalk
// (further from the intersection center) and outside the crossing road corridor, preventing vehicles from
// waiting inside the intersection and causing collisions with left-turning buses.
func TestSixLaneCrossingFourLaneStopLineBehindCrosswalk(t *testing.T) {
	laneWidth := 52.0

	// 1. EW is 6 lanes, NS is 4 lanes
	inter := &TrafficIntersection{
		ID:              1,
		Center:          Vec2{X: 0, Y: 0},
		ApproachHasRoad: [4]bool{true, true, true, true},
		ApproachLanes:   [4]int{6, 6, 4, 4},
		ApproachIs4Lane: [4]bool{true, true, true, true},
		Phase:           PhaseNSGreen, // EW has Red light
	}

	ewStop, ewCross, nsStop, nsCross := inter.GetApproachDimensions(laneWidth)

	// Crossing NS road (4 lanes) half-width: 2 * 52 + 12 = 116.0
	nsRoadHalfWidth := laneWidth*2.0 + 12.0
	// Crossing EW road (6 lanes) half-width: 3 * 52 + 12 = 168.0
	ewRoadHalfWidth := laneWidth*3.0 + 12.0

	// For the 6-lane EW approach:
	// Crosswalk must be outside the 4-lane crossing road (>= 116.0px)
	if ewCross < nsRoadHalfWidth {
		t.Fatalf("EW 6-lane crosswalk (%.2f) must be outside NS 4-lane roadway (half-width %.2f)", ewCross, nsRoadHalfWidth)
	}
	// Stop line must be strictly BEHIND the crosswalk (ewStop > ewCross)
	if ewStop <= ewCross {
		t.Fatalf("EW 6-lane stop line (%.2f) must be placed behind the crosswalk (%.2f)", ewStop, ewCross)
	}
	// Stop line must provide adequate pedestrian buffer (at least 20px behind crosswalk)
	if ewStop-ewCross < 20.0 {
		t.Fatalf("EW 6-lane stop line (%.2f) too close to crosswalk (%.2f), delta=%.2f (expected >= 20)", ewStop, ewCross, ewStop-ewCross)
	}

	// For the 4-lane NS approach:
	// Crosswalk must be outside the 6-lane crossing road (>= 168.0px)
	if nsCross < ewRoadHalfWidth {
		t.Fatalf("NS 4-lane crosswalk (%.2f) must be outside EW 6-lane roadway (half-width %.2f)", nsCross, ewRoadHalfWidth)
	}
	// Stop line must be strictly BEHIND the crosswalk (nsStop > nsCross)
	if nsStop <= nsCross {
		t.Fatalf("NS 4-lane stop line (%.2f) must be placed behind the crosswalk (%.2f)", nsStop, nsCross)
	}
	if nsStop-nsCross < 20.0 {
		t.Fatalf("NS 4-lane stop line (%.2f) too close to crosswalk (%.2f), delta=%.2f (expected >= 20)", nsStop, nsCross, nsStop-nsCross)
	}

	// 2. Symmetric check: EW is 4 lanes, NS is 6 lanes
	interSym := &TrafficIntersection{
		ID:              2,
		Center:          Vec2{X: 0, Y: 0},
		ApproachHasRoad: [4]bool{true, true, true, true},
		ApproachLanes:   [4]int{4, 4, 6, 6},
		ApproachIs4Lane: [4]bool{true, true, true, true},
		Phase:           PhaseEWGreen,
	}
	sEWStop, sEWCross, sNSStop, sNSCross := interSym.GetApproachDimensions(laneWidth)
	if sEWCross < ewRoadHalfWidth || sEWStop <= sEWCross {
		t.Fatalf("Symmetric EW 4-lane approach: cross=%.2f, stop=%.2f, must clear 6-lane crossing road (%.2f)", sEWCross, sEWStop, ewRoadHalfWidth)
	}
	if sNSCross < nsRoadHalfWidth || sNSStop <= sNSCross {
		t.Fatalf("Symmetric NS 6-lane approach: cross=%.2f, stop=%.2f, must clear 4-lane crossing road (%.2f)", sNSCross, sNSStop, nsRoadHalfWidth)
	}

	// 3. Verify in real Metropolis map where 6-lane North Ring Avenue intersects 4-lane avenues
	w := NewWorld(laneWidth, MapThemeMetropolis)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)

	testedIntersections := 0
	for _, it := range tls.Intersections {
		ewL := math.Max(float64(it.ApproachLanes[0]), float64(it.ApproachLanes[1]))
		nsL := math.Max(float64(it.ApproachLanes[2]), float64(it.ApproachLanes[3]))
		if (ewL >= 6 && nsL >= 4) || (ewL >= 4 && nsL >= 6) {
			testedIntersections++
			eStop, eCross, nStop, nCross := it.GetApproachDimensions(laneWidth)

			// Ensure stop line is always strictly behind crosswalk
			if eStop <= eCross {
				t.Fatalf("Metropolis intersection %v: EW stop line (%.2f) not behind crosswalk (%.2f)", it.Center, eStop, eCross)
			}
			if nStop <= nCross {
				t.Fatalf("Metropolis intersection %v: NS stop line (%.2f) not behind crosswalk (%.2f)", it.Center, nStop, nCross)
			}

			// Ensure stop line is outside crossing road corridor
			crossWidthEW := laneWidth*2.0 + 12.0 // at least 4 lanes
			if nsL >= 6 {
				crossWidthEW = laneWidth*3.0 + 12.0
			}
			if eStop <= crossWidthEW {
				t.Fatalf("Metropolis intersection %v: EW stop line (%.2f) inside crossing road corridor (%.2f)", it.Center, eStop, crossWidthEW)
			}
		}
	}

	if testedIntersections == 0 {
		t.Fatal("Expected to find at least one 6-lane x 4-lane intersection in Metropolis world map")
	}
}

// TestPlanModePausesSimulationAndPassengerSpawning verifies that when planning routes (IsPlanMode = true),
// the entire game simulation (traffic lights, AI traffic, player bus motion, passenger arrivals at bus stops)
// is strictly paused, and no new passengers are automatically spawned.
func TestPlanModePausesSimulationAndPassengerSpawning(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(MapThemeMetropolis)

	// Ensure we start in Drive Mode
	if g.IsPlanMode {
		t.Fatal("Expected to start in Driver Mode")
	}

	// 1. In Driver Mode: Verify simulation ticks advance world & passenger timers
	initialSpawnTimers := make([]float64, len(g.World.Stops))
	initialPassengers := make([]int, len(g.World.Stops))
	for i, s := range g.World.Stops {
		initialSpawnTimers[i] = s.SpawnTimer
		initialPassengers[i] = s.WaitingPassengers
	}

	initialTLTimer := g.TrafficLights.Intersections[0].Timer

	// Run 10 ticks in Driver Mode
	for i := 0; i < 10; i++ {
		_ = g.Update()
	}

	// Verify timers advanced in Driver Mode
	advancedInDriver := false
	for i, s := range g.World.Stops {
		if s.SpawnTimer != initialSpawnTimers[i] {
			advancedInDriver = true
			break
		}
	}
	if !advancedInDriver {
		t.Fatal("Expected stop spawn timers to advance in Driver Mode")
	}
	if g.TrafficLights.Intersections[0].Timer == initialTLTimer {
		t.Fatal("Expected traffic light timer to advance in Driver Mode")
	}

	// 2. Enter Plan Mode: Verify simulation and passenger spawning are strictly paused
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be true after TogglePlanMode()")
	}

	// Record snapshot of all stops, traffic lights, and AI cars
	pausedSpawnTimers := make([]float64, len(g.World.Stops))
	pausedPassengers := make([]int, len(g.World.Stops))
	for i, s := range g.World.Stops {
		pausedSpawnTimers[i] = s.SpawnTimer
		pausedPassengers[i] = s.WaitingPassengers
	}

	pausedTLTimers := make([]float64, len(g.TrafficLights.Intersections))
	for i, it := range g.TrafficLights.Intersections {
		pausedTLTimers[i] = it.Timer
	}

	pausedCarPositions := make([]Vec2, len(g.TrafficMgr.Cars))
	for i, c := range g.TrafficMgr.Cars {
		pausedCarPositions[i] = c.Pos
	}

	// Run 100 ticks in Plan Mode (simulating 1.6+ seconds of player route planning)
	for i := 0; i < 100; i++ {
		_ = g.Update()
	}

	// Verify that passenger spawn timers did NOT advance at all
	for i, s := range g.World.Stops {
		if s.SpawnTimer != pausedSpawnTimers[i] {
			t.Fatalf("Stop #%d (%s) spawn timer advanced while in plan mode: got %.3f, expected %.3f",
				s.ID, s.Name, s.SpawnTimer, pausedSpawnTimers[i])
		}
		if s.WaitingPassengers != pausedPassengers[i] {
			t.Fatalf("Stop #%d (%s) waiting passengers changed while in plan mode: got %d, expected %d",
				s.ID, s.Name, s.WaitingPassengers, pausedPassengers[i])
		}
	}

	// Verify that traffic lights did NOT advance
	for i, it := range g.TrafficLights.Intersections {
		if it.Timer != pausedTLTimers[i] {
			t.Fatalf("Traffic intersection #%d timer advanced while in plan mode: got %.3f, expected %.3f",
				it.ID, it.Timer, pausedTLTimers[i])
		}
	}

	// Verify that civilian traffic cars did NOT move
	for i, c := range g.TrafficMgr.Cars {
		if c.Pos != pausedCarPositions[i] {
			t.Fatalf("Civilian car #%d moved while in plan mode: got %v, expected %v",
				c.ID, c.Pos, pausedCarPositions[i])
		}
	}

	// 3. Exit Plan Mode: Verify simulation resumes cleanly
	g.TogglePlanMode()
	if g.IsPlanMode {
		t.Fatal("Expected IsPlanMode to be false after TogglePlanMode()")
	}

	// Run 10 ticks in Driver Mode after resuming
	for i := 0; i < 10; i++ {
		_ = g.Update()
	}

	// Verify passenger spawn timers and traffic lights resume advancing
	resumedPassengerTimers := false
	for i, s := range g.World.Stops {
		if s.SpawnTimer != pausedSpawnTimers[i] {
			resumedPassengerTimers = true
			break
		}
	}
	if !resumedPassengerTimers {
		t.Fatal("Expected passenger spawn timers to resume advancing after exiting plan mode")
	}

	if g.TrafficLights.Intersections[0].Timer == pausedTLTimers[0] {
		t.Fatal("Expected traffic light timer to resume advancing after exiting plan mode")
	}
}

// TestRouteRemoveLastStopAndUndo verifies:
// 1. Route.RemoveLastStop removes the most recently added stop and adjusts CurrentIndex correctly.
// 2. UndoLastPlannedStop properly manages Route, marks planModeRouteModified, and notifies user via HUD.
// 3. ClearPlannedRoute properly clears all stops.
// 4. HUD interactive buttons for Plan Undo and Clear detect clicks correctly.
func TestRouteRemoveLastStopAndUndo(t *testing.T) {
	// 1. Direct Route method testing
	r := NewRoute()
	if _, ok := r.RemoveLastStop(); ok {
		t.Fatal("Expected RemoveLastStop on empty route to return false")
	}

	r.AddStop(0)
	r.AddStop(2)
	r.AddStop(10)
	if len(r.StopIDs) != 3 {
		t.Fatalf("Expected 3 stops, got %d", len(r.StopIDs))
	}

	r.CurrentIndex = 2 // Pointing to stop index 2 (stopID 10)
	removedID, ok := r.RemoveLastStop()
	if !ok || removedID != 10 {
		t.Fatalf("Expected removed stopID 10, got %d (ok=%v)", removedID, ok)
	}
	if len(r.StopIDs) != 2 || r.StopIDs[0] != 0 || r.StopIDs[1] != 2 {
		t.Fatalf("Expected stops [0, 2], got %v", r.StopIDs)
	}
	if r.CurrentIndex != 1 {
		t.Fatalf("Expected CurrentIndex clamped to 1, got %d", r.CurrentIndex)
	}

	// Remove remaining stops down to 0
	r.RemoveLastStop()
	r.RemoveLastStop()
	if len(r.StopIDs) != 0 || r.CurrentIndex != 0 {
		t.Fatalf("Expected 0 stops and CurrentIndex=0, got len=%d, idx=%d", len(r.StopIDs), r.CurrentIndex)
	}

	// 2. Game level UndoLastPlannedStop & ClearPlannedRoute testing
	g := NewGame(1280, 720)
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatal("Expected to enter Plan Mode")
	}

	g.Route.Clear()
	g.Route.AddStop(1)
	g.Route.AddStop(3)
	g.Route.AddStop(5)
	g.planModeRouteModified = false

	// Undo stop 5
	success := g.UndoLastPlannedStop()
	if !success {
		t.Fatal("Expected UndoLastPlannedStop to succeed")
	}
	if !g.planModeRouteModified {
		t.Fatal("Expected planModeRouteModified to be true after undo")
	}
	if len(g.Route.StopIDs) != 2 || g.Route.StopIDs[1] != 3 {
		t.Fatalf("Expected stops [1, 3], got %v", g.Route.StopIDs)
	}

	// Undo down to empty
	g.UndoLastPlannedStop()
	g.UndoLastPlannedStop()
	if len(g.Route.StopIDs) != 0 {
		t.Fatalf("Expected route to be empty, got %d stops", len(g.Route.StopIDs))
	}

	// Undo on empty route returns false
	if g.UndoLastPlannedStop() {
		t.Fatal("Expected UndoLastPlannedStop on empty route to return false")
	}

	// 3. ClearPlannedRoute testing
	g.Route.AddStop(1)
	g.Route.AddStop(2)
	g.ClearPlannedRoute()
	if len(g.Route.StopIDs) != 0 {
		t.Fatalf("Expected ClearPlannedRoute to empty route, got %d stops", len(g.Route.StopIDs))
	}

	// 4. HUD click handlers testing
	hud := NewHUD()
	hud.PlanUndoBtnBounds = Rect{X: 100, Y: 200, W: 140, H: 24}
	hud.PlanClearBtnBounds = Rect{X: 250, Y: 200, W: 100, H: 24}
	hud.PlanPanelBounds = Rect{X: 50, Y: 150, W: 400, H: 100}

	if !hud.HandlePlanUndoClick(Vec2{X: 150, Y: 210}) {
		t.Fatal("Expected HandlePlanUndoClick to return true for point inside bounds")
	}
	if hud.HandlePlanUndoClick(Vec2{X: 50, Y: 210}) {
		t.Fatal("Expected HandlePlanUndoClick to return false for point outside bounds")
	}
	if !hud.HandlePlanClearClick(Vec2{X: 280, Y: 210}) {
		t.Fatal("Expected HandlePlanClearClick to return true for point inside bounds")
	}
	if !hud.IsInsidePlanPanel(Vec2{X: 120, Y: 180}) {
		t.Fatal("Expected IsInsidePlanPanel to return true for point inside panel")
	}
}

