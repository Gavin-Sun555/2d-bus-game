package sim

import (
	"math"
	"strings"
	"testing"
)

func TestRoadCameraViolationsComprehensive(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)

	// Ensure there is at least one road camera on Metropolis
	if len(tls.RoadCameras) == 0 {
		t.Fatalf("Metropolis map should have road cameras")
	}

	cam := tls.RoadCameras[0]
	cam.Cooldown = 0

	bus := NewBus(cam.Pos.X, cam.Pos.Y, 0)
	bus.Speed = 40.0 // Driving speed

	if cam.IsEW {
		// South side of centerline (Y = cam.RoadCoord + 30): legal flow is East (heading = 0)
		bus.Pos = Vec2{X: cam.Pos.X, Y: cam.RoadCoord + 30.0}
		bus.Heading = 0 // Eastbound (legal)
		violated, _ := tls.CheckRoadCameraViolation(bus)
		if violated {
			t.Errorf("Lawful driving on right-hand side should not trigger road camera violation")
		}

		// Driving Westbound on the South side is wrong-way (逆行)!
		bus.Heading = math.Pi // Westbound (wrong-way on south side!)
		violated, notice := tls.CheckRoadCameraViolation(bus)
		if !violated {
			t.Errorf("Wrong-way driving past camera should trigger violation")
		}
		if notice == "" {
			t.Errorf("Expected violation notice message")
		}
		if cam.Cooldown <= 0 {
			t.Errorf("Camera cooldown should be set after violation")
		}

		// Subsequent check during cooldown should not fine again
		violatedCooldown, _ := tls.CheckRoadCameraViolation(bus)
		if violatedCooldown {
			t.Errorf("Camera on cooldown should not issue duplicate violation")
		}
	}

	// Bus stopped should not be fined as wrong-way driving
	cam.Cooldown = 0
	bus.Speed = 0
	bus.Heading = math.Pi
	violatedStopped, _ := tls.CheckRoadCameraViolation(bus)
	if violatedStopped {
		t.Errorf("Stopped vehicle should not be fined by road camera")
	}
}

func TestIsBusFacingWrongWayOnRoadways(t *testing.T) {
	// Bus in intersection is exempt
	interBus := NewBus(0, 0, math.Pi)
	if IsBusFacingWrongWay(interBus) {
		t.Errorf("Bus inside intersection should not be considered wrong-way")
	}

	// Bus on East-West road Y = 0 (away from intersection):
	// South side (Y = 50): legal flow East (heading 0), wrong-way West (heading Pi)
	legalBus := NewBus(600, 50, 0)
	legalBus.Speed = 30
	if IsBusFacingWrongWay(legalBus) {
		t.Errorf("Eastbound bus on south side should not be wrong-way")
	}

	wrongBus := NewBus(600, 50, math.Pi)
	wrongBus.Speed = 30
	if !IsBusFacingWrongWay(wrongBus) {
		t.Errorf("Westbound bus on south side must be detected as wrong-way")
	}

	// Reversing bus: heading is Pi but moving backwards (Speed < -1): effective direction is East!
	reversingBus := NewBus(600, 50, math.Pi)
	reversingBus.Speed = -20
	if IsBusFacingWrongWay(reversingBus) {
		t.Errorf("Bus reversing eastward on south side should not be wrong-way")
	}

	// Nil bus safety
	if IsBusFacingWrongWay(nil) {
		t.Errorf("Nil bus should return false")
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
