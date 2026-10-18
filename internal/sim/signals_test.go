package sim

import (
	"math"
	"testing"
)

func TestTrafficLightApproachingSignalInfoAndStopDecision(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)
	if tls == nil || len(tls.Intersections) == 0 {
		t.Fatalf("Metropolis map must have traffic intersections")
	}

	inter := tls.Intersections[0]
	// Position a vehicle approaching from the west heading East (heading = 0)
	stopLineDist := 52.0*1.6 + 18.0
	// Outer through lane is at Y = inter.Center.Y + 52.0 * 1.5
	throughApproachPos := Vec2{X: inter.Center.X - stopLineDist - 60.0, Y: inter.Center.Y + 52.0*1.5}
	// Inner left-turn lane is at Y = inter.Center.Y + 52.0 * 0.5
	leftApproachPos := Vec2{X: inter.Center.X - stopLineDist - 60.0, Y: inter.Center.Y + 52.0*0.5}

	// 1. Signal Info Query
	hasInter, thruSig, leftSig, isMajor, remSec, distToLine := tls.GetApproachingSignalInfo(throughApproachPos, 0, 52.0)
	if !hasInter {
		t.Fatalf("Expected intersection detection for approachPos %v heading East towards %v", throughApproachPos, inter.Center)
	}
	if distToLine <= 0 || distToLine > 100.0 {
		t.Errorf("Unexpected distToLine: %.2f", distToLine)
	}
	if remSec < 0 {
		t.Errorf("Remaining seconds should be non-negative, got %.2f", remSec)
	}
	_ = thruSig
	_ = leftSig
	_ = isMajor

	// Distant position should not detect approaching intersection
	farPos := Vec2{X: inter.Center.X - 1000.0, Y: inter.Center.Y}
	hasFar, _, _, _, _, _ := tls.GetApproachingSignalInfo(farPos, 0, 52.0)
	if hasFar {
		t.Errorf("Distant position %v should not detect approaching intersection", farPos)
	}

	// 2. Stop Decision under Green vs Red phases
	// In PhaseEWGreen: Through traffic gets Green, Left-turn gets Red
	inter.Phase = PhaseEWGreen
	inter.Timer = 1.0
	inter.PlayerInside = false
	inter.FinedThisPassage = false

	stopGreen, _ := tls.CheckStopForVehicle(throughApproachPos, 0, 120.0, 52.0)
	if stopGreen {
		t.Errorf("Vehicle should not stop under green phase")
	}

	// Dedicated left-turn lane under PhaseEWGreen gets Red signal
	stopLeftDuringThruGreen, _ := tls.CheckStopForVehicle(leftApproachPos, 0, 120.0, 52.0)
	if !stopLeftDuringThruGreen {
		t.Errorf("Vehicle in dedicated left lane should stop under red left arrow")
	}

	// Dedicated left-turn lane under PhaseEWLeftGreen gets Green signal
	inter.Phase = PhaseEWLeftGreen
	inter.Timer = 1.0
	stopLeftGreen, _ := tls.CheckStopForVehicle(leftApproachPos, 0, 120.0, 52.0)
	if stopLeftGreen {
		t.Errorf("Vehicle in dedicated left lane should not stop under left green arrow")
	}

	// Set Phase to North-South Green (East-West is Red)
	inter.Phase = PhaseNSGreen
	inter.Timer = 1.0
	stopRed, distRed := tls.CheckStopForVehicle(throughApproachPos, 0, 120.0, 52.0)
	if !stopRed {
		t.Errorf("Vehicle should stop under red phase")
	}
	if distRed <= 0 || distRed > 120.0 {
		t.Errorf("Expected positive distance to stop line under red, got %.2f", distRed)
	}

	// Wrapper method ShouldVehicleStop
	if !tls.ShouldVehicleStop(throughApproachPos, 0, 120.0) {
		t.Errorf("ShouldVehicleStop should return true under red phase")
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
	bayInter.Phase = PhaseEWGreen                     // NS is RED!
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
