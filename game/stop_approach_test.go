package game

import (
	"math"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

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

// TestAIMissedStopOvershootPenalizesAndSkips verifies that when an AI-driven bus overshoots a mandatory
// bus stop (passengers waiting or alighting) without docking, it directly advances to the next stop and
// deducts a $50 fine. In manual driving mode, it does NOT automatically fine or skip.
func TestAIMissedStopOvershootPenalizesAndSkips(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying
	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]

	// 1. Setup mandatory stop with waiting passengers
	targetStop.WaitingPassengers = 4
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.Cash = 100
	g.Settings.TrafficFinesEnabled = true

	// Position bus 75px past the forward berth, out in the road travel lane
	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
	g.Bus.Pos = targetStop.FrontBerthPosition().Add(sim.Vec2{X: cosH * 75.0, Y: sinH * 75.0}).Add(toRoadDir.Mul(30.0))
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 20.0

	// Case A: Manual driving mode (AutopilotEnabled = false)
	g.Settings.AutopilotEnabled = false
	prevRouteIdx := g.Route.CurrentIndex
	_ = g.Update()

	if g.Route.CurrentIndex != prevRouteIdx {
		t.Fatalf("Expected manual mode NOT to auto-skip missed stop, got new index %d", g.Route.CurrentIndex)
	}
	if g.Cash != 100 {
		t.Fatalf("Expected manual mode NOT to fine for overshooting, got cash %d", g.Cash)
	}

	// Case B: AI Autopilot driving mode (AutopilotEnabled = true)
	g.Settings.AutopilotEnabled = true
	g.skipCooldown = 0.0
	_ = g.Update()

	if g.Cash != 50 {
		t.Fatalf("Expected AI missed stop to deduct $50 fine, got cash %d", g.Cash)
	}
	if g.Route.CurrentIndex == prevRouteIdx {
		t.Fatalf("Expected AI missed stop to advance route to next stop, remained at %d", g.Route.CurrentIndex)
	}

	// Verify toast notification mentions fine and missed stop
	foundFineToast := false
	for _, toast := range g.HUD.Toasts {
		if strings.Contains(toast.Text, "漏站罚款 -$50") || strings.Contains(toast.Text, "Missed Stop Fine -$50") {
			foundFineToast = true
			break
		}
	}
	if !foundFineToast {
		t.Fatalf("Expected warning toast for missed stop fine, got toasts: %+v", g.HUD.Toasts)
	}
}

// TestAIMissedStopOppositeRoadNoFalseOvershoot verifies that when driving past on the opposite side of
// the road (e.g. oncoming lanes or parallel roads) or driving in the opposite direction, the game never
// prematurely identifies it as overshooting or skipping the bus stop.
func TestAIMissedStopOppositeRoadNoFalseOvershoot(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying
	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]

	// Target stop with waiting passengers
	targetStop.WaitingPassengers = 4
	g.stopBellRung = false
	g.Cash = 100
	g.Settings.TrafficFinesEnabled = true
	g.Settings.AutopilotEnabled = true
	g.skipCooldown = 0.0

	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	perpX := -sinH
	perpY := cosH

	// Scenario 1: Bus driving in the OPPOSITE direction along oncoming lanes of the road
	// Heading is flipped by Pi, position has forwardProj > 55.0 and distToTarget < 350.0
	oppHeading := sim.NormalizeAngle(targetStop.Heading + math.Pi)
	g.Bus.Heading = oppHeading
	g.Bus.Speed = 25.0
	// Position 80px ahead along stop heading, but on the opposite side of RoadCenter (sideDist < -25)
	curbDist := targetStop.Pos.Distance(targetStop.RoadCenter)
	g.Bus.Pos = targetStop.RoadCenter.Add(sim.Vec2{X: cosH * 80.0, Y: sinH * 80.0}).Add(sim.Vec2{X: perpX * (-curbDist * 0.7), Y: perpY * (-curbDist * 0.7)})

	prevRouteIdx := g.Route.CurrentIndex
	_ = g.Update()

	if g.Route.CurrentIndex != prevRouteIdx {
		t.Fatalf("Driving past on opposite road should NOT skip stop! Changed from %d to %d", prevRouteIdx, g.Route.CurrentIndex)
	}
	if g.Cash != 100 {
		t.Fatalf("Driving past on opposite road should NOT fine $50! Cash is %d", g.Cash)
	}

	// Verify no fine toast was posted
	for _, toast := range g.HUD.Toasts {
		if strings.Contains(toast.Text, "漏站罚款") || strings.Contains(toast.Text, "Missed Stop") {
			t.Fatalf("Unexpected missed stop toast posted when on opposite road: %s", toast.Text)
		}
	}

	// Scenario 2: Empty station (canSkip == true), driving in opposite direction on opposite road
	targetStop.WaitingPassengers = 0
	g.skipCooldown = 0.0
	_ = g.Update()

	if g.Route.CurrentIndex != prevRouteIdx {
		t.Fatalf("Driving past empty station on opposite road should NOT trigger drive-by fast skip! Index: %d", g.Route.CurrentIndex)
	}

	// Scenario 3: Station occupied by another bus (isOccupied == true), bus on opposite road within 380px
	targetStop.WaitingPassengers = 3
	g.skipCooldown = 0.0
	// Add an AI bus docked in the target stop bay
	aiBus := sim.NewBus(targetStop.Pos.X, targetStop.Pos.Y, targetStop.Heading)
	aiBus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{{Bus: aiBus}}

	if !g.IsStopOccupiedByOtherVehicle(targetStop) {
		t.Fatalf("Expected target stop to be recognized as occupied by docked AI bus")
	}

	_ = g.Update()
	if g.Route.CurrentIndex != prevRouteIdx {
		t.Fatalf("Bus on opposite road should NOT trigger proactive AI skip for occupied stop! Index: %d", g.Route.CurrentIndex)
	}

	// Scenario 4: Autopilot has a planned path to targetStop but is on an earlier route leg (LastRemaining > 220px)
	g.AutopilotDriver.PrepareLeg(g.World, g.World.Stops[g.Route.StopIDs[(g.Route.CurrentIndex-1+len(g.Route.StopIDs))%len(g.Route.StopIDs)]], targetStop, g.Settings.GetLaneWidth())
	g.AutopilotDriver.LastRemaining = 500.0 // 500px away along planned route
	// Even if bus heading were temporarily aligned, route remaining > 220 blocks premature skip
	g.Bus.Heading = targetStop.Heading
	_ = g.Update()

	if g.Route.CurrentIndex != prevRouteIdx {
		t.Fatalf("Bus on earlier route leg (LastRemaining=500) should NOT trigger skip! Index: %d", g.Route.CurrentIndex)
	}
	if g.Cash != 100 {
		t.Fatalf("Bus on earlier route leg should NOT be fined! Cash: %d", g.Cash)
	}

	// Scenario 5: Legitimate overshoot after overtaking:
	// Bus is on correct road side, heading aligned, route arriving (LastRemaining <= 220), past the stop by 75px
	g.AIBusMgr.Buses = nil // Clear occupying bus
	targetStop.WaitingPassengers = 4
	g.AutopilotDriver.LastRemaining = 0.0
	g.Bus.Heading = targetStop.Heading
	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
	g.Bus.Pos = targetStop.FrontBerthPosition().Add(sim.Vec2{X: cosH * 75.0, Y: sinH * 75.0}).Add(toRoadDir.Mul(30.0))
	g.skipCooldown = 0.0

	_ = g.Update()

	if g.Cash != 50 {
		t.Fatalf("Legitimate overshoot should deduct $50 fine! Cash is %d", g.Cash)
	}
	if g.Route.CurrentIndex == prevRouteIdx {
		t.Fatalf("Legitimate overshoot should advance route! Remained at %d", g.Route.CurrentIndex)
	}

	initialPos := g.Bus.Pos
	for frame := 0; frame < 60; frame++ {
		_ = g.Update()
	}
	if g.Bus.Speed <= 1.0 {
		t.Fatalf("Bus is stopped on the road after missed stop! Speed: %.2f, Pos: %+v (was %+v)", g.Bus.Speed, g.Bus.Pos, initialPos)
	}
}

// TestAIMissedStopCirclingNearStopBreaksOutAndAdvances verifies that when an autopilot bus
// overshoots or spins out near a station bay and faces sideways or backwards (e.g. attempting to loop/donut),
// it immediately triggers missed-stop breakout, advances to the next stop, and resumes driving forward along the road.
func TestAIMissedStopCirclingNearStopBreaksOutAndAdvances(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying
	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]

	targetStop.WaitingPassengers = 4
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.Cash = 100
	g.Settings.TrafficFinesEnabled = true
	g.Settings.AutopilotEnabled = true
	g.skipCooldown = 0.0

	// Set up the autopilot as having reached the destination stop
	g.AutopilotDriver.PrepareLeg(g.World, g.World.Stops[g.Route.StopIDs[(g.Route.CurrentIndex-1+len(g.Route.StopIDs))%len(g.Route.StopIDs)]], targetStop, g.Settings.GetLaneWidth())
	g.AutopilotDriver.LastRemaining = 10.0

	// Position the bus just past the stop along targetStop.Heading (forwardProj = 25px) on the correct road side
	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
	g.Bus.Pos = targetStop.FrontBerthPosition().Add(sim.Vec2{X: cosH * 25.0, Y: sinH * 25.0}).Add(toRoadDir.Mul(30.0))

	// The bus is angled sideways/backwards towards the bay (simulating the start of a circling donut or spinout)
	g.Bus.Heading = sim.NormalizeAngle(targetStop.Heading + math.Pi*0.6) // cosHeading < 0
	g.Bus.Speed = 15.0

	prevRouteIdx := g.Route.CurrentIndex
	_ = g.Update()

	if g.Route.CurrentIndex == prevRouteIdx {
		t.Fatalf("Expected circling near stop to break out and advance route, but remained at index %d", prevRouteIdx)
	}
	if g.Cash != 50 {
		t.Fatalf("Expected $50 fine for missing stop due to circling, got Cash=%d", g.Cash)
	}
	if g.AutopilotDriver.State != autopilot.AutopilotStateCruising {
		t.Fatalf("Expected AutopilotDriver to reset to Cruising state, got %v", g.AutopilotDriver.State)
	}

	// Verify the bus drives forward and does not get stuck in a full-lock donut
	for frame := 0; frame < 60; frame++ {
		_ = g.Update()
	}
	if math.Abs(g.Bus.SteeringAngle) >= g.Bus.MaxSteerAngle*0.95 && g.Bus.Speed < 2.0 {
		t.Fatalf("Bus is stuck in full lock steering after breakout! Steering=%.2f, Speed=%.2f", g.Bus.SteeringAngle, g.Bus.Speed)
	}
}

// TestAIAutopilotProactiveSkipInBerserkAndStandardMode verifies that whenever skip conditions are met
// (green prompt showing: empty station with no alighting, or occupied station with no alighting),
// AI mode automatically and proactively skips the station without stopping or docking, in BOTH
// Berserk mode and Standard mode.
func TestAIAutopilotProactiveSkipInBerserkAndStandardMode(t *testing.T) {
	for _, berserk := range []bool{true, false} {
		modeName := "Standard"
		if berserk {
			modeName = "Berserk"
		}

		// 1. Empty station with no alighting passengers -> AI must skip automatically
		t.Run(modeName+"_EmptyStationAutoSkip", func(t *testing.T) {
			g := NewGame(1280, 720)
			g.State = StatePlaying
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = berserk
			g.skipCooldown = 0.0

			targetStopID := g.Route.GetCurrentTargetStopID()
			targetStop := g.World.Stops[targetStopID]
			targetStop.WaitingPassengers = 0
			g.stopBellRung = false
			g.passengersAlighting = 0

			// Position bus 300px before the stop along heading, on the stop side of the road
			cosH := math.Cos(targetStop.Heading)
			sinH := math.Sin(targetStop.Heading)
			toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
			toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
			g.Bus.Pos = targetStop.FrontBerthPosition().Sub(sim.Vec2{X: cosH * 300.0, Y: sinH * 300.0}).Add(toRoadDir.Mul(20.0))
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = 60.0

			prevRouteIdx := g.Route.CurrentIndex
			_ = g.Update()

			if g.Route.CurrentIndex == prevRouteIdx {
				t.Fatalf("[%s] AI mode failed to proactively skip empty station! Index remained at %d", modeName, g.Route.CurrentIndex)
			}

			// Verify toast
			foundToast := false
			for _, toast := range g.HUD.Toasts {
				if strings.Contains(toast.Text, "AI自主飞站") || strings.Contains(toast.Text, "AI Skip") {
					foundToast = true
					break
				}
			}
			if !foundToast {
				t.Fatalf("[%s] Expected AI proactive skip toast, got toasts: %+v", modeName, g.HUD.Toasts)
			}
		})

		// 2. Station occupied by another vehicle with no alighting passengers -> AI must skip automatically
		t.Run(modeName+"_OccupiedStationAutoSkip", func(t *testing.T) {
			g := NewGame(1280, 720)
			g.State = StatePlaying
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = berserk
			g.skipCooldown = 0.0

			targetStopID := g.Route.GetCurrentTargetStopID()
			targetStop := g.World.Stops[targetStopID]
			targetStop.WaitingPassengers = 5 // Passengers waiting, but station occupied!
			g.stopBellRung = false
			g.passengersAlighting = 0

			// Dock an AI bus in the bay
			aiBus := sim.NewBus(targetStop.Pos.X, targetStop.Pos.Y, targetStop.Heading)
			aiBus.Speed = 0.0
			g.AIBusMgr.Buses = []*traffic.AIBus{{Bus: aiBus}}

			cosH := math.Cos(targetStop.Heading)
			sinH := math.Sin(targetStop.Heading)
			toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
			toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
			// Position in the passing lane (e.g. 50px towards road) 300px before the stop
			g.Bus.Pos = targetStop.FrontBerthPosition().Sub(sim.Vec2{X: cosH * 300.0, Y: sinH * 300.0}).Add(toRoadDir.Mul(50.0))
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = 60.0

			prevRouteIdx := g.Route.CurrentIndex
			_ = g.Update()

			if g.Route.CurrentIndex == prevRouteIdx {
				t.Fatalf("[%s] AI mode failed to proactively skip occupied station! Index remained at %d", modeName, g.Route.CurrentIndex)
			}

			foundToast := false
			for _, toast := range g.HUD.Toasts {
				if strings.Contains(toast.Text, "AI自主飞站") || strings.Contains(toast.Text, "AI Skip") {
					foundToast = true
					break
				}
			}
			if !foundToast {
				t.Fatalf("[%s] Expected AI proactive skip toast for occupied station, got: %+v", modeName, g.HUD.Toasts)
			}
		})

		// 3. Mandatory stop: Station has waiting passengers and is NOT occupied -> AI must NOT skip
		t.Run(modeName+"_MandatoryStopWaitingPaxNeverSkipped", func(t *testing.T) {
			g := NewGame(1280, 720)
			g.State = StatePlaying
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = berserk
			g.skipCooldown = 0.0

			targetStopID := g.Route.GetCurrentTargetStopID()
			targetStop := g.World.Stops[targetStopID]
			targetStop.WaitingPassengers = 5
			g.stopBellRung = false
			g.AIBusMgr.Buses = nil

			cosH := math.Cos(targetStop.Heading)
			sinH := math.Sin(targetStop.Heading)
			toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
			toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
			g.Bus.Pos = targetStop.FrontBerthPosition().Sub(sim.Vec2{X: cosH * 300.0, Y: sinH * 300.0}).Add(toRoadDir.Mul(20.0))
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = 60.0

			prevRouteIdx := g.Route.CurrentIndex
			_ = g.Update()

			if g.Route.CurrentIndex != prevRouteIdx {
				t.Fatalf("[%s] AI mode should NEVER skip mandatory stop with waiting passengers! Changed from %d to %d", modeName, prevRouteIdx, g.Route.CurrentIndex)
			}
		})

		// 4. Mandatory stop: Passenger pressed stop bell -> AI must NOT skip
		t.Run(modeName+"_MandatoryStopAlightingNeverSkipped", func(t *testing.T) {
			g := NewGame(1280, 720)
			g.State = StatePlaying
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = berserk
			g.skipCooldown = 0.0

			targetStopID := g.Route.GetCurrentTargetStopID()
			targetStop := g.World.Stops[targetStopID]
			targetStop.WaitingPassengers = 0
			g.stopBellRung = true
			g.passengersAlighting = 2

			cosH := math.Cos(targetStop.Heading)
			sinH := math.Sin(targetStop.Heading)
			toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
			toRoadDir := toRoad.Mul(1.0 / toRoad.Length())
			g.Bus.Pos = targetStop.FrontBerthPosition().Sub(sim.Vec2{X: cosH * 300.0, Y: sinH * 300.0}).Add(toRoadDir.Mul(20.0))
			g.Bus.Heading = targetStop.Heading
			g.Bus.Speed = 60.0

			prevRouteIdx := g.Route.CurrentIndex
			_ = g.Update()

			if g.Route.CurrentIndex != prevRouteIdx {
				t.Fatalf("[%s] AI mode should NEVER skip stop when passengers are alighting! Changed from %d to %d", modeName, prevRouteIdx, g.Route.CurrentIndex)
			}
		})
	}
}

func TestAIMissedStopStoppedOutsideBayResumesDriving(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = false
	g.AutopilotDriver.BerserkMode = false
	g.Settings.TrafficFinesEnabled = true

	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]
	targetStop.WaitingPassengers = 4
	g.stopBellRung = true
	g.passengersAlighting = 2
	g.Bus.Passengers = append(g.Bus.Passengers, sim.Passenger{TargetStopID: targetStopID})

	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	laneW := g.Settings.GetLaneWidth()

	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())

	// Place an AI bus docked in the station bay
	frontAIBus := sim.NewBus(targetStop.Pos.X, targetStop.Pos.Y, targetStop.Heading)
	frontAIBus.Speed = 0.0
	g.AIBusMgr.Buses = []*traffic.AIBus{{ID: 1, Bus: frontAIBus}}

	// Player bus is stopped alongside the station waiting in passing lane (-35px along heading, LaneOffset=-laneW, Speed=0)
	g.Bus.Pos = targetStop.Pos.Add(sim.Vec2{X: cosH * -35.0, Y: sinH * -35.0}).Add(toRoadDir.Mul(laneW))
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 0.0
	g.AutopilotDriver.LaneOffset = -laneW
	g.Cash = 100

	// 1. Update: Must NOT be falsely penalized or skipped while waiting alongside! (Point 2)
	_ = g.Update()

	if g.Route.GetCurrentTargetStopID() != targetStopID {
		t.Fatalf("Player bus was wrongfully skipped to next stop while waiting for front bus!")
	}
	if g.Cash != 100 {
		t.Fatalf("Player bus was wrongfully fined -$50 while waiting for front bus! Cash=%d", g.Cash)
	}

	// 2. Clear the front AI bus (simulating front AI bus departing or clearing)
	g.AIBusMgr.Buses = nil

	g.HUD.Toasts = nil
	docked := false
	for frame := 0; frame < 300; frame++ {
		if frame%10 == 0 || g.AutopilotDriver.State == autopilot.AutopilotStateDocked {
			t.Logf("frame %d: pos=%v spd=%.2f offset=%.2f targetOffset=%.2f state=%v",
				frame, g.Bus.Pos, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset, g.AutopilotDriver.State)
		}
		_ = g.Update()
		if targetStop.ContainsBus(g.Bus) || g.AutopilotDriver.State == autopilot.AutopilotStateDocked {
			docked = true
			break
		}
	}

	if !docked {
		toasts := []string{}
		for _, toast := range g.HUD.Toasts {
			toasts = append(toasts, toast.Text)
		}
		t.Fatalf("Player bus failed to dock after front bus cleared! Pos=%v, TargetStopPos=%v, LaneOffset=%.2f, State=%v, Toasts=%v",
			g.Bus.Pos, targetStop.Pos, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.State, toasts)
	}
	if g.Cash < 100 {
		t.Fatalf("Player bus should not be fined when successfully docking, got Cash=%d", g.Cash)
	}
}

// TestAIMissedStopFrontOvershotStalledInOuterLaneFliesPast verifies that when an AI bus in outer/passing lane
// has its reference point past the forward berth (e.g. forwardProj = 30px) and comes to a crawl/stop
// while unable to dock into the curb bay, it directly triggers fly-past, advances to next stop, and resumes driving.
func TestAIMissedStopFrontOvershotStalledInOuterLaneFliesPast(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying

	targetStopID := g.Route.GetCurrentTargetStopID()
	targetStop := g.World.Stops[targetStopID]

	// Mandatory stop: waiting passengers and alighting passengers
	targetStop.WaitingPassengers = 2
	g.stopBellRung = true
	g.passengersAlighting = 2
	g.Bus.Passengers = append(g.Bus.Passengers, sim.Passenger{TargetStopID: targetStopID})
	g.Cash = 100
	g.Settings.TrafficFinesEnabled = true
	g.Settings.AutopilotEnabled = true
	g.AutopilotDriver.BerserkMode = true

	cosH := math.Cos(targetStop.Heading)
	sinH := math.Sin(targetStop.Heading)
	laneW := g.Settings.GetLaneWidth()

	toRoad := targetStop.RoadCenter.Sub(targetStop.Pos)
	toRoadDir := toRoad.Mul(1.0 / toRoad.Length())

	// Bus is in outer/passing lane, 30px beyond the forward berth marker.
	g.Bus.Pos = targetStop.FrontBerthPosition().Add(sim.Vec2{X: cosH * 30.0, Y: sinH * 30.0}).Add(toRoadDir.Mul(laneW * 2.0))
	g.Bus.Heading = targetStop.Heading
	g.Bus.Speed = 0.0
	g.Bus.IsBraking = true
	g.AutopilotDriver.LaneOffset = -laneW

	// 1. Update frame: Must trigger fly-past (missed stop with -$50 fine)
	_ = g.Update()

	if g.Cash != 50 {
		t.Fatalf("Expected -$50 fine for overshot stop in outer lane, got cash: %d", g.Cash)
	}
	if g.Route.GetCurrentTargetStopID() == targetStopID {
		t.Fatalf("Expected route to advance to next stop after front overshot in outer lane")
	}

	// 2. Bus must not remain locked on brakes; autopilot state must become cruising
	if g.AutopilotDriver.State != autopilot.AutopilotStateCruising {
		t.Fatalf("Expected autopilot state to be Cruising, got: %v", g.AutopilotDriver.State)
	}

	// 3. Run further frames: Bus must accelerate away and not freeze at 0 km/h
	for frame := 0; frame < 60; frame++ {
		_ = g.Update()
	}
	if g.Bus.Speed < 10.0 {
		t.Fatalf("Expected bus to accelerate away towards next stop, speed is still %.2f", g.Bus.Speed)
	}
}
