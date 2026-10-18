package game

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestRoundaboutCarsYieldToCirculatingBusAndAutopilotNoFines tests that:
// 1. A civilian traffic car approaching a roundabout yields and stops behind the yield line while a bus is circulating.
// 2. In Autopilot mode, any collision contact is zero-fine protected.
// 3. Roundabout collisions with entering vehicles do not fine the circulating bus.
func TestRoundaboutCarsYieldToCirculatingBusAndAutopilotNoFines(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.TrafficFinesEnabled = true
	g.Settings.AutopilotEnabled = true
	w := g.World
	rb := w.Roundabouts[0]

	// Bus circulating inside roundabout at radius 214px
	g.Bus.Pos = sim.Vec2{X: rb.Center.X + 214.0, Y: rb.Center.Y - 20.0}
	g.Bus.Heading = -math.Pi / 2
	g.Bus.Speed = 40.0

	// Civilian car approaching from East road
	car := &sim.TrafficCar{
		Pos:         sim.Vec2{X: rb.Center.X + 300.0, Y: rb.Center.Y - 100.8},
		Heading:     math.Pi, // Westbound
		Speed:       80.0,
		TargetSpeed: 80.0,
		Length:      48.0,
		Width:       22.0,
		IsEW:        true,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{car}

	dt := 1.0 / 60.0
	// Simulate 120 frames: car should yield behind yield line (distToCenter > rb.Radius + 15)
	for i := 0; i < 120; i++ {
		g.TrafficMgr.StepCar(dt, car, 0, g.Bus, nil, 1, 52.0, w)
	}

	distToCenter := car.Pos.Distance(rb.Center)
	if distToCenter < rb.Radius+15.0 {
		t.Fatalf("Car failed to yield at roundabout entrance! Distance to center = %.2f, expected >= %.2f",
			distToCenter, rb.Radius+15.0)
	}
	if car.Speed > 10.0 {
		t.Fatalf("Car should be stopped/braking at yield line, got speed = %.2f", car.Speed)
	}

	// 2. Autopilot mode collision protection test: force collision contact
	initialCash := g.Cash
	g.Bus.CrashCooldown = 0
	car.Pos = g.Bus.Pos.Add(sim.Vec2{X: 10.0, Y: -10.0}) // Direct contact
	g.checkBusCarCollisions()

	if g.Cash != initialCash {
		t.Fatalf("Autopilot mode collision must NOT deduct fine! Expected %d, got %d", initialCash, g.Cash)
	}
}

func TestBusCarCollisionSeparationAndNoOverlap(t *testing.T) {
	g := NewGame(1280, 720)

	// Set up player bus and target car on an East-West road segment
	// Bus length = 96 (front bumper at +48 from center), Width = 36 (half-width 18)
	// Car length = 46~50 (rear bumper at -24 from center), Width = 22 (half-width 11)
	g.Bus.Pos = sim.Vec2{X: 100, Y: 0}
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
	targetCar.Pos = sim.Vec2{X: 155, Y: 0}

	// 1. Verify SAT detects the collision before resolution
	busOBB := sim.OBB{
		Center:  g.Bus.Pos,
		HalfLen: g.Bus.Length / 2,
		HalfWid: g.Bus.Width / 2,
		Heading: g.Bus.Heading,
	}
	carOBB := sim.OBB{
		Center:  targetCar.Pos,
		HalfLen: targetCar.Length / 2,
		HalfWid: targetCar.Width / 2,
		Heading: targetCar.Heading,
	}

	collides, _, depth := sim.CheckOBBCollision(busOBB, carOBB)
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
	collidesAfter, _, depthAfter := sim.CheckOBBCollision(busOBB, carOBB)
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
	g.Bus.Pos = sim.Vec2{X: targetCar.Pos.X - 60, Y: 0} // Overlapping moving car
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
	g.Bus.Pos = sim.Vec2{X: 500, Y: 63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 0
	g.Bus.Length = 96.0
	g.Bus.Width = 36.0

	if sim.IsBusFacingWrongWay(g.Bus) {
		t.Fatalf("Bus on South side (Y=63) facing East must NOT be considered wrong-way")
	}

	approachingCar := &sim.TrafficCar{
		Pos:         sim.Vec2{X: 250, Y: 63.0}, // 250px away
		Heading:     0,
		Speed:       120.0,
		TargetSpeed: 120.0,
		Length:      48.0,
		Width:       22.0,
		IsEW:        true,
		RoadCenter:  0,
		LaneIndex:   1,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{approachingCar}

	dt := 1.0 / 60.0
	// Simulate 4.0 seconds (240 frames) of traffic AI
	for frame := 0; frame < 240; frame++ {
		g.TrafficMgr.StepCar(dt, approachingCar, 0, g.Bus, g.TrafficLights, 1, 42.0, g.World)
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
	g.Bus.Pos = sim.Vec2{X: 500, Y: 63.0}
	g.Bus.Speed = 0

	// Force car into collision contact from behind
	approachingCar.Pos = sim.Vec2{X: 435, Y: 63.0} // overlapping
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
	g.Bus.Pos = sim.Vec2{X: 500, Y: -63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 30.0
	g.Bus.CrashCooldown = 0

	if !sim.IsBusFacingWrongWay(g.Bus) {
		t.Fatalf("Bus facing East on North side (Y=-63) MUST be identified as wrong-way (逆行)")
	}

	oncomingCar := &sim.TrafficCar{
		Pos:     sim.Vec2{X: 540, Y: -63.0},
		Heading: math.Pi,
		Speed:   40.0,
		Length:  48.0,
		Width:   22.0,
		IsEW:    true,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{oncomingCar}
	cashBeforeWrongWay := g.Cash

	g.checkBusCarCollisions()

	if g.Cash != cashBeforeWrongWay-50 {
		t.Fatalf("Wrong-way collision must deduct $50 fine! Expected %d, got %d",
			cashBeforeWrongWay-50, g.Cash)
	}

	// 4. Bus actively front-ramming stationary car at speed MUST be penalized
	g.Bus.Pos = sim.Vec2{X: 200, Y: 63.0}
	g.Bus.Heading = 0
	g.Bus.Speed = 45.0 // High speed front impact
	g.Bus.CrashCooldown = 0

	victimCar := &sim.TrafficCar{
		Pos:     sim.Vec2{X: 260, Y: 63.0},
		Heading: 0,
		Speed:   0.0,
		Length:  48.0,
		Width:   22.0,
		IsEW:    true,
	}
	g.TrafficMgr.Cars = []*sim.TrafficCar{victimCar}
	cashBeforeFrontRam := g.Cash

	g.checkBusCarCollisions()

	if g.Cash != cashBeforeFrontRam-50 {
		t.Fatalf("Active front-ramming by bus must deduct $50 fine! Expected %d, got %d",
			cashBeforeFrontRam-50, g.Cash)
	}
}
