package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestHeadonConflictDetection(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 4, Type: 0},
		},
	}

	ad := NewAutopilotDriver()

	// 1. Two buses head-to-head facing each other (Ego Westbound, Other Eastbound)
	ego := sim.NewBus(50.0, 0.0, math.Pi) // Heading West (pi)
	other := sim.NewBus(-50.0, 0.0, 0.0)  // Heading East (0)
	other.Speed = 0

	f := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(other)},
		speedAbs:  0,
	}

	hasConflict, _ := ad.checkHeadonConflict(f)
	if !hasConflict {
		t.Fatal("expected head-on conflict detection between opposing stationary buses")
	}

	// 2. Normal following queue: Ego behind other, both heading West
	egoFollower := sim.NewBus(100.0, 0.0, math.Pi)
	otherLeader := sim.NewBus(0.0, 0.0, math.Pi)
	fQueue := autopilotFrame{
		bus:       egoFollower,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(otherLeader)},
		speedAbs:  0,
	}
	if conflict, _ := ad.checkHeadonConflict(fQueue); conflict {
		t.Fatal("normal following queue must NOT be flagged as head-on conflict")
	}

	// 3. Far distance: opposing buses 300px apart
	farOther := sim.NewBus(-300.0, 0.0, 0.0)
	fFar := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(farOther)},
		speedAbs:  0,
	}
	if conflict, _ := ad.checkHeadonConflict(fFar); conflict {
		t.Fatal("distant vehicles must NOT be flagged as head-on conflict")
	}
}

func TestHeadonRearClearance(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 4, Type: 0},
		},
	}
	ad := NewAutopilotDriver()

	// Ego bus at (0, 0) heading East (0 rad)
	ego := sim.NewBus(0.0, 0.0, 0.0)

	// 1. Completely clear behind
	fClear := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: nil,
		speedAbs:  0,
	}
	if !ad.headonRearClear(fClear, 70.0) {
		t.Fatal("expected clear rear when no obstacles exist")
	}

	// 2. Obstacle tailgating 20px behind ego (Pos.X = -egoHalfLen - obHalfLen - 20)
	rearTailgater := sim.NewBus(-120.0, 0.0, 0.0)
	fBlocked := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(rearTailgater)},
		speedAbs:  0,
	}
	if ad.headonRearClear(fBlocked, 70.0) {
		t.Fatal("expected rear clearance to be blocked by tailgating vehicle")
	}

	// 3. Obstacle far behind (> 200px)
	farRear := sim.NewBus(-300.0, 0.0, 0.0)
	fFarRear := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(farRear)},
		speedAbs:  0,
	}
	if !ad.headonRearClear(fFarRear, 70.0) {
		t.Fatal("expected clear rear when obstacle is 300px behind")
	}
}

func TestHeadonRecoveryLifecycle(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 4, Type: 0},
		},
	}
	ad := NewAutopilotDriver()

	// Ego bus at X=-50 (smaller X than other at X=50 -> ego yields first)
	ego := sim.NewBus(-50.0, 0.0, 0.0)
	other := sim.NewBus(50.0, 0.0, math.Pi)
	other.Speed = 0

	f := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{ObserveBus(other)},
		speedAbs:  0,
	}

	// Simulate 9.9 seconds: should not activate yet
	for i := 0; i < 99; i++ {
		if ad.tryHeadonDeadlockRecovery(f, 0.1) {
			t.Fatalf("recovery activated prematurely at %.1fs", float64(i)*0.1)
		}
	}
	if ad.HeadonRecoveryActive() {
		t.Fatal("recovery must not be active before 10s")
	}

	// Advance to 10.0 seconds: should activate!
	if !ad.tryHeadonDeadlockRecovery(f, 0.1) {
		t.Fatal("recovery should activate at 10.0s")
	}
	if !ad.HeadonRecoveryActive() {
		t.Fatal("HeadonRecoveryActive should be true")
	}

	// Phase 1: Verify reverse controls
	th, br, st, done := ad.headonRecoveryControls(f, 0.016)
	if done {
		t.Fatal("recovery should not be done on initial tick")
	}
	if ego.Gear != sim.GearReverse {
		t.Fatalf("expected GearReverse, got %v", ego.Gear)
	}
	if th <= 0 || br > 0 {
		t.Fatalf("expected reverse throttle > 0 and brake == 0, got th=%.2f br=%.2f", th, br)
	}
	_ = st

	// Simulate bus moving back by 70px (target distance reached)
	ego.Pos.X -= 71.0 // moving West is moving backward when heading is East (0)
	th2, br2, _, done2 := ad.headonRecoveryControls(f, 0.016)
	if done2 {
		t.Fatal("recovery should transition to yield hold, not be done immediately")
	}
	if ego.Gear != sim.GearDrive {
		t.Fatalf("expected GearDrive after reversing, got %v", ego.Gear)
	}
	if br2 != 1.0 || th2 != 0.0 {
		t.Fatalf("expected hold brake=1.0 th=0.0 during yield hold, got br=%.2f th=%.2f", br2, th2)
	}

	// Remove conflicting obstacle (as other vehicle drives past) and advance yield hold timer
	fClear := f
	fClear.obstacles = nil
	for tick := 0; tick < 260; tick++ { // 4.16 seconds
		_, _, _, done := ad.headonRecoveryControls(fClear, 0.016)
		if done {
			break
		}
	}

	if ad.HeadonRecoveryActive() {
		t.Fatal("recovery should complete and clear after yield hold")
	}
}

func TestMergeConflictDetection(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 6, Type: 0},
		},
	}
	ad := NewAutopilotDriver()

	// Ego bus in Lane 1 (Y = -78) merging toward Lane 2 (Y = -130), heading West at -2.79 rad
	ego := sim.NewBus(-569.92, -83.53, -2.79)
	ego.Speed = 0
	ego.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 1, Remaining: 0.28}

	// Car in Lane 2 (Y = -130), heading West at pi rad, stopped next to bus
	car := &sim.TrafficCar{
		ID:      31566,
		Pos:     sim.Vec2{X: -601.94, Y: -130.00},
		Heading: math.Pi,
		Length:  46,
		Width:   22,
		Speed:   0,
	}

	ob := VehicleObstacle{
		Pos:     car.Pos,
		Heading: car.Heading,
		Length:  car.Length,
		Width:   car.Width,
		Speed:   car.Speed,
	}

	f := autopilotFrame{
		bus:        ego,
		world:      world,
		laneWidth:  52,
		obstacles:  []VehicleObstacle{ob},
		speedAbs:   0,
		distToStop: 500.0,
	}

	hasConflict, isEgoYielding := ad.checkMergeConflict(f)
	if !hasConflict {
		t.Fatal("expected merge wedge conflict detection")
	}
	if !isEgoYielding {
		t.Fatal("merging bus should yield to straight vehicle in target lane")
	}

	conflict, yielding, revDist := ad.checkDeadlockConflict(f)
	if !conflict || !yielding || revDist != 22.0 {
		t.Fatalf("expected checkDeadlockConflict=(true, true, 22.0), got (%v, %v, %.1f)", conflict, yielding, revDist)
	}
}

func TestMergeDeadlockReverseLifecycle(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 6, Type: 0},
		},
	}
	ad := NewAutopilotDriver()

	ego := sim.NewBus(-569.92, -83.53, -2.79)
	ego.Speed = 0
	ego.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 1, Remaining: 0.28}

	car := &sim.TrafficCar{
		ID:      31566,
		Pos:     sim.Vec2{X: -601.94, Y: -130.00},
		Heading: math.Pi,
		Length:  46,
		Width:   22,
		Speed:   0,
	}

	ob := VehicleObstacle{
		Pos:     car.Pos,
		Heading: car.Heading,
		Length:  car.Length,
		Width:   car.Width,
		Speed:   car.Speed,
	}

	f := autopilotFrame{
		bus:        ego,
		world:      world,
		laneWidth:  52,
		obstacles:  []VehicleObstacle{ob},
		speedAbs:   0,
		distToStop: 500.0,
	}

	// 9.9 seconds: should not activate yet
	for i := 0; i < 99; i++ {
		if ad.tryHeadonDeadlockRecovery(f, 0.1) {
			t.Fatalf("merge deadlock recovery activated prematurely at %.1fs", float64(i)*0.1)
		}
	}
	if ad.HeadonRecoveryActive() {
		t.Fatal("recovery must not be active before 10s")
	}

	// 10.0 seconds: should activate!
	if !ad.tryHeadonDeadlockRecovery(f, 0.1) {
		t.Fatal("recovery should activate at 10.0s for merge conflict")
	}
	if !ad.HeadonRecoveryActive() {
		t.Fatal("HeadonRecoveryActive should be true")
	}
	if ad.headonRecovery.TargetDistance != 22.0 {
		t.Fatalf("expected TargetDistance 22.0 for merge recovery, got %.1f", ad.headonRecovery.TargetDistance)
	}

	// Reverse controls
	th, br, _, done := ad.headonRecoveryControls(f, 0.016)
	if done {
		t.Fatal("recovery should not be done on initial tick")
	}
	if ego.Gear != sim.GearReverse {
		t.Fatalf("expected GearReverse, got %v", ego.Gear)
	}
	if th <= 0 || br > 0 {
		t.Fatalf("expected th > 0, br == 0, got th=%.2f br=%.2f", th, br)
	}
}

func TestSideBySideDeadlockReverseLifecycle(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -6000, Y: -7100}, P2: sim.Vec2{X: -4000, Y: -7100}, Lanes: 4, Type: 0},
		},
	}
	ad := NewAutopilotDriver()

	// Bus A (AI#23) at (-5155, -7123) turning slightly toward stop
	busA := sim.NewBus(-5155.0, -7123.0, 0.05)
	busA.Speed = 0

	// Bus B (AI#24) at (-5158, -7084) in outer lane heading straight
	busB := sim.NewBus(-5158.0, -7084.0, 0.0)
	busB.Speed = 0

	obB := ObserveBus(busB)

	f := autopilotFrame{
		bus:       busA,
		world:     world,
		laneWidth: 40,
		obstacles: []VehicleObstacle{obB},
		speedAbs:  0,
	}

	conflict, yielding, revDist := ad.checkDeadlockConflict(f)
	if !conflict {
		t.Fatal("expected conflict detection for side-by-side wedged buses")
	}
	if revDist <= 0 {
		t.Fatalf("expected positive reverse distance, got %.1f", revDist)
	}
	_ = yielding

	// If ego is not the primary yielding vehicle, threshold is 13.5s
	// Simulate up to 14.0 seconds (140 ticks)
	for i := 0; i < 140; i++ {
		ad.tryHeadonDeadlockRecovery(f, 0.1)
	}
	if !ad.HeadonRecoveryActive() {
		t.Fatal("expected recovery active after deadlock timeout")
	}

	// Verify reverse controls
	th, br, _, done := ad.headonRecoveryControls(f, 0.016)
	if done {
		t.Fatal("should not be done on initial tick")
	}
	if busA.Gear != sim.GearReverse {
		t.Fatalf("expected GearReverse, got %v", busA.Gear)
	}
	if th <= 0 || br > 0 {
		t.Fatalf("expected reverse th > 0, br == 0, got th=%.2f br=%.2f", th, br)
	}
}

func TestArticulatedRearClearanceChecksTrailer(t *testing.T) {
	world := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: 9000, Y: 7252}, P2: sim.Vec2{X: 12000, Y: 7252}, Lanes: 4, Type: 0},
		},
	}
	ego := sim.NewBus(10832.9, 7228.6, -0.001)
	ego.SetModel(sim.BusArticulated)
	ego.TrailerHeading = 0.010

	ad := NewAutopilotDriver()
	ad.LaneOffset = -45.9

	// Car sitting ~9px behind the articulated trailer (~176px behind tractor Pos).
	rearCar := VehicleObstacle{
		Pos:     sim.Vec2{X: 10656.8, Y: 7226.0},
		Heading: 0,
		Length:  46,
		Width:   22,
		Speed:   0,
	}
	fBlocked := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{rearCar},
		speedAbs:  0,
	}
	if ad.headonRearClear(fBlocked, 22.0) {
		t.Fatal("expected headonRearClear=false when a car is 9px behind articulated trailer")
	}

	// Front leader creating a merge conflict in checkMergeConflict while rearCar blocks reversing.
	frontLeader := sim.NewBus(11040.2, 7225.2, -0.004)
	frontLeader.SetModel(sim.BusArticulated)
	fDeadlock := fBlocked
	fDeadlock.obstacles = []VehicleObstacle{ObserveBus(frontLeader), rearCar}
	for i := 0; i < 150; i++ {
		if ad.tryHeadonDeadlockRecovery(fDeadlock, 0.1) {
			t.Fatal("articulated bus must NOT initiate reverse recovery when rear car blocks its trailer")
		}
	}

	// Immediate rear guard in headonRecoveryControls must abort if a rear car is close behind the trailer
	// (including swept/parallel retreats when within stopping margin).
	for _, parallel := range []bool{false, true} {
		bus := *ego
		bus.ShiftTo(sim.GearReverse)
		bus.Speed = -15.0
		drv := NewAutopilotDriver()
		drv.headonRecovery = &HeadonDeadlockRecovery{
			StartPos:           bus.Pos.Add(sim.Vec2{X: 8}),
			TargetDistance:     22.0,
			YieldHoldRemaining: 4.0,
			ParallelRetreat:    parallel,
		}
		veryCloseCar := rearCar
		veryCloseCar.Pos.X = 10665.0 // ~1px behind trailer rear edge (~10688.9)
		fClose := autopilotFrame{
			bus:       &bus,
			world:     world,
			laneWidth: 52,
			obstacles: []VehicleObstacle{veryCloseCar},
			speedAbs:  15.0,
		}
		th, br, _, done := drv.headonRecoveryControls(fClose, 1.0/60.0)
		if done || !drv.headonRecovery.ReversingDone {
			t.Fatalf("parallel=%v: expected reversing to abort cleanly into yield hold", parallel)
		}
		if bus.Gear != sim.GearDrive || br != 1.0 || th != 0.0 {
			t.Fatalf("parallel=%v: expected GearDrive brake=1 th=0 on abort, got gear=%v br=%.2f th=%.2f", parallel, bus.Gear, br, th)
		}
	}

	// Clear rear when car is well behind the trailer.
	farCar := rearCar
	farCar.Pos.X = 10550.0
	fClear := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{farCar},
		speedAbs:  0,
	}
	if !ad.headonRearClear(fClear, 22.0) {
		t.Fatal("expected headonRearClear=true when rear car is well behind trailer")
	}

	// Approaching rear car moving fast toward the trailer must also block a long reverse.
	approachingCar := rearCar
	approachingCar.Pos.X = 10580.0
	approachingCar.Speed = 200.0
	fApproaching := autopilotFrame{
		bus:       ego,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{approachingCar},
		speedAbs:  0,
	}
	if ad.headonRearClear(fApproaching, 70.0) {
		t.Fatal("expected headonRearClear=false when a car is approaching the trailer from behind")
	}
}

func TestJunctionReverseControlsFullUnwindAndEntryAlignment(t *testing.T) {
	// Articulated jackknife unwind uses full [-1, 1] steer range.
	b := sim.NewBus(0, 0, 0.25)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = 0
	b.Speed = 0
	_, _, steer := junctionReverseControls(b, false, 0)
	if steer != 1.0 {
		t.Fatalf("expected full +1.0 unwind steer for ang=0.25, got %.3f", steer)
	}

	b.Heading = -0.25
	_, _, steer = junctionReverseControls(b, false, 0)
	if steer != -1.0 {
		t.Fatalf("expected full -1.0 unwind steer for ang=-0.25, got %.3f", steer)
	}

	// With alignEntry enabled and small articulation angle, reverse steer aligns heading toward entryHeading.
	bAligned := sim.NewBus(0, 0, 0.20)
	bAligned.SetModel(sim.BusArticulated)
	bAligned.TrailerHeading = 0.20
	bAligned.Speed = 0
	_, _, alignSteer := junctionReverseControls(bAligned, true, 0)
	if math.Abs(alignSteer-0.8) > 1e-6 {
		t.Fatalf("expected entry-alignment steer 0.8 for hdgErr=0.20, got %.6f", alignSteer)
	}
}
