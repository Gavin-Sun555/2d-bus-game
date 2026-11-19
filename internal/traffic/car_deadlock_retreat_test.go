package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func newDeadlockRetreatFixture(busHeading, tailHeading float64) (*sim.World, *TrafficManager, *sim.Bus, *sim.TrafficCar, float64) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{{
			P1:    sim.Vec2{X: -2000, Y: 7200},
			P2:    sim.Vec2{X: 2000, Y: 7200},
			Lanes: 6,
		}},
		Intersections: []sim.Vec2{{X: -2000, Y: 7200}, {X: 2000, Y: 7200}},
	}
	b := sim.NewBus(96.33, 7236.01, busHeading)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = tailHeading

	car := &sim.TrafficCar{
		ID:          1,
		Pos:         sim.Vec2{X: 4.46, Y: 7226.0},
		Heading:     0,
		TargetSpeed: 166.67,
		Length:      46,
		Width:       22,
		RoadCenter:  7200,
		IsEW:        true,
		LaneIndex:   0,
	}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	return w, tm, b, car, lw
}

func TestDeadlockedCarReversesBehindWedgedBusAndLetsItDepart(t *testing.T) {
	w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "origin", sim.Vec2{X: -800, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "dest", sim.Vec2{X: 1200, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
	}
	driver := autopilot.NewAutopilotDriver()
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	driver.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)

	const dt = 1.0 / 60.0
	carMinX := car.Pos.X
	prevX := car.Pos.X
	sawReverse := false
	busUnstuck := false

	for tick := 0; tick < 900; tick++ {
		obs := []autopilot.VehicleObstacle{{
			Pos:     car.Pos,
			Heading: car.Heading,
			Speed:   car.Speed,
			Length:  car.Length,
			Width:   car.Width,
		}}
		throttle, brake, steer, handbrake, _, _ := driver.ComputeControls(bus, route, w, nil, obs, lw, dt)
		bus.Update(dt, throttle, brake, steer, handbrake, false)
		tm.Update(dt, bus, nil, 1, lw, w)

		stepDelta := math.Abs(car.Pos.X - prevX)
		if stepDelta > 10.0 {
			t.Fatalf("unexpected teleport discontinuity at tick %d: delta=%.2f", tick, stepDelta)
		}
		prevX = car.Pos.X
		if car.Pos.X < carMinX {
			carMinX = car.Pos.X
		}
		if car.Speed < -1.0 {
			sawReverse = true
		}
		if bus.Pos.X > 240.0 && bus.Speed > 25.0 {
			busUnstuck = true
		}
		if busUnstuck && car.Pos.X > 40.0 && car.Speed > 20.0 {
			break
		}
	}

	if !sawReverse {
		t.Fatalf("expected deadlocked car to reverse smoothly (minX=%.2f, finalCar=%+v)", carMinX, car)
	}
	backed := 4.46 - carMinX
	if backed < 40.0 || backed > 65.0 {
		t.Fatalf("expected car to back up ~40-65px, backed=%.2f", backed)
	}
	if !busUnstuck {
		obs := []autopilot.VehicleObstacle{{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width}}
		diag := driver.DiagnoseStop(bus, route, w, nil, obs, lw)
		t.Fatalf("expected articulated bus to unstick after car backed up; bus pos=(%.2f,%.2f) spd=%.2f tgt=%.2f limits=%v",
			bus.Pos.X, bus.Pos.Y, bus.Speed, driver.LastTargetSpeed, diag.Limits)
	}
	if car.Pos.X <= 40.0 {
		t.Fatalf("expected car to resume forward after bus departed; car pos=(%.2f,%.2f) spd=%.2f hold=%.2f",
			car.Pos.X, car.Pos.Y, car.Speed, car.ReverseHold)
	}
}

func TestDeadlockedCarReverseSafetyGuards(t *testing.T) {
	const dt = 1.0 / 60.0

	t.Run("BlockedByRearCar", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		rear := &sim.TrafficCar{
			ID: 2, Pos: sim.Vec2{X: -48.0, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
			Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
		}
		tm.Cars = append(tm.Cars, rear)
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 || car.Pos.X < 4.0 {
			t.Fatalf("car reversed despite rear obstacle: posX=%.2f spd=%.2f rem=%.2f", car.Pos.X, car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("ApproachingRearCarReservesStopDistance", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		approaching := &sim.TrafficCar{
			ID: 2, Pos: sim.Vec2{X: -120.0, Y: 7226.0}, Heading: 0, Speed: 140.0, TargetSpeed: 166.67,
			Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
		}
		tm.Cars = append(tm.Cars, approaching)
		car.DeadlockTimer = 6.0
		tm.StepCar(dt, car, 0, bus, nil, 2, lw, w)
		if car.ReverseRemaining > 0 || car.Speed < 0 {
			t.Fatalf("car reversed in front of fast approaching rear car: rem=%.2f spd=%.2f", car.ReverseRemaining, car.Speed)
		}
	})

	t.Run("JunctionBehindBlocksReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		w.Intersections = []sim.Vec2{{X: -65, Y: 7200}}
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 || car.Pos.X < 4.0 {
			t.Fatalf("car reversed into intersection behind it: posX=%.2f spd=%.2f rem=%.2f", car.Pos.X, car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("DoorsOpenBusDoesNotTriggerReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		bus.DoorsOpen = true
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 {
			t.Fatalf("car reversed behind bus with open doors: spd=%.2f rem=%.2f", car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("RedLightQueueDoesNotReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(0, 0)
		bus.Pos = sim.Vec2{X: 90.0, Y: 7226.0}
		inter := &sim.TrafficIntersection{
			Center: sim.Vec2{X: 185, Y: 7200},
			Phase:  sim.PhaseNSGreen,
		}
		w.Intersections = []sim.Vec2{inter.Center}
		tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, tls, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 || car.Pos.X < 4.0 {
			t.Fatalf("car reversed unnecessarily in red light queue: posX=%.2f spd=%.2f rem=%.2f", car.Pos.X, car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("DynamicRearObstacleAbortsReverseCleanly", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 20; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining <= 0 || car.Speed >= 0 {
			t.Fatalf("expected car to be reversing before obstacle arrives: rem=%.2f spd=%.2f", car.ReverseRemaining, car.Speed)
		}
		// Another car appears directly behind while reversing.
		rear := &sim.TrafficCar{
			ID: 2, Pos: sim.Vec2{X: car.Pos.X - 52.0, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
			Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
		}
		tm.Cars = append(tm.Cars, rear)
		for tick := 0; tick < 30; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Pos.X-rear.Pos.X < (car.Length+rear.Length)/2 {
			t.Fatalf("expected mid-reverse abort without collision: rem=%.2f gap=%.2f",
				car.ReverseRemaining, car.Pos.X-rear.Pos.X-(car.Length+rear.Length)/2)
		}
	})

	t.Run("JunctionBehindTailCarBlocksChain", func(t *testing.T) {
		w, tm, bus, car1, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
		car2 := &sim.TrafficCar{
			ID: 2, Pos: sim.Vec2{X: -59.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
			Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
		}
		car3 := &sim.TrafficCar{
			ID: 3, Pos: sim.Vec2{X: -123.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
			Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
		}
		tm.Cars = append(tm.Cars, car2, car3)
		w.Intersections = append(w.Intersections, sim.Vec2{X: -320, Y: 7200})
		car1.DeadlockTimer = 6.0

		for tick := 0; tick < 180; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
			if car1.Pos.X-car2.Pos.X < 46.0 || car2.Pos.X-car3.Pos.X < 46.0 {
				t.Fatalf("chain cars overlapped at tick %d: xs=(%.2f, %.2f, %.2f)", tick, car1.Pos.X, car2.Pos.X, car3.Pos.X)
			}
		}
		if car3.Pos.X < -124.0 || car2.Pos.X < -60.0 || car1.Pos.X < 4.0 {
			t.Fatalf("chain backed into junction behind tail car: xs=(%.2f, %.2f, %.2f)", car1.Pos.X, car2.Pos.X, car3.Pos.X)
		}
	})
}

func TestDeadlockedCarCascadingChainRetreatUnsticksBus(t *testing.T) {
	w, tm, bus, car1, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
	car2 := &sim.TrafficCar{
		ID: 2, Pos: sim.Vec2{X: -59.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
		Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
	}
	car3 := &sim.TrafficCar{
		ID: 3, Pos: sim.Vec2{X: -123.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
		Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
	}
	tm.Cars = append(tm.Cars, car2, car3)

	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "origin", sim.Vec2{X: -800, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "dest", sim.Vec2{X: 1200, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
	}
	driver := autopilot.NewAutopilotDriver()
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	driver.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)

	const dt = 1.0 / 60.0
	cars := []*sim.TrafficCar{car1, car2, car3}
	prevX := []float64{car1.Pos.X, car2.Pos.X, car3.Pos.X}
	sawReverse := make([]bool, 3)
	minX := []float64{car1.Pos.X, car2.Pos.X, car3.Pos.X}
	busUnstuck := false

	for tick := 0; tick < 1200; tick++ {
		obs := make([]autopilot.VehicleObstacle, 0, len(cars))
		for _, c := range cars {
			obs = append(obs, autopilot.VehicleObstacle{
				Pos:     c.Pos,
				Heading: c.Heading,
				Speed:   c.Speed,
				Length:  c.Length,
				Width:   c.Width,
			})
		}
		throttle, brake, steer, handbrake, _, _ := driver.ComputeControls(bus, route, w, nil, obs, lw, dt)
		bus.Update(dt, throttle, brake, steer, handbrake, false)
		tm.Update(dt, bus, nil, 1, lw, w)

		for i, c := range cars {
			if delta := math.Abs(c.Pos.X - prevX[i]); delta > 10.0 {
				t.Fatalf("car %d teleport discontinuity at tick %d: delta=%.2f", i+1, tick, delta)
			}
			prevX[i] = c.Pos.X
			if c.Pos.X < minX[i] {
				minX[i] = c.Pos.X
			}
			if c.Speed < -1.0 {
				sawReverse[i] = true
			}
		}
		if car1.Pos.X-car2.Pos.X < 46.0 || car2.Pos.X-car3.Pos.X < 46.0 {
			t.Fatalf("chain cars overlapped at tick %d: xs=(%.2f, %.2f, %.2f)", tick, car1.Pos.X, car2.Pos.X, car3.Pos.X)
		}
		if bus.Pos.X > 240.0 && bus.Speed > 25.0 {
			busUnstuck = true
		}
		if busUnstuck && car1.Pos.X > 30.0 && car2.Pos.X > -30.0 && car3.Pos.X > -90.0 &&
			car1.Speed > 20.0 && car2.Speed > 20.0 && car3.Speed > 20.0 {
			break
		}
	}

	for i := range cars {
		if !sawReverse[i] {
			t.Fatalf("expected car %d in chain to reverse smoothly; minX=%v cars=%v", i+1, minX, cars)
		}
	}
	if !busUnstuck {
		t.Fatalf("expected wedged bus to depart after chain reverse; bus pos=(%.2f,%.2f) spd=%.2f minX=%v",
			bus.Pos.X, bus.Pos.Y, bus.Speed, minX)
	}
	if car1.Pos.X <= 30.0 || car2.Pos.X <= -30.0 || car3.Pos.X <= -90.0 {
		t.Fatalf("expected entire chain to resume forward after bus departed: xs=(%.2f, %.2f, %.2f)",
			car1.Pos.X, car2.Pos.X, car3.Pos.X)
	}
}

func TestDeadlockedCarChainContinuesWhenFirstStepInsufficient(t *testing.T) {
	w, tm, bus, car1, lw := newDeadlockRetreatFixture(-0.2673, -0.3187)
	car2 := &sim.TrafficCar{
		ID: 2, Pos: sim.Vec2{X: -59.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
		Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
	}
	car3 := &sim.TrafficCar{
		ID: 3, Pos: sim.Vec2{X: -123.5, Y: 7226.0}, Heading: 0, TargetSpeed: 166.67,
		Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0,
	}
	tm.Cars = append(tm.Cars, car2, car3)
	car1.DeadlockTimer = 6.0

	const dt = 1.0 / 60.0
	cars := []*sim.TrafficCar{car1, car2, car3}
	minX := []float64{car1.Pos.X, car2.Pos.X, car3.Pos.X}
	maxAttempt := 0
	busReleased := false

	for tick := 0; tick < 1200; tick++ {
		if !busReleased {
			// Keep the bus wedged until car1 retreats past 70px so a single step (~60px max)
			// is insufficient and the chain must perform a second retreat step.
			if car1.Pos.X < 4.46-70.0 {
				busReleased = true
			}
		}
		if busReleased {
			bus.Speed = 60.0
			bus.Pos.X += bus.Speed * dt
		}

		tm.Update(dt, bus, nil, 1, lw, w)

		for i, c := range cars {
			if c.Pos.X < minX[i] {
				minX[i] = c.Pos.X
			}
		}
		if car1.ReverseAttempts > maxAttempt {
			maxAttempt = car1.ReverseAttempts
		}
		if !busReleased && car1.Pos.X > 4.46+1.0 {
			t.Fatalf("car1 surged forward into still-wedged bus at tick %d: posX=%.2f", tick, car1.Pos.X)
		}
		if car1.Pos.X-car2.Pos.X < 46.0 || car2.Pos.X-car3.Pos.X < 46.0 {
			t.Fatalf("chain cars overlapped during multi-step retreat at tick %d: xs=(%.2f, %.2f, %.2f)",
				tick, car1.Pos.X, car2.Pos.X, car3.Pos.X)
		}
		if busReleased && car1.Pos.X > 40.0 && car1.Speed > 20.0 {
			break
		}
	}

	if !busReleased {
		t.Fatalf("expected multi-step chain retreat to push car1 back >70px; maxAttempt=%d minX=%v car1=%+v",
			maxAttempt, minX, car1)
	}
	if maxAttempt < 2 {
		t.Fatalf("expected >=2 retreat attempts before bus unblocked, got %d", maxAttempt)
	}
	if car1.Pos.X <= 40.0 {
		t.Fatalf("expected car1 to stop retreating and drive forward once bus moved: posX=%.2f spd=%.2f", car1.Pos.X, car1.Speed)
	}
}

func TestCarBlockedByAngledOverhangingBusInitiatesDeadlockReverse(t *testing.T) {
	w := &sim.World{
		Roads: []sim.RoadSegment{{
			P1: sim.Vec2{X: -2000, Y: 7200}, P2: sim.Vec2{X: 2000, Y: 7200}, Lanes: 6,
		}},
		Intersections: []sim.Vec2{{X: -2000, Y: 7200}, {X: 2000, Y: 7200}},
	}
	// Angled bus whose center X (860.7) is behind car X (882.8) while front nose overhangs ahead.
	bus := sim.NewBus(860.7, 7284.2, -0.55)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -0.15

	car := &sim.TrafficCar{
		ID:            1,
		Pos:           sim.Vec2{X: 882.8, Y: 7226.0},
		Heading:       0,
		TargetSpeed:   166.67,
		Length:        46,
		Width:         22,
		RoadCenter:    7200,
		IsEW:          true,
		LaneIndex:     0,
		DeadlockTimer: 6.0,
	}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	frame := carFrame{
		car: car, buses: []*sim.Bus{bus}, parkedBuses: []bool{false}, world: w, laneWidth: 52,
	}
	if !tm.carBlockedByStoppedBus(frame, false) {
		t.Fatal("expected car behind angled overhanging bus nose to be detected as blocked")
	}

	// Bus in far lane without lateral overlap must not trigger false block.
	farBus := sim.NewBus(860.7, 7380.0, -0.55)
	farBus.SetModel(sim.BusArticulated)
	farBus.TrailerHeading = -0.55
	farFrame := carFrame{
		car: car, buses: []*sim.Bus{farBus}, parkedBuses: []bool{false}, world: w, laneWidth: 52,
	}
	if tm.carBlockedByStoppedBus(farFrame, false) {
		t.Fatal("expected non-overlapping far-lane bus behind car not to block")
	}

	sawReverse := false
	for tick := 0; tick < 180; tick++ {
		tm.Update(1./60., bus, nil, 1, 52, w)
		if car.Speed < -1.0 {
			sawReverse = true
			break
		}
	}
	if !sawReverse {
		t.Fatalf("expected deadlocked car to start reversing behind angled overhanging bus; car=%+v", car)
	}
}

func TestCarReverseOnlyForMergeBlockOrDeadlockNotRedLightOrNonConflictingQueue(t *testing.T) {
	const dt = 1.0 / 60.0

	t.Run("FarRedLightQueueDoesNotReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(-0.20, -0.25)
		// Place intersection stop line ~450px ahead (beyond old 220px red-check horizon).
		inter := &sim.TrafficIntersection{
			Center: sim.Vec2{X: 700, Y: 7200},
			Phase:  sim.PhaseNSGreen,
		}
		w.Intersections = []sim.Vec2{inter.Center}
		tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
		bus.Pos = sim.Vec2{X: 110, Y: 7226}
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, tls, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 || car.Pos.X < 4.0 {
			t.Fatalf("car reversed in red-light queue 450px+ behind stop line: posX=%.2f spd=%.2f rem=%.2f",
				car.Pos.X, car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("StraightNonMergingSameLaneBusDoesNotTriggerReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(0, 0)
		bus.Pos = sim.Vec2{X: 110, Y: 7226}
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 {
			t.Fatalf("car reversed behind straight non-merging bus on open road: spd=%.2f rem=%.2f",
				car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("BusMergingAwayInAdjacentLaneDoesNotTriggerReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(0, 0.09)
		bus.Pos = sim.Vec2{X: 96.0, Y: 7301.0} // ~75px to the right of car in Y=7226
		bus.LaneMergeRequest = &sim.BusLaneMergeRequest{
			Target:    sim.Vec2{X: 0, Y: 7330.0},
			Heading:   0,
			RightTurn: true,
			Remaining: 0.3,
		}
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 {
			t.Fatalf("car in lane 0 reversed for bus merging right away from it: spd=%.2f rem=%.2f",
				car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("OppositeLaneBusDoesNotTriggerReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(math.Pi-0.25, math.Pi)
		bus.Pos = sim.Vec2{X: 60.0, Y: 7174.0} // oncoming lane across centerline (52px away)
		car.DeadlockTimer = 6.0
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
		}
		if car.ReverseRemaining > 0 || car.Speed < 0 {
			t.Fatalf("car reversed for bus on opposite carriageway: spd=%.2f rem=%.2f",
				car.Speed, car.ReverseRemaining)
		}
	})

	t.Run("CarBlockingActiveBusLaneChangeDoesReverse", func(t *testing.T) {
		w, tm, bus, car, lw := newDeadlockRetreatFixture(0, 0)
		// Bus in adjacent lane 1 (Y=7278) actively requesting merge left into lane 0 (Y=7226).
		bus.Pos = sim.Vec2{X: 70.0, Y: 7278.0}
		bus.LaneMergeRequest = &sim.BusLaneMergeRequest{
			Target:    sim.Vec2{X: 0, Y: 7226.0},
			Heading:   0,
			LeftTurn:  true,
			Remaining: 0.3,
		}
		car.DeadlockTimer = 6.0
		sawReverse := false
		for tick := 0; tick < 60; tick++ {
			tm.Update(dt, bus, nil, 1, lw, w)
			if car.Speed < -1.0 {
				sawReverse = true
				break
			}
		}
		if !sawReverse {
			t.Fatalf("expected car blocking active bus lane change into its lane to reverse; car=%+v", car)
		}
	})
}
