package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestPassingCurveChecksStoppedSourceLaneLeader(t *testing.T) {
	car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -1316.8047830578012, Y: -5478}, Heading: math.Pi, Length: 46, Width: 22}
	leader := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: -1432.4904906275794, Y: -5478}, Heading: math.Pi, Length: 46, Width: 22}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car, leader}}
	f := carFrame{car: car, laneWidth: 52, activeCount: 2}
	if tm.stoppedTrafficClearsPassingCurve(f, sim.Vec2{X: -1}, 1) {
		t.Fatal("passing curve intersects its source-lane leader")
	}
	leader.Pos.X = car.Pos.X - 260
	if !tm.stoppedTrafficClearsPassingCurve(f, sim.Vec2{X: -1}, 1) {
		t.Fatal("ample passing runway rejected")
	}
}

func TestRoundaboutExitDoesNotPushCirculatingCar(t *testing.T) {
	rb := sim.Roundabout{Radius: 300, InnerRadius: 90}
	exit := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: 170, Y: 78}, Heading: 0, Length: 46, Width: 22, InRoundabout: true, RoundaboutTargetR: 250, RoundaboutOuterRadius: 300, RoundaboutExitHeading: 0}
	exit.RoundaboutCurve = newRoundaboutCurve(exit.Pos, sim.Vec2{X: 240, Y: 78}, sim.Vec2{X: 310, Y: 78}, sim.Vec2{X: 380, Y: 78}, true)
	circulating := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: 230, Y: 97}, Heading: -1.17, Length: 46, Width: 22, InRoundabout: true, RoundaboutOuterRadius: 300, RoundaboutTargetR: 250}
	circulating.RoundaboutRadius = circulating.Pos.Length()
	circulating.RoundaboutAngle = math.Atan2(97, 230)
	tm := &TrafficManager{Cars: []*sim.TrafficCar{exit, circulating}}
	f := carFrame{car: exit, index: 0, activeCount: 2, world: &sim.World{Roundabouts: []sim.Roundabout{rb}}, laneWidth: 52}
	for tick := 0; tick < 300; tick++ {
		target := math.Min(90, tm.roundaboutExitCarSpeed(f))
		if exit.Speed < target {
			exit.Speed = math.Min(target, exit.Speed+120./60)
		} else {
			exit.Speed = math.Max(target, exit.Speed-280./60)
		}
		moveCar(exit, 1./60)
		a := sim.OBB{Center: exit.Pos, Heading: exit.Heading, HalfLen: 23, HalfWid: 11}
		b := sim.OBB{Center: circulating.Pos, Heading: circulating.Heading, HalfLen: 23, HalfWid: 11}
		if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
			t.Fatal("exit drove through circulating body")
		}
	}
	if exit.Pos.X > 220 || exit.Pos.X < 175 {
		t.Fatalf("did not hold a useful safe gap: %v", exit.Pos)
	}
	// A car already behind the outward path does not hold that exit.
	circulating.Pos = sim.Vec2{X: 100, Y: -100}
	circulating.RoundaboutRadius = circulating.Pos.Length()
	circulating.RoundaboutAngle = math.Atan2(-100, 100)
	if !math.IsInf(tm.roundaboutExitCarSpeed(f), 1) {
		t.Fatal("diverging circulation blocked exit")
	}
}

func TestCompletedTurnAndCrossingCarClearStoppedBus(t *testing.T) {
	cases := []struct {
		name string
		car  *sim.TrafficCar
		bus  *sim.Bus
	}{
		{"completed right turn", &sim.TrafficCar{ID: 119, Pos: sim.Vec2{X: -3557.2011279728654, Y: 7278}, Heading: 0, Length: 46, Width: 22, JunctionTurn: &sim.CarJunctionTurn{EntryHeading: math.Pi / 2, Center: sim.Vec2{X: -3900, Y: 7200}, Points: []sim.Vec2{{X: -3562, Y: 7278}, {X: -3354, Y: 7278}}, Progress: 4.798872027134795, ExitHeading: 0}}, sim.NewBus(-3584.416526375922, 7248.903712410465, -.266)},
		{"committed crossing", &sim.TrafficCar{ID: 3463, Pos: sim.Vec2{X: -143.8566260179176, Y: 1670}, Heading: math.Pi, Length: 46, Width: 22, JunctionTurn: &sim.CarJunctionTurn{Straight: true, EntryHeading: math.Pi, Center: sim.Vec2{Y: 1800}, Points: []sim.Vec2{{X: -127, Y: 1670}, {X: -556, Y: 1670}}, Progress: 16.8566260179176, ExitHeading: math.Pi}}, sim.NewBus(-165.74350477402402, 1602.2315646324976, 2.171)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			c.bus.JunctionCorridor = &sim.JunctionCorridor{Center: c.car.JunctionTurn.Center, Remaining: .25}
			tm := &TrafficManager{}
			f := carFrame{car: c.car, buses: []*sim.Bus{c.bus}}
			if v := tm.busFollowingSpeed(f, 100); v <= 0 {
				t.Fatal("physically clear committed car held")
			}
			original := c.bus.Pos
			c.bus.Pos = c.car.Pos
			if carClearsParallelStoppedBus(c.car, c.bus) {
				t.Fatal("actual body collision ignored")
			}
			c.bus.Pos = original
			c.bus.Speed = 2
			if carClearsParallelStoppedBus(c.car, c.bus) {
				t.Fatal("moving turn ignored")
			}
		})
	}
}

func TestOptionalPassRollsBackItsActualPrefix(t *testing.T) {
	for _, scenario := range []string{"clear rear", "blocked rear", "rear yield"} {
		t.Run(scenario, func(t *testing.T) {
			blocked := scenario == "blocked rear"
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 6}}}
			c := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{Y: 78}, Heading: 0, Length: 46, Width: 22, CanOvertake: true, LaneIndex: 1, IsEW: true, TargetSpeed: 200}
			leader := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: 108, Y: 78}, Heading: 0, Length: 46, Width: 22}
			rear := &sim.TrafficCar{ID: 3, Pos: sim.Vec2{X: -200, Y: 78}, Heading: 0, Length: 46, Width: 22, IsEW: true, LaneIndex: 1}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{c, leader, rear}}
			f := carFrame{car: c, world: w, laneWidth: 52, activeCount: 3}
			tm.beginCarLaneChangeWithLength(f, 2, 220)
			c.Speed = 30
			moveCarThroughJunction(c, 1.4)
			c.Speed = 0
			c.DeadlockTimer = 5
			start := c.Pos
			if scenario == "rear yield" {
				rear.Pos = sim.Vec2{X: c.Pos.X - 70, Y: 78}
			}
			if blocked {
				rear.Pos = c.Pos.Sub(sim.Vec2{X: 52})
			}
			sawReverse := false
			for tick := 0; tick < 60*12; tick++ {
				before := c.Pos
				tm.StepCar(1./60, c, 0, nil, nil, 3, 52, w)
				if scenario == "rear yield" {
					tm.StepCar(1./60, rear, 2, nil, nil, 3, 52, w)
				}
				if c.Speed < 0 {
					sawReverse = true
				}
				if c.Pos.Distance(before) > math.Abs(c.Speed)/60+.1 {
					t.Fatal("rollback jumped")
				}
				for _, other := range []*sim.TrafficCar{leader, rear} {
					if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: 23, HalfWid: 11}); hit {
						t.Fatal("rollback collision")
					}
				}
				if !blocked && c.JunctionTurn == nil && !c.PassRetreatActive && math.Abs(c.Speed) < 1 {
					if !sawReverse || c.Pos.Distance(sim.Vec2{Y: 78}) > 1 {
						t.Fatalf("did not return to its real start: %v", c.Pos)
					}
					return
				}
			}
			if blocked {
				if sawReverse || c.Pos.Distance(start) > .1 {
					t.Fatal("backed through occupied rear gap")
				}
				return
			}
			t.Fatalf("pass retreat unfinished pos%v v%.2f turn%v rem%.2f hold%.2f attempts%d", c.Pos, c.Speed, c.JunctionTurn != nil, c.ReverseRemaining, c.ReverseHold, c.ReverseAttempts)
		})
	}
}

func TestShortCarYieldAfterRearClearsJunction(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{Y: -2000}, P2: sim.Vec2{Y: 2000}, Lanes: 6}, {P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 6}}}
	w.RebuildIntersections()
	g := w.JunctionAt(sim.Vec2{}, 52)
	y := g.HalfY + 46
	c := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -78, Y: y}, Heading: math.Pi / 2, Length: 46, Width: 22, RoadCenter: 0, LaneIndex: 1, TargetSpeed: 100, YieldReverseRequest: .6}
	c.JunctionTurn = &sim.CarJunctionTurn{Straight: true, Points: []sim.Vec2{{X: -78, Y: -600}, {X: -78, Y: g.HalfY}, {X: -78, Y: g.HalfY + 200}, {X: -78, Y: g.HalfY + 400}}, Segment: 1, Progress: 46, ExitHeading: math.Pi / 2, ExitRoadCenter: 0, ExitLane: 1}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
	f := carFrame{car: c, world: w, laneWidth: 52, activeCount: 1}
	if !finishCarTurnExitBeforeReverse(f, c) || c.JunctionTurn != nil {
		t.Fatal("finished straight tail prevented rear-slot relay")
	}
	for tick := 0; tick < 150; tick++ {
		c.YieldReverseRequest = .6
		tm.StepCar(1./60, c, 0, nil, nil, 1, 52, w)
	}
	moved := y - c.Pos.Y
	if moved < 4 || moved > 12 {
		t.Fatalf("short proved rear slot not used safely: %.2f", moved)
	}
}

func TestAlignedOptionalPassTailReleasesWithoutMoving(t *testing.T) {
	for _, unsafe := range []string{"", "lateral", "yaw", "curved", "moving", "retreat"} {
		c := &sim.TrafficCar{Pos: sim.Vec2{X: 473.4824, Y: 78.085}, Heading: -.0064136, Length: 46, Width: 22, DeadlockTimer: 5, Passing: true, LaneIndex: 2, IsEW: true}
		c.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true, Points: []sim.Vec2{{Y: 130}, {X: 468.75, Y: 78.11535}, {X: 484.375, Y: 78.01513}, {X: 500, Y: 78}}, Segment: 1, Progress: 4.732, ExitHeading: 0, ExitEW: true, ExitLane: 1}
		switch unsafe {
		case "lateral":
			c.Pos.Y += 1
		case "yaw":
			c.Heading = .04
		case "curved":
			c.JunctionTurn.Points[2].Y += 2
		case "moving":
			c.Speed = 10
		case "retreat":
			c.PassRetreatActive = true
		}
		w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 6}}}
		pos, h := c.Pos, c.Heading
		changed := finishAlignedCarPass(carFrame{car: c, world: w, laneWidth: 52})
		if changed != (unsafe == "") {
			t.Fatalf("%s changed%v", unsafe, changed)
		}
		if c.Pos != pos || c.Heading != h {
			t.Fatal("completion changed physical pose")
		}
		if changed && (c.LaneIndex != 1 || c.JunctionTurn != nil) {
			t.Fatal("did not release destination lane")
		}
	}
}

func TestCommittedStraightCarHoldsSmallBusAlignmentGap(t *testing.T) {
	for _, guard := range []string{"", "moving", "different_junction", "turning", "wide_merge"} {
		center := sim.Vec2{X: -11700, Y: -3600}
		c := &sim.TrafficCar{Pos: sim.Vec2{X: -11238.8144, Y: -3678}, Heading: math.Pi, Length: 46, Width: 22, LaneIndex: 1, IsEW: true, RoadCenter: -3600}
		c.JunctionTurn = &sim.CarJunctionTurn{Straight: true, Center: center, ExitHeading: math.Pi}
		b := sim.NewBus(-11337.8035, -3668.8362, 3.13927)
		b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: -3678}, Heading: math.Pi, Junction: center, Remaining: .3}
		switch guard {
		case "moving":
			c.Speed = 70
		case "different_junction":
			b.LaneMergeRequest.Junction.X += 1000
		case "turning":
			c.JunctionTurn.Straight = false
		case "wide_merge":
			b.LaneMergeRequest.Target.Y -= 52
		}
		tm := &TrafficManager{}
		f := carFrame{car: c, world: &sim.World{}, laneWidth: 52, buses: []*sim.Bus{b}}
		target := tm.busMergeYieldSpeed(f, 100)
		if guard == "" {
			if target != 0 || c.YieldingToBus != b || c.JunctionTurn == nil {
				t.Fatal("committed rear car closed the small merge gap")
			}
		} else if target != 100 || c.YieldingToBus != nil {
			t.Fatalf("%s changed priority", guard)
		}
	}
}

// Entering the roundabout flag must not hide the car still directly ahead
// on its straight approach from the following driver.
func TestStraightFollowerKeepsNewRoundaboutEntryHeadway(t *testing.T) {
	follower := &sim.TrafficCar{ID: 1668, Pos: sim.Vec2{X: 78, Y: 416}, Heading: -math.Pi / 2, Speed: 30, Length: 46, Width: 22}
	lead := &sim.TrafficCar{ID: 2610, Pos: sim.Vec2{X: 78, Y: 367}, Heading: -math.Pi / 2, Length: 46, Width: 22, InRoundabout: true}
	lead.RoundaboutCurve = newRoundaboutCurve(lead.Pos, sim.Vec2{X: 78, Y: 330}, sim.Vec2{X: 100, Y: 300}, sim.Vec2{X: 130, Y: 270}, false)
	tm := &TrafficManager{Cars: []*sim.TrafficCar{follower, lead}}
	f := carFrame{car: follower, index: 0, activeCount: 2, laneWidth: 52}
	if speed := tm.carFollowingSpeed(f, 150); speed != 0 || follower.Speed != 0 {
		t.Fatalf("close entering leader ignored: target%.1f actual%.1f", speed, follower.Speed)
	}
	lead.Pos.X += 104
	follower.Speed = 30
	if speed := tm.carFollowingSpeed(f, 150); speed != 150 {
		t.Fatal("separate entry lane blocked", speed)
	}
}

func TestRetreatBlockedCarPassValidatesPredictedSegmentsAndLongerReturn(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 1000, Y: 3600}, P2: sim.Vec2{X: 3000, Y: 3600}, Lanes: 4}}}
	car := &sim.TrafficCar{
		ID: 2400, Pos: sim.Vec2{X: 2111.211449216759, Y: 3538.4524075642694},
		Heading: 2.7597631045883304, TargetSpeed: 222, CanOvertake: true,
		Length: 46, Width: 22, RoadCenter: 3600, IsEW: true, LaneIndex: 1,
	}
	bus := sim.NewBus(2156.0278310974663, 3574.0006626795885, 3.1415302472338666)
	// Start with a blocked optional pass whose continuation hits the stopped bus,
	// and verify a naive 60px return would clip the bus along turn.Predict(18..21).
	u := sim.UnitFromAngle(math.Pi)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	offset := car.Pos.Sub(sim.Vec2{Y: 3600}).Dot(r)
	source := 1.5 * 52.0
	yaw := sim.NormalizeAngle(car.Heading - math.Pi)
	shortPts := make([]sim.Vec2, 25)
	for i := range shortPts {
		tt := float64(i) / 24.0
		shift := (source-offset)*tt*tt*(3-2*tt) + math.Tan(yaw)*60*tt*(1-tt)*(1-tt)
		shortPts[i] = car.Pos.Add(u.Mul(60 * tt)).Add(r.Mul(shift))
	}
	shortTurn := &sim.CarJunctionTurn{
		LaneChange: true, Points: shortPts, ExitHeading: math.Pi,
		ExitEW: true, ExitRoadCenter: 3600, ExitLane: 1, SpeedLimit: carRecoveryLaneChangeSpeed,
	}
	if recoveryLaneChangeClearsStoppedBus(
		&sim.TrafficCar{Pos: car.Pos, Heading: car.Heading, Length: 46, Width: 22, JunctionTurn: shortTurn}, bus,
	) {
		t.Fatal("expected short 60px return curve to collide with stopped bus along predicted segments")
	}

	car.JunctionTurn = shortTurn
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	for tick := 0; tick < 180; tick++ {
		tm.StepCar(1./60, car, 0, bus, nil, 1, 52, w)
		body := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
		if hit, _, _ := bus.CollidesBody(body); hit {
			t.Fatalf("recovery curve struck stopped bus at tick %d", tick)
		}
	}
	if car.Pos.X > 2040 || math.Abs(car.Pos.Y-3522) > 2 {
		t.Fatalf("car failed to complete longer safe return into source lane: pos=%v speed=%.1f", car.Pos, car.Speed)
	}
}
