package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestArticulatedTurnsAndDocking(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, turn := range []float64{-math.Pi / 2, math.Pi / 2, math.Pi} {
					t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f/turn%.2f", lw, lanes, h, turn), func(t *testing.T) {
						in := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						out := sim.Vec2{X: math.Cos(h + turn), Y: math.Sin(h + turn)}
						rotate := func(p sim.Vec2) sim.Vec2 { return in.Mul(p.X).Add(right.Mul(p.Y)) }
						typ := sim.RoadStreet2Lane
						if lanes == 8 {
							typ = sim.RoadBusBoulevard8Lane
						}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -2500}), P2: rotate(sim.Vec2{X: 2500}), Lanes: lanes, Type: typ}, {P1: rotate(sim.Vec2{Y: -2500}), P2: rotate(sim.Vec2{Y: 2500}), Lanes: lanes, Type: typ}}}
						w.RebuildIntersections()
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", in.Mul(-1500), h, lanes, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", out.Mul(1500), h+turn, lanes, sim.RGBA(0, 0, 0, 255), lw)}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						sim.EnsureLongBusBays(w, route)
						b := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, h)
						b.SetModel(sim.BusArticulated)
						ad := NewAutopilotDriver()
						ad.DisallowOncomingOvertake = true
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						docked := false
						maxBend := 0.
						restored := false
						for tick := 0; tick < 6000; tick++ {
							tls.Update(1. / 60)
							th, br, steer, hb, doors, dock := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
							b.Update(1./60, th, br, steer, hb, doors)
							maxBend = math.Max(maxBend, math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())))
							if !restored && maxBend > .2 {
								data, err := json.Marshal(ad.Snapshot())
								if err != nil {
									t.Fatal(err)
								}
								var snap DriverSnapshot
								if err = json.Unmarshal(data, &snap); err != nil {
									t.Fatal(err)
								}
								ad, err = snap.Restore(w)
								if err != nil {
									t.Fatal(err)
								}
								if !ad.articulatedPath {
									t.Fatal("lost long vehicle path on load")
								}
								restored = true
							}
							if r := b.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
								t.Fatalf("body struck curb tick%d front%v heading%.2f tail%v tailH%.2f barrier%s", tick, b.Pos, b.Heading, b.TrailerBody().Center, b.TailHeading(), r.ObstacleName)
							}
							if dock {
								docked = true
								break
							}
						}
						if !docked {
							t.Fatalf("did not dock front%v h%.2f tail%v tailH%.2f bend%.2f diag%+v", b.Pos, b.Heading, b.TrailerBody().Center, b.TailHeading(), maxBend, ad.DiagnoseStop(b, route, w, tls, nil, lw))
						}
						if maxBend < .05 {
							t.Fatal("trailer never articulated")
						}
					})
				}
			}
		}
	}
}

func TestArticulatedLaneReturnKeepsTailClear(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, berserk := range []bool{false, true} {
				for _, blocked := range []bool{false, true} {
					t.Run(fmt.Sprintf("w%.0f/h%.2f/berserk%v/blocked%v", lw, h, berserk, blocked), func(t *testing.T) {
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
						w.RebuildIntersections()
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-1600), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2000), h, 8, sim.RGBA(0, 0, 0, 255), lw)}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						sim.EnsureLongBusBays(w, route)
						p := u.Mul(-700).Add(r.Mul(2.5 * lw))
						b := sim.NewBus(p.X, p.Y, h)
						b.SetModel(sim.BusArticulated)
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						// Fleet buses still return to their default lane; berserk player
						// retention on equal-speed lanes has separate policy coverage.
						ad.FleetBus = true
						ad.DisallowOncomingOvertake = true
						ad.CanSkipTargetStop = true
						ad.articulatedPath = true
						ad.drivePath = []sim.Vec2{u.Mul(-1600).Add(r.Mul(3.5 * lw)), u.Mul(2000).Add(r.Mul(3.5 * lw))}
						ad.cumLen = []float64{0, 3600}
						ad.pathWorld = w
						ad.pathFromID = 0
						ad.pathToID = 1
						ad.pathLane = lw
						ad.pathSkipTarget = true
						ad.pathRevision = currentDrivePathRevision
						ad.bypassStopID = 1
						ad.holdLaneOffset(-lw)
						var obs []VehicleObstacle
						if blocked {
							obs = []VehicleObstacle{{Pos: u.Mul(-700).Add(r.Mul(3.5 * lw)), Heading: h, Length: 46, Width: 22}}
						}
						settled := false
						for tick := 0; tick < 3600; tick++ {
							th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, obs, lw, 1./60)
							b.Update(1./60, th, br, st, hb, doors)
							for _, ob := range obs {
								if hit, _, _ := b.CollidesBody(sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
									t.Fatal("lane return struck car with tail", tick, b.Pos, b.TrailerBody())
								}
							}
							if b.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
								t.Fatalf("lane return hit road edge tick%d front%v h%.3f tail%+v error%+v", tick, b.Pos, b.Heading, b.TrailerBody(), b.CheckBoundaries(w, lw))
							}
							aligned := true
							for _, part := range b.BodyParts() {
								if math.Abs(part.Center.Dot(r)-3.5*lw) > 5 || math.Abs(sim.NormalizeAngle(part.Heading-h)) > .07 {
									aligned = false
								}
							}
							if aligned && b.Pos.Dot(u) > -100 {
								settled = true
								break
							}
						}
						if !settled {
							t.Fatalf("return stuck at%v h%.2f tail%v diagnosis%+v", b.Pos, b.Heading, b.TrailerBody(), ad.DiagnoseStop(b, route, w, nil, obs, lw))
						}
					})
				}
			}
		}
	}
}

func TestArticulatedBarrierUnwindSteer(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}},
		Stops: []*sim.BusStop{
			sim.NewBusStop(0, "from", sim.Vec2{X: -1200, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 255), lw),
			sim.NewBusStop(1, "to", sim.Vec2{X: 1200, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 255), lw),
		},
	}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	sim.EnsureLongBusBays(w, route)
	// Place articulated bus on the shoulder outside the right guardrail angled back toward the road,
	// where full left lock (-1) swings the trailer rear corner into the roadside guardrail.
	bus := sim.NewBus(-307.8, 127.6, -0.156)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = 0.090
	bus.SteeringAngle = -bus.MaxSteerAngle

	ad := NewAutopilotDriver()
	ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)

	f := autopilotFrame{
		bus:       bus,
		world:     w,
		laneWidth: lw,
		remaining: 1500,
	}
	// Find an exact lateral offset along Y where desired=-1 clips the barrier while unwind clears.
	found := false
	for dy := -20.0; dy <= 20.0; dy += 2.0 {
		bus.Pos.Y = 127.6 + dy
		if ad.articulatedSweptSpeed(f, -1.0) == 0 {
			if alt, ok := ad.articulatedBarrierUnwindSteer(f, -1.0); ok && alt > -1.0 {
				found = true
				break
			}
		}
	}
	if !found {
		t.Fatal("failed to construct barrier-clipped steer scenario")
	}

	origin := bus.Pos
	for tick := 0; tick < 240; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if r := bus.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("unwind struck hard barrier at tick %d: %s", tick, r.ObstacleName)
		}
	}
	if moved := bus.Pos.Sub(origin).X; moved < 40 {
		t.Fatalf("articulated bus stayed pinned beside barrier (dx=%.1f)", moved)
	}
}

func TestPursuitSteeringBackwardsAlphaUsesLocalRoadBeforeDistantStop(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: 0, Y: 1000}, P2: sim.Vec2{X: 0, Y: -2000}, Lanes: 4},
			{P1: sim.Vec2{X: 0, Y: -2000}, P2: sim.Vec2{X: 3000, Y: -2000}, Lanes: 4},
		},
		Stops: []*sim.BusStop{
			sim.NewBusStop(0, "from", sim.Vec2{X: 0, Y: 800}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw),
			// Distant target stop lies Eastbound (heading 0), orthogonal to the local Northbound road.
			sim.NewBusStop(1, "to", sim.Vec2{X: 2500, Y: -2000}, 0, 4, sim.RGBA(0, 0, 0, 255), lw),
		},
	}
	bus := sim.NewBus(140, -350, -0.10)
	bus.SetModel(sim.BusArticulated)

	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	s, _ := ad.trackPath(bus.Pos)

	f := autopilotFrame{
		bus:        bus,
		world:      w,
		laneWidth:  lw,
		targetStop: w.Stops[1],
		s:          s,
		remaining:  ad.cumLen[len(ad.cumLen)-1] - s,
		distToStop: bus.Pos.Distance(w.Stops[1].Pos),
	}
	steer, _, targetPt, _ := ad.pursuitSteering(f)
	toTarget := targetPt.Sub(bus.Pos)
	rawAlpha := sim.NormalizeAngle(math.Atan2(toTarget.Y, toTarget.X) - bus.Heading)
	if math.Abs(rawAlpha) <= math.Pi*0.5 {
		t.Fatalf("expected |rawAlpha| > pi/2 in setup, got %.3f", rawAlpha)
	}
	if steer >= -0.5 {
		t.Fatalf("expected strong left steer back toward northbound road (-pi/2), got steer=%.3f", steer)
	}
}

func TestBarrierReverseRecoveryUnpinsArticulatedBusWithFollower(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}},
		Stops: []*sim.BusStop{
			sim.NewBusStop(0, "from", sim.Vec2{X: -1200, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 255), lw),
			sim.NewBusStop(1, "to", sim.Vec2{X: 1200, Y: 0}, 0, 4, sim.RGBA(0, 0, 0, 255), lw),
		},
	}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	sim.EnsureLongBusBays(w, route)

	// Place an articulated bus close to the bottom hard barrier angled steeply toward it (+0.85 rad),
	// with full right steering angle, where any forward rollout clips ZoneHardBarrier.
	var bus *sim.Bus
	ad := NewAutopilotDriver()
	found := false
	for y := 150.0; y >= 90.0; y -= 1.0 {
		cand := sim.NewBus(120, y, 0.85)
		cand.SetModel(sim.BusArticulated)
		cand.SteeringAngle = cand.MaxSteerAngle
		if cand.HitsHardBarrier(w, lw) {
			continue
		}
		ad = NewAutopilotDriver()
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		s, lat := ad.trackPath(cand.Pos)
		f := autopilotFrame{
			bus:        cand,
			world:      w,
			laneWidth:  lw,
			s:          s,
			lateralErr: lat,
			remaining:  ad.cumLen[len(ad.cumLen)-1] - s,
			distToStop: cand.Pos.Distance(w.Stops[1].Pos),
		}
		if ad.articulatedSweptSpeed(f, -1.0, true) <= .5 && ad.tryBarrierReverseRecovery(f, -1.0, 1./60) {
			bus = cand
			found = true
			break
		}
	}
	if !found {
		t.Fatal("failed to construct pinned articulated bus scenario for barrier reverse recovery")
	}

	follower := sim.NewBus(-110, 78, 0)
	follower.SetModel(sim.BusArticulated)
	obs := []VehicleObstacle{{
		Pos:     follower.Pos,
		Heading: follower.Heading,
		Speed:   0,
		Length:  follower.OverallLength(),
		Width:   follower.Width,
		Bus:     follower,
	}}

	origin := bus.Pos
	ad.barrierReverse = nil
	sawReverse := false
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Gear == sim.GearReverse {
			sawReverse = true
		}
		if r := bus.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("struck hard barrier at tick %d: %s", tick, r.ObstacleName)
		}
		for _, pa := range bus.BodyParts() {
			for _, pb := range follower.BodyParts() {
				if hit, _, _ := sim.CheckOBBCollision(pa, pb); hit {
					t.Fatalf("collided with follower during recovery at tick %d", tick)
				}
			}
		}
	}
	if !sawReverse {
		t.Fatalf("expected barrier reverse recovery to engage GearReverse")
	}
	if dx := bus.Pos.X - origin.X; dx < 50 {
		t.Fatalf("bus did not recover and resume forward travel (dx=%.1f, pos=%v, h=%.3f)", dx, bus.Pos, bus.Heading)
	}
}

func TestYawedArticulatedDepartureClearsTrailingBayBus(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -3900, Y: -7200}, P2: sim.Vec2{X: -3900, Y: 7200}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, HasLines: true},
		},
	}
	w.RebuildIntersections()
	stopFrom := sim.NewBusStop(0, "BayFrom", sim.Vec2{X: -3900, Y: -6300}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	stopFrom.Length = 336
	stopTo := sim.NewBusStop(1, "BayTo", sim.Vec2{X: -3900, Y: 4500}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	stopTo.Length = 336
	w.Stops = []*sim.BusStop{stopFrom, stopTo}

	// Leader articulated bus has pulled into the adjacent lane at a sharp east-northeast yaw,
	// while a trailing articulated bus sits stationary at the departure mouth behind it.
	lead := sim.NewBus(-4029.1, -6109.2, 0.527)
	lead.SetModel(sim.BusArticulated)
	lead.TrailerHeading = 1.353
	lead.TrailerInitialized = true
	lead.SteeringAngle = 0.54

	rear := sim.NewBus(-4129.0, -6175.7, 1.569)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading = 1.571
	rear.TrailerInitialized = true

	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.State = AutopilotStateCruising
	ad.pathFromID = 0
	ad.pathToID = 1
	ad.drivePath = []sim.Vec2{
		{X: -4129.12, Y: -6300},
		{X: -4082, Y: -5950},
		{X: -4082, Y: 4000},
	}
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	obs := []VehicleObstacle{ObserveBus(rear)}

	startY := lead.Pos.Y
	for tick := 0; tick < 600; tick++ {
		th, br, st, hb, dr, _ := ad.ComputeControls(lead, route, w, nil, obs, lw, 1./60)
		lead.Update(1./60, th, br, st, hb, dr)
		if r := lead.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("struck hard barrier at tick %d: %s", tick, r.ObstacleName)
		}
		for _, pa := range lead.BodyParts() {
			for _, pb := range rear.BodyParts() {
				if hit, _, depth := sim.CheckOBBCollision(pa, pb); hit && depth > 1e-6 {
					t.Fatalf("collided with trailing bus at tick %d: depth=%g", tick, depth)
				}
			}
		}
	}
	if lead.Pos.Y-startY < 800 || lead.Speed < 50 {
		t.Fatalf("yawed articulated departure stalled beside trailing bay bus: pos=%v spd=%.1f", lead.Pos, lead.Speed)
	}
}

func TestYawedArticulatedBusUnwindsAwayFromCurbShelter(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -7800, Y: 0}, P2: sim.Vec2{X: -7800, Y: 5400}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, HasLines: true},
		},
	}
	w.RebuildIntersections()
	stopFrom := sim.NewBusStop(0, "BayFrom", sim.Vec2{X: -7800, Y: 2700}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	stopTo := sim.NewBusStop(1, "BayTo", sim.Vec2{X: -7800, Y: 4500}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{stopFrom, stopTo}

	// Articulated bus yawed inward at bay exit where straightening toward zero
	// hits the right-side glass shelter while full left steer clears.
	bus := sim.NewBus(-8028.2, 2851.7, 1.331)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = 1.644
	bus.TrailerInitialized = true
	bus.Gear = sim.GearDrive

	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.PrepareLeg(w, stopFrom, stopTo, lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	startY := bus.Pos.Y
	for tick := 0; tick < 720; tick++ {
		th, br, st, hb, dr, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, dr)
		if r := bus.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("struck hard barrier at tick %d: %s (pos=%v h=%.3f)", tick, r.ObstacleName, bus.Pos, bus.Heading)
		}
	}
	if bus.Pos.Y-startY < 60 || bus.Speed < 20 {
		d := ad.DiagnoseStop(bus, route, w, nil, nil, lw)
		t.Fatalf("yawed bus stayed pinned at bay shelter: pos=%v spd=%.1f limits=%v", bus.Pos, bus.Speed, d.Limits)
	}
}

func TestArticulatedParallelLeaderUnwindsPastStoppedFollower(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -15150, Y: 7200}, P2: sim.Vec2{X: -6254, Y: 7200}, Lanes: 6},
		},
	}
	w.RebuildIntersections()
	stopFrom := sim.NewBusStop(0, "From", sim.Vec2{X: -15150, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)
	stopTo := sim.NewBusStop(1, "To", sim.Vec2{X: -6500, Y: 7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{stopFrom, stopTo}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	leader := sim.NewBus(-11067.42, 7280.41, 0.1972)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading = -0.0950
	leader.TrailerInitialized = true
	leader.SteeringAngle = 0.5400
	leader.Gear = sim.GearDrive

	follower := sim.NewBus(-11139.87, 7228.38, 0.1814)
	follower.SetModel(sim.BusArticulated)
	follower.TrailerHeading = -0.0427
	follower.TrailerInitialized = true
	follower.SteeringAngle = 0.2205
	follower.Gear = sim.GearDrive

	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.PrepareLeg(w, stopFrom, stopTo, lw)
	obs := []VehicleObstacle{ObserveBus(follower)}

	f := autopilotFrame{
		bus:       leader,
		world:     w,
		laneWidth: lw,
		obstacles: obs,
		remaining: 4000,
	}
	if alt, ok := ad.articulatedBarrierUnwindSteer(f, 1.0); !ok || alt >= 1.0 {
		t.Fatalf("expected unwind(1.0) beside stopped follower, got alt=%.2f ok=%v", alt, ok)
	}

	// Verify a fast car far behind in our lane does not veto unwind/pull-away
	// at the partially unwound pose, while a fast rear car in the target right
	// lane still blocks a lateral cut-in.
	l2 := sim.NewBus(-11063.86, 7281.16, 0.2154)
	l2.SetModel(sim.BusArticulated)
	l2.TrailerHeading = -0.0908
	l2.TrailerInitialized = true
	l2.SteeringAngle = 0.3000
	l2.Gear = sim.GearDrive
	f2 := sim.NewBus(-11139.54, 7228.44, 0.1824)
	f2.SetModel(sim.BusArticulated)
	f2.TrailerHeading = -0.0424
	f2.TrailerInitialized = true
	carBehind := VehicleObstacle{Pos: sim.Vec2{X: -11378.78, Y: 7278.00}, Heading: 0, Speed: 222.2, Length: 46, Width: 22}
	ad2 := NewAutopilotDriver()
	ad2.FleetBus = true
	ad2.PrepareLeg(w, stopFrom, stopTo, lw)
	fr2WithCar := autopilotFrame{bus: l2, world: w, laneWidth: lw, obstacles: []VehicleObstacle{ObserveBus(f2), carBehind}, remaining: 4000}
	if alt2, ok2 := ad2.articulatedBarrierUnwindSteer(fr2WithCar, 1.0); !ok2 || alt2 >= 1.0 {
		t.Fatalf("rear follower car blocked unwind(1.0): alt=%.2f ok=%v", alt2, ok2)
	}
	adjCar := VehicleObstacle{Pos: sim.Vec2{X: -11220, Y: 7330}, Heading: 0, Speed: 222.2, Length: 46, Width: 22}
	if isClosingRearFollowerHit(fr2WithCar, adjCar, 160) {
		t.Fatal("adjacent-lane fast rear car should not be classified as in-lane rear hit")
	}

	startX := leader.Pos.X
	for tick := 0; tick < 180; tick++ {
		th, br, st, hb, dr, _ := ad.ComputeControls(leader, route, w, nil, obs, lw, 1./60)
		leader.Update(1./60, th, br, st, hb, dr)
		if r := leader.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("leader hit hard barrier at tick %d: %s", tick, r.ObstacleName)
		}
		for _, a := range leader.BodyParts() {
			if a.HalfLen <= 0 {
				continue
			}
			for _, b := range follower.BodyParts() {
				if b.HalfLen <= 0 {
					continue
				}
				if hit, _, depth := sim.CheckOBBCollision(a, b); hit && depth > 0 {
					t.Fatalf("leader touched follower at tick %d (depth=%.2f)", tick, depth)
				}
			}
		}
	}
	if leader.Pos.X-startX < 40 {
		d := ad.DiagnoseStop(leader, route, w, nil, obs, lw)
		t.Fatalf("leader stayed pinned beside stopped follower: dx=%.1f spd=%.1f limits=%v", leader.Pos.X-startX, leader.Speed, d.Limits)
	}
}
