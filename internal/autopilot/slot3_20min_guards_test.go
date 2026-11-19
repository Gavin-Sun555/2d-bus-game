package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSteepYawRearBerthRetreatRecoversWithoutStrikingShelter(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		LargeGrid: true,
		Roads:     []sim.RoadSegment{{P1: sim.Vec2{X: -3500, Y: -7200}, P2: sim.Vec2{X: 0, Y: -7200}, Lanes: 6, Type: sim.RoadExpressway6Lane}},
	}
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -3000, Y: -7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)
	stop := sim.NewBusStop(1, "target", sim.Vec2{X: -1950, Y: -7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)
	next := sim.NewBusStop(2, "next", sim.Vec2{X: -600, Y: -7200}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, stop, next}
	route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	sim.StandardizeStopBays(w, lw)
	sim.EnsureLongBusBays(w, route)

	bus := sim.NewBus(stop.Pos.X-27.2245, stop.Pos.Y-16.0903, 1.1147147)
	bus.SteeringAngle = -0.68
	targetPt := sim.Vec2{X: stop.Pos.X - 24.2553, Y: stop.Pos.Y}

	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.FleetBus = true
	ad.BerserkMode = true
	ad.stationDock = &stationDockPlan{
		stopID: stop.ID,
		target: targetPt,
		points: []sim.Vec2{{X: stop.Pos.X - 44.3, Y: stop.Pos.Y - 61.2}, targetPt},
	}

	sawRetreat := false
	docked := false
	for tick := 0; tick < 60*25; tick++ {
		th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Speed < -1 {
			sawRetreat = true
		}
		if r := bus.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
			t.Fatalf("steep rear-berth recovery struck hard barrier (%s) at tick %d", r.ObstacleName, tick)
		}
		if dock && bus.DoorsOpen {
			docked = true
			break
		}
	}
	if !sawRetreat || !docked || !stop.ContainsBus(bus) {
		t.Fatalf("compact bus failed steep rear-berth retreat/dock: retreat=%v docked=%v inBay=%v pos=%v hdg=%.3f",
			sawRetreat, docked, stop.ContainsBus(bus), bus.Pos, bus.Heading)
	}
}

func TestMedianRightMergeDefersAcrossCenterBRTPlatform(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		CenterBRT: true,
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: 6000, Y: 5400}, P2: sim.Vec2{X: 12000, Y: 5400}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
			{P1: sim.Vec2{X: 7800, Y: 3000}, P2: sim.Vec2{X: 7800, Y: 7000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
		},
		Intersections: []sim.Vec2{{X: 7800, Y: 5400}},
	}
	from := sim.NewBusStop(0, "from", sim.Vec2{X: 11200, Y: 5400}, math.Pi, 8, sim.RGBA(0, 0, 0, 255), lw)
	brtMid := sim.NewBusStop(1, "mid-brt", sim.Vec2{X: 9750, Y: 5400}, math.Pi, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(2, "to", sim.Vec2{X: 7800, Y: 4200}, -math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, brtMid, to}
	w.ConfigureCenterBRT(lw)
	sim.StandardizeStopBays(w, lw)

	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	bus := sim.NewBus(9995, 5374, math.Pi)
	bus.SetModel(sim.BusArticulated)
	bus.ResetArticulation()
	bus.Speed = 160

	queuedMedian := sim.NewBus(8350, 5374, math.Pi)
	obs := []VehicleObstacle{ObserveBus(queuedMedian)}

	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.FleetBus = true
	ad.BerserkMode = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	f := autopilotFrame{
		bus: bus, world: w, tls: tls, obstacles: obs,
		laneWidth: lw, s: ad.LastPathS, speedAbs: math.Abs(bus.Speed), remaining: 3000,
	}
	ad.selectMedianRightTurn(f)
	if ad.curbBusRight {
		t.Fatal("selected curb right merge directly into upcoming Center BRT platform divider")
	}
	if ad.updateRequiredApproachMerge(f, 1./60) {
		t.Fatal("started required right merge across Center BRT platform divider")
	}

	// Player autopilot borrowing a general lane on Center BRT also receives outward station flare.
	playerDriver := NewAutopilotDriver()
	if flare := playerDriver.medianGeneralLaneFlare(w, sim.Vec2{X: 9750, Y: 5374}, sim.Vec2{X: 9750, Y: 5374}, math.Pi, lw, lw); flare <= 0 {
		t.Fatalf("expected positive BRT station flare for player autopilot in general lane, got %.2f", flare)
	}
}

func TestStraightJunctionHighSpeedReturnKeepsLegalSide(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		CenterBRT: true,
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: 7800, Y: -4000}, P2: sim.Vec2{X: 7800, Y: 4000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
			{P1: sim.Vec2{X: 4000, Y: -1800}, P2: sim.Vec2{X: 11000, Y: -1800}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
		},
		Intersections: []sim.Vec2{{X: 7800, Y: -1800}},
	}
	from := sim.NewBusStop(0, "from", sim.Vec2{X: 7800, Y: -2700}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: 7800, Y: 2700}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	w.ConfigureCenterBRT(lw)
	sim.StandardizeStopBays(w, lw)

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 1000
	ad.PrepareLeg(w, from, to, lw)

	// 1. Inside the junction, holdStraightJunctionLane must never lock an oncoming negative offset.
	displaced := sim.NewBus(7841, -1800, math.Pi/2)
	s, _ := ad.trackPath(displaced.Pos)
	ad.holdStraightJunctionLane(autopilotFrame{
		bus: displaced, world: w, laneWidth: lw, s: s, remaining: 4200,
	}, 1./60)
	if ad.TargetLaneOffset < -1e-6 {
		t.Fatalf("holdStraightJunctionLane froze wrong-way negative TargetLaneOffset=%.2f", ad.TargetLaneOffset)
	}

	// 2. Returning across 3 lanes at high speed approaching a straight-through junction stays on X <= 7800.
	bus := sim.NewBus(7618, -2250, math.Pi/2)
	bus.Speed = 340
	ad.LaneOffset = 3 * lw
	ad.LaneOffsetStart = 3 * lw
	ad.TargetLaneOffset = 3 * lw
	ad.lastTargetLaneOffset = 3 * lw
	ad.LaneOffsetProgress = 1
	ad.OvertakeState = OvertakeReturning
	ad.TargetLaneOffset = 0

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for tick := 0; tick < 180; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		for _, c := range sim.BodyCorners(bus.BodyEnvelope()) {
			if c.X > 7800.5 {
				t.Fatalf("high-speed straight junction return crossed centerline at tick %d: corner=%v pos=%v hdg=%.3f",
					tick, c, bus.Pos, bus.Heading)
			}
		}
	}
}

func TestStraightBusYieldsToAngledRecoveringCarInAdjacentCorridor(t *testing.T) {
	const lw = 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 1000, Y: 3600}, P2: sim.Vec2{X: 3000, Y: 3600}, Lanes: 4}},
	}
	bus := sim.NewBus(2156.03, 3574.0, math.Pi)
	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.BerserkMode = true
	ad.drivePath = []sim.Vec2{{X: 2800, Y: 3522}, {X: 1200, Y: 3522}}
	ad.cumLen = []float64{0, 1600}
	ad.LaneOffset, ad.TargetLaneOffset = -lw, -lw
	ad.OvertakeState = OvertakePassing

	angledCar := VehicleObstacle{
		Pos: sim.Vec2{X: 2111.21, Y: 3538.45}, Heading: 2.76, Speed: 2, Length: 46, Width: 22,
	}
	f := autopilotFrame{
		bus: bus, world: w, obstacles: []VehicleObstacle{angledCar}, laneWidth: lw,
	}
	if spd := ad.obstacleSpeed(f, 222); spd > 5 {
		t.Fatalf("expected straight bus to hold behind angled recovering car overlapping corridor, got %.1f", spd)
	}

	alignedCar := VehicleObstacle{
		Pos: sim.Vec2{X: 2111.21, Y: 3522.0}, Heading: math.Pi, Speed: 0, Length: 46, Width: 22,
	}
	f.obstacles = []VehicleObstacle{alignedCar}
	if spd := ad.obstacleSpeed(f, 222); spd < 100 {
		t.Fatalf("aligned car in separate adjacent lane unexpectedly held straight bus: %.1f", spd)
	}
}
