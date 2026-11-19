package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSlot2ApproachFollowerPresteersBeforeRetreat(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	leader := sim.NewBus(-15231.1575977, -7212.8920868, 2.8943448436)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading, leader.TrailerInitialized, leader.SteeringAngle = 3.10485656, true, -.13867869
	follower := sim.NewBus(-15043.0306246, -7263.9380888, -3.1366133444)
	follower.SetModel(sim.BusArticulated)
	follower.TrailerHeading, follower.TrailerInitialized, follower.SteeringAngle = 3.13349388, true, .47430803
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	f := autopilotFrame{bus: follower, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(leader)}}
	if hit, _, _ := sim.CheckBusesCollision(follower, leader); hit {
		t.Fatal("fixture bodies already overlap")
	}
	if !sweepBodiesCollision(follower.BodyParts(), ObserveBus(leader), 0, .1, .1) {
		t.Fatal("fixture must touch prediction padding")
	}
	distance := ad.laneChangeFollowerRetreatDistance(f)
	if distance <= 0 {
		t.Fatal("approach follower did not plan a safe retreat")
	}
	ad.headonRecovery = &HeadonDeadlockRecovery{StartPos: follower.Pos, TargetDistance: distance, YieldHoldRemaining: 4, ParallelRetreat: true}
	start := follower.Pos
	for tick := 0; tick < 60*8; tick++ {
		f.speedAbs = math.Abs(follower.Speed)
		th, br, st, done := ad.headonRecoveryControls(f, 1./60)
		if done {
			break
		}
		follower.Update(1./60, th, br, st, false, false)
		if tick == 0 && follower.Pos != start {
			t.Fatal("bus moved before centering its wheels")
		}
		if hit, _, _ := sim.CheckBusesCollision(follower, leader); hit {
			t.Fatal("retreat hit leader")
		}
	}
	if follower.Pos.Distance(start) < distance-2 {
		t.Fatalf("retreat stalled: %.2f of %.2f", follower.Pos.Distance(start), distance)
	}
	// Existing prediction padding is distinct from an actual overlap: the
	// latter must still prevent a reverse, including at its initial pose.
	overlapping := *follower
	overlapping.Pos = leader.Pos
	f.bus = &overlapping
	if _, clear := ad.simulateReversePose(f, 8, 0); clear {
		t.Fatal("reverse accepted overlapping bodies")
	}
}

func slot2LaneChangePair() (*AutopilotDriver, autopilotFrame, *sim.Bus) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -20000, Y: -7200}, P2: sim.Vec2{X: -10000, Y: -7200}, Lanes: 6}}}
	b := sim.NewBus(-14770.9592656, -7271.3152164, 2.7699707)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized, b.SteeringAngle = 2.82978308, true, .02697326
	rear := sim.NewBus(-14639.9779218, -7265.0248346, 3.11408416)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading, rear.TrailerInitialized = 3.00613225, true
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	f := autopilotFrame{bus: rear, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(b)}}
	return ad, f, b
}

func TestSlot2StoppedLeaderSweep(t *testing.T) {
	ad, f, b := slot2LaneChangePair()
	distance := ad.laneChangeFollowerRetreatDistance(f)
	if distance <= 0 {
		t.Fatal("follower did not offer checked trailer clearance")
	}
	retreated, clear := ad.simulateReversePose(f, distance, 0)
	if !clear {
		t.Fatal("follower retreat is not collision free")
	}
	leaderFrame := autopilotFrame{bus: b, world: f.world, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(retreated)}}
	if ad.articulatedSweptSpeed(leaderFrame, .05, true) < 16 {
		t.Fatal("retreat did not release leader")
	}
}

func TestLaneChangeFollowerRetreatSafetyGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "moving follower", "moving leader", "straight queue", "rear occupied", "player", "leader doors"} {
		t.Run(name, func(t *testing.T) {
			ad, f, leader := slot2LaneChangePair()
			switch name {
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 1)
			case "moving follower":
				f.speedAbs, f.bus.Speed = 5, 5
			case "moving leader":
				leader.Speed = 5
				f.obstacles[0] = ObserveBus(leader)
			case "straight queue":
				leader.Heading = f.bus.Heading
				leader.ResetArticulation()
				f.obstacles[0] = ObserveBus(leader)
			case "leader doors":
				leader.DoorsOpen = true
			case "player":
				ad.FleetBus = false
			case "rear occupied":
				body := f.bus.TrailerBody()
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: body.Center.Sub(sim.UnitFromAngle(body.Heading).Mul(body.HalfLen + 25)), Heading: body.Heading, Length: 46, Width: 24})
			}
			if distance := ad.laneChangeFollowerRetreatDistance(f); distance != 0 {
				t.Fatalf("unsafe retreat for %s: %.1f", name, distance)
			}
		})
	}
}

func TestTrailerRecenterDoesNotReserveStoppedFollowerPassingGap(t *testing.T) {
	b := sim.NewBus(0, 78, -.0066)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = .1176, true
	f := autopilotFrame{bus: b, laneWidth: 52, obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: -196.41, Y: 78}, Heading: 0, Length: 46, Width: 22}}}
	if !requiredMergeLaneClear(f, 0, 0, 1.118) {
		t.Fatal("stopped rear follower reserved an unnecessary passing gap during cab recenter")
	}
	f.obstacles[0].Speed = 45
	if requiredMergeLaneClear(f, 0, 0, 1.118) {
		t.Fatal("recenter ignored moving rear traffic")
	}
	f.obstacles[0].Speed = 0
	f.obstacles[0].Pos = sim.Vec2{X: 75, Y: 78}
	if requiredMergeLaneClear(f, 0, 0, 1.118) {
		t.Fatal("recenter bypassed front queue")
	}
}

func TestAlignedLeaderWithTurnedWheelsCanRequestRearRetreat(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -20000, Y: -7200}, P2: sim.Vec2{X: -10000, Y: -7200}, Lanes: 6}}}
	leader := sim.NewBus(-15055.520673969202, -7263.962203601109, -3.139195941862948)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading, leader.TrailerInitialized, leader.SteeringAngle = 3.1361074117784336, true, .47786673660446466
	follower := sim.NewBus(-14980.374047843023, -7225.1025912623445, -3.1255254256255576)
	follower.SetModel(sim.BusArticulated)
	follower.TrailerHeading, follower.TrailerInitialized, follower.SteeringAngle = -3.1339273625668764, true, -.03470521240274877
	rear := sim.NewBus(-14806.063693781334, -7262.212446750992, -3.139443606823854)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading, rear.TrailerInitialized = 3.1131665285209777, true
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	f := autopilotFrame{bus: follower, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(leader), ObserveBus(rear)}}
	if distance := ad.laneChangeFollowerRetreatDistance(f); distance <= 0 {
		t.Fatal("aligned cab's turned wheels still froze its rear follower")
	}

}

func TestRecenterKeepsSeparatedCloseFollowerOutOfPassingReservation(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3900, Y: 0}, P2: sim.Vec2{X: -3900, Y: 10000}, Lanes: 8}}}
	b := sim.NewBus(-4082.787917288528, 5048.488375098108, 1.618958857303519)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized, b.SteeringAngle = 2.0093918423611683, true, -.09599189748974811
	rear := sim.NewBus(-4085.660070273174, 4853.393830024341, 1.4086861793329672)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading, rear.TrailerInitialized = 1.3137079100313918, true
	f := autopilotFrame{bus: b, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(rear)}}
	if !requiredMergeLaneClear(f, math.Pi/2, 0, -.787917288528) {
		t.Fatal("less than one pixel of cab recenter was blocked by a separated stationary follower")
	}
	rear.Pos.Y += 8
	f.obstacles[0] = ObserveBus(rear)
	if requiredMergeLaneClear(f, math.Pi/2, 0, -.787917288528) {
		t.Fatal("recenter ignored a follower with overlapping longitudinal bodies")
	}
}
