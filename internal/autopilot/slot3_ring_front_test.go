package autopilot

import (
	_ "embed"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot3-ring-front-20261117.json.gz
var slot3RingFront []byte

//go:embed testdata/slot3-brt-exit-hold-20261117.json.gz
var slot3BRTExitHold []byte

func TestSlot3ArticulatedExitHoldFollowsBRTFlare(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3BRTExitHold)
	s.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.World, 52)
	a := s.Fleet[0]
	b := a.State.Bus
	b.Pos, b.Heading, b.TrailerHeading = sim.Vec2{X: 10300, Y: 5322}, math.Pi, math.Pi
	b.Speed, b.SteeringAngle = 100, 0
	b.ShiftTo(sim.GearDrive)
	ad, err := a.State.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	ad.exitCruise.Target = sim.Vec2{Y: 5322}
	ad.exitCruise.Remaining = 6
	ad.BeforeRecovery = func() { t.Fatal("route recovery during straight platform pass") }
	start := b.Pos
	for tick := 0; tick < 30*60; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, nil, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if b.HitsHardBarrier(s.World, 52) {
			t.Fatal("outgoing lane drove into BRT island", b.Pos)
		}
		if b.Pos.Distance(start) > 1000 {
			return
		}
	}
	t.Fatal("outgoing bus failed to pass platform", b.Pos)
}

func TestBRTFlareDoesNotRequireRearOvertakingSlot(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3BRTExitHold)
	s.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.World, 52)
	a := s.Fleet[0]
	b := a.State.Bus
	b.Pos, b.Heading, b.TrailerHeading = sim.Vec2{X: 10020, Y: 5322}, math.Pi, math.Pi
	b.Speed, b.SteeringAngle = 30, 0
	ad, err := a.State.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	ad.exitCruise.Target = sim.Vec2{Y: 5322}
	ad.exitCruise.Remaining = 6
	rear := sim.NewBus(10210, 5322, math.Pi)
	obs := []VehicleObstacle{ObserveBus(rear)}
	s0, _ := ad.trackPath(b.Pos)
	f := autopilotFrame{bus: b, world: s.World, laneWidth: 52, s: s0, speedAbs: 30, remaining: 5000, obstacles: obs}
	actual := b.Pos.Sub(ad.pointAt(s0)).Dot(sim.Vec2{Y: -1})
	if requiredMergeLaneClear(f, math.Pi, actual, actual+26) {
		t.Fatal("fixture rear bus leaves an overtaking slot")
	}
	if !ad.updateExitCruise(f, 1./60) {
		t.Fatal("fixture did not hold outgoing road lane")
	}
	if ad.pathReturnHold == nil || math.Abs(ad.pathReturnHold.Target.Y-5296) > 1 {
		t.Fatal("same lane flare cancelled by rear bus", ad.pathReturnHold)
	}
}

func TestBRTDividerCrossReservesSpeedAndRearRunway(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3BRTExitHold)
	s.World.ConfigureCenterBRT(52)
	b := s.Fleet[0].State.Bus
	b.Pos, b.Heading = sim.Vec2{X: 10250, Y: 5374}, math.Pi
	b.Speed = 333
	if !brtPlatformBlocksDividerCross(s.World, b, math.Pi, 52, 26, 78) {
		t.Fatal("fast bus admitted a late platform divider crossing")
	}
	b.Speed = 0
	if brtPlatformBlocksDividerCross(s.World, b, math.Pi, 52, 26, 78) {
		t.Fatal("stationary bus with full runway rejected")
	}
	b.Speed = 333
	b.Pos.X = 11000
	if brtPlatformBlocksDividerCross(s.World, b, math.Pi, 52, 26, 78) {
		t.Fatal("distant platform froze a safe crossing")
	}
	b.Pos.X = 9400
	if brtPlatformBlocksDividerCross(s.World, b, math.Pi, 52, 26, 78) {
		t.Fatal("passed platform still blocked rear-clear bus")
	}
	b.Pos.X = 10000
	if brtPlatformBlocksDividerCross(s.World, b, math.Pi, 52, 130, 182) {
		t.Fatal("outer lanes do not cross the platform divider")
	}
}

func TestSlot3FrontBusEntersBesideStoppedRear(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3RingFront)
	s.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.World, 52)
	b := s.Player.Bus
	rear := s.Fleet[0].State.Bus
	ad, err := s.Player.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	route := &sim.Route{StopIDs: []int{ad.pathFromID, ad.pathToID}, CurrentIndex: 1}
	obs := []VehicleObstacle{ObserveBus(rear)}
	start := b.Pos
	for tick := 0; tick < 60*15; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, s.World, s.Lights, obs, 52, 1./60)
		before := b.Pos
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 {
			t.Fatal("bus relocated")
		}
		if hit, _, _ := sim.CheckBusesCollision(b, rear); hit {
			t.Fatal("entry touched rear bus")
		}
		if b.HitsHardBarrier(s.World, 52) {
			t.Fatal("entry crossed barrier")
		}
		if b.Pos.Distance(start) > 300 {
			return
		}
	}
	t.Fatal("front bus remained blocked by stopped rear")
}

func TestOuterBoardingPermissionSurvivesItsApproach(t *testing.T) {
	w, u, r := serviceTestWorld(0, 52)
	stop := w.Stops[1]
	pos := stop.FrontBerthPosition().Sub(r.Mul(47.12))
	b := sim.NewBus(pos.X, pos.Y, 0)
	b.Speed = 20
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.outerDockAllowed = true
	ad.stationDock = &stationDockPlan{stopID: stop.ID, rank: 1, target: pos.Add(u.Mul(10))}
	f := autopilotFrame{bus: b, world: w, targetStop: stop, laneWidth: 52, speedAbs: 20}
	if !stop.ContainsBusInOuterLane(b, 52) || stop.ContainsBus(b) {
		t.Fatal("fixture is not an outer berth")
	}
	ad.updateOuterLaneDock(f, 1./60)
	if !ad.outerDockAllowed {
		t.Fatal("active outer approach lost permission while moving")
	}
	b.Pos = b.Pos.Add(u.Mul(stop.Length + 200))
	ad.updateOuterLaneDock(f, 1./60)
	if ad.outerDockAllowed {
		t.Fatal("permission survived leaving station")
	}
}

func TestRoundaboutArcLeaderIsNotAdjacentCartesianLane(t *testing.T) {
	rb := sim.Roundabout{Radius: 300, InnerRadius: 115}
	for _, rotation := range []float64{0, math.Pi / 2, math.Pi, 3 * math.Pi / 2} {
		angle := 2.5 + rotation
		bus := sim.NewBus(260*math.Cos(angle), 260*math.Sin(angle), angle-math.Pi/2)
		leadAngle := angle - .3
		ob := VehicleObstacle{Pos: sim.Vec2{X: 260 * math.Cos(leadAngle), Y: 260 * math.Sin(leadAngle)}, Heading: leadAngle - math.Pi/2, Length: 46, Width: 22}
		f := autopilotFrame{bus: bus, world: &sim.World{Roundabouts: []sim.Roundabout{rb}}, laneWidth: 52, nearRoundabout: true}
		u := sim.UnitFromAngle(bus.Heading)
		delta := ob.Pos.Sub(bus.Pos)
		if roundaboutLanesSeparated(f, ob, u.X, u.Y, math.Abs(delta.X*u.Y-delta.Y*u.X)) {
			t.Fatal("same circulating lane classified as separate", rotation)
		}
	}
}
