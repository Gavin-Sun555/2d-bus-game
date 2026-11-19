package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestStationMedianReverseRetainsBodyClearance(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		world := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: 4}}}
		world.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-190), heading, 4, sim.RGBA(0, 0, 0, 255), 52)}
		bus := sim.NewBus(r.X*38.082746675336, r.Y*38.082746675336, heading-.47135105693)
		bus.SetModel(sim.BusArticulated)
		bus.TrailerHeading = heading - .0576
		bus.SteeringAngle = -.65 * bus.MaxSteerAngle
		ad := NewAutopilotDriver()
		ad.pathFromID, ad.roadSafetyHold = 0, true
		initialBus := *bus
		blockerBody := bus.TrailerBody()
		blockerPos := blockerBody.Center.Sub(sim.UnitFromAngle(blockerBody.Heading).Mul(blockerBody.HalfLen + 25))
		blocked := autopilotFrame{bus: &initialBus, world: world, laneWidth: 52, obstacles: []VehicleObstacle{{Pos: blockerPos, Heading: heading, Length: 48, Width: 22}}}
		blockedDriver := *ad
		if th, br, _, active := blockedDriver.roadReverseControls(blocked); !active || th != 0 || br != 1 {
			t.Fatal("blocked rear did not hold the bus stationary")
		}
		reversed, finished := false, false
		for tick := 0; tick < 600; tick++ {
			f := autopilotFrame{bus: bus, world: world, laneWidth: 52, speedAbs: math.Abs(bus.Speed)}
			th, br, st, active := ad.roadReverseControls(f)
			if !active {
				finished = true
				break
			}
			before := bus.Pos
			bus.Update(1./60, th, br, st, false, false)
			reversed = reversed || bus.Speed < 0
			if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.001 {
				t.Fatal("reverse relocated the vehicle")
			}
			for _, body := range bus.BodyParts() {
				if body.HalfLen <= 0 {
					continue
				}
				for _, corner := range sim.BodyCorners(body) {
					if corner.Dot(r) < -1e-8 {
						t.Fatal("reverse crossed the median")
					}
				}
			}
			if tick == 10 {
				snapshot := ad.Snapshot()
				restored, err := snapshot.Restore(world)
				if err != nil || !restored.roadReverseActive {
					t.Fatalf("reverse state lost on restore: %v", err)
				}
				ad = restored
			}
		}
		if !reversed || !finished || bus.Gear != sim.GearDrive {
			t.Fatalf("heading %.2f reverse failed to finish", heading)
		}
		if !roadReverseClear(autopilotFrame{bus: bus, world: world, laneWidth: 52}, heading, 0, math.Abs(u.X) > .5) {
			t.Fatal("finished pose lacks rear clearance")
		}
		// A stationary car immediately behind must veto the whole reverse.
		blocker := bus.TrailerBody()
		rear := blocker.Center.Sub(sim.UnitFromAngle(blocker.Heading).Mul(blocker.HalfLen + 24))
		f := autopilotFrame{bus: bus, world: world, laneWidth: 52, obstacles: []VehicleObstacle{{Pos: rear, Heading: heading, Length: 48, Width: 22}}}
		if roadReverseClear(f, heading, 0, math.Abs(u.X) > .5) {
			t.Fatal("reverse ignored the rear car")
		}
	}
}

func TestMidRoadOvertakeReturnReverseClearsHold(t *testing.T) {
	u := sim.UnitFromAngle(0)
	world := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: 4}}}
	bus := sim.NewBus(0, 41.8, -0.57)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -0.10
	bus.SteeringAngle = -bus.MaxSteerAngle

	// Position bus so roadMedianClearance is inside [0, 2] just like an angled overtake returner.
	center, ew, _, _ := sim.CurrentRoadInfo(world, bus.Pos, 0, 52)
	clearance := roadMedianClearance(bus, bus.Pos, bus.Heading, 0, center, ew)
	bus.Pos.Y += 0.25 - clearance

	ad := NewAutopilotDriver()
	ad.pathFromID = -1 // Mid-road, not on any departure road from a stop.
	ad.roadSafetyHold = true

	blockedBus := *bus
	blockerBody := bus.TrailerBody()
	blockerPos := blockerBody.Center.Sub(sim.UnitFromAngle(blockerBody.Heading).Mul(blockerBody.HalfLen + 25))
	blockedFrame := autopilotFrame{
		bus:       &blockedBus,
		world:     world,
		laneWidth: 52,
		obstacles: []VehicleObstacle{{Pos: blockerPos, Heading: 0, Length: 48, Width: 22}},
	}
	blockedDriver := *ad
	if th, br, _, active := blockedDriver.roadReverseControls(blockedFrame); !active || th != 0 || br != 1 {
		t.Fatal("blocked rear did not hold mid-road reversing bus stationary")
	}

	reversed, finished := false, false
	for tick := 0; tick < 600; tick++ {
		f := autopilotFrame{bus: bus, world: world, laneWidth: 52, speedAbs: math.Abs(bus.Speed)}
		th, br, st, active := ad.roadReverseControls(f)
		if !active {
			finished = true
			break
		}
		bus.Update(1./60, th, br, st, false, false)
		reversed = reversed || bus.Speed < 0
		for _, body := range bus.BodyParts() {
			if body.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(body) {
				if corner.Y < -1e-8 {
					t.Fatal("mid-road reverse crossed the median")
				}
			}
		}
	}
	if !reversed || !finished || bus.Gear != sim.GearDrive {
		t.Fatalf("mid-road reverse did not finish cleanly (reversed=%v finished=%v gear=%v)", reversed, finished, bus.Gear)
	}
}

func TestYawedSouthboundArticulatedRoadReverseAndUnwind(t *testing.T) {
	world := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 0, Y: -3000}, P2: sim.Vec2{X: 0, Y: 3000}, Lanes: 6}}}
	bus := sim.NewBus(-28, 0, 0.571)
	bus.SetModel(sim.BusArticulated)
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.roadSafetyHold = true
	ad.drivePath = []sim.Vec2{{X: -26, Y: -200}, {X: -26, Y: 400}}
	ad.cumLen = []float64{0, 600}

	f := autopilotFrame{bus: bus, world: world, laneWidth: 52, s: 200}
	th, br, _, active := ad.roadReverseControls(f)
	if !active || bus.Gear != sim.GearReverse || th <= 0 || br != 0 {
		t.Fatalf("expected Southbound yawed articulated bus to start roadReverse, got active=%v gear=%v th=%.2f br=%.2f", active, bus.Gear, th, br)
	}
	bus.Heading = 0.896
	if !ad.roadReverseForwardUnwindClear(f, math.Pi/2, 0, false, -16, 0.896-math.Pi/2, 1) {
		t.Fatal("expected clear forward unwind for yawed Southbound bus on its own side")
	}
}
