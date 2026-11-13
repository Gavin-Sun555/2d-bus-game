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
