package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStoppedMidLaneBusFindsClearLane(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/h%.2f/berserk%v", lw, h, berserk), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-5000), P2: u.Mul(5000), Lanes: 6}}}
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-3000), h, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(3000), h, 6, sim.RGBA(0, 0, 0, 255), lw)}
					bus := sim.NewBus(0, 0, h)
					bus.Pos = r.Mul(2 * lw)
					ad := NewAutopilotDriver()
					ad.DisallowOncomingOvertake = true
					ad.BerserkMode = berserk
					ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
					ad.LaneOffset = -.5 * lw
					ad.TargetLaneOffset = 0
					ad.OvertakeState = OvertakeReturning
					obs := []VehicleObstacle{{Pos: u.Mul(100).Add(r.Mul(2.5 * lw)), Heading: h, Length: 48, Width: 22}, {Pos: u.Mul(-180).Add(r.Mul(1.5 * lw)), Heading: h, Length: 48, Width: 22}}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					cleared := false
					for tick := 0; tick < 2400; tick++ {
						before := bus.Pos
						th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, d)
						if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.001 {
							t.Fatal("bus teleported")
						}
						for _, ob := range obs {
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
								t.Fatalf("collision tick%d pos%v", tick, bus.Pos)
							}
						}
						if bus.Pos.Dot(u) > 250 && math.Abs(bus.Pos.Dot(r)-(math.Round(bus.Pos.Dot(r)/lw-.5)+.5)*lw) < 8 && math.Abs(sim.NormalizeAngle(bus.Heading-h)) < .1 {
							cleared = true
							break
						}
					}
					if !cleared {
						t.Fatalf("stuck pos%v offset%.2f target%.2f speed%.1f", bus.Pos, ad.LaneOffset, ad.TargetLaneOffset, ad.LastTargetSpeed)
					}
				})
			}
		}
	}

}

func TestMidLaneRecoveryDoesNotForceIntoOccupiedLane(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 6}}}
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -3000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{X: 3000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw)}
	bus := sim.NewBus(0, 104, 0)
	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	ad.LaneOffset = -26
	ad.TargetLaneOffset = 0
	ad.OvertakeState = OvertakeReturning
	obstacles := []VehicleObstacle{{Pos: sim.Vec2{X: 100, Y: 130}, Heading: 0, Length: 48, Width: 22}, {Pos: sim.Vec2{X: 100, Y: 78}, Heading: 0, Length: 48, Width: 22}}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for tick := 0; tick < 300; tick++ {
		th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, d)
	}
	if bus.Pos.Distance(sim.Vec2{Y: 104}) > 1 || ad.midLaneRecovery != nil {
		t.Fatalf("forced into occupied lane: pos%v", bus.Pos)
	}
}
