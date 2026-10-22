package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStoppedObliqueBusRecoversWithoutWorseningMedianClearance(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, yaw := range []float64{-.23, .23} {
				t.Run(fmt.Sprintf("width%.0f/h%.2f/yaw%.2f", lw, h, yaw), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 4}}}
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-2000), h, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2000), h, 4, sim.RGBA(0, 0, 0, 255), lw)}
					bus := sim.NewBus(0, 0, h+yaw)
					extent := bus.Length/2*math.Abs(math.Sin(yaw)) + bus.Width/2*math.Cos(yaw)
					bus.Pos = r.Mul(extent - 2)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
					ad.LaneOffset = bus.Pos.Dot(r) - 1.5*lw
					ad.TargetLaneOffset = 0
					ad.OvertakeState = OvertakeReturning
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					recovered := false
					for tick := 0; tick < 1800; tick++ {
						before := bus.Pos
						th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, d)
						if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.001 {
							t.Fatal("recovery teleported")
						}
						clearance := roadMedianClearance(bus, bus.Pos, bus.Heading, h, 0, math.Abs(u.X) > .5)
						if clearance < -2.05 {
							t.Fatalf("recovery moved farther into opposing lane tick%d clearance%.3f", tick, clearance)
						}
						if bus.Pos.Dot(u) > 120 && clearance > 2 && math.Abs(sim.NormalizeAngle(bus.Heading-h)) < .1 {
							recovered = true
							break
						}
					}
					if !recovered {
						t.Fatalf("still stuck: pos%v yaw%.2f hold%v speed%.2f", bus.Pos, sim.NormalizeAngle(bus.Heading-h), ad.roadSafetyHold, bus.Speed)
					}
				})
			}
		}
	}
}

func TestRoadRecoveryRetainsRedLightAndFrontQueueStops(t *testing.T) {
	for _, scenario := range []string{"red light", "stopped front car"} {
		t.Run(scenario, func(t *testing.T) {
			const lw = 52.
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 4}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4}}}
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{X: 2000}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)}
			w.RebuildIntersections()
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			tls.Intersections[0].Phase = sim.PhaseNSGreen
			line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)
			bus := sim.NewBus(-line-48-8, 26, .23)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			var obs []VehicleObstacle
			if scenario == "stopped front car" {
				tls = nil
				bus.Pos.X = -1000
				obs = []VehicleObstacle{{Pos: bus.Pos.Add(sim.Vec2{X: 85}), Heading: 0, Length: 48, Width: 22}}
			}
			before := bus.Pos
			for tick := 0; tick < 120; tick++ {
				th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
				bus.Update(1./60, th, br, st, hb, d)
			}
			if bus.Pos.Distance(before) > .1 {
				t.Fatalf("recovery bypassed %s: moved%.2f", scenario, bus.Pos.Distance(before))
			}
		})
	}
}

func TestRoadRecoveryRejectsOncomingBodyInSweep(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 4}}}
	bus := sim.NewBus(0, 26, .23)
	ad := NewAutopilotDriver()
	f := autopilotFrame{bus: bus, world: w, laneWidth: 52, obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: 70, Y: 26}, Heading: math.Pi, Speed: 40, Length: 48, Width: 22}}}
	if _, ok := ad.roadLaneRecovery(f, -.1, 0, 0, true); ok {
		t.Fatal("recovery ignored oncoming vehicle in swept body")
	}
}
