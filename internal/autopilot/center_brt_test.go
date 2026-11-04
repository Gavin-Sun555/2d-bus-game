package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestCenterBRTCruiseAndStraightApproach(t *testing.T) {
	const lw = 52.
	for _, berserk := range []bool{false, true} {
		w := busLaneTestWorld(lw)
		w.CenterBRT = true
		w.Roads[0].MedianBusLane = true
		for _, s := range w.Stops {
			s.CenterBRT = true
			s.UpdateGeometry(lw)
		}
		ad := NewAutopilotDriver()
		ad.BerserkMode = berserk
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)
		for _, x := range []float64{-1300, -500} {
			bus := sim.NewBus(x, lw*.5, 0)
			bus.Speed = 100
			ad.trackPath(bus.Pos)
			f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: 100, remaining: 2600, targetStop: w.Stops[1]}
			if ad.updateRequiredApproachMerge(f, .016) {
				t.Fatal("central BRT forced out of lane before straight junction")
			}
			if !ad.updateBusLane(f, .016) {
				t.Fatal("BRT policy not applied")
			}
			if math.Abs(ad.TargetLaneOffset) > 1 {
				t.Fatal("bus left central lane", ad.TargetLaneOffset)
			}
		}
		for _, p := range ad.drivePath {
			if math.Abs(p.Y-lw*.5) > 1e-6 {
				t.Fatal("straight BRT route leaves central lane", p)
			}
		}
	}
}

func TestCenterBRTDrivingTurnsAndDocking(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
		for _, turn := range []float64{0, -math.Pi / 2, math.Pi / 2, math.Pi} {
			for _, h := range []float64{0, math.Pi / 2} {
				t.Run(fmt.Sprintf("model%d/turn%.2f/h%.2f", model, turn, h), func(t *testing.T) {
					const lw = 52.
					in := sim.UnitFromAngle(h)
					right := sim.Vec2{X: -in.Y, Y: in.X}
					out := sim.UnitFromAngle(h + turn)
					w := &sim.World{CenterBRT: true, Roads: []sim.RoadSegment{
						{P1: in.Mul(-3500), P2: in.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
						{P1: right.Mul(-3500), P2: right.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
					}}
					w.RebuildIntersections()
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", in.Mul(-2200), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", out.Mul(2200), h+turn, 8, sim.RGBA(0, 0, 0, 255), lw)}
					for _, s := range w.Stops {
						s.CenterBRT = true
						s.UpdateGeometry(lw)
					}
					b := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, h)
					b.SetModel(model)
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					ad := NewAutopilotDriver()
					ad.DisallowOncomingOvertake = true
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					docked := false
					for tick := 0; tick < 9000; tick++ {
						tls.Update(1. / 60)
						th, br, st, hb, doors, dock := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
						b.Update(1./60, th, br, st, hb, doors)
						if r := b.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
							t.Fatalf("struck boundary tick%d pos%v heading%.2f %s", tick, b.Pos, b.Heading, r.ObstacleName)
						}
						if dock {
							docked = true
							break
						}
					}
					if !docked {
						t.Fatalf("did not dock pos%v h%.2f diag%+v", b.Pos, b.Heading, ad.DiagnoseStop(b, route, w, tls, nil, lw))
					}
				})
			}
		}
	}
}
