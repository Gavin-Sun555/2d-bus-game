package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLeftTurnUsesGeneralLaneBesideStationaryBusQueue(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
			t.Run(fmt.Sprintf("h%.2f/model%d", heading, model), func(t *testing.T) {
				const lw = 52.
				w, _ := turnTestWorld(8, 8, heading, lw)
				for i := range w.Roads {
					w.Roads[i].Type = sim.RoadBusBoulevard8Lane
				}
				in := sim.UnitFromAngle(heading)
				rIn := sim.Vec2{X: -in.Y, Y: in.X}
				out := sim.Vec2{X: in.Y, Y: -in.X}
				rOut := sim.Vec2{X: -out.Y, Y: out.X}
				g := w.JunctionAt(sim.Vec2{}, lw)
				half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				pos := in.Mul(-1000).Add(rIn.Mul(3.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.SetModel(model)
				ad := NewAutopilotDriver()
				ad.CruiseSpeedLimit = 666.67
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
				if math.Abs(in.Y) > .5 {
					tls.Intersections[0].Phase = sim.PhaseNSLeftGreen
				}
				var obstacles []VehicleObstacle
				for _, along := range []float64{half - 220, half + 80, half + 320, half + 560} {
					p := out.Mul(along).Add(rOut.Mul(3.5 * lw))
					other := sim.NewBus(p.X, p.Y, heading-math.Pi/2)
					other.SetModel(sim.BusArticulated)
					other.JunctionCorridor = &sim.JunctionCorridor{Center: sim.Vec2{}, EntryHeading: heading - math.Pi/2, Entered: true, Remaining: 1,
						Bodies: sim.AppendPathBodies(nil, []sim.Vec2{p, p.Add(out.Mul(800))}, other.Length+8, other.Width+6)}
					other.JunctionCorridor.RebuildBounds()
					obstacles = append(obstacles, ObserveBus(other))
				}
				for tick := 0; tick < 5400; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					for _, ob := range obstacles {
						if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
							t.Fatalf("hit fixed outgoing queue tick%d", tick)
						}
					}
					if bus.Pos.Dot(out) > half+350 {
						// Lane membership, not exact centering: a compact bus may
						// leave the turn a few pixels inside the lane center.
						if lateral := bus.Pos.Dot(rOut); math.Min(math.Abs(lateral-2.5*lw), math.Abs(lateral-1.5*lw)) > lw/4 {
							t.Fatalf("did not retain selected general lane: %v", bus.Pos)
						}
						return
					}
				}
				d := ad.DiagnoseStop(bus, route, w, tls, obstacles, lw)
				t.Fatalf("left turn stayed blocked by curb queue: pos%v speed%.1f limits%v", bus.Pos, bus.Speed, d.Limits)
			})
		}
	}
}

func TestLeftTurnExitChoiceRetainsOccupiedLaneAndCommitmentGuards(t *testing.T) {
	for _, allBlocked := range []bool{false, true} {
		t.Run(fmt.Sprint(allBlocked), func(t *testing.T) {
			const lw = 52.
			w, _ := turnTestWorld(8, 8, 0, lw)
			for i := range w.Roads {
				w.Roads[i].Type = sim.RoadBusBoulevard8Lane
			}
			bus := sim.NewBus(-1000, 3.5*lw, 0)
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			f := autopilotFrame{bus: bus, world: w, laneWidth: lw}
			f.s, _ = ad.trackPath(bus.Pos)
			for lane := 3; lane >= 1; lane-- {
				if lane != 3 && !allBlocked {
					continue
				}
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: (float64(lane) + .5) * lw, Y: -600}, Heading: -math.Pi / 2, Length: 96, Width: 36})
			}
			ad.chooseLeftTurnExit(f, sim.Vec2{}, 0)
			if allBlocked {
				if ad.exitCruise != nil {
					t.Fatal("selected an occupied outgoing lane")
				}
				return
			}
			if ad.exitCruise == nil || !ad.exitCruise.LeftTurn {
				t.Fatal("did not choose clear outgoing general lane")
			}
			target := ad.exitCruise.Target
			restored, err := ad.Snapshot().Restore(w)
			if err != nil {
				t.Fatal(err)
			}
			f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: target.X, Y: -600}, Heading: -math.Pi / 2, Length: 96, Width: 36}}
			restored.chooseLeftTurnExit(f, sim.Vec2{}, 0)
			if restored.exitCruise.Target != target {
				t.Fatal("traffic change reversed committed turn exit after restore")
			}
		})
	}
}
