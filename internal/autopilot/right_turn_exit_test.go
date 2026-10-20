package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkRightTurnDoesNotCarryPassingOffsetIntoOncomingLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, road := range []struct {
			lanes int
			kind  sim.RoadType
		}{{4, sim.RoadAvenue4Lane}, {6, sim.RoadExpressway6Lane}, {8, sim.RoadBoulevard8Lane}, {8, sim.RoadBusBoulevard8Lane}} {
			lanes := road.lanes
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for passing := 0; passing < lanes/2; passing++ {
					offset := -float64(passing) * lw
					t.Run(fmt.Sprintf("width%.0f/lanes%d/type%d/h%.2f/offset%.0f", lw, lanes, road.kind, heading, offset), func(t *testing.T) {
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						out := sim.Vec2{X: -in.Y, Y: in.X}
						rightOut := in.Mul(-1)
						w := &sim.World{Roads: []sim.RoadSegment{
							{P1: in.Mul(-2400), P2: in.Mul(2400), Lanes: lanes, Type: road.kind},
							{P1: out.Mul(-2400), P2: out.Mul(2400), Lanes: 2},
						}}
						w.RebuildIntersections()
						from := sim.NewBusStop(0, "from", in.Mul(-1500), heading, lanes, sim.RGBA(0, 0, 0, 255), lw, 0)
						to := sim.NewBusStop(1, "to", out.Mul(1500), heading+math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw, 0)
						w.Stops = []*sim.BusStop{from, to}
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.PrepareLeg(w, from, to, lw)
						// Start on the right-turn curve with an unresolved passing offset,
						// as can happen when approach traffic prevents the normal return.
						var index int
						for i := 1; i < len(ad.drivePath)-1; i++ {
							before := ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize()
							after := ad.drivePath[i+1].Sub(ad.drivePath[i]).Normalize()
							if before.Dot(in) > .7 && after.Dot(out) > .1 && w.InsideJunction(ad.drivePath[i], lw, 0) {
								index = i
								break
							}
						}
						if index == 0 {
							t.Fatal("right-turn curve missing")
						}
						tangent := ad.drivePath[index+1].Sub(ad.drivePath[index]).Normalize()
						pos := ad.drivePath[index].Add(sim.Vec2{X: -tangent.Y, Y: tangent.X}.Mul(offset))
						bus := sim.NewBus(pos.X, pos.Y, math.Atan2(tangent.Y, tangent.X))
						bus.Speed = 75
						ad.LaneOffset, ad.TargetLaneOffset = offset, offset
						ad.OvertakeState = OvertakePassing
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						exit := w.JunctionAt(sim.Vec2{}, lw)
						extent := exit.HalfY
						if math.Abs(out.X) > .5 {
							extent = exit.HalfX
						}
						opposing := VehicleObstacle{Pos: out.Mul(extent + sim.JunctionStopInset + 24).Sub(rightOut.Mul(lw / 2)), Heading: heading - math.Pi/2, Speed: 0, Length: 48, Width: 22}
						docked := false
						for step := 0; step < 2400; step++ {
							th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{opposing}, lw, .016)
							bus.Update(.016, th, br, st, hb, doors)
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: opposing.Pos, Heading: opposing.Heading, HalfLen: 24, HalfWid: 11}); hit {
								t.Fatalf("right turn collided with opposing queue at step%d pos%v", step, bus.Pos)
							}
							if bus.Pos.Dot(out) > extent+bus.Length/2 {
								yaw := sim.NormalizeAngle(bus.Heading - heading - math.Pi/2)
								clearance := bus.Pos.Dot(rightOut) - bus.Length/2*math.Abs(math.Sin(yaw)) - bus.Width/2*math.Abs(math.Cos(yaw))
								if clearance < 0 {
									t.Fatalf("right turn entered oncoming lane: step%d pos%v clearance%.1f offset%.1f/%.1f target%v", step, bus.Pos, clearance, ad.LaneOffset, ad.TargetLaneOffset, ad.LastPursuitPt)
								}
							}
							if dock {
								docked = true
								break
							}
						}
						if !docked {
							t.Fatalf("right turn failed to finish: pos%v heading%.2f offset%.1f", bus.Pos, bus.Heading, ad.LaneOffset)
						}
					})
				}
			}
		}
	}
}

func TestRightTurnOffsetResetWaitsForOccupiedApproachLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("width%.0f/h%.2f", lw, heading), func(t *testing.T) {
				in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
				right := sim.Vec2{X: -in.Y, Y: in.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: in.Mul(-2400), P2: in.Mul(2400), Lanes: 6}, {P1: right.Mul(-2400), P2: right.Mul(2400), Lanes: 2}}}
				w.RebuildIntersections()
				from := sim.NewBusStop(0, "from", in.Mul(-1500), heading, 6, sim.RGBA(0, 0, 0, 255), lw, 0)
				to := sim.NewBusStop(1, "to", right.Mul(1500), heading+math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw, 0)
				w.Stops = []*sim.BusStop{from, to}
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, from, to, lw)
				geometry := w.JunctionAt(sim.Vec2{}, lw)
				half := geometry.HalfX
				if math.Abs(in.Y) > .5 {
					half = geometry.HalfY
				}
				pos := in.Mul(-half - 20).Add(right.Mul(1.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				ad.LaneOffset = -lw
				ad.TargetLaneOffset = ad.LaneOffset
				ad.trackPath(ad.basePathPosition(pos, w, heading, lw))
				car := VehicleObstacle{Pos: pos.Add(right.Mul(lw)), Heading: heading, Length: 48, Width: 22, Speed: 60}
				frame := autopilotFrame{bus: bus, world: w, obstacles: []VehicleObstacle{car}, laneWidth: lw, s: ad.LastPathS, speedAbs: 60, remaining: 2000, distToStop: 2000}
				if !ad.updateRightTurnOffset(frame) || !ad.rightTurnWaiting || ad.LaneOffset != -lw || ad.busTurnHoldDistance <= 0 {
					t.Fatal("right-turn correction forced entry through an occupied lane")
				}
				steer, _, target, _ := ad.pursuitSteering(frame)
				if math.Abs(target.Sub(bus.Pos).Dot(right)) > .001 || math.Abs(steer) > .001 {
					t.Fatal("waiting turn did not keep its incoming direction")
				}
				frame.obstacles = nil
				ad.updateRightTurnOffset(frame)
				if ad.rightTurnWaiting || ad.LaneOffset != 0 || ad.TargetLaneOffset != 0 {
					t.Fatal("right turn did not use the safe gap")
				}
				ad.Reset()
				if ad.rightTurnJunction != nil || ad.rightTurnWaiting {
					t.Fatal("reset retained the old right-turn state")
				}
			})
		}
	}
}
