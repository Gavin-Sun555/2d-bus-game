package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestIntersectionNeverForcesRightMergeIntoBlockedLanes(t *testing.T) {
	const lw = 52.0
	bus := sim.NewBus(-500, lw/2, 0)
	bus.Speed = 80
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.LaneOffset = -2 * lw
	ad.TargetLaneOffset = ad.LaneOffset
	ad.LaneOffsetProgress = 1
	inter := &sim.TrafficIntersection{IsMajor: true, ApproachLanes: [4]int{6, 6, 6, 6}, ApproachHasRoad: [4]bool{true, true, true, true}}
	obstacles := []VehicleObstacle{
		{Pos: sim.Vec2{X: -500, Y: 1.5 * lw}, Heading: 0, Speed: 80, Length: 48, Width: 22},
		{Pos: sim.Vec2{X: -500, Y: 2.5 * lw}, Heading: 0, Speed: 80, Length: 48, Width: 22},
	}
	f := autopilotFrame{bus: bus, obstacles: obstacles, laneWidth: lw, speedAbs: 80}
	ad.updateIntersectionLane(f, intersectionApproach{approachingInter: inter, isApproachingIntersection: true, distToStopLine: 300}, false, true, 2.5*lw, .016)
	if math.Abs(ad.TargetLaneOffset+2*lw) > .01 {
		t.Fatal("blocked through lanes forced a right merge")
	}
}

func TestBlockedRightReturnKeepsDrivingAndRetriesAfterGap(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, road := range []struct {
			lanes int
			kind  sim.RoadType
		}{{4, sim.RoadAvenue4Lane}, {6, sim.RoadExpressway6Lane}, {8, sim.RoadBoulevard8Lane}, {8, sim.RoadBusBoulevard8Lane}} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, speed := range []float64{80, 200} {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/type%d/h%.2f/speed%.0f", lw, road.lanes, road.kind, heading, speed), func(t *testing.T) {
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: in.Mul(-10000), P2: in.Mul(10000), Lanes: road.lanes, Type: road.kind}}}
						from := sim.NewBusStop(0, "from", in.Mul(-8000), heading, road.lanes, sim.RGBA(0, 0, 0, 255), lw, 0)
						to := sim.NewBusStop(1, "to", in.Mul(8000), heading, road.lanes, sim.RGBA(0, 0, 0, 255), lw, 0)
						w.Stops = []*sim.BusStop{from, to}
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.BerserkMaxSpeed = 200
						ad.PrepareLeg(w, from, to, lw)
						base := 2.5 * lw
						if road.lanes == 4 {
							base = 1.5 * lw
						}
						if road.kind == sim.RoadBusBoulevard8Lane {
							base = 3.5 * lw
						}
						lateral := base - lw
						pos := in.Mul(-4000).Add(right.Mul(lateral))
						bus := sim.NewBus(pos.X, pos.Y, heading)
						bus.Speed = speed
						ad.LaneOffset = -lw
						ad.TargetLaneOffset = 0 // A return selected before a car closed the gap.
						ad.OvertakeState = OvertakeReturning
						ad.LaneOffsetProgress = .2
						ad.LaneOffsetStart = -lw
						ad.lastTargetLaneOffset = 0
						ad.LaneOffsetDuration = .5
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						first := bus.Pos
						for step := 0; step < 180; step++ {
							obs := VehicleObstacle{Pos: bus.Pos.Add(right.Mul(lw)), Heading: heading, Speed: bus.Speed, Length: 48, Width: 22}
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{obs}, lw, .016)
							bus.Update(.016, th, br, st, hb, doors)
							if math.Abs(bus.Pos.Dot(right)-lateral) > 3 {
								t.Fatalf("forced right return at step%d lateral%.1f want%.1f offset%.1f/%.1f", step, bus.Pos.Dot(right), lateral, ad.LaneOffset, ad.TargetLaneOffset)
							}
							if math.Abs(sim.NormalizeAngle(bus.Heading-heading)) > .12 {
								t.Fatal("blocked return caused a sudden yaw")
							}
							a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							b := sim.OBB{Center: obs.Pos, Heading: obs.Heading, HalfLen: 24, HalfWid: 11}
							if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
								t.Fatal("blocked return collided with adjacent car")
							}
						}
						if bus.Pos.Sub(first).Dot(in) < 120 {
							t.Fatal("bus stopped rather than continuing in its clear lane")
						}
						// Once the gap clears, a fresh return request must be
						// accepted. Ordinary six/eight-lane policy may choose to
						// keep its current through lane when both are empty.
						ad.TargetLaneOffset = 0
						ad.holdBlockedRightMerge(autopilotFrame{bus: bus, world: w, laneWidth: lw, s: ad.LastPathS, speedAbs: bus.Speed})
						if ad.TargetLaneOffset != 0 {
							t.Fatal("right merge remained locked after gap opened")
						}
						if road.lanes == 6 || road.kind == sim.RoadBoulevard8Lane {
							return
						}
						returned := false
						for step := 0; step < 600; step++ {
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, .016)
							bus.Update(.016, th, br, st, hb, doors)
							if bus.Pos.Dot(right) > base-5 {
								returned = true
								break
							}
						}
						if !returned {
							t.Fatal("did not return to preferred curb lane after gap opened")
						}

					})
				}
			}
		}
	}
}

func TestRightMergeChecksActualPositionAndEveryCrossedLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, shift := range []float64{1, 2, 3} {
				for _, blocked := range []bool{false, true} {
					t.Run(fmt.Sprintf("width%.0f/h%.2f/shift%.0f/blocked%v", lw, heading, shift, blocked), func(t *testing.T) {
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-3000), P2: forward.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
						base := right.Mul(3.5 * lw)
						pos := base.Sub(right.Mul(shift * lw))
						bus := sim.NewBus(pos.X, pos.Y, heading)
						bus.Speed = 100
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.drivePath = []sim.Vec2{base.Sub(forward.Mul(3000)), base.Add(forward.Mul(3000))}
						ad.cumLen = []float64{0, 6000}
						ad.LaneOffset = -.2 * lw // Planned interpolation has run ahead of the body.
						ad.TargetLaneOffset = 0
						along := 1000.0
						if blocked {
							along = 0
						}
						obstacle := VehicleObstacle{Pos: pos.Add(right.Mul(lw)).Add(forward.Mul(along)), Heading: heading, Speed: 100, Length: 48, Width: 22}
						ad.holdBlockedRightMerge(autopilotFrame{bus: bus, world: w, obstacles: []VehicleObstacle{obstacle}, laneWidth: lw, s: 3000, speedAbs: 100})
						if blocked {
							if math.Abs(ad.TargetLaneOffset+shift*lw) > .001 || ad.TargetLaneOffset != ad.LaneOffset {
								t.Fatal("return ignored actual body position or an intermediate lane")
							}
						} else if ad.TargetLaneOffset != 0 {
							t.Fatal("distant traffic unnecessarily blocked a safe return")
						}
					})
				}
			}
		}
	}
}
