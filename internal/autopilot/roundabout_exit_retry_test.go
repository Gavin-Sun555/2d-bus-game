package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBlockedInnerRoundaboutExitUsesSafeDepartureOrAnotherLap(t *testing.T) {
	for _, missed := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, turn := range []float64{-math.Pi / 2, 0, math.Pi / 2} {
					t.Run(fmt.Sprintf("missed%v/width%.0f/heading%.2f/turn%.2f", missed, lw, heading, turn), func(t *testing.T) {
						w := sim.NewWorld(lw, sim.MapThemeMetropolis)
						rb := w.Roundabouts[0]
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						outgoing := sim.Vec2{X: math.Cos(heading + turn), Y: math.Sin(heading + turn)}
						w.Stops = []*sim.BusStop{
							sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw),
							sim.NewBusStop(1, "to", rb.Center.Add(outgoing.Mul(800)), heading+turn, 4, sim.RGBA(0, 0, 0, 255), lw),
						}
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
						exit := 0
						for i := 1; i < len(ad.drivePath)-1; i++ {
							if ad.drivePath[i].Distance(rb.Center) <= rb.Radius+15 && ad.drivePath[i+1].Distance(rb.Center) > rb.Radius+15 {
								exit = i
								break
							}
						}
						if exit == 0 {
							t.Fatal("no roundabout exit in route")
						}
						end := ad.drivePath[exit].Sub(rb.Center)
						exitAngle := math.Atan2(end.Y, end.X)
						angle := exitAngle + .55
						if missed {
							angle = exitAngle - .25
						}
						radius := roundaboutOuterLaneRadius(&rb) + roundaboutLaneOffset(&rb, sim.NewBus(0, 0, 0), -lw)
						pos := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius))
						bus := sim.NewBus(pos.X, pos.Y, angle-math.Pi/2)
						bus.Speed = 90
						ad.LaneOffset = radius - roundaboutOuterLaneRadius(&rb)
						ad.TargetLaneOffset = ad.LaneOffset
						ad.OvertakeState = OvertakePassing
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						lastAngle := angle
						traveled := 0.0
						docked := false
						usedDirectExit := false
						for step := 0; step < 3000; step++ {
							// A circulating car occupies the outer lane through the first exit.
							// It then continues tangentially away, without a position or yaw jump.
							carAngle := angle + .04 - 120/roundaboutOuterLaneRadius(&rb)*float64(step)/60
							var obstacles []VehicleObstacle
							if carAngle >= exitAngle-.05 {
								obstacles = []VehicleObstacle{{Pos: rb.Center.Add(sim.Vec2{X: math.Cos(carAngle), Y: math.Sin(carAngle)}.Mul(roundaboutOuterLaneRadius(&rb))), Heading: carAngle - math.Pi/2, Speed: 120, Length: 48, Width: 22}}
							} else {
								elapsed := (exitAngle - .05 - carAngle) * roundaboutOuterLaneRadius(&rb) / 120
								start := rb.Center.Add(sim.Vec2{X: math.Cos(exitAngle - .05), Y: math.Sin(exitAngle - .05)}.Mul(roundaboutOuterLaneRadius(&rb)))
								obstacles = []VehicleObstacle{{Pos: start.Add(sim.Vec2{X: math.Sin(exitAngle - .05), Y: -math.Cos(exitAngle - .05)}.Mul(elapsed * 120)), Heading: exitAngle - .05 - math.Pi/2, Speed: 120, Length: 48, Width: 22}}
							}
							if missed {
								obstacles = nil
							}
							th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
							bus.Update(1.0/60, th, br, st, hb, doors)
							usedDirectExit = usedDirectExit || ad.directRoundaboutExit != nil
							if bus.Pos.Distance(rb.Center) < rb.Radius+20 {
								current := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
								delta := sim.NormalizeAngle(lastAngle - current)
								if delta < -.03 {
									t.Fatalf("went clockwise after missing exit at step%d", step)
								}
								traveled += math.Max(0, delta)
								lastAngle = current
								if bus.Pos.Distance(rb.Center) < rb.InnerRadius+bus.Width/2 {
									t.Fatal("cut across central island")
								}
							}
							for _, obs := range obstacles {
								a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}
								b := sim.OBB{Center: obs.Pos, Heading: obs.Heading, HalfLen: 24, HalfWid: 11}
								if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
									t.Fatalf("forced exit into occupied lane at step%d bus%v h%.2f offset%.1f/%.1f target%v car%v pathS%.1f pathLen%d", step, bus.Pos, bus.Heading, ad.LaneOffset, ad.TargetLaneOffset, ad.LastPursuitPt, obs.Pos, ad.LastPathS, len(ad.drivePath))
								}
							}
							if dock {
								docked = true
								break
							}
						}
						if !docked {
							t.Fatalf("never recovered the exit: pos%v", bus.Pos)
						}
						if traveled < 5.8 && (missed || !usedDirectExit) {
							t.Fatalf("should go around again instead of forcing exit: traveled%.2f initial radius%.1f rb inner/outer%.1f/%.1f angle%.2f exit%.2f exitpoint%v", traveled, radius, rb.InnerRadius, rb.Radius, angle, exitAngle, end)
						}
					})
				}
			}
		}
	}
}
