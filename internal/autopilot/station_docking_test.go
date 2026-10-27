package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkDockChoosesAvailableForwardBerthAndDepartsSmoothly(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, lanes := range []int{2, 4, 6} {
				for blocked := 0; blocked <= lanes/2+2; blocked++ {
					// The final case on multi-lane roads blocks every legal berth.
					t.Run(fmt.Sprintf("width%.0f/heading%.2f/lanes%d/blocked%d", lw, heading, lanes, blocked), func(t *testing.T) {
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						rotate := func(p sim.Vec2) sim.Vec2 { return forward.Mul(p.X).Add(right.Mul(p.Y)) }
						w := &sim.World{Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -1500}), P2: rotate(sim.Vec2{X: 1800}), Lanes: lanes}}}
						for i, x := range []float64{-900, 0, 900} {
							w.Stops = append(w.Stops, sim.NewBusStop(i, "test", rotate(sim.Vec2{X: x}), heading, lanes, sim.RGBA(0, 0, 0, 255), lw))
						}
						stop := w.Stops[1]
						targets := []sim.Vec2{stop.FrontBerthPosition(), stop.RearBerthPosition()}
						for lane := 0; lane < lanes/2; lane++ {
							targets = append(targets, forward.Mul(96).Add(right.Mul((float64(lanes/2-lane)-0.5)*lw)))
						}
						if lanes == 2 {
							targets = append(targets, forward.Mul(96).Add(right.Mul(-lw/2)))
						}
						obstacles := []VehicleObstacle{}
						for i := 0; i < blocked; i++ {
							obstacles = append(obstacles, VehicleObstacle{Pos: targets[i], Heading: heading, Length: 96, Width: 36})
						}
						bus := sim.NewBus(-340, lw/2, 0)
						if lw == 78 && lanes == 6 {
							bus.Pos.Y = lw * 1.5
						}
						bus.Speed = 70
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.LaneOffset, ad.TargetLaneOffset = -float64(lanes/2-1)*lw, -float64(lanes/2-1)*lw
						if lanes == 2 {
							bus.Pos.Y = -lw / 2
							ad.LaneOffset = -lw
							ad.TargetLaneOffset = -lw
						}
						bus.Pos = rotate(bus.Pos)
						bus.Heading = heading
						route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
						docked := false
						for step := 0; step < 1800; step++ {
							// Full platforms now stage in the bay instead of boarding in
							// a traffic lane. Clear the passenger berths after a wait.
							if blocked >= 2 && blocked < len(targets) && step == 600 {
								obstacles[0].Pos = obstacles[0].Pos.Add(forward.Mul(600))
								obstacles[1].Pos = obstacles[1].Pos.Add(forward.Mul(600))
							}
							th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
							if blocked >= 2 && step < 600 && dock {
								t.Fatal("boarded beside a full platform")
							}
							bus.Update(1.0/60, th, br, st, hb, doors)
							body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							for _, ob := range obstacles {
								other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
								if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
									t.Fatalf("collision step%d pos%v", step, bus.Pos)
								}
							}
							if dock {
								docked = true
								break
							}
						}
						if blocked == len(targets) {
							if docked {
								t.Fatal("docked in occupied lane")
							}
							return
						}
						if !docked {
							t.Fatalf("did not dock: pos%v speed%.1f plan%+v", bus.Pos, bus.Speed, ad.stationDock)
						}
						// A rear berth occupant can obstruct the swept approach to
						// the nearest roadway lane; select a free reachable lane.
						matched := false
						availableTargets := targets[blocked:]
						if blocked >= 2 {
							availableTargets = targets[:1]
						}
						for _, target := range availableTargets {
							if bus.Pos.Distance(target) < 12 {
								matched = true
							}
						}
						if !matched {
							t.Fatalf("did not use a free forward berth: %v", bus.Pos)
						}
						if blocked < 2 && bus.Pos.Distance(targets[blocked]) > 12 {
							t.Fatal("did not prioritize platform berth")
						}
						// The rear berth may load in parallel but its leader must
						// clear before it can pull out.
						if blocked == 1 {
							obstacles[0].Pos = obstacles[0].Pos.Add(forward.Mul(600))
						}
						// Vehicles in the roadway must also clear before the queued
						// bus can merge out of the newly available platform.
						if blocked >= 2 {
							for i := 2; i < len(obstacles); i++ {
								obstacles[i].Pos = obstacles[i].Pos.Add(forward.Mul(600))
							}
						}
						parkedHeading := bus.Heading
						departed := false
						for step := 0; step < 500; step++ {
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
							oldHeading := bus.Heading
							bus.Update(1.0/60, th, br, st, hb, doors)
							body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							for _, ob := range obstacles {
								other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
								if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
									t.Fatalf("departure collision step%d pos%v", step, bus.Pos)
								}
							}
							if ad.State == AutopilotStateDocked && math.Abs(sim.NormalizeAngle(bus.Heading-parkedHeading)) > 0.01 {
								t.Fatal("yaw while docked")
							}
							if math.Abs(sim.NormalizeAngle(bus.Heading-oldHeading)) > 0.035 || math.Abs(sim.NormalizeAngle(bus.Heading-heading)) > 0.65 {
								t.Fatalf("departure swerved step%d heading%.2f pos%v", step, bus.Heading, bus.Pos)
							}
							if route.CurrentIndex == 2 && bus.Pos.Dot(forward) > 220 {
								departed = true
								break
							}
						}
						if !departed {
							t.Fatalf("did not depart: pos%v speed%.1f diag%+v", bus.Pos, bus.Speed, ad.DiagnoseStop(bus, route, w, nil, obstacles, lw))
						}
					})
				}
			}
		}
	}
}

func TestBerserkFullPlatformQueuesWithoutBorrowingOncomingLane(t *testing.T) {
	const lw = 52.0
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -1500}, P2: sim.Vec2{X: 1800}, Lanes: 2}}}
	for i, x := range []float64{-900, 0, 900} {
		w.Stops = append(w.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 2, sim.RGBA(0, 0, 0, 255), lw))
	}
	route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	bus := sim.NewBus(-340, lw/2, 0)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	yielded, docked := false, false
	for step := 0; step < 1800; step++ {
		obstacles := []VehicleObstacle{
			{Pos: w.Stops[1].Pos, Heading: 0, Length: 96, Width: 36},
			{Pos: sim.Vec2{Y: lw / 2}, Heading: 0, Length: 96, Width: 36},
			{Pos: sim.Vec2{X: 220 - float64(step)*100/60, Y: -lw / 2}, Heading: math.Pi, Speed: 100, Length: 48, Width: 24},
		}
		if step >= 600 {
			obstacles[0].Pos.X += 600
			obstacles[1].Pos.X += 600
		}
		th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
		if step < 600 && dock {
			t.Fatal("boarded on a full platform")
		}
		bus.Update(1.0/60, th, br, st, hb, doors)
		if step < 600 && br > 0 && bus.Pos.Y > 0 {
			yielded = true
		}
		body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
		for _, ob := range obstacles {
			other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
			if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
				t.Fatalf("collision step%d pos%v", step, bus.Pos)
			}
		}
		if dock {
			docked = true
			break
		}
	}
	if !yielded || !docked || bus.Pos.Distance(w.Stops[1].FrontBerthPosition()) > 12 {
		t.Fatalf("did not queue and dock in passenger bay: waited%v docked%v pos%v", yielded, docked, bus.Pos)
	}
}

func TestBerserkDockContinuesBehindMovingLeadCar(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f", lw, lanes, h), func(t *testing.T) {
					forward := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-1500), P2: forward.Mul(1500), Lanes: lanes}}}
					for i, x := range []float64{-900, 0, 900} {
						w.Stops = append(w.Stops, sim.NewBusStop(i, "test", forward.Mul(x), h, lanes, sim.RGBA(0, 0, 0, 255), lw))
					}
					lane := right.Mul((float64(lanes)/2 - .5) * lw)
					pos := forward.Mul(-340).Add(lane)
					bus := sim.NewBus(pos.X, pos.Y, h)
					bus.Speed = 70
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
					for step := 0; step < 1200; step++ {
						car := VehicleObstacle{Pos: forward.Mul(-240 + 100*float64(step)/60).Add(lane), Heading: h, Speed: 100, Length: 48, Width: 22}
						th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{car}, lw, 1.0/60)
						bus.Update(1.0/60, th, br, st, hb, doors)
						body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
						other := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}
						if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
							t.Fatal("collided with moving lead car")
						}
						if w.Stops[1].Pos.Sub(bus.Pos).Dot(forward) > 80 && bus.Speed < 20 {
							t.Fatalf("unnecessary stop before docking step%d speed%.1f pos%v", step, bus.Speed, bus.Pos)
						}
						if dock {
							if ad.stationDock.rank != 0 {
								t.Fatal("did not choose free bay")
							}
							return
						}
					}
					t.Fatal("did not dock")
				})
			}
		}
	}
}

func TestStationDockCurveStillYieldsToMovingConflict(t *testing.T) {
	bus := sim.NewBus(0, 0, 0)
	bus.Speed = 80
	points := []sim.Vec2{{}, {X: 100}, {X: 200}}
	for _, ob := range []VehicleObstacle{
		{Pos: sim.Vec2{X: 100, Y: -100}, Heading: math.Pi / 2, Speed: 80, Length: 48, Width: 22},
		{Pos: sim.Vec2{X: 200}, Heading: math.Pi, Speed: 80, Length: 48, Width: 22},
		{Pos: sim.Vec2{X: 100}, Length: 48, Width: 22},
	} {
		if stationDockCurveClear(bus, points, []VehicleObstacle{ob}) {
			t.Fatalf("ignored conflict: %+v", ob)
		}
	}
}
