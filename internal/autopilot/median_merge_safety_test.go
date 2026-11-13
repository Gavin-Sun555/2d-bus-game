package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func medianMergeScenario(t *testing.T, heading, lw float64, model sim.BusModel) (*sim.World, *sim.Bus, *AutopilotDriver, *sim.Route) {
	t.Helper()
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	center := forward.Mul(4000)
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: forward.Mul(-3000), P2: forward.Mul(7000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
		{P1: center.Sub(right.Mul(3000)), P2: center.Add(right.Mul(3000)), Lanes: 4},
	}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "from", forward.Mul(-2000), heading, 8, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "to", center.Sub(right.Mul(2000)), heading-math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw),
	}
	p := right.Mul(2.5 * lw)
	bus := sim.NewBus(p.X, p.Y, heading)
	bus.SetModel(model)
	bus.MaxSpeedFwd = 1000
	ad := NewAutopilotDriver()
	ad.articulatedPath = bus.IsArticulated()
	ad.BerserkMode, ad.BerserkMaxSpeed = true, 1000
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	ad.ordinaryBusLeft = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	if !ad.joinMedianLeftApproach(autopilotFrame{bus: bus, world: w, laneWidth: lw}, heading, center) {
		t.Fatal("failed to create median merge")
	}
	ad.busLaneJunction = &center
	return w, bus, ad, &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
}

func TestFastMedianApproachKeepsBusOnItsCarriageway(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
				for _, speed := range []float64{300, 667, 1000} {
					t.Run(fmt.Sprintf("h%.2f/width%.0f/model%d/speed%.0f", heading, lw, model, speed), func(t *testing.T) {
						w, bus, ad, route := medianMergeScenario(t, heading, lw, model)
						forward := sim.UnitFromAngle(heading)
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						bus.Speed = speed
						for tick := 0; tick < 600; tick++ {
							th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
							bus.Update(1./60, th, br, st, hb, d)
							for _, part := range bus.BodyParts() {
								if part.HalfLen <= 0 {
									continue
								}
								for _, corner := range sim.BodyCorners(part) {
									if corner.Dot(right) < -.1 {
										t.Fatalf("crossed median at tick%d pos%v heading%.3f speed%.1f hold%v", tick, bus.Pos, bus.Heading, bus.Speed, ad.roadSafetyHold)
									}
								}
							}
							if bus.Pos.Dot(forward) > 800 && math.Abs(sim.NormalizeAngle(bus.Heading-heading)) < .08 {
								return
							}
						}
						t.Fatalf("merge did not complete pos%v speed%.1f hold%v", bus.Pos, bus.Speed, ad.roadSafetyHold)
					})
				}
			}
		}
	}
}

func TestStrandedMedianMergeDrivesBackWithPassengers(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", heading), func(t *testing.T) {
			const lw = 52.
			w, bus, ad, route := medianMergeScenario(t, heading, lw, sim.BusArticulated)
			forward := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			// Pose reduced from the November 7 slot-one save.
			bus.Pos = forward.Mul(560).Sub(right.Mul(85.8278416417))
			bus.Heading = heading - .0310605449
			bus.TrailerHeading = heading - .2450447534
			bus.SteeringAngle = .0767453885
			bus.BoardPassengers(2, []int{1, 1})
			initial := bus.Pos
			for tick := 0; tick < 2400; tick++ {
				before := bus.Pos
				th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
				bus.Update(1./60, th, br, st, hb, d)
				if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.001 {
					t.Fatal("recovery relocated the bus")
				}
				if bus.Pos.Sub(initial).Dot(right) < -.3 || bus.PassengerCount != 2 {
					t.Fatal("recovery moved farther into opposing traffic or lost passengers")
				}
				clear := true
				for _, part := range bus.BodyParts() {
					for _, corner := range sim.BodyCorners(part) {
						clear = clear && corner.Dot(right) > 0
					}
				}
				if clear && math.Abs(sim.NormalizeAngle(bus.Heading-heading)) < .1 && bus.Pos.Sub(initial).Dot(forward) > 400 {
					return
				}
			}
			t.Fatalf("saved pose did not recover pos%v heading%.2f", bus.Pos, bus.Heading)
		})
	}
}

func TestStrandedMedianMergeStopsForVehicleInReturnSweep(t *testing.T) {
	w, bus, ad, _ := medianMergeScenario(t, 0, 52, sim.BusArticulated)
	bus.Pos = sim.Vec2{X: 560, Y: -85.8278416417}
	bus.Heading = -.0310605449
	bus.TrailerHeading = -.2450447534
	bus.SteeringAngle = .0767453885
	ob := VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{X: 78}), Heading: math.Pi, Length: 46, Width: 22}
	f := autopilotFrame{bus: bus, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ob}}
	if _, safe := ad.roadLaneRecovery(f, .65, 0, 0, true); safe {
		t.Fatal("stranded recovery ignored vehicle in checked crawl")
	}
}

func TestStrandedMedianMergeStillWaitsAtRed(t *testing.T) {
	const lw = 52.
	w, bus, ad, route := medianMergeScenario(t, 0, lw, sim.BusArticulated)
	lights := sim.NewTrafficLightSystem()
	lights.BuildForWorld(w)
	inter := lights.Intersections[0]
	inter.Phase = sim.PhaseNSGreen
	bus.Pos = sim.Vec2{X: inter.Center.X - inter.GetStopLineDistForHeading(0, lw) - bus.Length/2 - 10, Y: -85.8278416417}
	bus.Heading = -.0310605449
	bus.TrailerHeading = -.2450447534
	bus.SteeringAngle = .0767453885
	initial := bus.Pos
	for tick := 0; tick < 120; tick++ {
		th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, lights, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, d)
	}
	if bus.Pos.Distance(initial) > .01 {
		t.Fatal("return from opposing side bypassed the red light")
	}
}
