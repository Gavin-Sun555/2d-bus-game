package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestTwoBerthArrivalAllVehicleModels(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for first := sim.BusStandard; first < sim.BusModelCount; first++ {
			for second := sim.BusStandard; second < sim.BusModelCount; second++ {
				t.Run(fmt.Sprintf("h%.2f/%d/%d", h, first, second), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
					w.RebuildIntersections()
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-1500), h, 8, sim.RGBA(0, 0, 0, 255), 52), sim.NewBusStop(1, "to", u.Mul(1000), h, 8, sim.RGBA(0, 0, 0, 255), 52)}
					stop := w.Stops[1]
					p := stop.FrontBerthPosition()
					a := sim.NewBus(p.X, p.Y, h)
					a.SetModel(first)
					a.DoorsOpen = true
					p = stop.Pos.Sub(u.Mul(600))
					b := sim.NewBus(p.X, p.Y, h)
					b.SetModel(second)
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					ad := NewAutopilotDriver()
					ad.DisallowOncomingOvertake = true
					docked := false
					for tick := 0; tick < 3600; tick++ {
						obs := []VehicleObstacle{ObserveBus(a)}
						th, br, st, hb, d, dock := ad.ComputeControls(b, route, w, nil, obs, 52, 1./60)
						b.Update(1./60, th, br, st, hb, d)
						if hit, _, _ := sim.CheckBusesCollision(a, b); hit {
							t.Fatalf("collision tick%d", tick)
						}
						if b.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
							t.Fatalf("curb tick%d pose%v", tick, b.Pos)
						}
						if dock {
							rear, _ := stationBodyRange(a, u)
							expected := stop.Pos.Add(u.Mul(rear - stop.Pos.Dot(u) - sim.StationBerthGap - b.Length/2))
							if b.Pos.Distance(expected) > 12 {
								t.Fatal("wrong berth", b.Pos, expected)
							}
							if !stop.ContainsBus(b) || !b.DoorsOpen {
								t.Fatal("second vehicle did not become a boarding bus")
							}
							docked = true
							break
						}
					}
					if !docked {
						t.Logf("available%v contains%v heading%g speed%g state%v", availableStationBerths(b, stop, []VehicleObstacle{ObserveBus(a)}), stop.ContainsBus(b), b.Heading, b.Speed, ad.State)
						t.Fatalf("failed to dock pos%v diag%+v", b.Pos, ad.DiagnoseStop(b, route, w, nil, []VehicleObstacle{ObserveBus(a)}, 52))
					}
				})
			}
		}
	}
}

func TestLongBerthManeuverKeepsBoundaryBraking(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -1500}, 0, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)}
	p := w.Stops[1].Pos.Sub(sim.Vec2{X: 200})
	b := sim.NewBus(p.X, p.Y, 0)
	b.SetModel(sim.BusArticulated)
	b.Speed = 100
	ad := NewAutopilotDriver()
	ad.articulatedPath = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	target := w.Stops[1].FrontBerthPosition()
	ad.stationDock = &stationDockPlan{stopID: 1, target: target, points: stationDockCurve(b, target, sim.Vec2{X: 1})}
	w.Buildings = []sim.Building{{Pos: p.Add(sim.Vec2{X: 75}), W: 20, H: 80}}
	f := autopilotFrame{bus: b, world: w, targetStop: w.Stops[1], laneWidth: lw, speedAbs: 100, remaining: 296}
	inputs, active := ad.stationDockControls(f)
	if !active || inputs.brake == 0 || inputs.throttle != 0 {
		t.Fatalf("berth controller skipped boundary braking: %+v active%v target%.1f", inputs, active, ad.LastTargetSpeed)
	}
}

func TestStationaryBoardingPoseClearWithAdjacentStoppedCar(t *testing.T) {
	b := sim.NewBus(-108.66261425537517, -936.3812606794801, 1.5426619823700037)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = 1.566, true
	ob := VehicleObstacle{
		Pos:     sim.Vec2{X: -78, Y: -1020.7852968274712},
		Heading: math.Pi / 2,
		Speed:   0,
		Length:  46,
		Width:   22,
	}
	stop := sim.NewBusStop(3, "stop", sim.Vec2{X: 0, Y: -900}, math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), 52)
	w := &sim.World{
		Roads: []sim.RoadSegment{{P1: sim.Vec2{Y: -2000}, P2: sim.Vec2{Y: 0}, Lanes: 4}},
		Stops: []*sim.BusStop{stop},
	}
	f := autopilotFrame{
		bus:        b,
		world:      w,
		targetStop: stop,
		laneWidth:  52,
		speedAbs:   0,
		obstacles:  []VehicleObstacle{ob},
	}
	if !stationBoardingPoseClear(f) {
		t.Fatalf("stationary articulated bus should be clear: stopPos=%v contains=%v bound=%+v", stop.Pos, stop.ContainsBus(b), b.CheckBoundaries(w, 52))
	}
	if !stationFrontArrivalClear(f) {
		t.Fatal("stationFrontArrivalClear should allow non-overlapping stationary obstacle")
	}
}
