package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func serviceTestWorld(h, lw float64) (*sim.World, sim.Vec2, sim.Vec2) {
	u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
	for i, x := range []float64{-1800, 0, 1800} {
		w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
	}
	return w, u, r
}

func TestBerserkAngledBayArrivalServesAndLeaves(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("h%.2f/w%.0f", h, lw), func(t *testing.T) {
				w, u, r := serviceTestWorld(h, lw)
				stop := w.Stops[1]
				p := stop.FrontBerthPosition().Sub(u.Mul(18)).Sub(r.Mul(14))
				bus := sim.NewBus(p.X, p.Y, h+.297)
				bus.SetModel(sim.BusArticulated)
				bus.TrailerHeading = h + .328
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				opened := false
				for tick := 0; tick < 1800; tick++ {
					th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
					if tick == 0 && (!dock || !doors) {
						t.Fatal("safe angled arrival did not immediately open")
					}
					bus.Update(1./60, th, br, st, hb, doors)
					opened = opened || (dock && bus.DoorsOpen)
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary tick%d", tick)
					}
					if opened && bus.Pos.Sub(p).Dot(u) > 800 {
						return
					}
				}
				t.Fatalf("failed departure, opened%v distance%.1f", opened, bus.Pos.Sub(p).Dot(u))
			})
		}
	}
}

func TestBerserkOuterBoardingBeforeBlockedFrontMarker(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("h%.2f/w%.0f", h, lw), func(t *testing.T) {
				w, u, r := serviceTestWorld(h, lw)
				stop := w.Stops[1]
				lane := stop.RoadCenter.Add(r.Mul(3.5 * lw)) // actual curb-lane center, not the bay
				p := lane.Sub(u.Mul(240))
				bus := sim.NewBus(p.X, p.Y, h)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 60
				ahead := sim.NewBus(lane.X+u.X*180, lane.Y+u.Y*180, h)
				bay := sim.NewBus(stop.FrontBerthPosition().X, stop.FrontBerthPosition().Y, h)
				obs := []VehicleObstacle{ObserveBus(ahead), ObserveBus(bay)}
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				opened, restored := false, false
				for tick := 0; tick < 2400; tick++ {
					th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					for _, ob := range []*sim.Bus{ahead, bay} {
						if hit, _, _ := sim.CheckBusesCollision(bus, ob); hit {
							t.Fatalf("contact tick%d", tick)
						}
					}
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary tick%d", tick)
					}
					if dock && bus.DoorsOpen && !opened {
						if !stop.ContainsBusInOuterLane(bus, lw) {
							t.Fatal("served outside platform")
						}
						if bus.Pos.Sub(stop.FrontBerthPosition()).Dot(u) > -20 {
							t.Fatal("did not use earlier boarding position")
						}
						opened = true
					}
					if ad.midLaneRecovery != nil && !restored {
						data, err := json.Marshal(ad.Snapshot())
						if err != nil {
							t.Fatal(err)
						}
						var snapshot DriverSnapshot
						if err = json.Unmarshal(data, &snapshot); err != nil {
							t.Fatal(err)
						}
						ad, err = snapshot.Restore(w)
						if err != nil {
							t.Fatal(err)
						}
						if ad.skippedDeparture == nil || !ad.skippedDeparture.served {
							t.Fatal("lost served station departure on load")
						}
						restored = true
					}
					if opened && restored && bus.Pos.Sub(lane).Dot(u) > 700 {
						return
					}
				}
				t.Errorf("did not leave earlier position: pos%v plan%+v diag%+v overtake%d served%+v", bus.Pos, ad.stationDock, ad.DiagnoseStop(bus, route, w, nil, obs, lw), ad.OvertakeState, ad.skippedDeparture)
				t.Logf("mid%+v clear%v wheel%.3f heading%.3f", ad.midLaneRecovery, ad.midLaneSweepClear, bus.SteeringAngle, bus.Heading)
			})
		}
	}
}

func TestArticulatedSkippedStopClearsOccupiedForwardBerth(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			t.Run(fmt.Sprintf("h%.2f/model%d", h, model), func(t *testing.T) {
				const lw = 42.
				w, u, r := serviceTestWorld(h, lw)
				stop := w.Stops[1]
				p := u.Mul(-700).Add(r.Mul(3.5 * lw))
				bus := sim.NewBus(p.X, p.Y, h)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 150
				p = stop.FrontBerthPosition()
				leader := sim.NewBus(p.X, p.Y, h)
				leader.SetModel(model)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.CanSkipTargetStop = true
				ad.BerserkMaxSpeed = 150
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				obs := []VehicleObstacle{ObserveBus(leader)}
				for tick := 0; tick < 600; tick++ {
					th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if br > .001 || doors || dock {
						t.Fatalf("unnecessary stop tick%d speed%.1f target%.1f", tick, bus.Speed, ad.LastTargetSpeed)
					}
					if hit, _, _ := sim.CheckBusesCollision(bus, leader); hit {
						t.Fatal("contact")
					}
					if bus.Pos.Dot(u) > 40 {
						return
					}
				}
				t.Fatal("did not pass occupied forward berth")
			})
		}
	}
}

func TestSkippedStationBayChecksTrailerAndDepartureMotion(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, _, _ := serviceTestWorld(h, 52)
		stop := w.Stops[1]
		p := stop.FrontBerthPosition()
		b := sim.NewBus(p.X, p.Y, h)
		b.SetModel(sim.BusArticulated)
		if !obstacleInsideStationBay(stop, ObserveBus(b)) {
			t.Fatal("aligned forward articulated berth rejected")
		}
		b.TrailerHeading = h + .3
		if obstacleInsideStationBay(stop, ObserveBus(b)) {
			t.Fatal("trailer intruding into road was ignored")
		}
	}
}

func TestServedStationExitStillWaitsForTargetLaneTraffic(t *testing.T) {
	for _, speed := range []float64{0, 180} {
		w, u, r := serviceTestWorld(0, 52)
		stop := w.Stops[1]
		p := stop.RoadCenter.Add(r.Mul(3.5 * 52))
		bus := sim.NewBus(p.X, p.Y, 0)
		bus.SetModel(sim.BusArticulated)
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		ad.PrepareLeg(w, stop, w.Stops[2], 52)
		ad.skippedDeparture = &skippedStopDeparture{stop: stop, position: p, served: true}
		front := sim.NewBus(p.X+180, p.Y, 0)
		side := sim.NewBus(p.X, p.Y-52, 0)
		side.Speed = speed
		f := autopilotFrame{bus: bus, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(front), ObserveBus(side)}}
		ad.prepareServedStationMerge(f)
		if ad.midLaneRecovery != nil {
			t.Fatalf("reserved occupied target lane speed%.0f", speed)
		}
		// No straight-headway relaxation without a successful live actuator sweep.
		ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: p.Sub(r.Mul(52)), Heading: 0}
		if ad.clearsServedStationQueue(f, ObserveBus(front)) {
			t.Fatal("bypassed sweep requirement")
		}
		ad.midLaneSweepClear = true
		if ad.clearsServedStationQueue(f, ObserveBus(side)) {
			t.Fatal("ignored target lane vehicle")
		}
		front.Speed = 20
		if ad.clearsServedStationQueue(f, ObserveBus(front)) {
			t.Fatal("ignored moving front traffic")
		}
		_ = u
	}
}

func TestNarrowStationExitReservesEveryCrossedLane(t *testing.T) {
	for _, offset := range []float64{42, 84} {
		w, _, r := serviceTestWorld(0, 42)
		stop := w.Stops[1]
		p := stop.RoadCenter.Add(r.Mul(3.5 * 42))
		bus := sim.NewBus(p.X, p.Y, 0)
		bus.SetModel(sim.BusArticulated)
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		ad.PrepareLeg(w, stop, w.Stops[2], 42)
		ad.skippedDeparture = &skippedStopDeparture{stop: stop, position: p, served: true}
		front := sim.NewBus(p.X+180, p.Y, 0)
		side := sim.NewBus(p.X, p.Y-offset, 0)
		side.Speed = 180
		f := autopilotFrame{bus: bus, world: w, laneWidth: 42, obstacles: []VehicleObstacle{ObserveBus(front), ObserveBus(side)}}
		ad.prepareServedStationMerge(f)
		if ad.midLaneRecovery != nil {
			t.Fatalf("failed to reserve crossed lane at offset%.0f", offset)
		}
	}
}
