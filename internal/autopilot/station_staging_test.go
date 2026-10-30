package autopilot

import (
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A full passenger platform must not force the next bus to wait in traffic.
// Start on the adjacent lane at the exact longitudinal position in the save.
func TestFullStationQueuesInBayThenBoards(t *testing.T)       { testFullStationQueue(t, false) }
func TestLateArticulatedQueueWaitsForSafeRunway(t *testing.T) { testFullStationQueue(t, true) }

func testFullStationQueue(t *testing.T, late bool) {
	t.Helper()
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			if late && model != sim.BusArticulated {
				continue
			}
			for _, berserk := range []bool{false, true} {
				t.Run(fmt.Sprintf("h%.2f/model%d/berserk%v", h, model, berserk), func(t *testing.T) {
					const lw = 52.
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3500), P2: u.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
					w.RebuildIntersections()
					for i, x := range []float64{-1500, 0, 1500} {
						w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
					}
					stop := w.Stops[1]
					p := stop.FrontBerthPosition()
					lead := sim.NewBus(p.X, p.Y, h)
					lead.SetModel(sim.BusArticulated)
					lead.DoorsOpen = true
					rear := sim.NewBus(0, 0, h)
					sim.PlaceBusInStationQueue(rear, stop, []*sim.Bus{lead}, lw)
					rear.DoorsOpen = true
					distance := 212.547
					if !late {
						distance = 500
					}
					p = stop.Pos.Sub(u.Mul(distance)).Sub(r.Mul(47.12))
					bus := sim.NewBus(p.X, p.Y, h)
					bus.SetModel(model)
					ad := NewAutopilotDriver()
					ad.BerserkMode = berserk
					route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
					queued, docked := false, false
					for tick := 0; tick < 2400; tick++ {
						// Hold for 10 seconds, then move forward continuously.
						if tick >= 600 {
							lead.DoorsOpen = false
							lead.Speed = 100
							lead.Pos = lead.Pos.Add(u.Mul(100. / 60))
							rear.Pos = rear.Pos.Add(u.Mul(100. / 60))
							rear.Speed = 100
							rear.DoorsOpen = false
						}
						obs := []VehicleObstacle{ObserveBus(lead), ObserveBus(rear)}
						th, br, st, hb, d, dk := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
						before := bus.Pos
						bus.Update(1./60, th, br, st, hb, d)
						if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.01 {
							t.Fatal("position discontinuity")
						}
						if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
							t.Fatalf("contact tick%d front%v lead%v", tick, bus.Pos, lead.Pos)
						}
						if b := bus.CheckBoundaries(w, lw); b.Zone == sim.ZoneHardBarrier {
							t.Fatalf("barrier tick%d %s", tick, b.ObstacleName)
						}
						if tick < 600 {
							if late && !berserk && bus.Pos.Distance(p) > 1 {
								t.Fatal("late trailer entered an unsafe short slot")
							}
							if dk || bus.DoorsOpen {
								if late && berserk && stop.ContainsBusInOuterLane(bus, lw) && !stop.ContainsBus(bus) {
									// Berserk long buses may serve safely beside a full bay;
									// the physical collision/barrier checks above still apply.
									docked = true
									break
								}
								t.Fatal("boarding on full platform")
							}
							if tick > 450 && math.Abs(bus.Pos.Sub(stop.Pos).Dot(r)) < 10 && math.Abs(bus.Speed) < 3 {
								queued = true
							}
						}
						if tick == 500 {
							data, err := json.Marshal(ad.Snapshot())
							if err != nil {
								t.Fatal(err)
							}
							var snap DriverSnapshot
							if err = json.Unmarshal(data, &snap); err != nil {
								t.Fatal(err)
							}
							ad, err = snap.Restore(w)
							if err != nil {
								t.Fatal(err)
							}
						}
						if tick >= 600 && dk {
							docked = true
							break
						}
					}
					if (!late && !queued) || !docked {
						t.Fatalf("queue%v dock%v pose%v speed%.1f heading%.2f", queued, docked, bus.Pos, bus.Speed, bus.Heading)
					}
				})
			}
		}
	}
}

func TestStationQueueTargetIgnoresRearFollower(t *testing.T) {
	stop := sim.NewBusStop(0, "test", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), 52)
	front := stop.FrontBerthPosition()
	lead := sim.NewBus(front.X, front.Y, 0)
	lead.SetModel(sim.BusArticulated)
	p := stop.Pos.Sub(sim.Vec2{X: 124})
	bus := sim.NewBus(p.X, p.Y, 0)
	rear := sim.NewBus(p.X-120, p.Y, 0)
	target, ok := stationQueueTarget(bus, stop, []VehicleObstacle{ObserveBus(lead), ObserveBus(rear)})
	want := stop.Pos.Sub(sim.Vec2{X: 120})
	if !ok || target.Distance(want) > .01 {
		t.Fatalf("queue follows rear bus: target%v want%v", target, want)
	}
}
