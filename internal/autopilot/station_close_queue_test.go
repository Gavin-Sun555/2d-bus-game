package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestArticulatedStationQueueClosesToLeader(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			for _, dispatched := range []bool{false, true} {
				t.Run(fmt.Sprintf("h%.2f/lead%d/dispatched%v", h, model, dispatched), func(t *testing.T) {
					w, u, _ := serviceTestWorld(h, 52)
					stop := w.Stops[1]
					p := stop.FrontBerthPosition()
					lead := sim.NewBus(p.X, p.Y, h)
					lead.SetModel(model)
					lead.DoorsOpen = true
					p = stop.Pos.Sub(u.Mul(550))
					bus := sim.NewBus(p.X, p.Y, h)
					bus.SetModel(sim.BusArticulated)
					ad := NewAutopilotDriver()
					if dispatched {
						ad.QueueAtStation(stop.ID)
					}
					route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
					for tick := 0; tick < 1800; tick++ {
						th, br, st, hb, d, dock := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(lead)}, 52, 1./60)
						bus.Update(1./60, th, br, st, hb, d)
						if dock {
							ad.DwellTimer = 60
						}
						if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
							t.Fatalf("collision tick%d", tick)
						}
						if bus.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
							t.Fatalf("boundary tick%d", tick)
						}
					}
					tail := math.Inf(1)
					for _, part := range lead.BodyParts() {
						if part.HalfLen <= 0 {
							continue
						}
						for _, c := range sim.BodyCorners(part) {
							tail = math.Min(tail, c.Dot(u))
						}
					}
					gap := tail - bus.Pos.Dot(u) - bus.Length/2
					if gap < 20 || gap > 40 || math.Abs(bus.Speed) > 1 {
						t.Fatalf("queue gap%.1f speed%.1f pos%v target%.1f", gap, bus.Speed, bus.Pos, ad.LastTargetSpeed)
					}
					// Both models can board together. After save/load, closing doors
					// must still permit safe departure behind the moving lead.
					if !bus.DoorsOpen || ad.State != AutopilotStateDocked {
						t.Fatal("second vehicle never boarded")
					}
					ad.DwellTimer = 0
					var err error
					ad, err = ad.Snapshot().Restore(w)
					if err != nil {
						t.Fatal(err)
					}
					departureStart := bus.Pos
					departed := false
					for tick := 0; tick < 1800; tick++ {
						lead.DoorsOpen = false
						lead.Speed = 90
						lead.Pos = lead.Pos.Add(u.Mul(90. / 60))
						th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(lead)}, 52, 1./60)
						bus.Update(1./60, th, br, st, hb, d)
						if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
							t.Fatalf("departure contact tick%d", tick)
						}
						if bus.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
							t.Fatalf("departure boundary tick%d", tick)
						}
						if bus.Pos.Sub(departureStart).Dot(u) > 200 {
							departed = true
							break
						}
					}
					if !departed {
						t.Fatal("second bus did not depart after service")
					}
				})
			}
		}
	}
}

func TestArticulatedRoadAndTaperApproachClosesToDockedArticulatedLeader(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, tc := range []struct {
			name             string
			along, lateral   float64
			yaw, speed       float64
			cruiseSpeedLimit float64
		}{
			{name: "high_speed_road_approach", along: -900, lateral: -47.12, yaw: 0, speed: 444.44, cruiseSpeedLimit: 444.44},
			{name: "standard_road_approach", along: -700, lateral: -47.12, yaw: 0, speed: 150, cruiseSpeedLimit: 0},
			{name: "taper_mid_approach", along: -264, lateral: -27.7, yaw: 0.138, speed: 0, cruiseSpeedLimit: 444.44},
			{name: "taper_close_saved_pose", along: -214.44, lateral: -22.0, yaw: 0.1363, speed: 0, cruiseSpeedLimit: 444.44},
		} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, tc.name), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3500), P2: u.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				w.RebuildIntersections()
				for i, x := range []float64{-1500, 0, 1500} {
					w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
				}
				stop := w.Stops[1]
				front := stop.FrontBerthPosition()
				lead := sim.NewBus(front.X, front.Y, h)
				lead.SetModel(sim.BusArticulated)
				lead.DoorsOpen = true

				start := stop.Pos.Add(u.Mul(tc.along)).Add(r.Mul(tc.lateral))
				bus := sim.NewBus(start.X, start.Y, h+tc.yaw)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = tc.speed

				ad := NewAutopilotDriver()
				ad.CruiseSpeedLimit = tc.cruiseSpeedLimit
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}

				for tick := 0; tick < 900; tick++ {
					th, br, st, hb, d, dock := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(lead)}, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, d)
					if dock {
						ad.DwellTimer = 60
					}
					if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
						t.Fatalf("collision tick%d", tick)
					}
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary tick%d", tick)
					}
				}

				tail, _ := stationBodyRange(lead, u)
				gap := tail - bus.Pos.Dot(u) - bus.Length/2
				lat := math.Abs(bus.Pos.Sub(stop.Pos).Dot(r))
				if gap < 20 || gap > 40 || lat > 10 || math.Abs(bus.Speed) > 1 {
					t.Fatalf("did not close up behind docked articulated bus: gap=%.1f lat=%.1f speed=%.1f pos=%v", gap, lat, bus.Speed, bus.Pos)
				}

				ad.DwellTimer = 0
				departureStart := bus.Pos
				docked := false
				for tick := 0; tick < 1800; tick++ {
					lead.DoorsOpen = false
					lead.Speed = 100
					lead.Pos = lead.Pos.Add(u.Mul(100. / 60))
					th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(lead)}, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, d)
					if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
						t.Fatalf("departure contact tick%d", tick)
					}
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("departure boundary tick%d", tick)
					}
					if bus.Pos.Sub(departureStart).Dot(u) > 200 {
						docked = true
						break
					}
				}
				if !docked {
					t.Fatal("second bus did not depart after service")
				}
			})
		}
	}
}

func TestTwoMovingArticulatedBusesDockAndCloseQueueInBay(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2} {
		for _, cruiseLimit := range []float64{0, 444.44} {
			t.Run(fmt.Sprintf("h%.2f/limit%.0f", h, cruiseLimit), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3500), P2: u.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				w.RebuildIntersections()
				for i, x := range []float64{-1500, 0, 1500} {
					w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
				}
				stop := w.Stops[1]

				leadStart := stop.Pos.Sub(u.Mul(650)).Sub(r.Mul(47.12))
				lead := sim.NewBus(leadStart.X, leadStart.Y, h)
				lead.SetModel(sim.BusArticulated)
				lead.Speed = 150
				leadAd := NewAutopilotDriver()
				leadAd.CruiseSpeedLimit = cruiseLimit
				leadRoute := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}

				followStart := stop.Pos.Sub(u.Mul(1000)).Sub(r.Mul(47.12))
				bus := sim.NewBus(followStart.X, followStart.Y, h)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 150
				ad := NewAutopilotDriver()
				ad.CruiseSpeedLimit = cruiseLimit
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}

				for tick := 0; tick < 1200; tick++ {
					lth, lbr, lst, lhb, ld, ldk := leadAd.ComputeControls(lead, leadRoute, w, nil, []VehicleObstacle{ObserveBus(bus)}, lw, 1./60)
					if ldk {
						leadAd.DwellTimer = 20 // hold lead docked while follower closes in
					}
					lead.Update(1./60, lth, lbr, lst, lhb, ld)

					th, br, st, hb, d, fdk := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(lead)}, lw, 1./60)
					if fdk {
						ad.DwellTimer = 20
					}
					bus.Update(1./60, th, br, st, hb, d)

					if hit, _, _ := sim.CheckBusesCollision(bus, lead); hit {
						t.Fatalf("collision tick%d", tick)
					}
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary tick%d", tick)
					}
				}

				tail, _ := stationBodyRange(lead, u)
				gap := tail - bus.Pos.Dot(u) - bus.Length/2
				lat := math.Abs(bus.Pos.Sub(stop.Pos).Dot(r))
				if gap < 20 || gap > 40 || lat > 10 || math.Abs(bus.Speed) > 1 {
					t.Fatalf("moving pair did not close up in bay: gap=%.1f lat=%.1f speed=%.1f pos=%v", gap, lat, bus.Speed, bus.Pos)
				}
			})
		}
	}
}
