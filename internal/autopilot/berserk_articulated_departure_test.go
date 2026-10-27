package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBerserkArticulatedDepartureClearanceAndPace(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f", lw, lanes, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3500), P2: u.Mul(3500), Lanes: lanes}}}
					if lanes == 8 {
						w.Roads[0].Type = sim.RoadBusBoulevard8Lane
					}
					w.RebuildIntersections()
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{}, h, lanes, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2500), h, lanes, sim.RGBA(0, 0, 0, 255), lw)}
					stop := w.Stops[0]
					p := stop.FrontBerthPosition()
					b := sim.NewBus(p.X, p.Y, h)
					b.SetModel(sim.BusArticulated)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.BerserkMaxSpeed = 667
					ad.State = AutopilotStateDocked
					ad.DockedStopID = 0
					ad.DwellFinished = true
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
					peakYaw, peakBend, peakSpeed := 0., 0., 0.
					for tick := 0; tick < 1200; tick++ {
						th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
						before := b.Pos
						b.Update(1./60, th, br, st, hb, d)
						if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 {
							t.Fatal("position discontinuity")
						}
						if r := b.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
							t.Fatalf("barrier tick%d %s front%v tail%+v", tick, r.ObstacleName, b.Pos, b.TrailerBody())
						}
						if tick == 120 {
							data, err := json.Marshal(ad.Snapshot())
							if err != nil {
								t.Fatal(err)
							}
							var saved DriverSnapshot
							if err = json.Unmarshal(data, &saved); err != nil {
								t.Fatal(err)
							}
							ad, err = saved.Restore(w)
							if err != nil {
								t.Fatal(err)
							}
							if !ad.smoothDockDeparture {
								t.Fatal("smooth departure lost on load")
							}
						}
						peakYaw = math.Max(peakYaw, math.Abs(sim.NormalizeAngle(b.Heading-h)))
						peakBend = math.Max(peakBend, math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())))
						peakSpeed = math.Max(peakSpeed, b.Speed)
						if b.Pos.Sub(p).Dot(u) > 620 {
							if float64(tick)/60 > 5 || peakYaw > .26 || peakBend > .16 {
								t.Fatalf("departure too slow or sharp: elapsed%.2f yaw%.3f bend%.3f", float64(tick)/60, peakYaw, peakBend)
							}
							t.Logf("elapsed%.2f yaw%.3f bend%.3f speed%.1f", float64(tick)/60, peakYaw, peakBend, peakSpeed)
							return
						}
					}
					t.Fatalf("departure stuck front%v speed%.1f", b.Pos, b.Speed)
				})
			}
		}
	}
}

func TestBerserkArticulatedDepartureStillYieldsToTraffic(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, length := range []float64{46, 96} {
			t.Run(fmt.Sprintf("h%.2f/obstacle%.0f", h, length), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				w.RebuildIntersections()
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2500), h, 8, sim.RGBA(0, 0, 0, 255), lw)}
				p := w.Stops[0].FrontBerthPosition()
				b := sim.NewBus(p.X, p.Y, h)
				b.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.BerserkMaxSpeed = 667
				ad.State = AutopilotStateDocked
				ad.DockedStopID = 0
				ad.DwellFinished = true
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 0}
				width := 22.
				if length > 65 {
					width = 36
				}
				for tick := 0; tick < 1500; tick++ {
					movingTicks := math.Max(0, float64(tick-300))
					speed := 0.
					if tick >= 300 {
						speed = 120
					}
					ob := VehicleObstacle{Pos: u.Mul(400 + movingTicks*2).Add(r.Mul(3.5 * lw)), Heading: h, Speed: speed, Length: length, Width: width}
					th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, nil, []VehicleObstacle{ob}, lw, 1./60)
					b.Update(1./60, th, br, st, hb, d)
					other := sim.OBB{Center: ob.Pos.Add(u.Mul(speed / 60)), Heading: h, HalfLen: length / 2, HalfWid: width / 2}
					if hit, _, _ := b.CollidesBody(other); hit {
						t.Fatalf("contact tick%d bus%v", tick, b.Pos)
					}
					if result := b.CheckBoundaries(w, lw); result.Zone == sim.ZoneHardBarrier {
						t.Fatalf("barrier tick%d %s", tick, result.ObstacleName)
					}
					if tick > 300 && b.Pos.Sub(p).Dot(u) > 620 {
						return
					}
				}
				t.Fatalf("failed to resume after traffic moved: %v speed%.1f", b.Pos, b.Speed)
			})
		}
	}
}

func TestFastDeparturePreservesModeAndShortRunCaps(t *testing.T) {
	b := sim.NewBus(0, 0, 0)
	b.SetModel(sim.BusArticulated)
	ad := NewAutopilotDriver()
	ad.dockDepartureRun = 600
	ad.smoothDockDeparture = true
	f := autopilotFrame{bus: b, s: 400}
	if ad.dockDepartureSpeed(f) != 80 {
		t.Fatal("ordinary departure changed")
	}
	ad.BerserkMode = true
	ad.smoothDockDeparture = false
	if ad.dockDepartureSpeed(f) != 80 {
		t.Fatal("legacy departure got an unvalidated speed increase")
	}
	ad.smoothDockDeparture = true
	b.SetModel(sim.BusStandard)
	if ad.dockDepartureSpeed(f) != 80 {
		t.Fatal("standard bus departure changed")
	}
	b.SetModel(sim.BusArticulated)
	ad.dockDepartureRun = 200
	if ad.dockDepartureSpeed(f) != 80 {
		t.Fatal("short departure lost conservative cap")
	}
	ad.dockDepartureRun = 600
	b.TrailerHeading = .3
	if ad.dockDepartureSpeed(f) > 100 {
		t.Fatal("bent tail got fast departure")
	}
	b.ResetArticulation()
	b.SteeringAngle = .2
	if ad.dockDepartureSpeed(f) > 100 {
		t.Fatal("sharp steering got fast departure")
	}
	ad.CruiseSpeedLimit = 90
	f.targetStop = sim.NewBusStop(0, "test", sim.Vec2{X: 2000}, 0, 8, sim.RGBA(0, 0, 0, 255), 52)
	f.remaining = 2000
	if v := ad.targetSpeed(f); v > 90 {
		t.Fatalf("configured cruise limit lost %.1f", v)
	}
}
