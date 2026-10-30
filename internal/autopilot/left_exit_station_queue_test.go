package autopilot

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/left-exit-station-queue.json
var leftExitStationQueueFixture []byte

type leftExitStationScene struct {
	Fleet []struct {
		ID     int
		Bus    *sim.Bus
		Route  *sim.Route
		Driver DriverSnapshot
	}
	Lights *sim.TrafficLightSystem
}

func loadLeftExitStationScene(t *testing.T) (*sim.World, leftExitStationScene) {
	t.Helper()
	var s leftExitStationScene
	if err := json.Unmarshal(leftExitStationQueueFixture, &s); err != nil {
		t.Fatal(err)
	}
	return sim.NewWorld(52, sim.MapThemeMetropolis), s
}

func TestSavedLeftExitStationQueueResumes(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(fmt.Sprintf("reverse%v", reverse), func(t *testing.T) {
			w, s := loadLeftExitStationScene(t)
			drivers := make([]*AutopilotDriver, len(s.Fleet))
			distance := make([]float64, len(s.Fleet))
			served := make([]bool, len(s.Fleet))
			for i, a := range s.Fleet {
				var err error
				drivers[i], err = a.Driver.Restore(w)
				if err != nil {
					t.Fatal(err)
				}
			}
			for tick := 0; tick < 7200; tick++ {
				s.Lights.Update(1. / 60)
				for n := range s.Fleet {
					i := n
					if reverse {
						i = len(s.Fleet) - 1 - n
					}
					a := s.Fleet[i]
					obs := []VehicleObstacle{}
					for j, b := range s.Fleet {
						if i != j {
							obs = append(obs, ObserveBus(b.Bus))
						}
					}
					before := a.Bus.Pos
					th, br, st, hb, doors, docked := drivers[i].ComputeControls(a.Bus, a.Route, w, s.Lights, obs, 52, 1./60)
					a.Bus.Update(1./60, th, br, st, hb, doors)
					distance[i] += a.Bus.Pos.Distance(before)
					served[i] = served[i] || (docked && a.Bus.DoorsOpen && drivers[i].DockedStopID == 278)
					if a.Bus.Pos.Distance(before) > a.Bus.MaxSpeedFwd/60+.001 {
						t.Fatal("position jumped")
					}
					if result := a.Bus.CheckBoundaries(w, 52); result.Zone == sim.ZoneHardBarrier {
						t.Fatalf("AI%d boundary tick%d: %+v", a.ID, tick, result)
					}
					for j, b := range s.Fleet {
						if i != j {
							if hit, _, depth := sim.CheckBusesCollision(a.Bus, b.Bus); hit {
								t.Fatalf("AI%d/%d contact tick%d depth%.4f", a.ID, b.ID, tick, depth)
							}
						}
					}
				}
				if tick == 300 {
					for i, ad := range drivers {
						raw, err := json.Marshal(ad.Snapshot())
						if err != nil {
							t.Fatal(err)
						}
						var snap DriverSnapshot
						if err = json.Unmarshal(raw, &snap); err != nil {
							t.Fatal(err)
						}
						drivers[i], err = snap.Restore(w)
						if err != nil {
							t.Fatal(err)
						}
					}
				}
				complete := tick > 300
				for i, a := range s.Fleet {
					complete = complete && served[i] && a.Route.GetCurrentTargetStopID() != 278 && distance[i] > 800
				}
				if complete {
					return
				}
			}
			for i, a := range s.Fleet {
				t.Logf("AI%d distance%.1f served%v target%d", a.ID, distance[i], served[i], a.Route.GetCurrentTargetStopID())
			}
			t.Fatal("captured queue remained blocked")
		})
	}
}

func TestStationaryAngledRearQueueDoesNotReserveLeadBerth(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, u, r := serviceTestWorld(h, 52)
		stop := w.Stops[1]
		p := stop.FrontBerthPosition()
		lead := sim.NewBus(p.X, p.Y, h)
		lead.SetModel(sim.BusArticulated)
		rearPos := stop.Pos.Sub(u.Mul(180)).Sub(r.Mul(42))
		rear := sim.NewBus(rearPos.X, rearPos.Y, h+.55)
		if len(availableStationBerths(lead, stop, []VehicleObstacle{ObserveBus(rear)})) == 0 {
			t.Fatal("separated stationary rear queue reserved the lead berth")
		}
		rear.Speed = 20
		if stationFollowerBehind(lead, ObserveBus(rear), h) {
			t.Fatal("moving angled traffic received stationary-queue exception")
		}
		rear.Speed = 0
		rear.Heading = h + 1.2
		if stationFollowerBehind(lead, ObserveBus(rear), h) {
			t.Fatal("crossing traffic received follower exception")
		}
		rear.Heading = h + .55
		rear.Pos = lead.Pos.Sub(u.Mul(80))
		if stationFollowerBehind(lead, ObserveBus(rear), h) {
			t.Fatal("overlapping envelopes received follower exception")
		}
	}
}

func TestLateBayArrivalStillRejectsFrontConflictAndRoadsidePose(t *testing.T) {
	for _, outside := range []bool{false, true} {
		w, s := loadLeftExitStationScene(t)
		a := s.Fleet[0]
		ad, err := a.Driver.Restore(w)
		if err != nil {
			t.Fatal(err)
		}
		obs := []VehicleObstacle{}
		if outside {
			a.Bus.Pos.X -= 100
		} else {
			p := a.Bus.Pos.Add(sim.UnitFromAngle(w.Stops[278].Heading).Mul(85))
			obs = append(obs, VehicleObstacle{Pos: p, Heading: w.Stops[278].Heading, Length: 46, Width: 22})
		}
		_, _, _, _, doors, docked := ad.ComputeControls(a.Bus, a.Route, w, s.Lights, obs, 52, 1./60)
		if doors || docked || ad.State == AutopilotStateDocked {
			t.Fatalf("unsafe arrival accepted outside%v", outside)
		}
	}
}

func TestLongLeftArrivalHasNoBackwardsStationTaper(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				w, _, out, _ := wideLeftExitWorld(lw, h, true)
				w.Stops[1] = sim.NewBusStop(1, "to", out.Mul(math.Max(900, 900*lw/52)), h-math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
				path := buildAutopilotDrivePath(w, w.Stops[0], w.Stops[1], lw, true, false, sim.RoadPathOptions{LongVehicle: true, OrdinaryBusLaneLeft: true})
				g := w.JunctionAt(sim.Vec2{}, lw)
				mouth := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				for i := 1; i < len(path); i++ {
					if path[i-1].Dot(out) > mouth+1 && path[i].Dot(out) > mouth+1 && path[i].Sub(path[i-1]).Dot(out) < -.01 {
						t.Fatalf("backwards arrival at%d: %v -> %v tail%v target%v heading%.3f geom%+v", i, path[i-1], path[i], path[len(path)-4:], w.Stops[1].FrontBerthPosition(), w.Stops[1].Heading, g)
					}
				}
			})
		}
	}
}

func TestLongLeftArrivalReachesBayAcrossWidthsAndHeadings(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				w, _, out, _ := wideLeftExitWorld(lw, h, true)
				w.Stops[1] = sim.NewBusStop(1, "to", out.Mul(math.Max(900, 900*lw/52)), h-math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
				g := w.JunctionAt(sim.Vec2{}, lw)
				mouth := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				right := sim.Vec2{X: -out.Y, Y: out.X}
				p := out.Mul(mouth + 240).Add(right.Mul(1.5 * lw))
				b := sim.NewBus(p.X, p.Y, h-math.Pi/2)
				b.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.articulatedPath = true
				ad.ordinaryBusLeft = true
				ad.pathFromID = 0
				ad.pathToID = 1
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for tick := 0; tick < 3600; tick++ {
					th, br, st, hb, doors, docked := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
					before := b.Pos
					b.Update(1./60, th, br, st, hb, doors)
					if b.Pos.Distance(before) > b.MaxSpeedFwd/60+.001 {
						t.Fatal("position jumped")
					}
					if result := b.CheckBoundaries(w, lw); result.Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary at tick%d: %+v", tick, result)
					}
					if docked && b.DoorsOpen && ad.DockedStopID == 1 {
						return
					}
				}
				t.Fatalf("arrival did not open doors: pos%v yaw%.3f target%.1f", b.Pos, b.Heading, ad.LastTargetSpeed)
			})
		}
	}
}
