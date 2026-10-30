package autopilot

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/block279-station-yield.json
var block279YieldFixture []byte

func TestBlock279SavedQueueResumes(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(fmt.Sprintf("reverse%v", reverse), func(t *testing.T) {
			var scene leftExitStationScene
			if err := json.Unmarshal(block279YieldFixture, &scene); err != nil {
				t.Fatal(err)
			}
			w := sim.NewWorld(52, sim.MapThemeMetropolis)
			drivers := make([]*AutopilotDriver, len(scene.Fleet))
			served := make([]bool, len(drivers))
			distance := make([]float64, len(drivers))
			for i, a := range scene.Fleet {
				var err error
				drivers[i], err = a.Driver.Restore(w)
				if err != nil {
					t.Fatal(err)
				}
			}
			for tick := 0; tick < 7200; tick++ {
				scene.Lights.Update(1. / 60)
				for n := range scene.Fleet {
					i := n
					if reverse {
						i = len(scene.Fleet) - 1 - n
					}
					a := scene.Fleet[i]
					obs := []VehicleObstacle{}
					for j, b := range scene.Fleet {
						if i != j {
							obs = append(obs, ObserveBus(b.Bus))
						}
					}
					before := a.Bus.Pos
					th, br, st, hb, doors, docked := drivers[i].ComputeControls(a.Bus, a.Route, w, scene.Lights, obs, 52, 1./60)
					a.Bus.Update(1./60, th, br, st, hb, doors)
					distance[i] += a.Bus.Pos.Distance(before)
					if docked && a.Bus.DoorsOpen && drivers[i].DockedStopID == 278 && !served[i] {
						served[i] = true
						t.Logf("AI%d served %.2fs", a.ID, float64(tick)/60)
					}
					if a.Bus.Pos.Distance(before) > a.Bus.MaxSpeedFwd/60+.01 {
						t.Fatal("jump")
					}
					if r := a.Bus.CheckBoundaries(w, 52); r.Zone == sim.ZoneHardBarrier {
						t.Fatalf("AI%d boundary tick%d %+v", a.ID, tick, r)
					}
					for j, b := range scene.Fleet {
						if j != i {
							if hit, _, depth := sim.CheckBusesCollision(a.Bus, b.Bus); hit && depth > 1e-5 {
								t.Fatalf("AI%d/%d collision tick%d depth%g", a.ID, b.ID, tick, depth)
							}
						}
					}
				}
			}
			for i, a := range scene.Fleet {
				if !served[i] || distance[i] < 500 {
					t.Errorf("AI%d served%v distance%.1f final%v", a.ID, served[i], distance[i], a.Bus.Pos)
				}
			}
		})
	}
}

func TestBlock279LeadUsesAvailableStraighteningRunway(t *testing.T) {
	var scene leftExitStationScene
	if err := json.Unmarshal(block279YieldFixture, &scene); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	a := scene.Fleet[0]
	ad, err := a.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	stop := w.Stops[278]
	u := sim.UnitFromAngle(stop.Heading)
	if extra := stop.FrontBerthPosition().Sub(ad.stationDock.target).Dot(u); extra >= a.Bus.Length/2 {
		t.Fatal("fixture no longer exercises the short available runway")
	}
	obs := []VehicleObstacle{}
	for _, b := range scene.Fleet[1:] {
		obs = append(obs, ObserveBus(b.Bus))
	}
	f := autopilotFrame{bus: a.Bus, world: w, tls: scene.Lights, obstacles: obs, targetStop: stop, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining}
	in, active := ad.stationDockControls(f)
	if !active || in.throttle <= 0 || in.brake != 0 || ad.stationDock == nil || ad.stationDock.waiting || ad.stationDock.target.Distance(stop.FrontBerthPosition()) > 1 || !stationCurveCanSteer(a.Bus, ad.stationDock.points) {
		t.Fatalf("lead kept stale waiting point: %+v", in)
	}
}

func TestStationFollowerUsesTrueSectionProjection(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		lead := sim.NewBus(0, 0, h+.14)
		lead.SetModel(sim.BusArticulated)
		lead.TrailerHeading = h + .6
		lead.TrailerInitialized = true
		rear := sim.NewBus(0, 0, h+.035)
		rear.SetModel(sim.BusArticulated)
		rear.TrailerHeading = h + .15
		rear.TrailerInitialized = true
		u := sim.UnitFromAngle(h)
		leadRear, _ := stationBodyRange(lead, u)
		_, rearFront := stationBodyRange(rear, u)
		rear.Pos = u.Mul(leadRear - rearFront - 4)
		ob := ObserveBus(rear)
		if !stationFollowerBehind(lead, ob, h) {
			t.Fatalf("real 4px gap denied at h%g", h)
		}
		rear.Pos = rear.Pos.Add(u.Mul(5))
		ob = ObserveBus(rear)
		if stationFollowerBehind(lead, ob, h) {
			t.Fatalf("actual overlapping section granted at h%g", h)
		}
	}
}

func TestMovingFreeBayKeepsTaperBesideCar(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		t.Run(fmt.Sprintf("berserk%v", berserk), func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6, Type: sim.RoadExpressway6Lane}}}
			w.RebuildIntersections()
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -1500}, 0, 6, sim.RGBA(0, 0, 0, 255), 52), sim.NewBusStop(1, "to", sim.Vec2{}, 0, 6, sim.RGBA(0, 0, 0, 255), 52)}
			stop := w.Stops[1]
			p := stop.Pos.Sub(sim.Vec2{X: 180, Y: 17})
			b := sim.NewBus(p.X, p.Y, .3)
			b.SetModel(sim.BusArticulated)
			b.Speed = 90
			b.SteeringAngle = -.15
			b.TrailerHeading = .56
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			obs := []VehicleObstacle{{Pos: stop.Pos.Sub(sim.Vec2{Y: 120}), Heading: 0, Length: 46, Width: 22}}
			ad := NewAutopilotDriver()
			ad.BerserkMode = berserk
			ad.articulatedPath = true
			ad.PrepareLeg(w, w.Stops[0], stop, 52)
			ad.drivePath[len(ad.drivePath)-1] = stop.FrontBerthPosition()
			for tick := 0; tick < 1800; tick++ {
				th, br, st, hb, doors, docked := ad.ComputeControls(b, route, w, nil, obs, 52, 1./60)
				b.Update(1./60, th, br, st, hb, doors)
				if b.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
					t.Fatal("normal arrival hit barrier")
				}
				if docked {
					return
				}
			}
			t.Fatalf("passing traffic incorrectly changed free-bay arrival: pos%v heading%g", b.Pos, b.Heading)
		})
	}
}
