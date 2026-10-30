package autopilot

import (
	"embed"
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/departure-rear-lock-slot-*.json
var rearDepartureFixtures embed.FS

type rearDepartureFixture struct {
	RearBus    *sim.Bus
	RearDriver DriverSnapshot
	RearRoute  *sim.Route
	World      *sim.World
	Bus        *sim.Bus
	Driver     DriverSnapshot
	Route      *sim.Route
	OtherBuses []*sim.Bus
	Lights     *sim.TrafficLightSystem
}

func loadRearDeparture(t *testing.T, slot int) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	raw, err := rearDepartureFixtures.ReadFile(fmt.Sprintf("testdata/departure-rear-lock-slot-%d.json", slot))
	if err != nil {
		t.Fatal(err)
	}
	var s rearDepartureFixture
	if err = json.Unmarshal(raw, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: s.Bus, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS, speedAbs: math.Abs(s.Bus.Speed), remaining: ad.LastRemaining, targetStop: s.World.Stops[s.Route.GetCurrentTargetStopID()]}
	for _, b := range s.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, s.Route
}
func TestSavedStationDepartureEscape(t *testing.T) {
	for slot := 1; slot <= 2; slot++ {
		t.Run(fmt.Sprintf("slot%d", slot), func(t *testing.T) {
			ad, f, r := loadRearDeparture(t, slot)
			initial := f.bus.Pos
			desired, _, _, _ := ad.pursuitSteering(f)
			if _, ok := ad.stationDepartureEscape(f, desired); !ok {
				t.Fatal("no physically clear departure found")
			}
			for tick := 0; tick < 600; tick++ {
				before := f.bus.Pos
				th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, r, f.world, f.tls, f.obstacles, 52, 1./60)
				f.bus.Update(1./60, th, br, st, hb, doors)
				if f.bus.Pos.Distance(before) > f.bus.MaxSpeedFwd/60+1 {
					t.Fatal("recovery relocated the bus")
				}
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatal("recovery crossed a hard boundary")
				}
				for _, ob := range f.obstacles {
					for _, part := range predictedObstacleBodies(ob, 0) {
						if part.HalfLen > 0 {
							if hit, _, depth := f.bus.CollidesBody(part); hit && depth > 1e-6 {
								t.Fatalf("body contact at %.2fs: depth %g", float64(tick)/60, depth)
							}
						}
					}
				}
			}
			if d := f.bus.Pos.Distance(initial); d < 200 {
				t.Fatalf("departure remained blocked: travel %g", d)
			}
		})
	}
}
func TestDepartureEscapeRetainsSafetyGuards(t *testing.T) {
	cases := map[string]func(*autopilotFrame){
		"doors open":        func(f *autopilotFrame) { f.bus.DoorsOpen = true },
		"passenger service": func(f *autopilotFrame) { f.bus.AddPassengerService(2, 1) },
		"moving fast":       func(f *autopilotFrame) { f.speedAbs = 40; f.bus.Speed = 40 },
		"no stopped rear follower": func(f *autopilotFrame) {
			for i := range f.obstacles {
				f.obstacles[i].Speed = 10
			}
		},
		"physical obstruction": func(f *autopilotFrame) {
			f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos, Heading: f.bus.Heading, Length: 96, Width: 36})
		},
	}
	for name, change := range cases {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadRearDeparture(t, 1)
			change(&f)
			if _, ok := ad.stationDepartureEscape(f, -1); ok {
				t.Fatal("unsafe departure accepted")
			}
		})
	}
}

func TestSavedRearBerthCompletesAfterLeadClears(t *testing.T) {
	raw, err := rearDepartureFixtures.ReadFile("testdata/departure-rear-lock-slot-2.json")
	if err != nil {
		t.Fatal(err)
	}
	var scene rearDepartureFixture
	if err = json.Unmarshal(raw, &scene); err != nil {
		t.Fatal(err)
	}
	ad, err := scene.RearDriver.Restore(scene.World)
	if err != nil {
		t.Fatal(err)
	}
	stop := scene.World.Stops[278]
	scene.Bus.Pos = scene.Bus.Pos.Add(sim.UnitFromAngle(stop.Heading).Mul(500))
	scene.Bus.Heading = stop.Heading
	scene.Bus.ResetArticulation()
	obstacles := []VehicleObstacle{ObserveBus(scene.Bus)}
	for _, b := range scene.OtherBuses {
		if b.Pos.Distance(scene.RearBus.Pos) > .01 {
			obstacles = append(obstacles, ObserveBus(b))
		}
	}
	for tick := 0; tick < 1200; tick++ {
		scene.Lights.Update(1. / 60)
		th, br, st, hb, doors, docked := ad.ComputeControls(scene.RearBus, scene.RearRoute, scene.World, scene.Lights, obstacles, 52, 1./60)
		scene.RearBus.Update(1./60, th, br, st, hb, doors)
		if scene.RearBus.CheckBoundaries(scene.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("alignment crossed hard boundary")
		}
		for _, ob := range obstacles {
			for _, body := range predictedObstacleBodies(ob, 0) {
				if body.HalfLen > 0 {
					if hit, _, depth := scene.RearBus.CollidesBody(body); hit && depth > 1e-6 {
						t.Fatalf("alignment body contact depth%g", depth)
					}
				}
			}
		}
		if docked && scene.RearBus.DoorsOpen {
			if !stop.ContainsBus(scene.RearBus) {
				t.Fatal("opened doors outside the real boarding area")
			}
			snapshot := ad.Snapshot()
			restored, err := snapshot.Restore(scene.World)
			if err != nil || snapshot.Dock == nil || !snapshot.Dock.AlignmentRecovery || restored.stationDock == nil || !restored.stationDock.alignmentRecovery {
				t.Fatal("alignment plan did not survive save restoration")
			}
			t.Logf("saved rear bus docked after %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("saved rear-berth bus remained blocked after lead departed")
}
