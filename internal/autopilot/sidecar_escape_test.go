package autopilot

import (
	"embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/sidecar-*-2026-10-29.json
var sidecarFixtures embed.FS

func loadSidecarFixture(t *testing.T, name string) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	raw, err := sidecarFixtures.ReadFile("testdata/sidecar-" + name + "-2026-10-29.json")
	if err != nil {
		t.Fatal(err)
	}
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if err = json.Unmarshal(raw, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: x.Bus, world: x.World, tls: x.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: x.World.Stops[x.Route.GetCurrentTargetStopID()]}
	for _, b := range x.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	for _, c := range x.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
	}
	return ad, f, x.Route
}
func TestSavedSidecarLocksClearWithoutContact(t *testing.T) {
	for _, name := range []string{"player", "fleet"} {
		t.Run(name, func(t *testing.T) {
			ad, f, route := loadSidecarFixture(t, name)
			start := f.bus.Pos
			heading := approachRoadHeading(f.bus.Heading)
			u := sim.UnitFromAngle(heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			center, ew, _, _ := sim.CurrentRoadInfo(f.world, start, heading, 52)
			origin := sim.Vec2{Y: center}
			if !ew {
				origin = sim.Vec2{X: center}
			}
			for tick := 0; tick < 1200; tick++ {
				before := f.bus.Pos
				th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
				f.bus.Update(1./60, th, br, st, hb, doors)
				if before.Distance(f.bus.Pos) > f.bus.MaxSpeedFwd/60+1 {
					t.Fatal("sidecar recovery relocated bus")
				}
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatalf("sidecar recovery hit hard boundary tick%d pos%v heading%g tail%g speed%g wheel%g plan%+v", tick, f.bus.Pos, f.bus.Heading, f.bus.TailHeading(), f.bus.Speed, f.bus.SteeringAngle, ad.stationDock)
				}
				for _, part := range f.bus.BodyParts() {
					if part.HalfLen > 0 {
						yaw := part.Heading - heading
						if part.Center.Sub(origin).Dot(r)-part.HalfLen*math.Abs(math.Sin(yaw))-part.HalfWid*math.Abs(math.Cos(yaw)) < 0 {
							t.Fatal("sidecar recovery entered opposing traffic")
						}
					}
				}
				for _, ob := range f.obstacles {
					for _, part := range predictedObstacleBodies(ob, 0) {
						if part.HalfLen > 0 {
							if hit, _, depth := f.bus.CollidesBody(part); hit && depth > 1e-6 {
								t.Fatalf("body contact tick%d depth%g", tick, depth)
							}
						}
					}
				}
				if tick == 120 {
					if !ad.sidecarEscapeActive {
						t.Fatal("clearance intent ended before trailer passed car")
					}
					raw, _ := json.Marshal(ad.Snapshot())
					var saved DriverSnapshot
					if err := json.Unmarshal(raw, &saved); err != nil {
						t.Fatal(err)
					}
					var err error
					ad, err = saved.Restore(f.world)
					if err != nil {
						t.Fatal(err)
					}
					if !ad.sidecarEscapeActive {
						t.Fatal("save/load lost ongoing full-body clearance intent")
					}
				}
			}
			minimum := 20.
			if name == "player" {
				minimum = 160
			}
			if f.bus.Pos.Sub(start).Dot(u) < minimum {
				t.Fatalf("sidecar lock persisted at %v", f.bus.Pos)
			}
		})
	}
}
func TestSidecarRecoveryKeepsSafetyGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "speed", "admission", "frontal", "moving", "reversing"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadSidecarFixture(t, "player")
			switch name {
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 0)
			case "speed":
				f.speedAbs = 40
			case "admission":
				f.bus.RoundaboutAdmissionHold = &sim.RoundaboutAdmissionHold{}
			case "frontal":
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(30)), Heading: f.bus.Heading, Length: 46, Width: 22})
			case "moving":
				for i := range f.obstacles {
					if f.obstacles[i].Bus == nil {
						f.obstacles[i].Speed = 50
					}
				}
			case "reversing":
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(200)), Heading: f.bus.Heading, Speed: -10, Length: 46, Width: 22})
			}
			desired, _, _, _ := ad.pursuitSteering(f)
			if _, ok := ad.sidecarEscape(f, desired); ok || ad.sidecarPresteerValid {
				t.Fatal("sidecar recovery bypassed safety guard")
			}
		})
	}
}
