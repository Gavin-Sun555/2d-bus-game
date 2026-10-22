package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Replay an entry car stopped ahead of a waiting bus, including legacy states
// already past the yield plane. It must drive away without reset or contact.
func TestRoundaboutLeadCarClearsWaitingBusBehind(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for lane := 0; lane < 2; lane++ {
				for _, overshot := range []bool{false, true} {
					for _, fleet := range []bool{false, true} {
						t.Run(fmt.Sprintf("width%.0f/h%.2f/lane%d/overshot%v/fleet%v", lw, h, lane, overshot, fleet), func(t *testing.T) {
							center := sim.Vec2{X: 100, Y: -200}
							rb := sim.Roundabout{Center: center, Radius: 240, InnerRadius: 120}
							w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: center.Add(sim.Vec2{X: -2000}), P2: center.Add(sim.Vec2{X: 2000}), Lanes: 4}, {P1: center.Add(sim.Vec2{Y: -2000}), P2: center.Add(sim.Vec2{Y: 2000}), Lanes: 4}}}
							w.RebuildIntersections()
							u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
							right := sim.Vec2{X: -u.Y, Y: u.X}
							radius, gap := 248., 80.
							if overshot {
								radius, gap = 215, 100
							}
							lateral := (float64(lane) + .5) * lw
							pos := center.Sub(u.Mul(math.Sqrt(radius*radius - lateral*lateral))).Add(right.Mul(lateral))
							car := &sim.TrafficCar{ID: 1, Pos: pos, Heading: h, Length: 48, Width: 22, LaneIndex: lane, IsEW: math.Abs(u.X) > .5, TargetSpeed: 100}
							car.RoadCenter = center.X
							if car.IsEW {
								car.RoadCenter = center.Y
							}
							b := pos.Sub(u.Mul(gap))
							bus := sim.NewBus(b.X, b.Y, h)
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
							player := bus
							if fleet {
								player = nil
								tm.AIBusMgr = &traffic.AIBusManager{Buses: []*traffic.AIBus{{Bus: bus}}}
							}
							entered := false
							for tick := 0; tick < 1800; tick++ {
								previous := car.Pos
								tm.StepCar(1.0/60, car, 0, player, nil, 1, lw, w)
								if car.Pos.Distance(previous) > car.Speed/60+.05 {
									t.Fatal("car relied on reset/teleport")
								}
								if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: bus.Pos, Heading: h, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}); hit {
									t.Fatalf("lead car struck waiting bus at tick%d", tick)
								}
								entered = entered || car.InRoundabout
								if entered && !car.InRoundabout && car.Pos.Sub(center).Dot(u) > rb.Radius+60 {
									return
								}
							}
							t.Fatalf("entry deadlock: entered%v car%v heading%.2f speed%.1f", entered, car.Pos, car.Heading, car.Speed)
						})
					}
				}
			}
		}
	}
}

func TestAutopilotAndLeadCarResumeAtRoundaboutTogether(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", h), func(t *testing.T) {
			const lw = 52.
			rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}, {P1: sim.Vec2{Y: -2000}, P2: sim.Vec2{Y: 2000}, Lanes: 4}}}
			w.RebuildIntersections()
			for i, x := range []float64{-800, 800} {
				w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 4, sim.RGBA(0, 0, 0, 255), lw))
			}
			p := u.Mul(-math.Sqrt(215*215 - 26*26)).Add(r.Mul(26))
			car := &sim.TrafficCar{Pos: p, Heading: h, Length: 48, Width: 22, LaneIndex: 0, IsEW: math.Abs(u.X) > .5, TargetSpeed: 100}
			b := p.Sub(u.Mul(100))
			bus := sim.NewBus(b.X, b.Y, h)
			driver := autopilot.NewAutopilotDriver()
			driver.BerserkMode = true
			driver.BerserkMaxSpeed = 667
			driver.LaneOffset = -lw
			driver.TargetLaneOffset = -lw
			driver.OvertakeState = autopilot.OvertakePassing
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
			for tick := 0; tick < 2400; tick++ {
				tm.StepCar(1.0/60, car, 0, bus, nil, 1, lw, w)
				obs := []autopilot.VehicleObstacle{{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width}}
				th, br, st, hb, doors, _ := driver.ComputeControls(bus, route, w, nil, obs, lw, 1.0/60)
				bus.Update(1.0/60, th, br, st, hb, doors)
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
					t.Fatalf("following bus hit lead car at tick%d", tick)
				}
				if bus.Pos.Dot(u) > rb.Radius+80 && car.Pos.Dot(u) > rb.Radius+80 {
					return
				}
			}
			t.Fatalf("lead car / bus did not clear roundabout: car%v v%.1f bus%v v%.1f", car.Pos, car.Speed, bus.Pos, bus.Speed)
		})
	}
}
