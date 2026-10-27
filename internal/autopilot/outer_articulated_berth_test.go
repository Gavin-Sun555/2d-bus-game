package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBerserkArticulatedOuterBoardingReleasesBothBuses(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("heading%.2f/width%.0f", h, lw), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				for i, x := range []float64{-1500, 0, 1500} {
					w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
				}
				stop := w.Stops[1]
				p := stop.FrontBerthPosition()
				leader := sim.NewBus(p.X, p.Y, h)
				lp := p
				lr := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 2}
				ld := NewAutopilotDriver()
				ld.State = AutopilotStateDeparting
				ld.CruiseSpeedLimit = 180
				ld.PrepareLeg(w, stop, w.Stops[2], lw)
				ld.departureFromDock = true
				p = stop.Pos.Sub(r.Mul(61))
				bus := sim.NewBus(p.X, p.Y, h+.09)
				bus.SetModel(sim.BusArticulated)
				bus.TrailerHeading = h + .05
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				boarded := false
				for tick := 0; tick < 1800; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ObserveBus(leader)}, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.DoorsOpen && ad.DockedStopID == 1 {
						boarded = true
					}
					th, br, st, hb, doors, _ = ld.ComputeControls(leader, lr, w, nil, []VehicleObstacle{ObserveBus(bus)}, lw, 1./60)
					leader.Update(1./60, th, br, st, hb, doors)
					if hit, _, _ := sim.CheckBusesCollision(bus, leader); hit {
						t.Fatalf("contact tick%d", tick)
					}
					if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier || leader.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("boundary tick%d", tick)
					}
					if tick == 120 && !boarded {
						t.Fatal("outer long bus did not open doors promptly")
					}
					if boarded && bus.Pos.Sub(p).Dot(u) > 600 && leader.Pos.Sub(lp).Dot(u) > 600 {
						return
					}
				}
				t.Fatalf("did not release both buses: boarded%v player%.1f leader%.1f", boarded, bus.Pos.Sub(p).Dot(u), leader.Pos.Sub(lp).Dot(u))
			})
		}
	}
}

func TestOuterArticulatedBoardingRejectsUnsafePoses(t *testing.T) {
	for _, name := range []string{"normal mode", "moving", "tail outside", "overlap", "wrong direction"} {
		t.Run(name, func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -4000}, P2: sim.Vec2{X: 4000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
			stop := sim.NewBusStop(0, "test", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), 52)
			w.Stops = []*sim.BusStop{stop}
			bus := sim.NewBus(stop.Pos.X, stop.Pos.Y-61, 0)
			bus.SetModel(sim.BusArticulated)
			ad := NewAutopilotDriver()
			ad.BerserkMode = name != "normal mode"
			f := autopilotFrame{bus: bus, targetStop: stop, world: w, laneWidth: 52, remaining: 96}
			switch name {
			case "moving":
				bus.Speed = 20
				f.speedAbs = 20
			case "tail outside":
				bus.Pos.X -= 100
			case "overlap":
				f.obstacles = []VehicleObstacle{ObserveBus(sim.NewBus(bus.Pos.X, bus.Pos.Y, 0))}
			case "wrong direction":
				bus.Heading = math.Pi
				bus.TrailerHeading = math.Pi
			}
			input, _ := ad.stationDockControls(f)
			if input.isDocked || input.toggleDoors {
				t.Fatal("unsafe outer pose accepted for boarding")
			}
		})
	}
}
