package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestOccupiedStationDoesNotHoldUnfinishedArticulatedTurn(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("heading%.2f/width%.0f", h, lw), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				stop := sim.NewBusStop(1, "test", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), lw)
				leadPos := stop.Pos.Sub(u.Mul(148))
				lead := sim.NewBus(leadPos.X, leadPos.Y, h)
				pos := stop.Pos.Sub(u.Mul(469)).Sub(r.Mul(49))
				bus := sim.NewBus(pos.X, pos.Y, h+.224)
				bus.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				obs := []VehicleObstacle{ObserveBus(lead)}
				if len(availableStationBerths(bus, stop, obs)) != 0 {
					t.Fatal("fixture must have an occupied platform")
				}
				f := autopilotFrame{bus: bus, targetStop: stop, obstacles: obs, remaining: 592, laneWidth: lw}
				if _, active := ad.stationDockControls(f); active {
					t.Fatal("unreachable queue curve took over and held the road turn")
				}
				// An already aligned, late arrival waits, but an in-progress
				// passing/return maneuver must keep its road controller alive.
				bus.Pos = stop.Pos.Sub(u.Mul(212)).Sub(r.Mul(49))
				bus.Heading = h
				bus.ResetArticulation()
				f.remaining = 350
				if in, active := ad.stationDockControls(f); !active || in.brake != 1 {
					t.Fatal("aligned late arrival did not wait for queue runway")
				}
				ad.LaneOffset = -lw
				if _, active := ad.stationDockControls(f); active {
					t.Fatal("queue takeover interrupted the current lane maneuver")
				}
				bus.Pos = stop.FrontBerthPosition().Sub(u.Mul(30)).Sub(r.Mul(100))
				f.remaining = 30
				ad.BerserkMode = false // No alternative outer-lane boarding berth.
				if in, active := ad.stationDockControls(f); !active || in.brake != 1 || in.throttle != 0 {
					t.Fatalf("close occupied platform lost its stopping hold: inputs%+v active%v plan%+v distance%g", in, active, ad.stationDock, stop.FrontBerthPosition().Sub(bus.Pos).Dot(u))
				}
			})
		}
	}
}

func TestStationAlongsideLeaderKeepsPhysicalSweep(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			t.Run(fmt.Sprintf("heading%.2f/model%d", h, model), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				bus := sim.NewBus(0, 0, h)
				bus.SetModel(model)
				p := u.Mul(-4).Sub(r.Mul(45))
				other := sim.NewBus(p.X, p.Y, h+.114)
				ob := ObserveBus(other)
				if !stationAlongsideBodyClear(bus, ob) {
					t.Fatal("separated leading bus cannot finish straight arrival")
				}
				if stationAlongsideBodyClear(other, ObserveBus(bus)) {
					t.Fatal("follower also claimed priority")
				}
				if !stationDockCurveClear(bus, []sim.Vec2{{}, u.Mul(30), u.Mul(100)}, []VehicleObstacle{ob}) {
					t.Fatal("clear forward corridor rejected")
				}
				if stationDockCurveClear(bus, []sim.Vec2{{}, u.Mul(20), u.Mul(50).Sub(r.Mul(45))}, []VehicleObstacle{ob}) {
					t.Fatal("curve crossing adjacent vehicle was accepted")
				}
				moving := ob
				moving.Speed = 10
				if stationAlongsideBodyClear(bus, moving) {
					t.Fatal("moving neighbor lost normal margins")
				}
				other.Pos = r.Mul(-30)
				if stationAlongsideBodyClear(bus, ObserveBus(other)) {
					t.Fatal("overlapping neighbor was accepted")
				}
			})
		}
	}
}

func TestMisalignedRearBerthUsesFreeForwardRunway(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", h), func(t *testing.T) {
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			stop := sim.NewBusStop(1, "test", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), 52)
			pos := stop.RearBerthPosition().Sub(u.Mul(.356)).Add(r.Mul(1.94))
			bus := sim.NewBus(pos.X, pos.Y, h+.241)
			ad := NewAutopilotDriver()
			ad.stationDock = &stationDockPlan{stopID: 1, target: stop.RearBerthPosition(), points: []sim.Vec2{pos.Sub(u.Mul(21)), stop.RearBerthPosition()}}
			f := autopilotFrame{bus: bus, targetStop: stop, remaining: .356, distToStop: pos.Distance(stop.Pos), laneWidth: 52}
			in, active := ad.stationDockControls(f)
			if !active || in.throttle <= 0 || in.brake > 0 || ad.stationDock.target.Distance(stop.FrontBerthPosition()) > .01 {
				t.Fatalf("rear berth remained stuck: active%v inputs%+v plan%+v", active, in, ad.stationDock)
			}
			for tick := 0; tick < 1200; tick++ {
				in, _ = ad.stationDockControls(f)
				bus.Update(1./60, in.throttle, in.brake, in.steer, false, in.toggleDoors)
				f.speedAbs = math.Abs(bus.Speed)
				if in.isDocked {
					if !atSelectedStationBerth(bus, stop.FrontBerthPosition(), h) {
						t.Fatal("boarded before aligned")
					}
					return
				}
			}
			t.Fatalf("did not align at front berth: pos%v heading%g", bus.Pos, bus.Heading)
		})
	}
}
