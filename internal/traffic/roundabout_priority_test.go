package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestFastCarSlowsBeforeRoundaboutAndUsesBusCap(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(h), func(t *testing.T) {
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
			w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 4}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 4}}}
			c := &sim.TrafficCar{Pos: u.Mul(-1500).Add(r.Mul(78)), Heading: h, IsEW: math.Abs(u.X) > .5, LaneIndex: 1, Length: 46, Width: 22, Speed: 550, TargetSpeed: 550}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
			entered := false
			for tick := 0; tick < 1800; tick++ {
				tm.StepCar(1./60, c, 0, nil, nil, 1, 52, w)
				if c.InRoundabout {
					entered = true
					if c.Speed > sim.RoundaboutSpeedLimit+.01 {
						t.Fatalf("car entered/circulated too fast: %.1f", c.Speed)
					}
				} else if entered {
					return
				}
			}
			t.Fatal("car did not complete roundabout")
		})
	}
}

func TestJoiningCarRechecksStoppedCirculatingBusPriority(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(h), func(t *testing.T) {
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
			w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
			angle := h + math.Pi - math.Asin(78./214) - .3
			bus := sim.NewBus(214*math.Cos(angle), 214*math.Sin(angle), angle-math.Pi/2)
			join := sim.RoundaboutEntryJoin(rb, u, 78, 214, 52)
			start, _ := sim.RoundaboutEntryPoint(join, 0)
			c := &sim.TrafficCar{Pos: start, Heading: h, IsEW: math.Abs(u.X) > .5, LaneIndex: 1, Length: 46, Width: 22, TargetSpeed: 300, Speed: 90, InRoundabout: true, RoundaboutRadius: start.Length(), RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutAngle: math.Atan2(start.Y, start.X), RoundaboutExitHeading: h, RoundaboutExitAngle: sim.NormalizeAngle(h + math.Asin(78./214)), RoundaboutExitOffset: 78, RoundaboutExitLane: 1, RoundaboutExitEW: math.Abs(u.X) > .5}
			beginRoundaboutEntry(c, rb, 52)
			tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
			for tick := 0; tick < 180; tick++ {
				tm.StepCar(1./60, c, 0, bus, nil, 1, 52, w)
				yaw := c.Heading - h
				front := c.Length/2*math.Abs(math.Cos(yaw)) + c.Width/2*math.Abs(math.Sin(yaw))
				if c.Pos.Dot(u)+front > -rb.YieldLineDistance()+.5 {
					t.Fatalf("joining car crossed yield line into stopped circulating bus: %v", c.Pos)
				}
			}
			// Once the ring clears, the same live car must enter and finish.
			bus.Pos = u.Mul(5000).Add(r.Mul(5000))
			start = c.Pos
			for tick := 0; tick < 1200; tick++ {
				tm.StepCar(1./60, c, 0, bus, nil, 1, 52, w)
				if !c.InRoundabout && c.Pos.Dot(u) > 300 {
					return
				}
			}
			t.Fatalf("entry did not resume after priority vehicle left: moved%.1f", c.Pos.Distance(start))
		})
	}
}

func TestEntryCurveYieldsToBusArrivingFromLeft(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
				w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
				lateral := 1.5 * lw
				join := sim.RoundaboutEntryJoin(rb, u, lateral, 214, lw)
				start, _ := sim.RoundaboutEntryPoint(join, 0)
				c := &sim.TrafficCar{Pos: start, Heading: h, IsEW: math.Abs(u.X) > .5, LaneIndex: 1, Length: 46, Width: 22, TargetSpeed: 300, Speed: 90, InRoundabout: true, RoundaboutRadius: start.Length(), RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutAngle: math.Atan2(start.Y, start.X), RoundaboutExitHeading: h, RoundaboutExitAngle: sim.NormalizeAngle(h + math.Asin(lateral/214)), RoundaboutExitOffset: lateral, RoundaboutExitLane: 1, RoundaboutExitEW: math.Abs(u.X) > .5}
				beginRoundaboutEntry(c, rb, lw)
				tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
				bus := sim.NewBus(0, 0, 0)
				bus.Speed = 90
				for tick := 0; tick < 1200; tick++ {
					a := h + math.Pi + .5 - 90./214*float64(tick)/60
					bus.Pos = sim.Vec2{X: 214 * math.Cos(a), Y: 214 * math.Sin(a)}
					bus.Heading = a - math.Pi/2
					tm.StepCar(1./60, c, 0, bus, nil, 1, lw, w)
					next := a - 90./214/60
					body := sim.OBB{Center: sim.Vec2{X: 214 * math.Cos(next), Y: 214 * math.Sin(next)}, Heading: next - math.Pi/2, HalfLen: 48, HalfWid: 18}
					if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 23, HalfWid: 11}, body); hit {
						t.Fatalf("late ring bus collision tick%d car%v bus%v", tick, c.Pos, body.Center)
					}
					if !c.InRoundabout && c.Pos.Dot(u) > 300 {
						return
					}
				}
				t.Fatal("car did not use gap after priority bus passed")
			})
		}
	}
}
