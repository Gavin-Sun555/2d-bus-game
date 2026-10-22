package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestLeftCarClearsStoppedThroughQueueBesideCurve(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(h), func(t *testing.T) {
			w, tls := leftPathMatrixWorld(6, 6, false, h)
			it := tls.Intersections[0]
			it.Phase = sim.PhaseEWLeftGreen
			if math.Abs(math.Sin(h)) > .5 {
				it.Phase = sim.PhaseNSLeftGreen
			}
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			car := &sim.TrafficCar{Pos: u.Mul(-300).Add(r.Mul(26)), Heading: h, Length: 46, Width: 22, Speed: 85, TargetSpeed: 85, IsEW: math.Abs(u.X) > .5}
			f := carFrame{car: car, world: w, tls: tls, laneWidth: 52, activeCount: 2}
			car.JunctionTurn = planCarJunctionTurn(f, it)
			car.Pos = car.JunctionTurn.Points[0]
			moveCarThroughJunction(car, 50./85)
			car.Speed = 0
			start := car.Pos
			queue := &sim.TrafficCar{Pos: u.Mul(-42).Add(r.Mul(36)), Heading: h, Length: 46, Width: 22}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{car, queue}}
			for tick := 0; tick < 180; tick++ {
				tm.StepCar(1./60, car, 0, nil, tls, 2, 52, w)
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: queue.Pos, Heading: queue.Heading, HalfLen: 23, HalfWid: 11}); hit {
					t.Fatalf("contact tick %d", tick)
				}
			}
			if car.Pos.Distance(start) < 100 {
				t.Fatal("left-turn car waited for a nonconflicting stopped through queue")
			}
			// A stopped vehicle on the actual curve still blocks motion.
			turn := planCarJunctionTurn(carFrame{car: &sim.TrafficCar{Pos: u.Mul(-300).Add(r.Mul(26)), Heading: h, Length: 46, Width: 22, IsEW: math.Abs(u.X) > .5}, world: w, tls: tls, laneWidth: 52}, it)
			car.JunctionTurn = turn
			car.Pos = turn.Points[0]
			car.Heading = h
			queue.Pos, queue.Heading = turn.Predict(40)
			if tm.carFollowingSpeed(f, 85) != 0 {
				t.Fatal("stopped leader on turn lost protection")
			}
		})
	}
}
