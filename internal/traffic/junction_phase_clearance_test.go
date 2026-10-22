package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A turn can outlast its entry phase, especially across a wide arterial.
// Its remaining swept path must stay protected until the vehicle clears.
func TestCommittedTurnKeepsPriorityAcrossSignalChange(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, phase := range []sim.LightPhase{sim.PhaseEWGreen, sim.PhaseEWLeftGreen, sim.PhaseNSGreen, sim.PhaseNSLeftGreen} {
			t.Run(fmt.Sprintf("h%.2f/phase%d", h, phase), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w, tls := forcedLeftWorld(6, h)
				it := tls.Intersections[0]
				committed := &sim.TrafficCar{ID: 2, Heading: h, LaneIndex: 0, Length: 46, Width: 22, Speed: 60, TargetSpeed: 85, IsEW: math.Abs(u.X) > .5}
				committed.Pos = u.Mul(-it.GetStopLineDistForHeading(h, 52) - 25).Add(r.Mul(26))
				f := carFrame{car: committed, world: w, tls: tls, laneWidth: 52, activeCount: 2}
				committed.JunctionTurn = planCarJunctionTurn(f, it)
				if committed.JunctionTurn == nil || committed.JunctionTurn.Straight {
					t.Fatal("expected left turn")
				}
				committed.Pos = committed.JunctionTurn.Points[0]
				moveCarThroughJunction(committed, 1.4)
				entrant := &sim.TrafficCar{ID: 1, Heading: h + math.Pi/2, Length: 46, Width: 22}
				f.car = entrant
				it.Phase = phase
				if !otherHasJunctionPriority(f, it, committed, 1, 10) {
					t.Fatal("phase change revoked a turn already in the junction")
				}
			})
		}
	}
}

func TestThreeApproachesClearAfterPhaseHandoff(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		t.Run(fmt.Sprint(lw), func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -4000}, P2: sim.Vec2{X: 4000}, Lanes: 6}, {P1: sim.Vec2{Y: -4000}, P2: sim.Vec2{Y: 4000}, Lanes: 2}}}
			w.RebuildIntersections()
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			it := tls.Intersections[0]
			it.Phase = sim.PhaseEWLeftGreen
			// An east-approach left turn is still crossing when its phase ends.
			turning := &sim.TrafficCar{ID: 1, Heading: math.Pi, IsEW: true, Length: 46, Width: 22, Speed: 60, TargetSpeed: 85}
			turning.Pos = sim.Vec2{X: it.GetStopLineDistForHeading(math.Pi, lw) + 25, Y: -lw / 2}
			f := carFrame{car: turning, world: w, tls: tls, laneWidth: lw, activeCount: 3}
			turning.JunctionTurn = planCarJunctionTurn(f, it)
			turning.Pos = turning.JunctionTurn.Points[0]
			moveCarThroughJunction(turning, 1.4)
			eastbound := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: -it.GetStopLineDistForHeading(0, lw) - 40, Y: 1.5 * lw}, Heading: 0, IsEW: true, LaneIndex: 1, Length: 46, Width: 22, TargetSpeed: 100}
			northbound := &sim.TrafficCar{ID: 3, Pos: sim.Vec2{X: lw / 2, Y: it.GetStopLineDistForHeading(-math.Pi/2, lw) + 40}, Heading: -math.Pi / 2, Length: 46, Width: 22, TargetSpeed: 100}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{turning, eastbound, northbound}}
			it.Phase = sim.PhaseEWGreen
			entryFrame := carFrame{car: eastbound, world: w, tls: tls, laneWidth: lw, index: 1, activeCount: 3}
			if limit := tm.junctionQueueSpeed(entryFrame, 100); limit >= 100 {
				t.Fatal("new green traffic was admitted across the unfinished turn")
			}
			done := false
			for tick := 0; tick < 2400; tick++ {
				if tick == 900 {
					it.Phase = sim.PhaseNSGreen
				}
				for i, c := range tm.Cars {
					tm.StepCar(1./60, c, i, nil, tls, 3, lw, w)
				}
				for i, a := range tm.Cars {
					for _, b := range tm.Cars[i+1:] {
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 23, HalfWid: 11}); hit {
							t.Fatalf("contact at tick %d cars %d/%d", tick, a.ID, b.ID)
						}
					}
				}
				if turning.Pos.Y > 700 && eastbound.Pos.X > 700 && northbound.Pos.Y < -700 {
					done = true
					break
				}
			}
			if !done {
				for _, c := range tm.Cars {
					t.Logf("car %d pos %v speed %.2f heading %.2f turn %v", c.ID, c.Pos, c.Speed, c.Heading, c.JunctionTurn != nil)
				}
				t.Fatal("three approaches failed to clear without recovery")
			}
		})
	}
}

func TestEnteredCarBeforeTurnTangentKeepsPriority(t *testing.T) {
	w, tls := forcedLeftWorld(6, 0)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseNSGreen
	entered := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: -it.GetStopLineDistForHeading(0, 52) - 20, Y: 26}, Heading: 0, Length: 46, Width: 22, IsEW: true, Speed: 60}
	entrant := &sim.TrafficCar{ID: 1, Heading: math.Pi / 2, Length: 46, Width: 22}
	f := carFrame{car: entrant, world: w, tls: tls, laneWidth: 52}
	if !otherHasJunctionPriority(f, it, entered, 1, 10) {
		t.Fatal("vehicle past the entry line lost protection before reaching turn tangent")
	}
	entered.Pos.X -= 50
	if otherHasJunctionPriority(f, it, entered, 1, 10) {
		t.Fatal("waiting red approach reserved the box")
	}
}
