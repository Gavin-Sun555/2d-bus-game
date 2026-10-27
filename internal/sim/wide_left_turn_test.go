package sim

import (
	"fmt"
	"math"
	"testing"
)

func wideLeftBodies(path []Vec2) []OBB {
	var bodies []OBB
	for i := 1; i < len(path); i++ {
		delta := path[i].Sub(path[i-1])
		n := int(math.Ceil(delta.Length() / 4))
		for j := 0; j < n; j++ {
			p := path[i-1].Add(delta.Mul(float64(j) / float64(n)))
			bodies = append(bodies, OBB{Center: p, Heading: math.Atan2(delta.Y, delta.X), HalfLen: 48, HalfWid: 18})
		}
	}
	return bodies
}

func TestWideLeftTurnSeparatesOpposingBodies(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, inOffset := range []float64{.5 * lw, 3.5 * lw} {
			for _, outOffset := range []float64{1.5 * lw, 2.5 * lw, 3.5 * lw} {
				t.Run(fmt.Sprintf("w%.0f/in%.1f/out%.1f", lw, inOffset/lw, outOffset/lw), func(t *testing.T) {
					g := NewJunctionGeometry(4*lw+12, 4*lw+12, lw)
					east, north := Vec2{X: 1}, Vec2{Y: -1}
					a := wideLeftBodies(WideLeftTurnPath(Vec2{}, east, north, inOffset, outOffset, g))
					for _, otherIn := range []float64{.5 * lw, 3.5 * lw} {
						for _, otherOut := range []float64{1.5 * lw, 2.5 * lw, 3.5 * lw} {
							b := wideLeftBodies(WideLeftTurnPath(Vec2{}, east.Mul(-1), north.Mul(-1), otherIn, otherOut, g))
							assertWideCorridorsClear(t, a, b)
						}
					}
					// Civilian left-turn arcs keep their own target lane and must remain
					// separate from the new bus curve, including the closest median exit.
					for _, exitOffset := range []float64{.5 * lw, 1.5 * lw} {
						b := wideLeftBodies(QuarterTurnPath(Vec2{}, east.Mul(-1), north.Mul(-1), .5*lw, exitOffset, g.TurnRadius))
						for i := range b {
							b[i].HalfLen = 24
							b[i].HalfWid = 11
						}
						assertWideCorridorsClear(t, a, b)
					}
					// Queues on all four approaches remain behind their stop lines.
					for _, dir := range []Vec2{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}} {
						right := Vec2{X: -dir.Y, Y: dir.X}
						half := math.Abs(dir.X)*g.HalfX + math.Abs(dir.Y)*g.HalfY
						for lane := 0; lane < 4; lane++ {
							if dir == east && math.Abs((float64(lane)+.5)*lw-inOffset) < 1 {
								continue // the following queue must leave room for this bus
							}
							p := dir.Mul(-half - JunctionStopInset - 24).Add(right.Mul((float64(lane) + .5) * lw))
							assertWideCorridorsClear(t, a, []OBB{{Center: p, Heading: math.Atan2(dir.Y, dir.X), HalfLen: 24, HalfWid: 11}})
						}
					}
				})
			}
		}
	}
}

func assertWideCorridorsClear(t *testing.T, a, b []OBB) {
	t.Helper()
	for _, ba := range a {
		for _, bb := range b {
			if hit, _, _ := CheckOBBCollision(ba, bb); hit {
				t.Fatalf("turn bodies overlap at %v / %v", ba.Center, bb.Center)
			}
		}
	}
}

func TestWideLeftTurnKeepsParallelLanesSeparate(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		g := NewJunctionGeometry(4*lw+12, 4*lw+12, lw)
		in, out := Vec2{X: 1}, Vec2{Y: -1}
		a := wideLeftBodies(WideLeftTurnPath(Vec2{}, in, out, .5*lw, 1.5*lw, g))
		b := wideLeftBodies(WideLeftTurnPath(Vec2{}, in, out, 3.5*lw, 3.5*lw, g))
		assertWideCorridorsClear(t, a, b)
	}
}
