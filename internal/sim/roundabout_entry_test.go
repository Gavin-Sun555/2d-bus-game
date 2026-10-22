package sim

import (
	"fmt"
	"math"
	"testing"
)

// Check the whole pair of bodies at independent curve positions. Merely testing
// their centers at equal curve parameters misses corners and speed differences.
func TestRoundaboutParallelEntrySweptBodies(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, busLane := range []int{0, 1} {
				t.Run(fmt.Sprintf("width%.0f/h%.2f/bus%d", lw, h, busLane), func(t *testing.T) {
					rb := Roundabout{Center: Vec2{X: 130, Y: -240}, Radius: 240, InnerRadius: 120}
					f := Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := Vec2{X: -f.Y, Y: f.X}
					bus := OBB{Center: rb.Center.Sub(f.Mul(340)).Add(r.Mul((float64(busLane) + .5) * lw)), Heading: h, HalfLen: 48, HalfWid: 18}
					car := OBB{Center: rb.Center.Sub(f.Mul(340)).Add(r.Mul((float64(1-busLane) + .5) * lw)), Heading: h, HalfLen: 24, HalfWid: 11}
					if !SeparateRoundaboutEntries(rb, f, lw, bus, true, car, false) {
						t.Fatal("clear adjacent joins rejected")
					}
					_, a, _ := RoundaboutEntryCorridor(rb, f, lw, bus, true)
					_, b, _ := RoundaboutEntryCorridor(rb, f, lw, car, false)
					for i := 0; i <= 96; i++ {
						p, pa := RoundaboutEntryPoint(a, float64(i)/96)
						x := bus
						x.Center = p
						x.Heading = math.Atan2(pa.Y, pa.X)
						for j := 0; j <= 96; j++ {
							q, qa := RoundaboutEntryPoint(b, float64(j)/96)
							y := car
							y.Center = q
							y.Heading = math.Atan2(qa.Y, qa.X)
							if hit, _, _ := CheckOBBCollision(x, y); hit {
								t.Fatalf("sweeps intersect at %d/%d", i, j)
							}
						}
					}
					for _, c := range []RoundaboutJoin{a, b} {
						p, tangent := RoundaboutEntryPoint(c, 0)
						if math.Abs(NormalizeAngle(math.Atan2(tangent.Y, tangent.X)-h)) > .001 {
							t.Fatal("entry tangent is discontinuous")
						}
						end, tangent := RoundaboutEntryPoint(c, 1)
						radial := end.Sub(rb.Center)
						if tangent.Normalize().Dot(Vec2{X: radial.Y, Y: -radial.X}.Normalize()) < .999 {
							t.Fatal("join tangent is discontinuous")
						}
						if p.Distance(end) < 100 {
							t.Fatal("join is too short")
						}
					}
				})
			}
		}
	}
}

func TestParallelEntryRejectsConflictingBodies(t *testing.T) {
	rb := Roundabout{Radius: 240, InnerRadius: 120}
	f := Vec2{X: 1}
	a := OBB{Center: Vec2{X: -310, Y: 26}, HalfLen: 48, HalfWid: 18}
	cases := []struct {
		name string
		body OBB
	}{
		{"same lane", OBB{Center: Vec2{X: -240, Y: 26}, HalfLen: 24, HalfWid: 11}},
		{"crossing", OBB{Center: Vec2{X: -240, Y: 78}, Heading: math.Pi / 2, HalfLen: 24, HalfWid: 11}},
		{"oncoming", OBB{Center: Vec2{X: -240, Y: 78}, Heading: math.Pi, HalfLen: 24, HalfWid: 11}},
		{"wide vehicle", OBB{Center: Vec2{X: -240, Y: 78}, HalfLen: 60, HalfWid: 45}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if SeparateRoundaboutEntries(rb, f, 52, a, true, c.body, false) {
				t.Fatal("unsafe parallel entry accepted")
			}
		})
	}
}

func TestRoundaboutEntryIntentExpires(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.RoundaboutEntry = &BusRoundaboutEntry{Remaining: .1}
	b.Update(.2, 0, 1, 0, false, false)
	if b.RoundaboutEntry != nil {
		t.Fatal("stale entry intent retained")
	}
}
