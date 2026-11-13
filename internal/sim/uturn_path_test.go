package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestUTurnPathIsDrivableAndContained(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, lw := range []float64{42, 52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("lanes%d/width%.0f/h%.2f", lanes, lw, h), func(t *testing.T) {
					g := NewJunctionGeometry(float64(lanes)*lw/2, float64(lanes)*lw/2, lw)
					path := UTurnPath(Vec2{}, h, lw/2, (float64(lanes)/2-.5)*lw, g)
					if len(path) < 48 {
						t.Fatal("U-turn still has orthogonal corners")
					}
					previous := h
					for i := 1; i < len(path); i++ {
						delta := path[i].Sub(path[i-1])
						heading := math.Atan2(delta.Y, delta.X)
						turn := math.Abs(NormalizeAngle(heading - previous))
						if turn > .28 {
							t.Fatalf("path corner at %d: %.2f", i, turn)
						}
						if i > 1 {
							priorLength := path[i-1].Distance(path[i-2])
							curvature := turn / ((priorLength + delta.Length()) / 2)
							if curvature > math.Tan(.68)/60+.001 {
								t.Fatalf("unsteerable curvature at %d: %.4f", i, curvature)
							}
						}
						previous = heading
						forward := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := Vec2{X: -forward.Y, Y: forward.X}
						for _, along := range []float64{-48, 48} {
							for _, across := range []float64{-18, 18} {
								p := path[i].Add(forward.Mul(along)).Add(right.Mul(across))
								if math.Abs(p.X) > g.HalfX+1 || math.Abs(p.Y) > g.HalfY+1 {
									t.Fatalf("turn body leaves junction at%d: %v in%+v", i, p, g)
								}
							}
						}
					}
					if math.Cos(previous-h) > -.98 {
						t.Fatal("outgoing tangent is not reversed")
					}
				})
			}
		}
	}
}

func TestWideJunctionUTurnUsesNearHalf(t *testing.T) {
	const lw = 52
	g := NewJunctionGeometry(4*lw, 4*lw, lw)
	for _, offsets := range [][2]float64{{lw / 2, 2.5 * lw}, {3.5 * lw, 3.5 * lw}} {
		path := UTurnPath(Vec2{}, 0, offsets[0], offsets[1], g)
		for _, point := range path {
			if point.X > 1 {
				t.Fatalf("U-turn %v enters far half: %v", offsets, point)
			}
		}
		if path[49].Y < -3.5*lw+10 || math.Abs(path[len(path)-1].Y+offsets[1]) > .01 {
			t.Fatalf("U-turn %v misses inner receiving lane or final lane: %v to %v", offsets, path[49], path[len(path)-1])
		}
	}
}
