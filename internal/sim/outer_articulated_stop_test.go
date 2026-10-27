package sim

import (
	"math"
	"testing"
)

func TestOuterArticulatedBoardingRequiresWholePlatformLength(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := Vec2{X: math.Cos(h), Y: math.Sin(h)}
		r := Vec2{X: -u.Y, Y: u.X}
		s := NewBusStop(0, "test", Vec2{}, h, 8, RGBA(0, 0, 0, 255), 52)
		for _, along := range []float64{-100, 0, 96, 140} {
			p := s.Pos.Add(u.Mul(along)).Sub(r.Mul(61))
			b := NewBus(p.X, p.Y, h)
			b.SetModel(BusArticulated)
			want := along == 0 || along == 96
			if got := s.ContainsBusInOuterLane(b, 52); got != want {
				t.Fatalf("h%.2f along%.0f got%v want%v", h, along, got, want)
			}
		}
	}
}
