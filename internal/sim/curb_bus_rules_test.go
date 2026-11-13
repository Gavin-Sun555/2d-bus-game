package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestWideRoadCurbBusThroughAndLeftSignals(t *testing.T) {
	for _, lanes := range []int{6, 8} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, movement := range []string{"left", "through", "through-on-left-green"} {
				t.Run(fmt.Sprintf("lanes%d/h%.2f/%s", lanes, h, movement), func(t *testing.T) {
					const lw = 52.
					i := &TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{lanes, lanes, lanes, lanes}, ApproachIs4Lane: [4]bool{true, true, true, true}, GreenDuration: 8, LeftGreenDuration: 5.5, YellowDuration: 2.5}
					i.Phase = PhaseEWLeftGreen
					if math.Abs(math.Sin(h)) > .5 {
						i.Phase = PhaseNSLeftGreen
					}
					if movement == "through" {
						i.Phase = PhaseEWGreen
						if math.Abs(math.Sin(h)) > .5 {
							i.Phase = PhaseNSGreen
						}
					}
					tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{i}}
					in := UnitFromAngle(h)
					r := Vec2{X: -in.Y, Y: in.X}
					curb := (float64(lanes)/2 - .5) * lw
					line := i.GetStopLineDistForHeading(h, lw)
					p := in.Mul(-line - 45).Add(r.Mul(curb))
					b := NewBus(p.X, p.Y, h)
					b.Speed = 40
					if bad, why := tls.CheckPlayerViolation(b, lw); bad {
						t.Fatalf("legal curb entry fined: %s", why)
					}
					if !i.PlayerInside || !i.PlayerEntryBusLane {
						t.Fatal("wide curb bus permission not recognized")
					}
					if movement == "left" {
						out := Vec2{X: in.Y, Y: -in.X}
						rr := Vec2{X: -out.Y, Y: out.X}
						b.Pos = out.Mul(line + 40).Add(rr.Mul(curb))
						b.Heading = h - math.Pi/2
						// The left arrow may have changed after a lawful entry.
						i.Phase = PhaseEWGreen
						if math.Abs(in.Y) > .5 {
							i.Phase = PhaseNSGreen
						}
					} else {
						b.Pos = in.Mul(line + 40).Add(r.Mul(curb))
					}
					bad, why := tls.CheckPlayerViolation(b, lw)
					if movement == "through-on-left-green" {
						if !bad {
							t.Fatal("left arrow incorrectly authorized straight movement")
						}
					} else if bad {
						t.Fatalf("legal curb exit fined: %s", why)
					}
				})
			}
		}
	}
}
