package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCurbLeftTurnPreservesDistantStationEntry(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", h), func(t *testing.T) {
			w, _ := turnTestWorld(8, 8, h, 52)
			in := sim.UnitFromAngle(h)
			out := sim.Vec2{X: in.Y, Y: -in.X}
			ri, ro := sim.Vec2{X: -in.Y, Y: in.X}, sim.Vec2{X: -out.Y, Y: out.X}
			curb := 3.5 * 52.
			g := w.JunctionAt(sim.Vec2{}, 52)
			half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
			entry := out.Mul(7600).Add(ro.Mul(curb))
			bay := out.Mul(7770).Add(ro.Mul(curb + 47.12))
			target := bay.Add(out.Mul(60))
			b := sim.NewBus(0, 0, h)
			b.Pos = in.Mul(-1200).Add(ri.Mul(curb))
			ad := NewAutopilotDriver()
			ad.drivePath = []sim.Vec2{b.Pos, out.Mul(half).Add(ro.Mul(curb)), entry, bay, target}
			if !ad.joinCurbLeftTurn(autopilotFrame{bus: b, world: w, laneWidth: 52}, sim.Vec2{}, in, curb, true) {
				t.Fatal("turn not joined")
			}
			found := false
			for _, p := range ad.drivePath {
				if p.Distance(entry) < .001 {
					found = true
				}
			}
			if !found {
				t.Fatalf("lost station entry: tail%v", ad.drivePath[len(ad.drivePath)-4:])
			}
			for i := 1; i < len(ad.drivePath); i++ {
				a, z := ad.drivePath[i-1], ad.drivePath[i]
				if z.Sub(a).Dot(out) > 1000 && math.Abs(z.Sub(a).Dot(ro)) > 1 {
					t.Fatal("long diagonal toward shoulder")
				}
			}
		})
	}
}
