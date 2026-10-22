package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestOuterEntryYieldsToCirculatingBus(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("berserk%v/h%.2f", berserk, h), func(t *testing.T) {
				const lw = 52.
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				rb := w.Roundabouts[0]
				in := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				right := sim.Vec2{X: -in.Y, Y: in.X}
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", in.Mul(-800), h, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", in.Mul(800), h, 4, sim.RGBA(0, 0, 0, 255), lw)}
				p := in.Mul(-350).Add(right.Mul(1.5 * lw))
				b := sim.NewBus(p.X, p.Y, h)
				b.Speed = 90
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.DisallowOncomingOvertake = true
				r := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				radius := roundaboutOuterLaneRadius(&rb)
				for tick := 0; tick < 1800; tick++ {
					angle := h + math.Pi + .8 - 90/radius*float64(tick)/60
					ob := VehicleObstacle{Pos: sim.Vec2{X: math.Cos(angle) * radius, Y: math.Sin(angle) * radius}, Heading: angle - math.Pi/2, Speed: 90, Length: 96, Width: 36}
					th, br, st, hb, d, _ := ad.ComputeControls(b, r, w, nil, []VehicleObstacle{ob}, lw, 1./60)
					b.Update(1./60, th, br, st, hb, d)
					next := roundaboutObstacleAt(&rb, ob, 1./60)
					next.HalfLen = 48
					next.HalfWid = 18
					if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, next); hit {
						t.Fatalf("entry collided tick%d bus%v v%.1f ring%v", tick, b.Pos, b.Speed, next.Center)
					}
					if b.Pos.Dot(in) > rb.Radius+150 {
						return
					}
				}
				t.Fatalf("entry did not resume: %v target%.1f", b.Pos, ad.LastTargetSpeed)
			})
		}
	}
}
