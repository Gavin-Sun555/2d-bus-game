package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestMedianRightPathSeparatesCrossLeftTraffic(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			u := sim.UnitFromAngle(h)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{
				{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
				{P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
			}}
			w.RebuildIntersections()
			g := w.JunctionAt(sim.Vec2{}, lw)
			a := sim.NewBusStop(0, "a", u.Mul(-2200), h, 8, sim.RGBA(0, 0, 0, 255), lw)
			b := sim.NewBusStop(1, "b", r.Mul(2200), h+math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
			for _, s := range []*sim.BusStop{a, b} {
				s.CenterBRT = true
				s.UpdateGeometry(lw)
			}
			path := sim.ComputeRoadPathBetweenStops(w, a, b, lw)
			ours := sim.AppendPathBodies(nil, path, 96, 36)
			for _, crossH := range []float64{h + math.Pi/2, h - math.Pi/2} {
				in := sim.UnitFromAngle(crossH)
				out := sim.UnitFromAngle(crossH - math.Pi/2)
				carPath := sim.QuarterTurnPath(sim.Vec2{}, in, out, 1.5*lw, 1.5*lw, g.TurnRadius)
				theirs := sim.AppendPathBodies(nil, carPath, 46, 22)
				if !sim.CorridorsSeparated(ours, theirs) {
					t.Fatalf("protected right and cross left conflict width%.0f heading%.2f cross%.2f", lw, h, crossH)
				}
				busLeft := sim.WideLeftTurnPath(sim.Vec2{}, in, out, .5*lw, .5*lw, g)
				busBodies := sim.AppendPathBodies(nil, busLeft, 96, 36)
				// Compare physical bodies; corridor skins intentionally touch on the
				// narrowest 42px median lanes, where the live admission check yields.
				physicalOurs := append([]sim.OBB(nil), ours...)
				for i := range physicalOurs {
					physicalOurs[i].HalfLen -= 4
					physicalOurs[i].HalfWid -= 3
				}
				for i := range busBodies {
					busBodies[i].HalfLen -= 4
					busBodies[i].HalfWid -= 3
				}
				if !sim.CorridorsSeparated(physicalOurs, busBodies) {
					for i, a := range physicalOurs {
						for j, b := range busBodies {
							if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
								t.Fatalf("BRT conflict width%.0f heading%.2f cross%.2f ours%d %+v other%d %+v", lw, h, crossH, i, a, j, b)
							}
						}
					}
				}
			}
		}
	}
}
