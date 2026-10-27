package sim

import (
	"math"
	"testing"
)

func TestJunctionIndexPreservesOrderAndCoversCandidates(t *testing.T) {
	w := NewWorld(52)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)
	tls.PrepareJunctionIndex(52)
	var ids []int
	var found []*TrafficIntersection
	for _, it := range tls.Intersections {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, speed := range []float64{0, 182, 500, 1000} {
				radius := tls.CarJunctionQueryRadius(52, speed, 48, 1./60)
				p := it.Center.Sub(Vec2{X: math.Cos(h), Y: math.Sin(h)}.Mul(it.GetStopLineDistForHeading(h, 52) + speed*speed/440 + 80))
				ids, found = tls.NearbyJunctions(p, radius, ids, found)
				last := -1
				for i, id := range ids {
					if id <= last || found[i] != tls.Intersections[id] {
						t.Fatal("priority order changed")
					}
					last = id
				}
				for _, original := range tls.Intersections {
					if math.Abs(original.Center.X-p.X) <= radius && math.Abs(original.Center.Y-p.Y) <= radius {
						ok := false
						for _, candidate := range found {
							if candidate == original {
								ok = true
								break
							}
						}
						if !ok {
							t.Fatal("spatial broad phase omitted a junction")
						}
					}
				}
			}
		}
	}
	old := tls.junctionIndex
	tls.Intersections[0].Phase = PhaseEWLeftGreen
	tls.PrepareJunctionIndex(52)
	if tls.junctionIndex != old {
		t.Fatal("phase transition rebuilt static index")
	}
	tls.Intersections[0].Center.X += 9000
	tls.PrepareJunctionIndex(52)
	if tls.junctionIndex == old {
		t.Fatal("in-place center edit not detected")
	}
	old = tls.junctionIndex
	tls.Intersections[0].ApproachLanes[0] = 16
	tls.PrepareJunctionIndex(78)
	if tls.junctionIndex == old {
		t.Fatal("lane/width edit not detected")
	}
}
