package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestUnifiedStationQueueCapacity(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for first := BusStandard; first < BusModelCount; first++ {
			for next := BusStandard; next < BusModelCount; next++ {
				t.Run(fmt.Sprintf("h%.2f/%d/%d", h, first, next), func(t *testing.T) {
					st := NewBusStop(0, "test", Vec2{}, h, 8, RGBA(0, 0, 0, 255), 52)
					if st.Length != 336 {
						t.Fatal("platform is not uniform")
					}
					a, b, c := NewBus(0, 0, h), NewBus(0, 0, h), NewBus(0, 0, h)
					a.SetModel(first)
					b.SetModel(next)
					PlaceBusInStationQueue(a, st, nil, 52)
					PlaceBusInStationQueue(b, st, []*Bus{a}, 52)
					PlaceBusInStationQueue(c, st, []*Bus{a, b}, 52)
					if !st.ContainsBus(a) || a.Pos.Distance(st.FrontBerthPosition()) > .01 {
						t.Fatal("first bus not at front", a.Pos)
					}
					if first == BusArticulated || next == BusArticulated {
						if st.ContainsBus(b) {
							t.Fatal("long bus shares platform")
						}
					} else if !st.ContainsBus(b) || b.Pos.Distance(st.RearBerthPosition()) > .01 {
						t.Fatal("second short bus not at rear", b.Pos)
					}
					if st.ContainsBus(c) {
						t.Fatal("third vehicle admitted")
					}
					for _, pair := range [][2]*Bus{{a, b}, {a, c}, {b, c}} {
						if hit, _, _ := CheckBusesCollision(pair[0], pair[1]); hit {
							t.Fatal("queue body overlap")
						}
					}
				})
			}
		}
	}
}
func TestAllMapsUseUnifiedStationBays(t *testing.T) {
	for theme := MapThemeMetropolis; theme <= MapThemeCustom; theme++ {
		for _, lw := range []float64{42, 52, 78} {
			w := NewWorld(lw, theme)
			for _, st := range w.Stops {
				if st.Length != StandardStopLength {
					t.Fatalf("map%d stop%d length%.0f", theme, st.ID, st.Length)
				}
			}
		}
	}
}
