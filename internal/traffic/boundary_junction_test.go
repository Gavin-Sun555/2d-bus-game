package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// North-west corner: ordinary through lanes face out of the map, while the
// innermost lane can turn south. Both cars were observed past the stop line.
func TestMetropolisThroughCarsClearBoundaryJunction(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		t.Run(fmt.Sprint(lw), func(t *testing.T) {
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			center := sim.Vec2{X: -2600 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}
			var inter *sim.TrafficIntersection
			for _, it := range tls.Intersections {
				if it.Center.Distance(center) < 1 {
					inter = it
				}
			}
			if inter == nil {
				t.Fatal("missing boundary junction")
			}
			inter.Phase = sim.PhaseEWGreen
			stop := inter.GetStopLineDistForHeading(math.Pi, lw)
			green := &sim.TrafficCar{ID: 2, Pos: center.Add(sim.Vec2{X: stop - 35, Y: -2.5 * lw}), Heading: math.Pi, Length: 46, Width: 22, LaneIndex: 2, RoadCenter: center.Y, IsEW: true, TargetSpeed: 100}
			blue := &sim.TrafficCar{ID: 3, Pos: center.Add(sim.Vec2{X: stop - 240, Y: -1.5 * lw}), Heading: math.Pi, Length: 46, Width: 22, LaneIndex: 1, RoadCenter: center.Y, IsEW: true, TargetSpeed: 100}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{green, blue}}
			for step := 0; step < 1200; step++ {
				tm.Update(1./60, nil, tls, 1, lw, w)
				if green.Heading != math.Pi || blue.Heading != math.Pi {
					t.Fatal("through car turned at the map exit")
				}
				present := false
				for _, car := range tm.Cars {
					present = present || car == green || car == blue
				}
				if !present {
					return
				}
			}
			t.Fatalf("exit stuck: green%v speed%.1f blue%v speed%.1f", green.Pos, green.Speed, blue.Pos, blue.Speed)
		})
	}
}

func TestMapExitThroughCarsObeySignalsAndClearOnGreen(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("width%.0f/h%.2f", lw, h), func(t *testing.T) {
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				center := sim.Vec2{X: -2600 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}
				if h == 0 {
					center.X = -center.X
				}
				if h == math.Pi/2 {
					center = sim.Vec2{X: 2600 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}
				}
				var inter *sim.TrafficIntersection
				for _, it := range tls.Intersections {
					if it.Center.Distance(center) < 1 {
						inter = it
					}
				}
				if inter == nil {
					t.Fatal("missing junction")
				}
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				ew := math.Abs(u.X) > .5
				coord := center.X
				if ew {
					coord = center.Y
				}
				inter.Phase = sim.PhaseNSGreen
				if !ew {
					inter.Phase = sim.PhaseEWGreen
				}
				stop := inter.GetStopLineDistForHeading(h, lw)
				car := &sim.TrafficCar{ID: 5, Pos: center.Sub(u.Mul(stop + 100)).Add(r.Mul(1.5 * lw)), Heading: h, Length: 46, Width: 22, LaneIndex: 1, RoadCenter: coord, IsEW: ew, TargetSpeed: 100}
				tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
				for tick := 0; tick < 600; tick++ {
					tm.Update(1./60, nil, tls, 1, lw, w)
				}
				if center.Sub(car.Pos).Dot(u)-car.Length/2 < stop {
					t.Fatal("exit car crossed on red")
				}
				if car.Speed > 1 {
					t.Fatal("exit car did not wait for green")
				}
				inter.Phase = sim.PhaseEWGreen
				if !ew {
					inter.Phase = sim.PhaseNSGreen
				}
				removed := false
				for tick := 0; tick < 1800; tick++ {
					before := car.Pos
					tm.Update(1./60, nil, tls, 1, lw, w)
					if car.Pos.Distance(before) > car.Speed/60+.001 {
						t.Fatal("exit car teleported")
					}
					if math.Abs(sim.NormalizeAngle(car.Heading-h)) > .001 || car.LaneIndex != 1 {
						t.Fatal("exit changed lane or heading")
					}
					present := false
					for _, other := range tm.Cars {
						present = present || other == car
					}
					if !present {
						if !outsideRoadBounds(car, w, car.Length/2) {
							t.Fatal("removed before rear left the map")
						}
						if car.Pos.Sub(center).Dot(u) > car.Length/2+4 {
							t.Fatal("removal waited past the map exit")
						}
						removed = true
						break
					}
				}
				if !removed {
					t.Fatalf("car failed to leave: pos%v speed%.1f", car.Pos, car.Speed)
				}
				if len(tm.Cars) != 1 || tm.Cars[0] == car {
					t.Fatal("traffic count not restored with a new car")
				}
				if tm.Cars[0].Pos.Distance(center) < 100 {
					t.Fatal("replacement spawned at the exit")
				}
			})
		}
	}
}
