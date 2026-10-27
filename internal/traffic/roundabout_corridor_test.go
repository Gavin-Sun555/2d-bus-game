package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRoundaboutCorridorPredictsCurveWithoutAdvancingCar(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, phase := range []string{"entry", "circle", "exit"} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, phase), func(t *testing.T) {
				rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
				angle := .8 + h
				car := &sim.TrafficCar{Pos: sim.Vec2{X: 195.25 * math.Cos(angle), Y: 195.25 * math.Sin(angle)}, Heading: angle - math.Pi/2, Speed: 90, Length: 46, Width: 22, InRoundabout: true, RoundaboutRadius: 195.25, RoundaboutTargetR: 195.25, RoundaboutOuterRadius: 220, RoundaboutAngle: angle, RoundaboutExitAngle: -.8 + h, RoundaboutExitHeading: h, RoundaboutExitOffset: 78, RoundaboutTraveled: 2}
				if phase == "entry" {
					car.Pos = sim.Vec2{X: -300*math.Cos(h) - 78*math.Sin(h), Y: -300*math.Sin(h) + 78*math.Cos(h)}
					car.Heading = h
					car.RoundaboutTraveled = 0
					beginRoundaboutEntry(car, rb, 52)
				} else if phase == "exit" {
					beginRoundaboutExit(car)
				}
				before := *car
				curveDistance := 0.
				if car.RoundaboutCurve != nil {
					curveDistance = car.RoundaboutCurve.Distance
				}
				publishRoundaboutCorridor(car)
				if car.Pos != before.Pos || car.Heading != before.Heading || car.RoundaboutAngle != before.RoundaboutAngle || (car.RoundaboutCurve != nil && car.RoundaboutCurve.Distance != curveDistance) {
					t.Fatal("prediction advanced the live car")
				}
				actual := *car
				if car.RoundaboutCurve != nil {
					curve := *car.RoundaboutCurve
					actual.RoundaboutCurve = &curve
				}
				for tick := 0; tick <= 48; tick++ {
					body, ok := sim.PredictJunctionBody(car.JunctionCorridor, float64(tick)*1.5)
					if !ok || body.Center.Distance(actual.Pos) > 2.5 || math.Abs(sim.NormalizeAngle(body.Heading-actual.Heading)) > .12 {
						t.Fatalf("incorrect curved prediction tick%d predicted%+v actual%v h%.3f", tick, body, actual.Pos, actual.Heading)
					}
					moveCar(&actual, 1./60)
				}
			})
		}
	}
}
