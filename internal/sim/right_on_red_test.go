package sim

import (
	"fmt"
	"math"
	"testing"
)

func eightLaneRedApproach(lw, heading float64, busLane bool) (*TrafficLightSystem, *TrafficIntersection, Vec2, Vec2) {
	phase := PhaseNSGreen
	if math.Abs(math.Sin(heading)) > .5 {
		phase = PhaseEWGreen
	}
	inter := &TrafficIntersection{IsMajor: true, Phase: phase, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{8, 8, 8, 8}, ApproachBusLane: [4]bool{busLane, busLane, busLane, busLane}}
	forward := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	return &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}, inter, forward, Vec2{X: -forward.Y, Y: forward.X}
}

func TestEightLaneRightOnRedWaitsForActualTurn(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, busLane := range []bool{false, true} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/bus%v", lw, heading, busLane), func(t *testing.T) {
					tls, inter, forward, right := eightLaneRedApproach(lw, heading, busLane)
					line := inter.GetStopLineDistForHeading(heading, lw)
					bus := NewBus(0, 0, heading)
					bus.Speed = 60
					// A wide box leaves a long straight run between the stop line and
					// the actual right-turn tangent, with no right steering yet.
					curve := QuarterTurnPath(Vec2{}, forward, right, 3.5*lw, 1.5*lw, 90)
					tangent := curve[0].Dot(forward)
					for along := -line - 80; along < tangent; along += 3 {
						bus.Pos = forward.Mul(along).Add(right.Mul(3.5 * lw))
						if bad, notice := tls.CheckPlayerViolation(bus, lw); bad {
							t.Fatalf("lawful right turn fined before steering started: %s", notice)
						}
					}
					for i, point := range curve {
						bus.Pos = point
						next := min(i+1, len(curve)-1)
						previous := max(0, i-1)
						v := curve[next].Sub(curve[previous])
						bus.Heading = math.Atan2(v.Y, v.X)
						if bad, notice := tls.CheckPlayerViolation(bus, lw); bad {
							t.Fatal(notice)
						}
					}
					bus.Heading = NormalizeAngle(heading + math.Pi/2)
					for along := 0.0; along < line+160; along += 3 {
						bus.Pos = curve[len(curve)-1].Add(right.Mul(along))
						if bad, notice := tls.CheckPlayerViolation(bus, lw); bad {
							t.Fatalf("lawful right turn fined while exiting: %s", notice)
						}
					}
				})
			}
		}
	}
}

func TestEightLaneRightLaneDoesNotExcuseRedStraightOrLeft(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, busLane := range []bool{false, true} {
				for _, left := range []bool{false, true} {
					t.Run(fmt.Sprintf("width%.0f/heading%.2f/bus%v/left%v", lw, heading, busLane, left), func(t *testing.T) {
						tls, inter, forward, right := eightLaneRedApproach(lw, heading, busLane)
						line := inter.GetStopLineDistForHeading(heading, lw)
						bus := NewBus(0, 0, heading)
						bus.Speed = 60
						fined := false
						for along := -line - 80; along < line+100; along += 3 {
							bus.Pos = forward.Mul(along).Add(right.Mul(3.5 * lw))
							if left && along > -line+bus.Length {
								bus.Heading = NormalizeAngle(heading - .4)
							}
							if bad, _ := tls.CheckPlayerViolation(bus, lw); bad {
								fined = true
								break
							}
						}
						if !fined {
							t.Fatal("right-lane entry excused an actual red-light violation")
						}
						if bad, _ := tls.CheckPlayerViolation(bus, lw); bad {
							t.Fatal("same passage fined twice")
						}
					})
				}
			}
		}
	}
}
