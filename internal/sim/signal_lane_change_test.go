package sim

import (
	"fmt"
	"math"
	"testing"
)

// A lane-change countersteer is not a left turn: the front crossing the
// stop line in a through lane must use that approach's through signal.
func TestThroughGreenAfterLaneChangeIsNotRedLight(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, yellow := range []bool{false, true} {
					for _, atBoundary := range []bool{false, true} {
						t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f/yellow%v/boundary%v", lw, lanes, heading, yellow, atBoundary), func(t *testing.T) {
							phase := PhaseEWGreen
							if math.Abs(math.Sin(heading)) > .5 {
								phase = PhaseNSGreen
							}
							if yellow {
								phase++
							}
							inter := &TrafficIntersection{IsMajor: true, Phase: phase, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{lanes, lanes, lanes, lanes}, ApproachIs4Lane: [4]bool{true, true, true, true}}
							tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
							forward := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
							right := Vec2{X: -forward.Y, Y: forward.X}
							line := inter.GetStopLineDistForHeading(heading, lw)
							lateral := 1.5 * lw
							yaw := 0.0
							if atBoundary {
								lateral = .9 * lw
								yaw = .18
							}
							pos := forward.Mul(-line - 55).Add(right.Mul(lateral))
							bus := NewBus(pos.X, pos.Y, heading+yaw)
							bus.Speed = 35
							bus.SteeringAngle = -.06 // countersteering to straighten after moving outward
							tls.CheckPlayerViolation(bus, lw)
							if inter.PlayerInside {
								t.Fatal("lane-change yaw recorded entry before the front crossed the stop line")
							}
							bus.Pos = forward.Mul(-line - 35).Add(right.Mul(lateral))
							if violation, notice := tls.CheckPlayerViolation(bus, lw); violation {
								t.Fatal(notice)
							}
							if !inter.PlayerInside {
								t.Fatal("did not record stop-line entry")
							}
							if inter.PlayerEntryLane != 1 {
								t.Fatalf("front entered through lane but recorded lane%d", inter.PlayerEntryLane)
							}
							if inter.PlayerCrossedOnRed {
								t.Fatal("through green was mistaken for the red left arrow")
							}
							if inter.PlayerTurnedLeft {
								t.Fatal("countersteering was recorded as a left turn")
							}
							// Continue behind traffic; lights may change before the rear clears.
							if math.Abs(math.Sin(heading)) > .5 {
								inter.Phase = PhaseEWGreen
							} else {
								inter.Phase = PhaseNSGreen
							}
							bus.Heading = heading
							bus.SteeringAngle = 0
							for s := -line - 30; s < line+bus.Length+10; s += 5 {
								bus.Pos = forward.Mul(s).Add(right.Mul(1.5 * lw))
								if violation, notice := tls.CheckPlayerViolation(bus, lw); violation {
									t.Fatalf("lawful passage fined after signal change: %s", notice)
								}
							}
						})
					}
				}
			}
		}
	}
}

func TestLaneChangeCorrectionDoesNotExcuseActualRed(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, leftLane := range []bool{false, true} {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f/left%v", lw, lanes, heading, leftLane), func(t *testing.T) {
						phase := PhaseNSGreen
						if math.Abs(math.Sin(heading)) > .5 {
							phase = PhaseEWGreen
						}
						if leftLane {
							if math.Abs(math.Sin(heading)) > .5 {
								phase = PhaseNSGreen
							} else {
								phase = PhaseEWGreen
							}
						}
						inter := &TrafficIntersection{IsMajor: true, Phase: phase, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{lanes, lanes, lanes, lanes}, ApproachIs4Lane: [4]bool{true, true, true, true}}
						tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
						forward := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := Vec2{X: -forward.Y, Y: forward.X}
						line := inter.GetStopLineDistForHeading(heading, lw)
						lateral := 1.5 * lw
						if leftLane {
							lateral = .5 * lw
						}
						pos := forward.Mul(-line - 55).Add(right.Mul(lateral))
						bus := NewBus(pos.X, pos.Y, heading)
						bus.Speed = 35
						bus.SteeringAngle = -.06
						tls.CheckPlayerViolation(bus, lw)
						bus.Pos = forward.Mul(-line - 35).Add(right.Mul(lateral))
						if violation, notice := tls.CheckPlayerViolation(bus, lw); violation {
							t.Fatalf("fined before rear crossed: %s", notice)
						}
						bus.Pos = forward.Mul(-line + 55).Add(right.Mul(lateral))
						if violation, notice := tls.CheckPlayerViolation(bus, lw); !violation {
							t.Fatalf("actual red not detected: %s", notice)
						}
						if violation, _ := tls.CheckPlayerViolation(bus, lw); violation {
							t.Fatal("same passage fined twice")
						}
					})
				}
			}
		}
	}
}
