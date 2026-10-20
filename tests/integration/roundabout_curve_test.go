package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Check actual displacement against the body direction throughout entry, exit,
// and the return to straight driving, rather than only limiting position jumps.
func TestRoundaboutCarContinuousEntryAndExit(t *testing.T) {
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeTwinCities; theme++ {
		for _, lw := range []float64{42, 52, 78} {
			w := sim.NewWorld(lw, theme)
			for ri, rb := range w.Roundabouts {
				for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					ew := math.Abs(forward.X) > .5
					rc := rb.Center.Y
					if !ew {
						rc = rb.Center.X
					}
					pos := rb.Center.Sub(forward.Mul(rb.Radius + 160))
					lanes := w.GetRoadLanes(rc, ew, pos)
					for lane := 0; lane < lanes/2; lane++ {
						t.Run(fmt.Sprintf("map%d/width%.0f/ring%d/heading%.2f/lane%d", theme, lw, ri, heading, lane), func(t *testing.T) {
							car := &sim.TrafficCar{Pos: pos.Add(right.Mul((float64(lane) + .5) * lw)), Heading: heading, Speed: 90, TargetSpeed: 90, Length: 48, Width: 22, IsEW: ew, RoadCenter: rc, LaneIndex: lane}
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
							entered, exited := false, false
							afterExit := 0
							lastDirection := heading
							for frame := 0; frame < 2400; frame++ {
								previous, oldHeading := car.Pos, car.Heading
								tm.StepCar(1.0/60, car, 0, nil, nil, 1, lw, w)
								delta := car.Pos.Sub(previous)
								distance := delta.Length()
								if distance > .01 {
									direction := math.Atan2(delta.Y, delta.X)
									if err := math.Abs(sim.NormalizeAngle(direction - car.Heading)); err > .15 {
										t.Fatalf("frame%d movement/body mismatch %.3f", frame, err)
									}
									if turn := math.Abs(sim.NormalizeAngle(direction - lastDirection)); turn > .20 {
										t.Fatalf("frame%d trajectory corner %.3f", frame, turn)
									}
									lastDirection = direction
								}
								if turn := math.Abs(sim.NormalizeAngle(car.Heading - oldHeading)); turn > .20 {
									t.Fatalf("frame%d heading snap %.3f", frame, turn)
								}
								if math.Abs(distance-math.Abs(car.Speed)/60) > .10 {
									t.Fatalf("frame%d inconsistent speed: distance%.3f speed%.1f", frame, distance, car.Speed)
								}
								if car.InRoundabout {
									entered = true
									if car.Pos.Distance(rb.Center) < rb.InnerRadius+car.Width/2 {
										t.Fatalf("frame%d cut island radius%.1f inner%.1f", frame, car.Pos.Distance(rb.Center), rb.InnerRadius)
									}
								} else if entered {
									exited = true
									afterExit++
								}
								if w.CheckBoundaries(car.Pos, car.Width/2, lw).Zone == sim.ZoneHardBarrier {
									t.Fatalf("frame%d hit barrier at%v", frame, car.Pos)
								}
								if afterExit == 45 {
									break
								}
							}
							if !entered || !exited {
								t.Fatalf("did not complete entry/exit: %v/%v pos%v", entered, exited, car.Pos)
							}
						})
					}
				}
			}
		}
	}
}
