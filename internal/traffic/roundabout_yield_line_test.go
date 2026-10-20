package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRoundaboutYieldPlaneIncludesWholeNoseAcrossAllLanes(t *testing.T) {
	for _, radius := range []float64{240, 340, 480} {
		for _, lw := range []float64{42, 52, 78} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for lane := 0; lane < 4; lane++ {
					for _, yaw := range []float64{-.2, 0, .2} {
						rb := sim.Roundabout{Center: sim.Vec2{X: 100, Y: -250}, Radius: radius}
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						car := &sim.TrafficCar{Length: 58, Width: 26, Heading: heading + yaw, IsEW: math.Abs(forward.X) > .5}
						front := car.Length/2*math.Abs(math.Cos(yaw)) + car.Width/2*math.Abs(math.Sin(yaw))
						car.Pos = rb.Center.Sub(forward.Mul(rb.YieldLineDistance() + front + 12)).Add(right.Mul((float64(lane) + .5) * lw))
						if gap := roundaboutYieldGap(car, rb); math.Abs(gap-12) > 1e-6 {
							t.Fatalf("radius%.0f/lane%d/h%.2f/yaw%.2f: gap%.2f", radius, lane, heading, yaw, gap)
						}
					}
				}
			}
		}
	}
}
