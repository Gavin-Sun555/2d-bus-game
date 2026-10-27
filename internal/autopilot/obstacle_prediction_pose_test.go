package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStoppedObstaclePredictionRetainsPhysicalPose(t *testing.T) {
	for _, heading := range []float64{0, .7, 1.85, math.Pi} {
		ob := VehicleObstacle{Pos: sim.Vec2{X: 30, Y: 40}, Heading: heading, Length: 46, Width: 22, JunctionCorridor: &sim.JunctionCorridor{Remaining: .25, Bodies: []sim.OBB{{Center: sim.Vec2{X: 30, Y: 40}, Heading: 0, HalfLen: 23, HalfWid: 11}, {Center: sim.Vec2{X: 40, Y: 40}, Heading: 0, HalfLen: 23, HalfWid: 11}}}}
		actual := predictedObstacleBodies(ob, 0)[0]
		if actual.Center != ob.Pos || actual.Heading != heading || actual.HalfLen != 23 || actual.HalfWid != 11 {
			t.Fatalf("stationary vehicle rotated or moved: %+v", actual)
		}
		future := predictedObstacleBodies(ob, 5)[0]
		if future.Center.Distance(sim.Vec2{X: 35, Y: 40}) > .001 {
			t.Fatal("moving prediction lost its trajectory")
		}
	}
}
