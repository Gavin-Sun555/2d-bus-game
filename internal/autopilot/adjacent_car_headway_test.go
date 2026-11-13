package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestAdjacentCarWithOverlappingBodyRequiresHeadway(t *testing.T) {
	const laneWidth = 52.
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		forward := sim.UnitFromAngle(heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		world := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-2000), P2: forward.Mul(2000), Lanes: 6, Type: sim.RoadExpressway6Lane}}}
		pos := right.Mul(50.25)
		bus := sim.NewBus(pos.X, pos.Y, heading)
		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.BerserkMaxSpeed = 667
		driver.drivePath = []sim.Vec2{pos, pos.Add(forward.Mul(1000))}

		car := VehicleObstacle{Pos: pos.Add(forward.Mul(85)).Add(right.Mul(27.75)), Heading: heading, Speed: 166.7, Width: 22, Length: 46}
		frame := autopilotFrame{bus: bus, world: world, laneWidth: laneWidth, speedAbs: 168, obstacles: []VehicleObstacle{car}}
		if limit := driver.obstacleSpeed(frame, 667); limit <= 0 || limit >= car.Speed {
			t.Errorf("heading %.2f: overlapping corridor needs smooth braking below the leader speed, got %.1f", heading, limit)
		}

		frame.obstacles[0].Pos = pos.Add(forward.Mul(72)).Add(right.Mul(27.75))
		if limit := driver.obstacleSpeed(frame, 667); limit != 0 {
			t.Errorf("heading %.2f: almost touching the rear needs a stop, got %.1f", heading, limit)
		}

		frame.obstacles[0].Pos = pos.Add(forward.Mul(85)).Add(right.Mul(34))
		if limit := driver.obstacleSpeed(frame, 667); limit <= car.Speed {
			t.Errorf("heading %.2f: car with physical side clearance prevented passing, got %.1f", heading, limit)
		}
	}
}
