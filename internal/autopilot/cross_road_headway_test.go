package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestCrossRoadQueueDoesNotBecomeDistantFrontalLeader(t *testing.T) {
	for _, rotation := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(rotation), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(rotation) - v.Y*math.Sin(rotation), Y: v.X*math.Sin(rotation) + v.Y*math.Cos(rotation)}
			}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -4000}), P2: rotate(sim.Vec2{X: 4000}), Lanes: 8}}}
			p := rotate(sim.Vec2{X: -26, Y: 56})
			b := sim.NewBus(p.X, p.Y, math.Pi/2+rotation)
			b.SetModel(sim.BusArticulated)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			far := VehicleObstacle{Pos: rotate(sim.Vec2{X: -1300, Y: 78}), Heading: rotation, Length: 46, Width: 22}
			f := autopilotFrame{bus: b, world: w, laneWidth: 52, obstacles: []VehicleObstacle{far}}
			if speed := ad.obstacleSpeed(f, 150); speed != 150 {
				t.Fatalf("perpendicular road projected into frontal corridor: %.2f", speed)
			}
			f.obstacles = []VehicleObstacle{{Pos: p.Add(rotate(sim.Vec2{Y: 44})), Heading: b.Heading, Length: 46, Width: 22}}
			if speed := ad.obstacleSpeed(f, 150); speed != 0 {
				t.Fatalf("actual leader lost protection: %.2f", speed)
			}
		})
	}
}

func TestClearStoppedBusBesideTwoLaneCorridor(t *testing.T) {
	for _, rotation := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(rotation), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(rotation) - v.Y*math.Sin(rotation), Y: v.X*math.Sin(rotation) + v.Y*math.Cos(rotation)}
			}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -1950, Y: -4000}), P2: rotate(sim.Vec2{X: -1950, Y: 4000}), Lanes: 2}}}
			p := rotate(sim.Vec2{X: -1924, Y: -2408.287095})
			b := sim.NewBus(p.X, p.Y, -math.Pi/2+rotation)
			p = rotate(sim.Vec2{X: -1970.348896, Y: -2575.128366})
			other := sim.NewBus(p.X, p.Y, -1.565162615+rotation)
			other.SetModel(sim.BusArticulated)
			other.TrailerHeading = -1.618051551 + rotation
			ad := NewAutopilotDriver()
			ad.drivePath = []sim.Vec2{b.Pos, b.Pos.Add(rotate(sim.Vec2{Y: -800}))}
			ad.cumLen = []float64{0, 800}
			f := autopilotFrame{bus: b, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(other)}}
			if v := ad.obstacleSpeed(f, 150); v != 150 {
				t.Fatalf("clear parallel bus stopped traffic: %.2f", v)
			}
			other.Pos = b.Pos.Add(rotate(sim.Vec2{Y: -100}))
			other.Heading = b.Heading
			other.TrailerHeading = b.Heading
			f.obstacles = []VehicleObstacle{ObserveBus(other)}
			if ad.clearsStoppedParallelBus(f, f.obstacles[0]) {
				t.Fatal("real in-lane leader ignored")
			}
		})
	}
}
