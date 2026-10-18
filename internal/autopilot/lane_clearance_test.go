package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLeftLaneMergeIgnoresClearThroughQueue(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(heading) - v.Y*math.Sin(heading), Y: v.X*math.Sin(heading) + v.Y*math.Cos(heading)}
			}
			w := sim.NewWorld(52, sim.MapThemeMetropolis)
			w.Roundabouts = nil
			w.Roads = []sim.RoadSegment{{P1: sim.Vec2{X: -1000}, P2: sim.Vec2{X: 1000}, Lanes: 6, Type: sim.RoadExpressway6Lane}}
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.drivePath = []sim.Vec2{{X: -600, Y: 130}, {X: -450, Y: 78}, {X: -300, Y: 26}, {X: 100, Y: 26}, {X: 100, Y: -400}}
			for i := range w.Roads {
				w.Roads[i].P1 = rotate(w.Roads[i].P1)
				w.Roads[i].P2 = rotate(w.Roads[i].P2)
			}
			for i := range ad.drivePath {
				ad.drivePath[i] = rotate(ad.drivePath[i])
			}
			ad.cumLen = make([]float64, len(ad.drivePath))
			for i := 1; i < len(ad.drivePath); i++ {
				ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
			}
			bus := sim.NewBus(-375, 52, -math.Atan2(52, 150))
			bus.Pos = rotate(bus.Pos)
			bus.Heading += heading
			s, _ := ad.trackPath(bus.Pos)
			queue := VehicleObstacle{Pos: sim.Vec2{X: -285, Y: 78}, Heading: 0, Length: 48, Width: 24}
			queue.Pos = rotate(queue.Pos)
			queue.Heading = heading
			f := autopilotFrame{bus: bus, world: w, laneWidth: 52, s: s, obstacles: []VehicleObstacle{queue}}
			if got := ad.obstacleSpeed(f, 100); got != 100 {
				t.Fatalf("clear left merge blocked by through queue: targetSpeed=%.1f", got)
			}
			// A car occupying the actual merge must still make the bus stop.
			queue.Pos = bus.Pos.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(65))
			f.obstacles = []VehicleObstacle{queue}
			if got := ad.obstacleSpeed(f, 100); got != 0 {
				t.Fatalf("failed to stop for car in merge: targetSpeed=%.1f", got)
			}
		})
	}
}

func TestLeftLaneMergePassesQueueAndStillStopsForRed(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	w.Roundabouts = nil
	w.Roads = []sim.RoadSegment{
		{P1: sim.Vec2{X: -1000}, P2: sim.Vec2{X: 1000}, Lanes: 6, Type: sim.RoadExpressway6Lane},
		{P1: sim.Vec2{X: 100, Y: -1000}, P2: sim.Vec2{X: 100, Y: 1000}, Lanes: 6, Type: sim.RoadExpressway6Lane},
	}
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "from", sim.Vec2{X: -800}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "to", sim.Vec2{X: 100, Y: -800}, -math.Pi/2, 6, sim.RGBA(0, 0, 0, 255), lw),
	}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	inter := &sim.TrafficIntersection{Center: sim.Vec2{X: 100}, IsMajor: true, ApproachLanes: [4]int{6, 6, 6, 6}, Phase: sim.PhaseEWGreen}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.drivePath = []sim.Vec2{{X: -600, Y: 130}, {X: -450, Y: 78}, {X: -300, Y: 26}, {X: 126, Y: 26}, {X: 126, Y: -800}}
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	bus := sim.NewBus(-375, 52, -math.Atan2(52, 150))
	queue := VehicleObstacle{Pos: sim.Vec2{X: -285, Y: 78}, Heading: 0, Length: 48, Width: 24}
	for step := 0; step < 600; step++ {
		throttle, brake, steer, handbrake, doors, _ := ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{queue}, lw, 1.0/60)
		bus.Update(1.0/60, throttle, brake, steer, handbrake, doors)
		body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
		other := sim.OBB{Center: queue.Pos, Heading: queue.Heading, HalfLen: queue.Length / 2, HalfWid: queue.Width / 2}
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			t.Fatalf("hit through queue: step=%d pos=%v", step, bus.Pos)
		}
		if violated, notice := tls.CheckPlayerViolation(bus, lw); violated {
			t.Fatalf("ran red: step=%d %s", step, notice)
		}
	}
	if bus.Pos.X < queue.Pos.X+bus.Length/2 {
		t.Fatalf("failed to pass queue: pos=%v speed=%.1f target=%.1f", bus.Pos, bus.Speed, ad.LastTargetSpeed)
	}
	if bus.Speed > 1 {
		t.Fatalf("failed to stop for left red: pos=%v speed=%.1f", bus.Pos, bus.Speed)
	}
}
