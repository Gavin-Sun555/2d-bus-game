package traffic

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPlayerObstacleSnapshotWithoutAIBuses(t *testing.T) {
	for _, count := range []int{1, 40} {
		t.Run(fmt.Sprintf("cars%d", count), func(t *testing.T) {
			world := &sim.World{}
			tm := &TrafficManager{}
			for i := 0; i < count; i++ {
				tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: 100 + float64(i)*20}, Heading: 0, Speed: 200, Length: 46, Width: 22})
			}
			mgr := NewAIBusManager()
			mgr.Update(1./60, nil, world, nil, tm, 52, 4)
			obs := mgr.GetLocalObstacles(sim.Vec2{}, 667, nil)
			if len(obs) != count {
				t.Fatalf("player sees %d/%d cars with AI fleet disabled", len(obs), count)
			}
			// Cache and spatial buckets must keep following cars after the last AI
			// bus is removed, and after traffic moves into a different query region.
			for _, c := range tm.Cars {
				c.Pos.X += 6000
				c.Speed = 80
			}
			mgr.Update(1./60, nil, world, nil, tm, 52, 4)
			obs = mgr.GetLocalObstacles(sim.Vec2{X: 6000}, 667, obs)
			if len(obs) != count {
				t.Fatalf("moved traffic missing from player snapshot: %d/%d", len(obs), count)
			}
			for _, o := range obs {
				if o.Pos.X < 6000 || o.Speed != 80 {
					t.Fatal("player still sees stale car position/speed", o)
				}
			}
			tm.Cars = nil
			mgr.Update(1./60, nil, world, nil, tm, 52, 4)
			if obs = mgr.GetLocalObstacles(sim.Vec2{X: 6000}, 667, obs); len(obs) != 0 {
				t.Fatal("removed traffic remains in player snapshot")
			}
		})
	}
}
