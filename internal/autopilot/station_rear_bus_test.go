package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestDockIntoFreeBayAheadOfStoppedThroughBus(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, gap := range []float64{100, 105, 110, 120} {
					t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f/gap%.0f", lw, lanes, h, gap), func(t *testing.T) {
						forward := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-1500), P2: forward.Mul(1500), Lanes: lanes}}}
						for i, x := range []float64{-900, 0, 900} {
							w.Stops = append(w.Stops, sim.NewBusStop(i, "test", forward.Mul(x), h, lanes, sim.RGBA(0, 0, 0, 255), lw, 1))
						}
						stop := w.Stops[1]
						pos := stop.Pos.Sub(forward.Mul(90)).Sub(right.Mul(20))
						bus := sim.NewBus(pos.X, pos.Y, h+.24)
						obs := VehicleObstacle{Pos: forward.Mul(-90 - gap).Add(right.Mul((float64(lanes)/2 - .5) * lw)), Heading: h, Length: 96, Width: 36}
						route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						for tick := 0; tick < 1200; tick++ {
							th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{obs}, lw, 1./60)
							bus.Update(1./60, th, br, st, hb, doors)
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: obs.Pos, Heading: h, HalfLen: 48, HalfWid: 18}); hit {
								t.Fatalf("collision at%d", tick)
							}
							if dock {
								if ad.stationDock == nil || ad.stationDock.rank != 0 {
									t.Fatal("should use empty bay")
								}
								return
							}
						}
						t.Fatalf("stalled pos%v heading%.3f target%.2f plan%+v", bus.Pos, bus.Heading, ad.LastTargetSpeed, ad.stationDock)
					})
				}
			}
		}
	}
}
func TestStationDockCloseRearStillChecksPhysicalSweptBody(t *testing.T) {
	bus := sim.NewBus(0, 0, 0)
	rear := VehicleObstacle{Pos: sim.Vec2{X: -100}, Heading: 0, Length: 96, Width: 36}
	if !stationDockCurveClear(bus, []sim.Vec2{{}, {X: 30}, {X: 90}}, []VehicleObstacle{rear}) {
		t.Fatal("stationary safe rear blocked forward departure")
	}
	for _, ob := range []VehicleObstacle{
		{Pos: sim.Vec2{X: -90}, Length: 96, Width: 36},
		{Pos: sim.Vec2{X: -100}, Speed: 100, Length: 96, Width: 36},
		{Pos: sim.Vec2{X: 50}, Length: 96, Width: 36},
	} {
		if stationDockCurveClear(bus, []sim.Vec2{{}, {X: 30}, {X: 90}}, []VehicleObstacle{ob}) {
			t.Fatalf("ignored unsafe body %+v", ob)
		}
	}
	if stationDockCurveClear(bus, []sim.Vec2{{}, {X: -20}, {X: 90}}, []VehicleObstacle{rear}) {
		t.Fatal("tail swept into follower")
	}
}
