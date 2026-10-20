package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkEmptyRoundaboutKeepsOuterApproachLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, turn := range []float64{-math.Pi / 2, 0, math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/turn%.2f", lw, heading, turn), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					rb := w.Roundabouts[0]
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					out := sim.Vec2{X: math.Cos(heading + turn), Y: math.Sin(heading + turn)}
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(900)), heading, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", rb.Center.Add(out.Mul(900)), heading+turn, 4, sim.RGBA(0, 0, 0, 255), lw)}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					pos := rb.Center.Sub(forward.Mul(550)).Add(right.Mul(1.5 * lw))
					bus := sim.NewBus(pos.X, pos.Y, heading)
					bus.Speed = 90
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					entered := false
					for step := 0; step < 2400; step++ {
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1.0/60)
						bus.Update(1.0/60, th, br, st, hb, doors)
						if math.Abs(ad.LaneOffset) > .01 || math.Abs(ad.TargetLaneOffset) > .01 {
							t.Fatalf("empty approach changed lanes at step%d: %.2f/%.2f pos%v", step, ad.LaneOffset, ad.TargetLaneOffset, bus.Pos)
						}
						radius := bus.Pos.Distance(rb.Center)
						if radius <= rb.Radius+15 {
							entered = true
							if radius < (rb.InnerRadius+16+rb.Radius)/2 {
								t.Fatalf("moved into inner ring: radius%.1f", radius)
							}
						}
						if entered && bus.Pos.Sub(rb.Center).Dot(out) > rb.Radius+220 {
							return
						}
					}
					t.Fatalf("failed to pass roundabout pos%v entered%v", bus.Pos, entered)
				})
			}
		}
	}
}

func TestBerserkRoundaboutApproachChangesLaneOnlyForQueue(t *testing.T) {
	for _, test := range []struct {
		name                 string
		queued, innerBlocked bool
		wantChange           bool
	}{
		{name: "empty"}, {name: "outer queue", queued: true, wantChange: true}, {name: "both lanes blocked", queued: true, innerBlocked: true},
	} {
		t.Run(test.name, func(t *testing.T) {
			const lw = 52.0
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			rb := w.Roundabouts[0]
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", rb.Center.Add(sim.Vec2{X: -900}), 0, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", rb.Center.Add(sim.Vec2{X: 900}), 0, 4, sim.RGBA(0, 0, 0, 255), lw)}
			bus := sim.NewBus(rb.Center.X-430, rb.Center.Y+1.5*lw, 0)
			bus.Speed = 60
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			var obstacles []VehicleObstacle
			if test.queued {
				obstacles = append(obstacles, VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{X: 110}), Length: 48, Width: 22})
			}
			if test.innerBlocked {
				obstacles = append(obstacles, VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{Y: -lw}), Length: 48, Width: 22})
			}
			ad.ComputeControls(bus, &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}, w, nil, obstacles, lw, 1.0/60)
			if changed := ad.TargetLaneOffset < -lw*.3; changed != test.wantChange {
				t.Fatalf("change%v want%v offset%.1f", changed, test.wantChange, ad.TargetLaneOffset)
			}
		})
	}
}
