package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRoundaboutOuterLaneNeverStartsPassingSlowTraffic(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, speed := range []float64{0, 20, 50, 75} {
				t.Run(fmt.Sprintf("width%.0f/angle%.2f/speed%.0f", lw, angle, speed), func(t *testing.T) {
					rb := &sim.Roundabout{Center: sim.Vec2{X: 500, Y: -700}, Radius: 300, InnerRadius: 130}
					radius := roundaboutOuterLaneRadius(rb)
					pos := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius))
					bus := sim.NewBus(pos.X, pos.Y, angle-math.Pi/2)
					bus.Speed = 60
					ahead := angle - .4
					obstacle := VehicleObstacle{Pos: rb.Center.Add(sim.Vec2{X: math.Cos(ahead), Y: math.Sin(ahead)}.Mul(radius)), Heading: ahead - math.Pi/2, Speed: speed, Length: 48, Width: 22}
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					for step := 0; step < 120; step++ {
						ad.updateRoundaboutLane(bus, rb, []VehicleObstacle{obstacle}, lw, 0, 60, 1.0/60)
					}
					if ad.LaneOffset != 0 || ad.TargetLaneOffset != 0 || ad.OvertakeState != OvertakeNone {
						t.Fatal("slow outer traffic triggered a passing maneuver")
					}
				})
			}
		}
	}
}

func TestRoundaboutInnerLaneMergesEarlyOnlyWhenSafe(t *testing.T) {
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, occupied := range []bool{false, true} {
			t.Run(fmt.Sprintf("angle%.2f/occupied%v", angle, occupied), func(t *testing.T) {
				rb := &sim.Roundabout{Radius: 320, InnerRadius: 130}
				outer := roundaboutOuterLaneRadius(rb)
				pos := sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(outer - 52)
				bus := sim.NewBus(pos.X, pos.Y, angle-math.Pi/2)
				bus.Speed = 60
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.LaneOffset = -52
				ad.TargetLaneOffset = -52
				ad.OvertakeState = OvertakePassing
				var obstacles []VehicleObstacle
				if occupied {
					obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: math.Cos(angle - .15), Y: math.Sin(angle - .15)}.Mul(outer), Heading: angle - .15 - math.Pi/2, Speed: 60, Length: 48, Width: 22}}
				}
				ad.updateRoundaboutLane(bus, rb, obstacles, 52, 0, 60, .016)
				if occupied {
					if ad.TargetLaneOffset != -52 || ad.LaneOffset != -52 {
						t.Fatal("merged into occupied outer lane")
					}
				} else if ad.TargetLaneOffset != 0 || ad.OvertakeState != OvertakeReturning {
					t.Fatal("did not use an early safe gap")
				}
			})
		}
	}
}

func TestDirectRoundaboutExitChecksCompleteSweptPath(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			const lw = 78.0
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			rb := &w.Roundabouts[0]
			in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			out := sim.Vec2{X: -in.Y, Y: in.X}
			from := sim.NewBusStop(0, "from", rb.Center.Sub(in.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw)
			to := sim.NewBusStop(1, "to", rb.Center.Add(out.Mul(800)), heading+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
			w.Stops = []*sim.BusStop{from, to}
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.PrepareLeg(w, from, to, lw)
			index := ad.roundaboutExitIndex(rb, 0)
			if index < 0 {
				t.Fatal("exit not found")
			}
			exit := ad.drivePath[index].Sub(rb.Center)
			angle := math.Atan2(exit.Y, exit.X) + .55
			radius := roundaboutOuterLaneRadius(rb) - 52
			pos := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius))
			bus := sim.NewBus(pos.X, pos.Y, angle-math.Pi/2)
			bus.Speed = 90
			oldPath := append([]sim.Vec2(nil), ad.drivePath...)
			blocked := VehicleObstacle{Pos: pos, Heading: bus.Heading, Length: 48, Width: 22}
			if ad.tryDirectRoundaboutExit(bus, rb, []VehicleObstacle{blocked}, index, lw) {
				t.Fatal("direct exit ignored a vehicle in its swept path")
			}
			if len(ad.drivePath) != len(oldPath) || ad.drivePath[0] != oldPath[0] {
				t.Fatal("rejected exit mutated the active route")
			}
			if !ad.tryDirectRoundaboutExit(bus, rb, nil, index, lw) {
				t.Fatal("clear, drivable direct exit was rejected")
			}
			if bus.Pos != pos || bus.Heading != angle-math.Pi/2 {
				t.Fatal("direct exit snapped the bus pose")
			}
			if ad.drivePath[0].Distance(pos) > .01 || ad.directRoundaboutExit == nil {
				t.Fatal("direct exit did not start continuously from the current pose")
			}
			if ad.drivePath[len(ad.drivePath)-1] != oldPath[len(oldPath)-1] {
				t.Fatal("direct exit changed the destination")
			}
		})
	}
}

func TestRoundaboutDirectExitStateClearsOnNewLegAndReset(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
	center := w.Roundabouts[0].Center
	ad.directRoundaboutExit = &center
	ad.PrepareLeg(w, w.Stops[1], w.Stops[2], 52)
	if ad.directRoundaboutExit != nil {
		t.Fatal("direct exit leaked into the next leg")
	}
	ad.directRoundaboutExit = &center
	ad.Reset()
	if ad.directRoundaboutExit != nil {
		t.Fatal("reset retained a stale direct exit")
	}
}
