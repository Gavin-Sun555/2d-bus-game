package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Drive from each left approach lane through the real controller and physics.
// Check the whole bus envelope, circulation direction, lane hold and route exit.
func TestBerserkInnerLaneRoundaboutEntry(t *testing.T) {
	approaches := []struct {
		name    string
		heading float64
	}{{"west", 0}, {"north", math.Pi / 2}, {"east", math.Pi}, {"south", -math.Pi / 2}}
	exits := []struct {
		name string
		turn float64
	}{{"straight", 0}, {"right", math.Pi / 2}, {"left", -math.Pi / 2}}
	for _, lw := range []float64{40, 42, 52, 64, 78} {
		for _, approach := range approaches {
			for _, exit := range exits {
				t.Run(fmt.Sprintf("%s/%s/lane%.0f", approach.name, exit.name, lw), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					rb := w.Roundabouts[0]
					forward := sim.Vec2{X: math.Cos(approach.heading), Y: math.Sin(approach.heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					exitHeading := approach.heading + exit.turn
					outgoing := sim.Vec2{X: math.Cos(exitHeading), Y: math.Sin(exitHeading)}
					from := sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(800)), approach.heading, 4, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", rb.Center.Add(outgoing.Mul(800)), exitHeading, 4, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					pos := rb.Center.Sub(forward.Mul(450)).Add(right.Mul(lw * 0.5))
					bus := sim.NewBus(pos.X, pos.Y, approach.heading)
					bus.Speed = 120
					driver := NewAutopilotDriver()
					driver.BerserkMode = true
					driver.LaneOffset, driver.TargetLaneOffset = -lw, -lw
					driver.OvertakeState = OvertakePassing
					entered, exited := false, false
					lastAngle := math.Atan2(pos.Y-rb.Center.Y, pos.X-rb.Center.X)
					for step := 0; step < 1800; step++ {
						throttle, brake, steer, handbrake, doors, _ := driver.ComputeControls(bus, route, w, nil, nil, lw, 1.0/60)
						if !entered && driver.TargetLaneOffset > -lw*0.3 {
							t.Fatalf("returned to outer lane before entering: step=%d pos=%v targetOffset=%.1f", step, bus.Pos, driver.TargetLaneOffset)
						}
						bus.Update(1.0/60, throttle, brake, steer, handbrake, doors)
						radius := bus.Pos.Distance(rb.Center)
						angle := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
						toCenter := rb.Center.Sub(bus.Pos)
						along := math.Abs(toCenter.X*math.Cos(bus.Heading) + toCenter.Y*math.Sin(bus.Heading))
						across := math.Abs(-toCenter.X*math.Sin(bus.Heading) + toCenter.Y*math.Cos(bus.Heading))
						clearance := math.Hypot(math.Max(0, along-bus.Length/2), math.Max(0, across-bus.Width/2))
						if clearance < rb.InnerRadius+8 {
							t.Fatalf("body clips island: step=%d clearance=%.1f pos=%v heading=%.2f", step, clearance, bus.Pos, bus.Heading)
						}
						if radius < rb.Radius-10 {
							entered = true
							if delta := sim.NormalizeAngle(angle - lastAngle); delta > 0.002 {
								t.Fatalf("clockwise circulation: step=%d delta=%f pos=%v", step, delta, bus.Pos)
							}
						}
						lastAngle = angle
						if entered && radius > rb.Radius+110 && bus.Pos.Sub(rb.Center).Dot(outgoing) < rb.Radius*0.9 {
							t.Fatalf("left by wrong exit: step=%d pos=%v", step, bus.Pos)
						}
						if entered && bus.Pos.Sub(rb.Center).Dot(outgoing) > rb.Radius+150 {
							exited = true
							break
						}
					}
					if !entered || !exited {
						t.Fatalf("failed to complete chosen exit: entered=%v exited=%v pos=%v", entered, exited, bus.Pos)
					}
				})
			}
		}
	}
}
