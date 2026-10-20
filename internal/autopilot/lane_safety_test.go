package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBerserkFourLaneOvertakeBodyStaysOnOwnSide(t *testing.T) {
	for _, speed := range []float64{445, 667} {
		for _, dt := range []float64{.016, .04} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("speed%.0f/dt%.3f/h%.2f", speed, dt, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					right := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 4}}}
					from := sim.NewBusStop(0, "from", u.Mul(-3000), h, 4, sim.RGBA(0, 0, 0, 255), 52, 0)
					to := sim.NewBusStop(1, "to", u.Mul(3000), h, 4, sim.RGBA(0, 0, 0, 255), 52, 0)
					w.Stops = []*sim.BusStop{from, to}
					p := u.Mul(-1200).Add(right.Mul(78))
					bus := sim.NewBus(p.X, p.Y, h)
					bus.Speed = speed
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.BerserkMaxSpeed = speed
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					obs := VehicleObstacle{Pos: u.Mul(-850).Add(right.Mul(78)), Heading: h, Speed: 35, Width: 22, Length: 48}
					passed := false
					for step := 0; step < int(12/dt); step++ {
						obs.Pos = obs.Pos.Add(u.Mul(obs.Speed * dt))
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{obs}, 52, dt)
						bus.Update(dt, th, br, st, hb, doors)
						yaw := sim.NormalizeAngle(bus.Heading - h)
						clearance := bus.Pos.Dot(right) - math.Abs(math.Sin(yaw))*bus.Length/2 - math.Abs(math.Cos(yaw))*bus.Width/2
						if clearance < 0 {
							t.Fatalf("step%d body crossed centerline: clearance%.2f pos%v heading%.3f offset%.1f target%.1f", step, clearance, bus.Pos, yaw, ad.LaneOffset, ad.TargetLaneOffset)
						}
						if bus.Pos.Sub(obs.Pos).Dot(u) > 120 {
							passed = true
							break
						}
					}
					if !passed {
						t.Fatal("bus must actually finish the overtake")
					}
				})
			}
		}
	}
}
