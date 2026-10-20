package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkBayDepartureDoesNotCutAcrossMedianToPass(t *testing.T) {
	for _, lw := range []float64{42, 52, 64, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, ahead := range []float64{150, 250, 350} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/car%.0f", lw, heading, ahead), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(3000), Lanes: 4}}}
					for i, x := range []float64{0, 1800} {
						w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), heading, 4, sim.RGBA(0, 0, 0, 255), lw))
					}
					bus := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, heading)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.BerserkMaxSpeed = 667
					ad.State = AutopilotStateDeparting
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					car := VehicleObstacle{Pos: u.Mul(ahead).Add(right.Mul(lw * 1.5)), Heading: heading, Speed: 35, Length: 48, Width: 24}
					for tick := 0; tick < 1200; tick++ {
						car.Pos = car.Pos.Add(u.Mul(car.Speed / 60))
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{car}, lw, 1.0/60)
						bus.Update(1.0/60, th, br, st, hb, doors)
						yaw := sim.NormalizeAngle(bus.Heading - heading)
						clearance := bus.Pos.Dot(right) - math.Abs(math.Sin(yaw))*bus.Length/2 - math.Abs(math.Cos(yaw))*bus.Width/2
						if clearance < 0 {
							t.Fatalf("departure crossed median tick%d p%v yaw%.2f v%.1f clearance%.2f offset%.1f", tick, bus.Pos, yaw, bus.Speed, clearance, ad.LaneOffset)
						}
						if bus.Pos.Dot(u) > 700 {
							return
						}
					}
					t.Fatalf("departure did not resume safely: pos%v speed%.1f", bus.Pos, bus.Speed)
				})
			}
		}
	}
}
