package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestDepartureRearSideCarDoesNotVetoForwardLaneEntry(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			t.Run(fmt.Sprintf("h%.2f/model%d", h, model), func(t *testing.T) {
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				stop := sim.NewBusStop(0, "from", sim.Vec2{}, h, 6, sim.RGBA(0, 0, 0, 255), 52)
				w := &sim.World{Stops: []*sim.BusStop{stop}, Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: 6}}}
				pos := u.Mul(450).Add(r.Mul(94))
				b := sim.NewBus(pos.X, pos.Y, h-.536)
				b.SetModel(model)
				b.SteeringAngle = b.MaxSteerAngle
				ad := NewAutopilotDriver()
				ad.pathFromID = 0
				car := VehicleObstacle{Pos: pos.Sub(u.Mul(58)).Sub(r.Mul(16)), Heading: h, Length: 46, Width: 22}
				f := autopilotFrame{bus: b, world: w, laneWidth: 52, obstacles: []VehicleObstacle{car}}
				if !ad.isDepartureRearCar(f, car) {
					t.Fatal("stopped rear-side car still classified as leader")
				}
				if v := ad.controlSweptSpeed(f, 1); !math.IsInf(v, 1) {
					t.Fatalf("rear car vetoed forward controls: %v", v)
				}
				for _, tc := range []string{"ahead", "same corridor", "moving", "oncoming", "bus", "doors", "reverse", "junction", "narrow road", "far from departure", "parallel street"} {
					t.Run(tc, func(t *testing.T) {
						own := *b
						one := f
						one.bus = &own
						other := car
						world := *w
						one.world = &world
						switch tc {
						case "ahead":
							other.Pos = pos.Add(u.Mul(30)).Sub(r.Mul(16))
						case "same corridor":
							other.Pos = pos.Sub(sim.UnitFromAngle(b.Heading).Mul(58))
						case "moving":
							other.Speed = 90
						case "oncoming":
							other.Heading = h + math.Pi
						case "bus":
							other.Bus = sim.NewBus(0, 0, h)
						case "doors":
							own.DoorsOpen = true
						case "reverse":
							own.Speed = -5
						case "junction":
							world.Intersections = []sim.Vec2{pos}
						case "narrow road":
							copyStop := *stop
							copyStop.RoadLanes = 2
							world.Stops = []*sim.BusStop{&copyStop}
						case "parallel street":
							own.Pos = own.Pos.Add(r.Mul(1800))
							other.Pos = other.Pos.Add(r.Mul(1800))
						case "far from departure":
							own.Pos = own.Pos.Add(u.Mul(1000))
							other.Pos = other.Pos.Add(u.Mul(1000))
						}
						if ad.isDepartureRearCar(one, other) {
							t.Fatal("rear-side exception escaped its scope")
						}
					})
				}
			})
		}
	}
}
