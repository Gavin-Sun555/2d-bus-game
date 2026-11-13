package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCarsFollowBRTTaperInBothDirections(t *testing.T) {
	const laneWidth, dt = 52., 1. / 60
	for _, heading := range []float64{0, math.Pi, math.Pi / 2, -math.Pi / 2} {
		forward := sim.UnitFromAngle(heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		w := &sim.World{CenterBRT: true, Roads: []sim.RoadSegment{{
			P1: forward.Mul(-1500), P2: forward.Mul(1500), Lanes: 8,
			Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true,
		}}}
		stop := sim.NewBusStop(0, "BRT", sim.Vec2{}, heading, 8, sim.RGBA(0, 0, 0, 255), laneWidth)
		stop.CenterBRT = true
		w.Stops = []*sim.BusStop{stop}
		start := forward.Mul(-360).Add(right.Mul(3.5 * laneWidth))
		car := &sim.TrafficCar{Heading: heading, IsEW: math.Abs(forward.X) > .5, LaneIndex: 3, Pos: start, Speed: 300}
		tm := &TrafficManager{}
		outward, inward := false, false
		for tick := 0; tick < 145; tick++ {
			before, previousYaw := car.Pos, car.RoadCurveYaw
			moveCar(car, dt)
			tm.handleCarNavigation(car, laneWidth, w, before)
			want := 3.5*laneWidth + w.BRTStationFlareAt(car.Pos, laneWidth)
			if got := car.Pos.Dot(right); math.Abs(got-want) > .001 {
				t.Fatalf("heading %.2f tick %d: car offset %.3f, painted lane %.3f", heading, tick, got, want)
			}
			if math.Abs(car.RoadCurveYaw-previousYaw) > .04 {
				t.Fatalf("heading %.2f tick %d: yaw jumped from %.3f to %.3f", heading, tick, previousYaw, car.RoadCurveYaw)
			}
			if car.RoadCurveYaw > .12 && !outward {
				// A stopped car keeps its pose even if it inherits a slight lateral error.
				car.Pos = car.Pos.Add(right.Mul(2))
				stopped := car.Pos
				tm.handleCarNavigation(car, laneWidth, w, stopped)
				if car.Pos != stopped || car.RoadCurveYaw <= .12 {
					t.Fatal("stopped car slid or snapped straight in the taper")
				}
				car.Pos = car.Pos.Sub(right.Mul(2))
				outward = true
			}
			inward = inward || car.RoadCurveYaw < -.12
		}
		if !outward || !inward || car.RoadCurveYaw != 0 || car.Heading != heading {
			t.Fatalf("heading %.2f: expected both curves and a straight exit", heading)
		}
	}
}
