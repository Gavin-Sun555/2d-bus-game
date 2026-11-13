package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestFrozenInnerPassingOffsetRecentersAndClearsCar(t *testing.T) {
	const lw = 52.
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		forward := sim.UnitFromAngle(heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-6000), P2: forward.Mul(6000), Lanes: 6, Type: sim.RoadExpressway6Lane}}}
		for id, along := range []float64{-2200, 3500} {
			w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", forward.Mul(along), heading, 6, sim.RGBA(0, 0, 0, 255), lw))
		}
		sim.StandardizeStopBays(w, lw)
		route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
		pos := forward.Mul(-900).Add(right.Mul(47))
		bus := sim.NewBus(pos.X, pos.Y, heading)
		bus.SetModel(sim.BusArticulated)
		bus.Speed = 172
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		ad.BerserkMaxSpeed = 667
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		frozenOffset := 47 - 2.5*lw
		ad.holdLaneOffset(frozenOffset)
		car := VehicleObstacle{Pos: forward.Mul(-829).Add(right.Mul(1.5 * lw)), Heading: heading, Speed: 166.7, Length: 46, Width: 22}

		// A lead in the left lane would occupy the space needed for the correction.
		blocked := car
		blocked.Pos = bus.Pos.Add(forward.Mul(25)).Sub(right.Mul(21))
		ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{car, blocked}, lw, 1./60)
		if ad.TargetLaneOffset < frozenOffset-2 {
			t.Fatalf("heading %.2f: recentered into an occupied passing lane", heading)
		}

		passed := false
		for tick := 0; tick < 480; tick++ {
			car.Pos = car.Pos.Add(forward.Mul(car.Speed / 60))
			th, br, steer, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{car}, lw, 1./60)
			bus.Update(1./60, th, br, steer, hb, doors)
			body := sim.OBB{Center: car.Pos, Heading: heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
			if hit, _, _ := bus.CollidesBody(body); hit {
				t.Fatalf("heading %.2f: contacted car during pass at tick %d", heading, tick)
			}
			for _, part := range bus.BodyParts() {
				yaw := sim.NormalizeAngle(part.Heading - heading)
				if part.Center.Dot(right)-part.HalfLen*math.Abs(math.Sin(yaw))-part.HalfWid*math.Abs(math.Cos(yaw)) < -.1 {
					t.Fatalf("heading %.2f: body crossed centerline at tick %d", heading, tick)
				}
			}
			if math.Abs(bus.Pos.Dot(right)-lw/2) < 4 && bus.Pos.Sub(car.Pos).Dot(forward) > bus.OverallLength()+car.Length/2 {
				passed = true
				break
			}
		}
		if !passed {
			t.Fatalf("heading %.2f: did not center and overtake, lateral %.1f speed %.1f gap %.1f", heading, bus.Pos.Dot(right), bus.Speed, bus.Pos.Sub(car.Pos).Dot(forward))
		}
	}
}
