package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) stationDepartureRoad(f autopilotFrame) (*sim.BusStop, sim.Vec2, sim.Vec2, bool) {
	if f.world == nil || ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) || ad.State == AutopilotStateDocked {
		return nil, sim.Vec2{}, sim.Vec2{}, false
	}
	stop := f.world.Stops[ad.pathFromID]
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	relative := f.bus.Pos.Sub(stop.RoadCenter)
	curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
	if relative.Dot(forward) < -50 || relative.Dot(forward) > 350 || relative.Dot(right) < curb+3 || math.Cos(f.bus.Heading-stop.Heading) < .8 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return nil, forward, right, false
	}
	return stop, forward, right, true
}

// While the entire body still clears the road, give traffic in the target lane
// time to pass before starting the merge. Waiting here leaves that lane open.
func (ad *AutopilotDriver) stationDepartureTrafficSpeed(f autopilotFrame, target float64) float64 {
	if len(f.obstacles) == 0 {
		return target
	}
	stop, forward, right, active := ad.stationDepartureRoad(f)
	if !active {
		return target
	}
	curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
	yaw := f.bus.Heading - stop.Heading
	extent := f.bus.Length/2*math.Abs(math.Sin(yaw)) + f.bus.Width/2*math.Abs(math.Cos(yaw))
	for _, ob := range f.obstacles {
		if math.Cos(ob.Heading-stop.Heading) < .8 {
			continue
		}
		lane := ob.Pos.Sub(stop.RoadCenter).Dot(right)
		if math.Abs(lane-curb) > f.laneWidth*.4+ob.Width/2 {
			continue
		}
		clearance := f.bus.Pos.Sub(ob.Pos).Dot(right) - extent - ob.Width/2
		if clearance <= 2 {
			continue
		} // Already occupying the lane: finish through a safe gap.
		along := ob.Pos.Sub(f.bus.Pos).Dot(forward)
		behind := math.Max(180, math.Max(0, ob.Speed)*2.2+80)
		ahead := math.Max(120, (f.bus.Length+ob.Length)/2+40)
		if along >= -behind && along < ahead {
			return 0
		}
	}
	return target
}

// If a car has already yielded behind a partly merged bus, the bus may finish
// only if its complete remaining swept body clears that stationary car.
func (ad *AutopilotDriver) clearsYieldingCarDuringDeparture(f autopilotFrame, ob VehicleObstacle) bool {
	if math.Abs(ob.Speed) >= 3 || ob.Length > 65 || math.Cos(ob.Heading-f.bus.Heading) < .8 || len(ad.drivePath) < 2 {
		return false
	}
	_, forward, _, active := ad.stationDepartureRoad(f)
	if !active || ob.Pos.Sub(f.bus.Pos).Dot(forward) > f.bus.Length/2 {
		return false
	}
	offset := f.bus.Pos.Sub(ad.basePathPosition(f.bus.Pos, f.world, f.bus.Heading, f.laneWidth))
	error := f.bus.Pos.Sub(ad.pointAt(f.s).Add(offset))
	pointAt := func(distance float64) sim.Vec2 {
		return ad.pointAt(f.s + distance).Add(offset).Add(error.Mul(math.Max(0, 1-distance/80)))
	}
	other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 2, HalfWid: ob.Width/2 + 2}
	body := sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length/2 + 2, HalfWid: f.bus.Width/2 + 2}
	if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
		return false
	}
	for distance := 5.0; distance <= 220; distance += 5 {
		position := pointAt(distance)
		tangent := pointAt(distance + 2).Sub(position).Normalize()
		if tangent.Dot(forward) < .8 {
			return false
		}
		body.Center = position
		body.Heading = math.Atan2(tangent.Y, tangent.X)
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			return false
		}
	}
	return true
}
