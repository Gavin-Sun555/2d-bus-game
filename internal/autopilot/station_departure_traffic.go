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
	forward := sim.UnitFromAngle(stop.Heading)
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
	var departure departureSweepCache
	if f.departureCache == nil {
		f.departureCache = &departure
	}
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
			if ad.clearsStationYieldNeighbor(f, ob) || ad.clearsStoppedDepartureFollower(f, ob) || ad.clearsYieldingCarDuringDeparture(f, ob) {
				continue
			}
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
	if departureBodyCollision(f, 0, 0, body, other) {
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
		if departureBodyCollision(f, 0, int(distance/5), body, other) {
			return false
		}
	}
	return true
}

// A stopped bus behind or beside the rear of the bay cannot "pass first".
// Permit departure only when the full pull-out sweep clears its complete body.
// Moving traffic and vehicles ahead keep their normal priority.
func (ad *AutopilotDriver) clearsStoppedDepartureFollower(f autopilotFrame, ob VehicleObstacle) bool {
	if math.Abs(ob.Speed) >= 3 || len(ad.drivePath) < 2 {
		return false
	}
	_, forward, _, active := ad.stationDepartureRoad(f)
	if !active {
		return false
	}
	yaw := ob.Heading - math.Atan2(forward.Y, forward.X)
	nose := ob.Pos.Sub(f.bus.Pos).Dot(forward) + ob.Length/2*math.Abs(math.Cos(yaw)) + ob.Width/2*math.Abs(math.Sin(yaw))
	if nose >= -f.bus.Length/2-6 {
		// A stationary outer bus can be alongside our rear, rather than
		// wholly behind it. Do not make it advance to release us: allow the
		// full departure sweep below to prove a separated forward exit.
		if nose > f.bus.Length/2 || roadSideClearance(f.bus, ob) <= 2 {
			return false
		}
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		u := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		return p.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(ad.LaneOffset)), math.Atan2(u.Y, u.X)
	}
	base, heading := pointAt(0)
	errorPos, errorHeading := f.bus.Pos.Sub(base), sim.NormalizeAngle(f.bus.Heading-heading)
	other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 2, HalfWid: ob.Width/2 + 2}
	for distance := 0.; distance <= 240; distance += 4 {
		pos, heading := pointAt(distance)
		body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: f.bus.Length/2 + 2, HalfWid: f.bus.Width/2 + 2}
		if departureBodyCollision(f, 1, int(distance/4), body, other) {
			return false
		}
	}
	return true
}
