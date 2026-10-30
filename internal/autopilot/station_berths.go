package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Two front-first positions, independent of vehicle model. The rear position
// follows the lead's actual tail and may overhang the painted platform.
func availableStationBerths(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) []sim.Vec2 {
	if !stationBoardingCapacity(bus, stop, obstacles) {
		return nil
	}
	var storage [2]sim.Vec2
	var queue sim.Vec2
	var occupied bool
	if stop.HasTwoBerths() {
		queue, occupied = stationQueueTarget(bus, stop, obstacles)
	}
	return availableStationBerthsInto(bus, stop, obstacles, storage[:0], queue, occupied)
}

// The caller owns storage. Capacity and queue geometry may already have been
// established by an earlier branch of the same control decision.
func availableStationBerthsInto(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle, storage []sim.Vec2, queue sim.Vec2, occupied bool) []sim.Vec2 {
	targets := append(storage[:0], stop.FrontBerthPosition())
	if stop.HasTwoBerths() {
		rear := stop.RearBerthPosition()
		if occupied {
			rear = queue
		}
		candidate := *bus
		candidate.Pos, candidate.Heading = rear, stop.Heading
		candidate.ResetArticulation()
		if stop.ContainsBus(&candidate) {
			targets = append(targets, rear)
		}
	}
	clear := targets[:0]
	for _, target := range targets {
		candidate := *bus
		candidate.Pos, candidate.Heading = target, stop.Heading
		candidate.ResetArticulation()
		free := true
		for _, ob := range obstacles {
			reach := bus.OverallLength() + ob.Length/2 + 16
			offset := ob.Pos.Sub(target)
			if offset.Dot(offset) > reach*reach {
				continue
			}
			for _, body := range predictedObstacleBodies(ob, 0) {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen += 8
				body.HalfWid += 2
				if hit, _, _ := candidate.CollidesBody(body); hit {
					free = false
					break
				}
			}
			if !free {
				break
			}
		}
		if free {
			clear = append(clear, target)
		}
	}
	return clear
}

// Queued followers do not reserve their leader's free berth. Two buses ahead
// (or already serving alongside this stop) do prevent a third admission.
func stationBoardingCapacity(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) bool {
	if !stop.HasTwoBerths() {
		return true
	}
	u := sim.UnitFromAngle(stop.Heading)
	count := 0
	for _, ob := range obstacles {
		if ob.Length < 65 || math.Cos(ob.Heading-stop.Heading) < .8 {
			continue
		}
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		inBay := stop.OccupiesBerth(body)
		serving := ob.Bus != nil && ob.Bus.DoorsOpen && math.Abs(ob.Speed) < 3 && (stop.ContainsBus(ob.Bus) || (ob.Bus.PassengerService.Active && ob.Bus.PassengerService.StopID == stop.ID))
		if !inBay && !serving {
			continue
		}
		if !serving && (stationFollowerBehind(bus, ob, stop.Heading) || stationQueuedNoseClearsBerth(bus, ob, stop.Heading)) {
			continue
		}
		if !serving && ob.Pos.Sub(bus.Pos).Dot(u) < 0 {
			continue
		}
		count++
		if count >= 2 {
			return false
		}
	}
	return true
}

func atSelectedStationBerth(bus *sim.Bus, target sim.Vec2, heading float64) bool {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	d := bus.Pos.Sub(target)
	return math.Abs(d.Dot(u)) < 10 && math.Abs(d.Dot(r)) < 10 && math.Abs(sim.NormalizeAngle(bus.Heading-heading)) < .2
}

func stationBoardingPoseClear(f autopilotFrame) bool {
	if f.world != nil && f.bus.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
		return false
	}
	for _, ob := range f.obstacles {
		if ob.Bus == nil && ob.Length < 65 && math.Cos(ob.Heading-f.targetStop.Heading) > .9 {
			u := sim.UnitFromAngle(f.targetStop.Heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			d := ob.Pos.Sub(f.bus.Pos)
			if d.Dot(u) > 0 && math.Abs(d.Dot(r)) < (f.bus.Width+ob.Width)/2+8 && d.Dot(u)-(f.bus.Length+ob.Length)/2 < 35 {
				return false
			}
		}
		if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, .5, .5) {
			return false
		}
	}
	return true
}

// A full passenger platform can still have safe staging space behind its
// occupants. Reserve behind actual body envelopes, including articulated tails,
// and stay on the bay axis instead of holding in the adjacent traffic lane.
func stationQueueTarget(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) (sim.Vec2, bool) {
	if !stop.HasTwoBerths() {
		return sim.Vec2{}, false
	}
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	along := stop.FrontBerthPosition().Sub(stop.Pos).Dot(u)
	found := false
	for _, ob := range obstacles {
		if ob.Length < 65 || math.Cos(ob.Heading-stop.Heading) < .9 {
			continue
		}
		d := ob.Pos.Sub(stop.Pos)
		// Include the entire aligned approach queue, but not buses in traffic lanes.
		if math.Abs(d.Dot(r)) > stop.Width/2 || d.Dot(u) < -1000 || d.Dot(u) > stop.Length/2+ob.Length/2 {
			continue
		}
		yaw := ob.Heading - stop.Heading
		extent := math.Abs(math.Cos(yaw))*ob.Length/2 + math.Abs(math.Sin(yaw))*ob.Width/2
		if d.Dot(u)+extent <= bus.Pos.Sub(stop.Pos).Dot(u)+bus.Length/2-1 {
			continue
		}
		rear := d.Dot(u) - extent
		along = math.Min(along, rear-sim.StationBerthGap-bus.Length/2)
		found = true
	}
	if !found {
		return sim.Vec2{}, false
	}
	return stop.Pos.Add(u.Mul(along)), true
}

// A follower entering the rear of a long platform must not reserve the berth
// ahead of the bus already pulling in. Require separated full envelopes; the
// target-position collision checks below still protect the actual berth.
func stationFollowerBehind(bus *sim.Bus, ob VehicleObstacle, heading float64) bool {
	if math.Cos(bus.Heading-heading) < .9 {
		return false
	}
	if math.Cos(ob.Heading-heading) < .9 {
		// A stationary bus still straightening at the rear entrance does not
		// reserve the lead's berth. Keep moving/crossing traffic conservative;
		// separated full longitudinal envelopes remain mandatory below.
		if ob.Bus == nil || math.Abs(ob.Speed) >= 3 || math.Cos(ob.Heading-heading) < .8 {
			return false
		}
	}
	forward := sim.UnitFromAngle(heading)
	ownRear, _ := stationBodyRange(bus, forward)
	otherExtent := math.Abs(math.Cos(ob.Heading-heading))*ob.Length/2 + math.Abs(math.Sin(ob.Heading-heading))*ob.Width/2
	otherFront := ob.Pos.Dot(forward) + otherExtent
	if ob.Bus != nil {
		_, otherFront = stationBodyRange(ob.Bus, forward)
	}
	return otherFront+2 <= ownRear
}

func stationQueuedNoseClearsBerth(bus *sim.Bus, ob VehicleObstacle, heading float64) bool {
	if !bus.IsArticulated() || math.Cos(bus.Heading-heading) < .9 || math.Cos(ob.Heading-heading) < .9 {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	otherFront := math.Inf(1)
	if ob.Bus != nil {
		_, otherFront = stationBodyRange(ob.Bus, forward)
	}
	// A follower's nose may be beside a yawed trailer while still well
	// behind the leading powered section. Do not let that queued nose
	// reserve the lead berth; the candidate placement and complete curve
	// still have to clear every real body below.
	if ob.Bus == nil || ob.Bus.DoorsOpen || math.Abs(ob.Speed) >= 3 {
		return false
	}
	powered := bus.BodyParts()[0]
	yaw := powered.Heading - heading
	poweredRear := powered.Center.Dot(forward) - powered.HalfLen*math.Abs(math.Cos(yaw)) - powered.HalfWid*math.Abs(math.Sin(yaw))
	if otherFront+12 > poweredRear {
		return false
	}
	for _, body := range ob.Bus.BodyParts() {
		if body.HalfLen > 0 {
			body.HalfLen += .5
			body.HalfWid += .5
			if hit, _, _ := bus.CollidesBody(body); hit {
				return false
			}
		}
	}
	return true
}

// Project each real section directly onto the platform axis. Projecting the
// combined yawed envelope a second time invents extra rear/front reach and can
// make a separated follower reserve its leader's otherwise free boarding berth.
func stationBodyRange(bus *sim.Bus, forward sim.Vec2) (rear, front float64) {
	rear, front = math.Inf(1), math.Inf(-1)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		u := sim.UnitFromAngle(part.Heading)
		extent := math.Abs(u.Dot(forward))*part.HalfLen + math.Abs(u.Dot(right))*part.HalfWid
		center := part.Center.Dot(forward)
		rear, front = math.Min(rear, center-extent), math.Max(front, center+extent)
	}
	return rear, front
}
