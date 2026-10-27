package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Front-first, longitudinal platform positions. A long bus reserves the whole
// platform; cars in the roadway do not consume a passenger berth.
func availableStationBerths(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) []sim.Vec2 {
	targets := []sim.Vec2{stop.FrontBerthPosition()}
	if stop.HasTwoBerths() && !bus.IsArticulated() {
		targets = append(targets, stop.RearBerthPosition())
	}
	for _, ob := range obstacles {
		if ob.Length < 65 {
			continue
		}
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		if !stop.OccupiesBerth(body) {
			continue
		}
		long := ob.Length > 150 || (ob.Bus != nil && ob.Bus.IsArticulated())
		if stop.HasTwoBerths() && (bus.IsArticulated() || long) && !stationFollowerBehind(bus, ob, stop.Heading) {
			return nil
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
			body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 8, HalfWid: ob.Width/2 + 2}
			if hit, _, _ := candidate.CollidesBody(body); hit {
				free = false
				break
			}
		}
		if free {
			clear = append(clear, target)
		}
	}
	return clear
}

func atSelectedStationBerth(bus *sim.Bus, target sim.Vec2, heading float64) bool {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	d := bus.Pos.Sub(target)
	return math.Abs(d.Dot(u)) < 10 && math.Abs(d.Dot(r)) < 10 && math.Abs(sim.NormalizeAngle(bus.Heading-heading)) < .15
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
	if math.Cos(bus.Heading-heading) < .9 || math.Cos(ob.Heading-heading) < .9 {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	own := bus.BodyEnvelope()
	ownExtent := math.Abs(math.Cos(own.Heading-heading))*own.HalfLen + math.Abs(math.Sin(own.Heading-heading))*own.HalfWid
	otherExtent := math.Abs(math.Cos(ob.Heading-heading))*ob.Length/2 + math.Abs(math.Sin(ob.Heading-heading))*ob.Width/2
	return ob.Pos.Dot(forward)+otherExtent+2 <= own.Center.Dot(forward)-ownExtent
}
