package sim

import "math"

// PlaceBusInStationQueue reserves one longitudinal queue across all services.
// An adjacent bus using the bay's merge lane also reserves that station slot.
// Existing vehicles are never moved to make room for a new dispatch.
func PlaceBusInStationQueue(bus *Bus, stop *BusStop, others []*Bus, laneWidth float64) bool {
	forward := UnitFromAngle(stop.Heading)
	right := Vec2{X: -forward.Y, Y: forward.X}
	along := stop.FrontBerthPosition().Sub(stop.Pos).Dot(forward)
	for attempt := 0; attempt <= len(others); attempt++ {
		moved := false
		for _, other := range others {
			if other == nil || other == bus {
				continue
			}
			delta := other.Pos.Sub(stop.Pos)
			yaw := other.Heading - stop.Heading
			envelope := other.BodyEnvelope()
			delta = envelope.Center.Sub(stop.Pos)
			extent := math.Abs(math.Cos(yaw))*envelope.HalfLen + math.Abs(math.Sin(yaw))*envelope.HalfWid
			lateralExtent := math.Abs(math.Sin(yaw))*envelope.HalfLen + math.Abs(math.Cos(yaw))*envelope.HalfWid
			if math.Abs(delta.Dot(right)) >= laneWidth+bus.Width/2+lateralExtent {
				continue
			}
			// A long bus owns both boarding positions. Other vehicles wait
			// behind the platform until the long vehicle has departed.
			if stop.HasTwoBerths() && (bus.IsArticulated() || other.IsArticulated()) && stop.OccupiesBerth(envelope) {
				along = math.Min(along, -stop.Length/2-StationBerthGap-bus.Length/2)
			}
			ownFront := bus.Length / 2
			if stop.HasTwoBerths() && math.Abs(other.Speed) < 3 && other.Pos.Sub(stop.Pos).Dot(forward) > -1200 && other.Pos.Sub(stop.Pos).Dot(forward) < stop.RearBerthPosition().Sub(stop.Pos).Dot(forward)-48 {
				along = math.Min(along, delta.Dot(forward)-extent-StationBerthGap-ownFront)
			}
			ownRear := bus.OverallLength() - ownFront
			d := delta.Dot(forward)
			if along+ownFront > d-extent-24+1e-6 && along-ownRear < d+extent+24-1e-6 {
				along = d - extent - 24 - ownFront
				moved = true
			}
		}
		if !moved {
			break
		}
	}
	bus.Pos = stop.Pos.Add(forward.Mul(along))
	bus.Heading = stop.Heading
	bus.ResetArticulation()
	return along < stop.FrontBerthPosition().Sub(stop.Pos).Dot(forward)-.001
}
