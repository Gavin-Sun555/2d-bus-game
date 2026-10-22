package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Cars create a gap for route-required and station merges. Cars already
// alongside or too close to brake comfortably clear the bus first.
func (tm *TrafficManager) busMergeYieldSpeed(f carFrame, target float64) float64 {
	car := f.car
	if f.world == nil || car.InRoundabout || car.JunctionTurn != nil {
		car.YieldingToBus = nil
		return target
	}
	previous := car.YieldingToBus
	car.YieldingToBus = nil
	for _, bus := range f.buses {
		if bus == nil || bus.DoorsOpen {
			continue
		}
		if limit, ok := laneBusYieldLimit(f, bus, previous); ok {
			if car.YieldingToBus == nil || limit < target {
				car.YieldingToBus = bus
			}
			car.IsBraking = true
			target = math.Min(target, limit)
		}
		if bus.StationMergeRequest == nil || bus.StationMergeRequest.Remaining <= 0 {
			continue
		}
		var stop *sim.BusStop
		for _, candidate := range f.world.Stops {
			if candidate.ID == bus.StationMergeRequest.StopID {
				stop = candidate
				break
			}
		}
		if stop == nil || car.LaneIndex != max(0, stop.RoadLanes/2-1) || math.Cos(car.Heading-stop.Heading) < .98 || math.Cos(bus.Heading-stop.Heading) < .8 {
			continue
		}
		forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		curb := (float64(max(2, stop.RoadLanes))/2 - .5) * f.laneWidth
		if math.Abs(car.Pos.Sub(stop.RoadCenter).Dot(right)-curb) > f.laneWidth*.35 {
			continue
		}
		// Leave enough space for both full bodies and the bus controller's rear gap.
		gap := bus.Pos.Sub(car.Pos).Dot(forward) - (bus.Length+car.Length)/2 - 140
		if gap < 0 || gap > 750 {
			continue
		}
		braking := car.Speed*car.Speed/(2*220) + math.Abs(car.Speed)*.2 + 15
		if previous != bus && gap < braking {
			continue
		}
		limit := math.Sqrt(2 * 220 * math.Max(0, gap-8))
		if car.YieldingToBus == nil || limit < target {
			car.YieldingToBus = bus
		}
		car.IsBraking = true
		target = math.Min(target, limit)
	}
	return target
}
