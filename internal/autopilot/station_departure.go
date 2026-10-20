package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Join the next route from the actual berth, leaving room to merge gradually.
func (ad *AutopilotDriver) connectDockDeparture(bus *sim.Bus, stop *sim.BusStop) {
	if len(ad.drivePath) < 2 {
		return
	}
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	total := ad.cumLen[len(ad.cumLen)-1]
	joinS := 0.0
	for s := 10.0; s <= math.Min(total, 700); s += 10 {
		point := ad.pointAt(s)
		tangent := ad.pointAt(s + 2).Sub(point).Normalize()
		if tangent.Dot(forward) < 0.8 && point.Sub(bus.Pos).Dot(forward) > 80 {
			break
		}
		joinS = s
		if point.Sub(bus.Pos).Dot(forward) >= 600 {
			break
		}
	}
	goal := ad.pointAt(joinS)
	if goal.Sub(bus.Pos).Dot(forward) < 80 {
		return
	}
	runIn := math.Min(120, goal.Sub(bus.Pos).Dot(forward)*0.25)
	alignedBus := *bus
	alignedBus.Pos = bus.Pos.Add(forward.Mul(runIn))
	alignedBus.Heading = stop.Heading
	points := append([]sim.Vec2{bus.Pos}, stationDockCurve(&alignedBus, goal, forward)...)
	for i := 1; i < len(ad.drivePath); i++ {
		if ad.cumLen[i] > joinS {
			points = append(points, ad.drivePath[i])
		}
	}
	ad.drivePath = points
	ad.cumLen = make([]float64, len(points))
	for i := 1; i < len(points); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + points[i].Distance(points[i-1])
	}
	ad.WaypointIdx = 0
	ad.pathSynced = false
	ad.LastPathS = 0
	ad.LaneOffset, ad.TargetLaneOffset = 0, 0
}
