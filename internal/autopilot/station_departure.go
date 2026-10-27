package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Join the next route from the actual berth, leaving room to merge gradually.
func (ad *AutopilotDriver) connectDockDeparture(bus *sim.Bus, stop *sim.BusStop) {
	ad.smoothDockDeparture = false
	if len(ad.drivePath) < 2 {
		return
	}
	forward := sim.UnitFromAngle(stop.Heading)
	total := ad.cumLen[len(ad.cumLen)-1]
	joinS := 0.0
	for s := 10.0; s <= math.Min(total, 700); s += 10 {
		point := ad.pointAt(s)
		if ad.pathWorld != nil {
			if ad.pathWorld.InsideJunction(point, ad.pathLane, bus.OverallLength()/2+24) {
				break
			}
			nearRing := false
			for _, rb := range ad.pathWorld.Roundabouts {
				if point.Distance(rb.Center) < rb.Radius+bus.OverallLength() {
					nearRing = true
					break
				}
			}
			if nearRing {
				break
			}
		}
		tangent := ad.pointAt(s + 2).Sub(point).Normalize()
		if tangent.Dot(forward) < 0.8 && point.Sub(bus.Pos).Dot(forward) > 80 {
			break
		}
		joinS = s
		runway := 600.
		if ad.BerserkMode && bus.IsArticulated() {
			// Finish merging before the paved station exit narrows to the curb.
			runway = math.Max(400, 400+(ad.pathLane-52)*4)
		}
		if point.Sub(bus.Pos).Dot(forward) >= runway {
			break
		}
	}
	goal := ad.pointAt(joinS)
	if goal.Sub(bus.Pos).Dot(forward) < 80 {
		return
	}
	runIn := math.Min(120, goal.Sub(bus.Pos).Dot(forward)*0.25)
	if ad.BerserkMode && bus.IsArticulated() {
		runIn = math.Min(runIn, 80)
	}
	if bus.IsArticulated() {
		// Clear the shelter with the rear section before beginning the lateral
		// merge, including flexible service behind the usual front marker.
		rear := math.Inf(1)
		for _, body := range bus.BodyParts() {
			for _, corner := range sim.BodyCorners(body) {
				rear = math.Min(rear, corner.Sub(stop.Pos).Dot(forward))
			}
		}
		// The shelter is 68px long; leave additional room beyond its end.
		runIn = math.Min(goal.Sub(bus.Pos).Dot(forward)*.5, math.Max(runIn, 50-rear))
	}
	alignedBus := *bus
	alignedBus.Pos = bus.Pos.Add(forward.Mul(runIn))
	alignedBus.Heading = stop.Heading
	points := append([]sim.Vec2{bus.Pos}, stationDockCurve(&alignedBus, goal, forward)...)
	if ad.BerserkMode && bus.IsArticulated() && goal.Sub(alignedBus.Pos).Dot(forward) >= 300 {
		// Spread the lateral merge over the whole available runway instead of
		// reusing the arrival curve's short turn and long final straight. Keep
		// the initial pull-forward so the tail clears the shelter before turning.
		ad.smoothDockDeparture = true
		points = gentleDockDeparture(bus.Pos, alignedBus.Pos, goal, forward)
	}
	ad.dockDepartureRun = stationCurveLength(points)
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

// Quintic lateral blend: zero slope and curvature at both ends, with a
// monotonic forward coordinate. It cannot loop back or cut across the curb.
func gentleDockDeparture(origin, start, goal, forward sim.Vec2) []sim.Vec2 {
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	along := goal.Sub(start).Dot(forward)
	lateral := goal.Sub(start).Dot(right)
	points := []sim.Vec2{origin, start}
	for i := 1; i <= 64; i++ {
		t := float64(i) / 64
		blend := t * t * t * (10 + t*(-15+6*t))
		points = append(points, start.Add(forward.Mul(along*t)).Add(right.Mul(lateral*blend)))
	}
	return points
}

func (ad *AutopilotDriver) dockDepartureSpeed(f autopilotFrame) float64 {
	if !ad.BerserkMode || !f.bus.IsArticulated() || !ad.smoothDockDeparture || ad.dockDepartureRun < 400 {
		return 80
	}
	// Build speed as the bus pulls clear; retain a low cap if either section
	// has not settled. Signals, traffic and full-body sweeps still apply later.
	limit := math.Min(240, 100+math.Max(0, f.s)*.4)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-f.bus.TailHeading())) > .25 || math.Abs(f.bus.SteeringAngle) > .15 {
		limit = math.Min(limit, 100)
	}
	return limit
}
