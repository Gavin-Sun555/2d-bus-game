package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func stationBayLateralExtent(bus *sim.Bus, stop *sim.BusStop) float64 {
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	extent := 0.
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		for _, point := range sim.BodyCorners(part) {
			extent = math.Max(extent, math.Abs(point.Sub(stop.Pos).Dot(r)))
		}
	}
	return extent
}

// Fill an available forward berth before accepting the rear marker, leaving
// the runway and rear boarding space for the next bus in the queue.
func (ad *AutopilotDriver) preferForwardArticulatedBay(f autopilotFrame) bool {
	if ad.BerserkMode || !f.bus.IsArticulated() || f.targetStop == nil {
		return false
	}
	front := f.targetStop.FrontBerthPosition()
	u := sim.UnitFromAngle(f.targetStop.Heading)
	if front.Sub(f.bus.Pos).Dot(u) < 60 {
		return false
	}
	for _, berth := range ad.availableStationBerths(f.bus, f.targetStop, f.obstacles) {
		if berth.Distance(front) < 1 {
			return true
		}
	}
	return false
}

func (ad *AutopilotDriver) unfinishedArticulatedBayEntry(f autopilotFrame) bool {
	p := ad.stationDock
	return !ad.BerserkMode && f.bus.IsArticulated() && p != nil && p.rank == 0 && !p.waiting && len(p.points) > 1 &&
		p.target.Sub(f.bus.Pos).Dot(sim.UnitFromAngle(f.targetStop.Heading)) > 5 &&
		stationBayLateralExtent(f.bus, f.targetStop) > f.targetStop.Width/2+1
}

// Prefer an earlier, steerable lateral move followed by a straight run that
// lets the trailer settle. Evaluate the real hitch constraint, not just the
// powered section's endpoint. A short/occupied berth may still need its normal
// fallback; never demand an impossible lateral fit to open the doors.
func articulatedBayEntryCurve(f autopilotFrame, target sim.Vec2, original []sim.Vec2) ([]sim.Vec2, bool) {
	u := sim.UnitFromAngle(f.targetStop.Heading)
	distance := target.Sub(f.bus.Pos).Dot(u)
	if distance < 70 || distance > 650 {
		return original, false
	}
	score := func(points []sim.Vec2) float64 {
		pose := *f.bus
		for i := 1; i < len(points); i++ {
			d := points[i].Sub(points[i-1])
			if d.Length() < .01 {
				continue
			}
			pose = pose.AtPose(points[i], math.Atan2(d.Y, d.X))
			if pose.Pos.Distance(points[i]) > .1 || pose.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
				return math.Inf(1)
			}
		}
		return stationBayLateralExtent(&pose, f.targetStop)
	}
	best, bestScore := original, score(original)
	changed := false
	if bestScore <= f.targetStop.Width/2 {
		return original, false
	}
	for _, fraction := range []float64{.35, .45, .55, .65, .75} {
		straight := math.Min(220, distance*fraction)
		end := target.Sub(u.Mul(straight))
		points := stationAlignmentCurve(f.bus, end, u)
		for along := 5.; along < straight; along += 5 {
			points = append(points, end.Add(u.Mul(along)))
		}
		points = append(points, target)
		if !stationCurveCanSteer(f.bus, points) {
			continue
		}
		if extent := score(points); extent+1 < bestScore && stationDockCurveClear(f.bus, points, f.obstacles) {
			best, bestScore = points, extent
			changed = true
		}
	}
	return best, changed
}
