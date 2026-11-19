package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func (ad *AutopilotDriver) usesCurbRightAt(center sim.Vec2) bool {
	return ad.curbBusRight && ad.medianRightChoice != nil && ad.medianRightChoice.Distance(center) < 1
}
func (ad *AutopilotDriver) curbRightPathChoice() *sim.Vec2 {
	if ad.curbBusRight {
		return ad.medianRightChoice
	}
	return nil
}

// Choose once per junction, before committing to a crossing of the general
// lanes. Occupied right-side lanes keep the bus on its protected median turn.
func (ad *AutopilotDriver) selectMedianRightTurn(f autopilotFrame) {
	if f.world == nil || !f.world.CenterBRT || f.tls == nil || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok || !road.HasMedianBusLane() {
		return
	}
	a := ad.intersectionApproachWithRunway(f, multi, six, true)
	if a.approachingInter == nil || !a.isUpcomingRight || !a.isApproachingIntersection {
		return
	}
	junction := a.approachingInter.Center
	if ad.fallbackActiveAt(junction) || (ad.medianRightChoice != nil && ad.medianRightChoice.Distance(junction) < 1) {
		return
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	alreadyCurb := f.bus.Pos.Sub(origin).Dot(r) > 2.75*f.laneWidth
	busQueue, curbQueue, middleClear := medianRightQueues(f, origin, u, r, a.distToStopLine)
	current := f.bus.Pos.Sub(origin).Dot(r)
	if !alreadyCurb && brtPlatformBlocksDividerCross(f.world, f.bus, heading, f.laneWidth, current, 3.5*f.laneWidth) {
		return
	}
	enoughRunway := a.distToStopLine > math.Max(800, 3*f.laneWidth*4.5+f.bus.OverallLength())
	preferCurb := busQueue > curbQueue && middleClear && enoughRunway
	if preferCurb {
		preferCurb = requiredMergeLaneClear(f, heading, current, 3.5*f.laneWidth)
	}
	ad.medianRightChoice = &junction
	ad.curbBusRight = alreadyCurb || preferCurb
	// A saved curb merge is already a committed maneuver; keep its corridor.
	if ad.approachMerge != nil && ad.approachMerge.RightTurn && ad.approachMerge.Junction.Distance(junction) < 1 {
		ad.curbBusRight = true
	}
	if ad.pathFromID < 0 || ad.pathFromID >= len(f.world.Stops) || ad.pathToID < 0 || ad.pathToID >= len(f.world.Stops) {
		return
	}
	if !ad.curbBusRight {
		ad.approachMerge = nil
		ad.approachMergeJoined = false
	}
	// Rebuild the leg with only this junction's selected turn corridor.
	ad.drivePath = nil
	ad.PrepareLeg(f.world, f.world.Stops[ad.pathFromID], f.world.Stops[ad.pathToID], f.laneWidth)
	ad.trackPath(ad.basePathPosition(f.bus.Pos, f.world, f.bus.Heading, f.laneWidth))
}

// Compare actual waiting vehicles, not a weighted score: an empty or equally
// queued bus lane always wins. Only inspect middle-lane traffic in the runway
// we would cross; a distant car beyond the merge does not make a road busy.
func medianRightQueues(f autopilotFrame, origin, forward, right sim.Vec2, stopDistance float64) (busQueue, curbQueue int, middleClear bool) {
	middleClear = true
	mergeRunway := math.Min(stopDistance, math.Max(700, 3*f.laneWidth*4.5+f.bus.OverallLength()+f.speedAbs*1.5))
	heading := math.Atan2(forward.Y, forward.X)
	for _, ob := range f.obstacles {
		if ob.Bus == f.bus {
			continue
		}
		delta := ob.Pos.Sub(f.bus.Pos)
		along := delta.Dot(forward)
		yaw := ob.Heading - heading
		extent := ob.Length/2*math.Abs(math.Sin(yaw)) + ob.Width/2*math.Abs(math.Cos(yaw))
		lateral := ob.Pos.Sub(origin).Dot(right)
		low, high := lateral-extent, lateral+extent
		if high > f.laneWidth+1 && low < 3*f.laneWidth-1 && along > -math.Max(250, math.Abs(ob.Speed)*1.5) && along < mergeRunway {
			middleClear = false
		}
		if along <= 0 || along > stopDistance+50 || math.Cos(yaw) < .8 || math.Abs(ob.Speed) >= 20 {
			continue
		}
		if high > 0 && low < f.laneWidth {
			busQueue++
		}
		if high > 3*f.laneWidth && low < 4*f.laneWidth {
			curbQueue++
		}
	}
	return
}
