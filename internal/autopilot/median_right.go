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
	if ad.medianRightChoice != nil && ad.medianRightChoice.Distance(junction) < 1 {
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
	clear := true
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(f.bus.Pos)
		along := delta.Dot(u)
		lateral := ob.Pos.Sub(origin).Dot(r)
		// Reserve every crossed lane from behind the bus to the outgoing mouth.
		if along > -math.Max(250, math.Abs(ob.Speed)*1.5) && along < a.distToStopLine+900 && lateral > f.laneWidth-ob.Width/2 && lateral < 4*f.laneWidth+ob.Width/2 {
			clear = false
			break
		}
	}
	ad.medianRightChoice = &junction
	ad.curbBusRight = alreadyCurb || (clear && a.distToStopLine > math.Max(800, 3*f.laneWidth*4.5+f.bus.OverallLength()))
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
