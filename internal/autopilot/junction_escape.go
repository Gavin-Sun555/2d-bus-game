package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func corridorTurnAngle(c *sim.JunctionCorridor) float64 {
	return sim.NormalizeAngle(c.Bodies[len(c.Bodies)-1].Heading - c.EntryHeading)
}

// A bus already turning yields backwards to a conflicting, committed U-turn.
// The U-turn keeps priority so the two drivers cannot both elect to retreat.
func junctionRetreatNeeded(f autopilotFrame) bool {
	c := f.bus.JunctionCorridor
	if c == nil || c.Remaining <= 0 || !c.Entered || len(c.Bodies) < 2 || f.world == nil ||
		!f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return false
	}
	angle := math.Abs(corridorTurnAngle(c))
	if angle < .5 || angle >= 2.8 {
		return false
	}
	for _, ob := range f.obstacles {
		other := ob.JunctionCorridor
		if ob.Bus == nil || math.Abs(ob.Speed) >= 2 || ob.Pos.Distance(f.bus.Pos) > f.bus.OverallLength()+140 ||
			other == nil || other.Remaining <= 0 || !other.Entered || other.Center.Distance(c.Center) > 1 || len(other.Bodies) < 2 ||
			math.Cos(other.EntryHeading-c.EntryHeading) > -.98 || math.Abs(corridorTurnAngle(other)) < 2.8 {
			continue
		}
		if !sim.SeparatedJunctionCorridors(c, other) {
			return true
		}
	}
	return false
}

func junctionReverseControls(b *sim.Bus) (throttle, brake, steer float64) {
	steer = sim.Clamp(4*sim.NormalizeAngle(b.Heading-b.TailHeading()), -.8, .8)
	if b.Speed < -16 {
		return 0, .3, steer
	}
	return sim.Clamp((16+b.Speed)/15, .05, .35), 0, steer
}

func junctionReverseClear(f autopilotFrame, ticks int) bool {
	var scan forwardEscapeScan
	scan.prepare(f, 16)
	b := *f.bus
	// Physics ages reservations in place. Keep trial updates isolated.
	b.JunctionCorridor = nil
	b.RoundaboutEntry = nil
	b.LaneMergeRequest = nil
	b.StationMergeRequest = nil
	b.ShiftTo(sim.GearReverse)
	for tick := 1; tick <= ticks; tick++ {
		th, br, st := junctionReverseControls(&b)
		b.SetNextAccelerationScale(1)
		b.Update(1./60, th, br, st, false, false)
		if !scan.poseClear(f, &b, float64(tick)/60) {
			return false
		}
	}
	return true
}
