package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// An exhausted path or stalled late arrival cannot reverse into a missed bay.
// Recover from the actual pose, preserving collision/merge checks on departure.
// Fleet buses need this just as the player's existing missed-stop handler does.
func (ab *AIBus) recoverPassedStation(w *sim.World, laneWidth, dt float64, heldByController bool) bool {
	if ab == nil || w == nil || ab.Driver == nil || ab.Route == nil || ab.Bus == nil || len(ab.Route.StopIDs) < 2 || ab.Driver.State == autopilot.AutopilotStateDocked || ab.Driver.State == autopilot.AutopilotStateDeparting || ab.Bus.DoorsOpen || ab.Bus.DoorProgress > .05 || ab.Bus.PassengerServiceRemaining() > 0 || ab.Bus.TerminalReplacementHold {
		if ab != nil {
			ab.lateArrivalWait = 0
		}
		return false
	}
	id := ab.Route.GetCurrentTargetStopID()
	if id < 0 || id >= len(w.Stops) {
		ab.lateArrivalWait = 0
		return false
	}
	stop := w.Stops[id]
	path := ab.Driver.DrivePath()
	if len(path) < 2 || ab.Driver.LastTargetID != id || stop.ContainsBus(ab.Bus) {
		ab.lateArrivalWait = 0
		return false
	}
	u := sim.UnitFromAngle(stop.Heading)
	delta := ab.Bus.Pos.Sub(stop.FrontBerthPosition())
	// Require a genuinely passed endpoint on the same road, not an approach,
	// rear berth, adjacent street, or a route that still has turns ahead.
	along := delta.Dot(u)
	lateral := math.Abs(delta.X*u.Y - delta.Y*u.X)
	endpointPassed := ab.Bus.Pos.Sub(path[len(path)-1]).Dot(u)
	// A long bus may exhaust its taper past the last valid front marker
	// while its powered section is still yawed. Its straight tail, location
	// outside a junction, and passed endpoint distinguish this from a road turn.
	yawedPastTaper := ab.Bus.IsArticulated() && ab.Driver.State == autopilot.AutopilotStateApproaching && heldByController && math.Abs(ab.Bus.Speed) < .5 && ab.Driver.LastRemaining <= 2 && along > 32 && along <= ab.Bus.Length && endpointPassed > 32 && ab.Driver.LastLateralErr < stop.Width+laneWidth && math.Cos(ab.Bus.Heading-stop.Heading) > .8 && math.Cos(ab.Bus.TailHeading()-stop.Heading) > .95 && ab.Bus.Pos.Distance(stop.Pos) < stop.Length && !w.InsideJunction(ab.Bus.Pos, laneWidth, ab.Bus.OverallLength()/2)
	if yawedPastTaper {
		for _, rb := range w.Roundabouts {
			if ab.Bus.Pos.Distance(rb.Center) < rb.Radius+260 {
				yawedPastTaper = false
				break
			}
		}
	}
	if (!yawedPastTaper && math.Cos(ab.Bus.Heading-stop.Heading) < .98) || lateral > stop.Width/2+laneWidth {
		ab.lateArrivalWait = 0
		return false
	}
	exhausted := yawedPastTaper || (ab.Driver.LastRemaining <= 2 && along > ab.Bus.Length/2+12 && endpointPassed > ab.Bus.Length/2+12)
	// A late outer-lane arrival may project onto the final taper with a
	// positive remaining distance although its actual berth is behind it.
	// Give the normal dock planner time to finish first; never skip a moving
	// entry, a waiting rear berth inside the bay, or a stop still ahead.
	lateOutside := ab.Driver.State == autopilot.AutopilotStateApproaching && math.Abs(ab.Bus.Speed) < .5 && heldByController && ab.Driver.LastRemaining <= ab.Bus.OverallLength() && along > 8 && endpointPassed > 8 && lateral > stop.Width/2+4
	if lateOutside {
		ab.lateArrivalWait += math.Max(0, dt)
	} else {
		ab.lateArrivalWait = 0
	}
	if !exhausted && ab.lateArrivalWait < 3 {
		return false
	}
	ab.lateArrivalWait = 0
	ab.Route.AdvanceToNextStop()
	next := ab.Route.GetCurrentTargetStopID()
	for i := range ab.Bus.Passengers {
		if ab.Bus.Passengers[i].TargetStopID == id {
			ab.Bus.Passengers[i].TargetStopID = next
		}
	}
	ab.Driver.ContinuePastSkippedStop(ab.Bus, stop, laneWidth)
	ab.LastExchangedStopID = -1
	ab.Bus.StationMergeRequest = nil
	ab.Bus.LaneMergeRequest = nil
	return true
}
