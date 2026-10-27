package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// followingSpeed is a headway cap for bus-sized leaders, applied after path-based
// passing decisions. Their long, yawed bodies may still span our nose even when
// their centers have left the tracked lane. Cars retain the normal passing policy.
func (ad *AutopilotDriver) followingSpeed(f autopilotFrame) float64 {
	limit := math.Inf(1)
	forward := sim.UnitFromAngle(f.bus.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	for _, obs := range f.obstacles {
		if ad.clearsSkippedStopBay(f, obs) || ad.clearsServedStationQueue(f, obs) {
			continue
		}
		// The short ordinary-mode obstacle ray is measured to the leader's
		// envelope center. A long bus's rear is reached before that center
		// enters the ray, leaving only the last-resort contact stop. Keep the
		// same bumper headway for articulated leaders in every driving mode.
		if !ad.BerserkMode && (obs.Bus == nil || !obs.Bus.IsArticulated()) {
			continue
		}
		if ad.ordinaryBusLeft && ad.clearsThroughQueueDuringMerge(f, obs) {
			continue
		}
		if obs.Length < f.bus.Length*.8 {
			continue
		}
		angle := obs.Heading - f.bus.Heading
		if math.Cos(angle) < .85 {
			continue
		}
		delta := obs.Pos.Sub(f.bus.Pos)
		along := delta.Dot(forward)
		if along <= 0 {
			continue
		}
		halfLength := math.Abs(math.Cos(angle))*obs.Length/2 + math.Abs(math.Sin(angle))*obs.Width/2
		halfWidth := math.Abs(math.Sin(angle))*obs.Length/2 + math.Abs(math.Cos(angle))*obs.Width/2
		if math.Abs(delta.Dot(right)) >= f.bus.Width/2+halfWidth+3 {
			continue
		}
		if f.world != nil && !f.nearRoundabout && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
			// Temporary steering yaw can point at a bus far ahead in another
			// lane. Both physical bodies must also share the road corridor.
			axis := approachRoadHeading(f.bus.Heading)
			normal := sim.Vec2{X: -math.Sin(axis), Y: math.Cos(axis)}
			busExtent := math.Abs(math.Sin(f.bus.Heading-axis))*f.bus.Length/2 + math.Abs(math.Cos(f.bus.Heading-axis))*f.bus.Width/2
			leadExtent := math.Abs(math.Sin(obs.Heading-axis))*obs.Length/2 + math.Abs(math.Cos(obs.Heading-axis))*obs.Width/2
			if math.Abs(delta.Dot(normal)) > busExtent+leadExtent+3 {
				continue
			}
		}
		gap := along - f.bus.Length/2 - halfLength
		if gap > 80 && len(ad.drivePath) >= 2 {
			tracked := ad.basePathPosition(obs.Pos, f.world, f.bus.Heading, f.laneWidth)
			if minDistanceToPath(tracked, ad.drivePath, ad.WaypointIdx) > f.bus.Width/2+obs.Width/2+10 {
				continue
			}
		}
		lead := math.Max(0, obs.Speed*math.Cos(angle))
		// Match a moving leader continuously rather than repeatedly accelerating
		// into a stop threshold. Reserve space for reaction and leader braking.
		desired := 30 + .35*f.speedAbs
		speed := math.Max(0, lead+.7*(gap-desired))
		stopping := math.Sqrt(lead*lead + 2*135*math.Max(0, gap-18-.2*f.speedAbs))
		if gap < 12 || (lead < 1 && gap <= 38) {
			speed = 0
		}
		limit = math.Min(limit, math.Min(speed, stopping))
	}
	return limit
}
