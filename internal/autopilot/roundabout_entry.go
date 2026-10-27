package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

const apRoundaboutEntrySweep = 0.60

func roundaboutOuterLaneRadius(rb *sim.Roundabout) float64 {
	return (rb.InnerRadius + 16 + 3*rb.Radius) * 0.25
}

// Leave room for the long body's yaw during entry and outward lane changes.
func roundaboutLaneOffset(rb *sim.Roundabout, bus *sim.Bus, offset float64) float64 {
	minRadius := rb.InnerRadius + bus.Width*0.5 + 24
	minOffset := math.Min(0, minRadius-roundaboutOuterLaneRadius(rb))
	return sim.Clamp(offset, minOffset, 0)
}

// roundaboutEntry identifies the road-to-ring join on the active route. The
// approach direction comes from the route, not the bus's changing yaw angle.
func (ad *AutopilotDriver) roundaboutEntry(rb *sim.Roundabout) (int, sim.Vec2, bool) {
	if len(ad.drivePath) < 2 || ad.drivePath[0].Distance(rb.Center) <= rb.Radius+10 {
		return 0, sim.Vec2{}, false
	}
	for i := 1; i < len(ad.drivePath); i++ {
		if ad.drivePath[i].Distance(rb.Center) > rb.Radius+10 {
			continue
		}
		direction := ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize()
		if direction.Length() < 0.5 {
			return 0, sim.Vec2{}, false
		}
		return i, direction, true
	}
	return 0, sim.Vec2{}, false
}

func (ad *AutopilotDriver) joiningInnerRoundabout(rb *sim.Roundabout, s, laneWidth float64) bool {
	entry, _, ok := ad.roundaboutEntry(rb)
	if !ok || ad.LaneOffset >= -laneWidth*0.3 {
		return false
	}
	outerRadius := roundaboutOuterLaneRadius(rb)
	return s < ad.cumLen[entry]+outerRadius*(apRoundaboutEntrySweep+0.05)
}

// innerRoundaboutEntryTarget follows a tangent-continuous entry curve into the
// inner ring. Driving straight to the inner circle and then turning 90 degrees
// lets the bus's front cut into the island before the rear has joined the ring.
func (ad *AutopilotDriver) innerRoundaboutEntryTarget(f autopilotFrame, rb *sim.Roundabout, lookahead, offset float64) (sim.Vec2, bool) {
	points, forward, ok := ad.innerRoundaboutEntryCurve(f, rb, offset)
	if !ok {
		return sim.Vec2{}, false
	}
	return pursuitAlongEntry(points, f.bus.Pos, forward, lookahead), true
}

func (ad *AutopilotDriver) innerRoundaboutEntryCurve(f autopilotFrame, rb *sim.Roundabout, offset float64) ([]sim.Vec2, sim.Vec2, bool) {
	if !ad.joiningInnerRoundabout(rb, f.s, f.laneWidth) {
		return nil, sim.Vec2{}, false
	}
	return ad.roundaboutEntryCurve(f, rb, offset)
}

func (ad *AutopilotDriver) roundaboutEntryCurve(f autopilotFrame, rb *sim.Roundabout, offset float64) ([]sim.Vec2, sim.Vec2, bool) {
	entry, forward, ok := ad.roundaboutEntry(rb)
	if !ok || f.s >= ad.cumLen[entry]+roundaboutOuterLaneRadius(rb)*(apRoundaboutEntrySweep+.05) {
		return nil, sim.Vec2{}, false
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	radius := roundaboutOuterLaneRadius(rb) + offset
	var curve sim.RoundaboutJoin
	if ad.parallelRoundaboutJoin(f, rb) {
		lateral := ad.drivePath[entry].Sub(rb.Center).Dot(right) + ad.LaneOffset
		curve = sim.RoundaboutEntryJoin(*rb, forward, lateral, radius, f.laneWidth)
	} else {
		if ad.LaneOffset >= -f.laneWidth*.3 {
			return nil, sim.Vec2{}, false
		}
		base := ad.drivePath[entry]
		theta := math.Atan2(base.Y-rb.Center.Y, base.X-rb.Center.X) - apRoundaboutEntrySweep
		join := rb.Center.Add(sim.UnitFromAngle(theta).Mul(radius))
		tangent := sim.Vec2{X: math.Sin(theta), Y: -math.Cos(theta)}
		start := base.Sub(forward.Mul(50)).Add(right.Mul(ad.LaneOffset))
		curve = sim.RoundaboutJoin{Reference: [4]sim.Vec2{start, start.Add(forward.Mul(100)), join.Sub(tangent.Mul(60)), join}}
	}
	join, _ := sim.RoundaboutEntryPoint(curve, 1)
	angle := math.Atan2(join.Y-rb.Center.Y, join.X-rb.Center.X)
	points := make([]sim.Vec2, 0, 33)
	for i := 0; i <= 24; i++ {
		point, _ := sim.RoundaboutEntryPoint(curve, float64(i)/24)
		points = append(points, point)
	}
	// Continue counter-clockwise beyond the tangent join for pursuit lookahead.
	for i := 1; i <= 8; i++ {
		theta := angle - float64(i)*0.05
		points = append(points, rb.Center.Add(sim.UnitFromAngle(theta).Mul(radius)))
	}
	return points, forward, true
}

// A short right exit overlaps the outer entrance. Keep the existing yielded
// turn there; through/left routes have room to complete two separate joins.
func (ad *AutopilotDriver) parallelRoundaboutJoin(f autopilotFrame, rb *sim.Roundabout) bool {
	entry, _, ok := ad.roundaboutEntry(rb)
	if !ok {
		return false
	}
	exit := ad.roundaboutExitIndex(rb, 0)
	if exit <= entry {
		return false
	}
	// A road intersection can overlap a roundabout in synthetic worlds. Only
	// replace a route that actually follows the circular carriageway.
	onCircle := 0
	for i := entry; i <= exit; i++ {
		radius := ad.drivePath[i].Distance(rb.Center)
		if radius < rb.InnerRadius+16 {
			return false
		}
		if math.Abs(radius-roundaboutOuterLaneRadius(rb)) < 12 {
			onCircle++
		}
	}
	if onCircle < 4 {
		return false
	}
	a := ad.drivePath[entry].Sub(rb.Center)
	b := ad.drivePath[exit].Sub(rb.Center)
	sweep := math.Atan2(a.Y, a.X) - math.Atan2(b.Y, b.X)
	if sweep < 0 {
		sweep += 2 * math.Pi
	}
	if sweep <= 1.7 {
		return false
	}
	if ad.parallelEntryCenter != nil && *ad.parallelEntryCenter == rb.Center {
		return true
	}
	_, forward, _ := ad.roundaboutEntry(rb)
	join := sim.RoundaboutEntryJoin(*rb, forward, f.laneWidth/2, roundaboutOuterLaneRadius(rb), f.laneWidth)
	start, _ := sim.RoundaboutEntryPoint(join, 0)
	if rb.Center.Sub(f.bus.Pos).Dot(forward) < rb.YieldLineDistance()+f.bus.Length/2+12 || f.bus.Pos.Sub(start).Dot(forward) > 8 {
		return false
	}
	center := rb.Center
	ad.parallelEntryCenter = &center
	return true
}

// pursuitAlongEntry projects onto the entry curve before measuring lookahead;
// base-path arc length belongs to the outer lane and cannot parameterize it.
func pursuitAlongEntry(points []sim.Vec2, pos, forward sim.Vec2, lookahead float64) sim.Vec2 {
	return entryPointAt(points, forward, entryPosition(points, pos, forward)+lookahead)
}

func entryPosition(points []sim.Vec2, pos, forward sim.Vec2) float64 {
	bestDistance, bestS, total := math.Inf(1), 0.0, 0.0
	for i := 0; i < len(points)-1; i++ {
		length := points[i].Distance(points[i+1])
		t, d := projectToSegment(pos, points[i], points[i+1])
		if d < bestDistance {
			bestDistance, bestS = d, total+t*length
		}
		total += length
	}
	if before := pos.Sub(points[0]).Dot(forward); before < 0 {
		bestS = before
	}
	return bestS
}

func entryPointAt(points []sim.Vec2, forward sim.Vec2, targetS float64) sim.Vec2 {
	if targetS <= 0 {
		return points[0].Add(forward.Mul(targetS))
	}
	for i := 0; i < len(points)-1; i++ {
		length := points[i].Distance(points[i+1])
		if targetS <= length {
			return points[i].Add(points[i+1].Sub(points[i]).Mul(targetS / length))
		}
		targetS -= length
	}
	return points[len(points)-1]
}
