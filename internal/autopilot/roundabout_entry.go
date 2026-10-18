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
	entry, forward, _ := ad.roundaboutEntry(rb)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	outerRadius := roundaboutOuterLaneRadius(rb)
	radius := outerRadius + offset
	baseEntry := ad.drivePath[entry]
	angle := math.Atan2(baseEntry.Y-rb.Center.Y, baseEntry.X-rb.Center.X) - apRoundaboutEntrySweep
	join := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius))
	tangent := sim.Vec2{X: math.Sin(angle), Y: -math.Cos(angle)}
	start := baseEntry.Sub(forward.Mul(50)).Add(right.Mul(ad.LaneOffset))
	control1 := start.Add(forward.Mul(100))
	control2 := join.Sub(tangent.Mul(60))
	points := make([]sim.Vec2, 0, 33)
	for i := 0; i <= 24; i++ {
		u := float64(i) / 24
		v := 1 - u
		points = append(points, start.Mul(v*v*v).Add(control1.Mul(3*v*v*u)).Add(control2.Mul(3*v*u*u)).Add(join.Mul(u*u*u)))
	}
	// Continue counter-clockwise beyond the tangent join for pursuit lookahead.
	for i := 1; i <= 8; i++ {
		theta := angle - float64(i)*0.05
		points = append(points, rb.Center.Add(sim.Vec2{X: math.Cos(theta), Y: math.Sin(theta)}.Mul(radius)))
	}
	return points, forward, true
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
