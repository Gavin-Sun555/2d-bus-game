package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A short saved rear-berth curve can finish with the nose still yawed. Once a
// forward berth opens, use an actual maximum-curvature arc followed by a straight
// run instead of asking a tiny quintic to hit the exact bay center. A small
// lateral displacement is acceptable only inside the stop's real boarding area.
func stationActuatorAlignment(f autopilotFrame, target sim.Vec2) ([]sim.Vec2, sim.Vec2, bool) {
	stop := f.targetStop
	if stop == nil || f.world == nil || f.speedAbs > 8 {
		return nil, sim.Vec2{}, false
	}
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	b := *f.bus
	yaw := sim.NormalizeAngle(stop.Heading - b.Heading)
	if math.Abs(yaw) < .15 || math.Abs(yaw) > .9 || math.Abs(b.Pos.Sub(stop.Pos).Dot(right)) > stop.Width/2+4 || target.Sub(b.Pos).Dot(forward) < 25 {
		return nil, sim.Vec2{}, false
	}
	points := []sim.Vec2{b.Pos}
	for traveled := 0.; traveled < 240; traveled += .5 {
		remaining := target.Sub(b.Pos).Dot(forward)
		if remaining <= .5 {
			break
		}
		yaw = sim.NormalizeAngle(stop.Heading - b.Heading)
		steering := sim.Clamp(math.Atan(yaw*b.WheelBase/.5)/b.MaxSteerAngle, -1, 1)
		b = edgeRecoveryPose(b, .5, steering)
		if b.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier || math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
			return nil, sim.Vec2{}, false
		}
		points = append(points, b.Pos)
	}
	if target.Sub(b.Pos).Dot(forward) > 1 || math.Abs(sim.NormalizeAngle(b.Heading-stop.Heading)) > .025 || !stop.ContainsBus(&b) || !stationCurveCanSteer(f.bus, points) || !stationDockCurveClear(f.bus, points, f.obstacles) {
		return nil, sim.Vec2{}, false
	}
	return points, b.Pos, true
}

// Use the near side of the front berth when there is too little runway to
// reach its center. Leave room for steering error at the boarding boundary.
func stationRoadsideFrontAlignment(f autopilotFrame, berth, forward, right sim.Vec2) ([]sim.Vec2, sim.Vec2) {
	stop := f.targetStop
	if f.world == nil || stop.Width <= 12 {
		return nil, sim.Vec2{}
	}
	across := f.bus.Pos.Sub(berth).Dot(right)
	target := berth.Add(right.Mul(math.Copysign(stop.Width/2-6, across)))
	points := stationAlignmentCurve(f.bus, target, forward)
	if len(points) < 2 || !stationCurveCanSteer(f.bus, points) {
		return nil, sim.Vec2{}
	}
	for i := 1; i < len(points); i++ {
		delta := points[i].Sub(points[i-1])
		pose := f.bus.AtPose(points[i], math.Atan2(delta.Y, delta.X))
		if pose.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier ||
			(i == len(points)-1 && !stop.ContainsBus(&pose)) {
			return nil, sim.Vec2{}
		}
	}
	return points, target
}
