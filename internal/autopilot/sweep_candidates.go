package autopilot

import "math"

// Collect only for this one articulated sweep. Its 16 front-section steps each
// travel speed*.05, so every sampled position lies within speed*.8 of the source
// (a rejected AtPose stays at the previous position). Triangle inequality then
// expands each original broad-phase circle to cover all those sample positions.
// Live per-sample tests and original obstacle order are retained.
func articulatedSweepCandidates(f autopilotFrame, speed, steer float64, dst []int) ([]int, bool) {
	b := f.bus
	if len(f.obstacles) <= 64 || b == nil ||
		!(speed >= 0 && speed <= 10000) || !(math.Abs(steer) <= 1) ||
		!(math.Abs(b.Pos.X) <= 1e9 && math.Abs(b.Pos.Y) <= 1e9) ||
		!(math.Abs(b.Heading) <= 1e6 && math.Abs(b.SteeringAngle) <= 1.5 && math.Abs(b.MaxSteerAngle) <= 1.5) ||
		!(b.WheelBase >= 1 && b.WheelBase <= 1e9 && b.SteerTurnSpeed >= 0 && b.SteerTurnSpeed <= 1e6) ||
		!(math.Abs(b.OverallLength()) <= 8192) {
		return nil, false
	}
	dst = dst[:0]
	ownLength := b.OverallLength()
	travel := speed * .8
	for i, ob := range f.obstacles {
		radius := math.Abs(ownLength+ob.Length+100) + travel
		// Deliberately generous roundoff allowance, including source-coordinate ULPs.
		radius += 1e-7 * (1 + radius + math.Abs(b.Pos.X) + math.Abs(b.Pos.Y))
		if radius >= 0 && radius <= 8192 {
			delta := ob.Pos.Sub(b.Pos)
			if delta.Dot(delta) > radius*radius {
				continue
			}
		}
		// Bounded stack storage. Dense/abnormal scenes fall back without allocating.
		if len(dst) == cap(dst) {
			return nil, false
		}
		dst = append(dst, i)
	}
	return dst, true
}
