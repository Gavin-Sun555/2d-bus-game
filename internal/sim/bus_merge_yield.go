package sim

import "math"

// BusMergeYieldLimit lets a route bus reserve a modest rear slot for a bus
// returning ahead of it. Alongside, opposing and unbrakeable traffic cannot
// grant priority; a stopped follower can keep its existing safe queue gap.
func BusMergeYieldLimit(follower, merging *Bus, laneWidth float64, continuing bool) (float64, bool) {
	if follower == nil || merging == nil || follower == merging || merging.DoorsOpen {
		return 0, false
	}
	request := merging.LaneMergeRequest
	if request == nil || request.Remaining <= 0 || request.Joined || math.Cos(follower.Heading-request.Heading) < .98 || math.Cos(merging.Heading-request.Heading) < .9 {
		return 0, false
	}
	forward := Vec2{X: math.Cos(request.Heading), Y: math.Sin(request.Heading)}
	right := Vec2{X: -forward.Y, Y: forward.X}
	if math.Abs(follower.Pos.Sub(request.Target).Dot(right)) > laneWidth*.4 {
		return 0, false
	}
	yaw := merging.Heading - request.Heading
	extent := merging.Length/2*math.Abs(math.Cos(yaw)) + merging.Width/2*math.Abs(math.Sin(yaw))
	gap := merging.Pos.Sub(follower.Pos).Dot(forward) - extent - follower.Length/2
	if gap < 12 || gap > 750 {
		return 0, false
	}
	speed := math.Max(0, follower.Speed)
	braking := speed*speed/(2*135) + speed*.2
	if !continuing && gap < 20+braking {
		return 0, false
	}
	return math.Sqrt(2 * 135 * math.Max(0, gap-35)), true
}
