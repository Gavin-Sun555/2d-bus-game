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
	// A slow bay exit can still have a small yaw while occupying the rear
	// slot. Its projected body and braking distance remain part of the grant.
	minAlignment := .8
	if continuing {
		// Keep an already granted slot while the leader rotates through its
		// merge. Recheck projected full-body gap and braking below every tick.
		minAlignment = .25
	}
	if request == nil || request.Remaining <= 0 || request.Joined || (math.Cos(follower.Heading-request.Heading) < .98 && (math.Abs(follower.Speed) > 20 || math.Cos(follower.Heading-request.Heading) < .95)) || math.Cos(merging.Heading-request.Heading) < minAlignment {
		return 0, false
	}
	forward := UnitFromAngle(request.Heading)
	right := Vec2{X: -forward.Y, Y: forward.X}
	// A bus still finishing its bay exit can physically occupy the requested
	// lane while its center is outside the old narrow lane-center tolerance.
	// Include its full lateral footprint and the same 10px margin used by
	// the required-merge clearance check. Otherwise the leader can wait for
	// a follower which does not recognize that it occupies the requested slot.
	followerYaw := follower.Heading - request.Heading
	followerBody := follower.BodyEnvelope()
	mergingBody := merging.BodyEnvelope()
	followerHalfWidth := followerBody.HalfLen*math.Abs(math.Sin(followerYaw)) + followerBody.HalfWid*math.Abs(math.Cos(followerYaw))
	// A return across several lanes also needs the rear slot in each crossed
	// lane. A follower there can stop the merge even though it clears the final
	// lane center. Keep the same full-body and braking checks below.
	current := mergingBody.Center.Sub(request.Target).Dot(right)
	lateral := followerBody.Center.Sub(request.Target).Dot(right)
	margin := math.Max(laneWidth*.4, followerHalfWidth+merging.Width/2+10)
	if lateral < math.Min(0, current)-margin || lateral > math.Max(0, current)+margin {
		return 0, false
	}
	yaw := merging.Heading - request.Heading
	extent := mergingBody.HalfLen*math.Abs(math.Cos(yaw)) + mergingBody.HalfWid*math.Abs(math.Sin(yaw))
	followerExtent := followerBody.HalfLen*math.Abs(math.Cos(followerYaw)) + followerBody.HalfWid*math.Abs(math.Sin(followerYaw))
	gap := mergingBody.Center.Sub(followerBody.Center).Dot(forward) - extent - followerExtent
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
