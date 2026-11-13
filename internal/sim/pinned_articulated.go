package sim

import "math"

// PinnedArticulatedRearGap measures how far a follower's front bumper is
// behind a stopped articulated bus whose powered section is turned away from
// the road while its trailer still follows it. Such a bus can only recover by
// backing up a few pixels or swinging its tail, so a follower must not pull up
// beside that tail: it reports the gap to the rearmost body section whenever
// the bus lies within reach of the follower's own lateral extent. ok is false
// for aligned, moving or compact buses, distant lanes, and buses already
// alongside the follower.
func PinnedArticulatedRearGap(leader *Bus, pos Vec2, heading, halfLen, halfWid, reach float64) (float64, bool) {
	if leader == nil || !leader.IsArticulated() || math.Abs(leader.Speed) >= 1 || math.Cos(leader.TailHeading()-heading) < .95 {
		return 0, false
	}
	if yaw := math.Abs(NormalizeAngle(leader.Heading - heading)); yaw < .2 || yaw > 1.2 {
		return 0, false
	}
	u := UnitFromAngle(heading)
	r := Vec2{X: -u.Y, Y: u.X}
	rear := math.Inf(1)
	minLat, maxLat := math.Inf(1), math.Inf(-1)
	for _, part := range leader.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - heading
		along := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
		side := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
		delta := part.Center.Sub(pos)
		rear = math.Min(rear, delta.Dot(u)-along)
		minLat = math.Min(minLat, delta.Dot(r)-side)
		maxLat = math.Max(maxLat, delta.Dot(r)+side)
	}
	if math.IsInf(rear, 0) {
		return 0, false
	}
	lateralGap := math.Max(0, math.Max(minLat-halfWid, -halfWid-maxLat))
	gap := rear - halfLen
	if lateralGap > reach || gap < -halfLen {
		return 0, false
	}
	return gap, true
}
