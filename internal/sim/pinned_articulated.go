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
	if leader == nil || !leader.IsArticulated() || math.Abs(leader.Speed) >= 1 || math.Cos(leader.TailHeading()-heading) < .35 {
		return 0, false
	}
	if yaw := math.Abs(NormalizeAngle(leader.Heading - heading)); yaw < .2 || yaw > 1.2 {
		return 0, false
	}
	u := UnitFromAngle(heading)
	if leader.Pos.Sub(pos).Dot(u) <= 0 {
		return 0, false
	}
	r := Vec2{X: -u.Y, Y: u.X}
	latLimit := halfWid + reach
	rear := math.Inf(1)
	minLat, maxLat := math.Inf(1), math.Inf(-1)
	for _, part := range leader.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		dc := part.Center.Sub(pos)
		cU, cR := dc.Dot(u), dc.Dot(r)
		pU := UnitFromAngle(part.Heading)
		pR := Vec2{X: -pU.Y, Y: pU.X}
		lU, lR := pU.Dot(u)*part.HalfLen, pU.Dot(r)*part.HalfLen
		wU, wR := pR.Dot(u)*part.HalfWid, pR.Dot(r)*part.HalfWid
		corners := [4]Vec2{
			{X: cU + lU + wU, Y: cR + lR + wR},
			{X: cU + lU - wU, Y: cR + lR - wR},
			{X: cU - lU - wU, Y: cR - lR - wR},
			{X: cU - lU + wU, Y: cR - lR + wR},
		}
		for i, a := range corners {
			minLat = math.Min(minLat, a.Y)
			maxLat = math.Max(maxLat, a.Y)
			if math.Abs(a.Y) <= latLimit {
				rear = math.Min(rear, a.X)
			}
			b := corners[(i+1)&3]
			if dy := b.Y - a.Y; math.Abs(dy) > 1e-9 {
				for _, bound := range [2]float64{-latLimit, latLimit} {
					if t := (bound - a.Y) / dy; t > 0 && t < 1 {
						rear = math.Min(rear, a.X+(b.X-a.X)*t)
					}
				}
			}
		}
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
