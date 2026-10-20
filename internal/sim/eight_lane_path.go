package sim

import "math"

// Buses cruise in the outer through lane and enter the right-turn lane before
// the stop line. A path taper avoids treating a turn-lane change as overtaking.
func appendEightLaneRightTurn(points []Vec2, w *World, center, in, out Vec2, inCoord, outCoord float64, inEW, outEW bool, inProbe, outProbe Vec2, lw float64) []Vec2 {
	right := Vec2{X: -in.Y, Y: in.X}
	geometry := w.JunctionAt(center, lw)
	stop := geometry.HalfX + JunctionStopInset
	if !inEW {
		stop = geometry.HalfY + JunctionStopInset
	}
	end := stop + 120
	start := end + 360
	if len(points) > 0 {
		start = math.Min(start, center.Sub(points[len(points)-1]).Dot(in))
	}
	end = math.Min(end, start)
	base := RoadLaneOffset(inCoord, inEW, lw, w, inProbe)
	for i := 0; i <= 12; i++ {
		u := float64(i) / 12
		offset := base + (3.5*lw-base)*u*u*(3-2*u)
		point := center.Sub(in.Mul(start + (end-start)*u)).Add(right.Mul(offset))
		if len(points) == 0 || point.Sub(points[len(points)-1]).Dot(in) >= -.01 {
			points = append(points, point)
		}
	}
	outOffset := RoadLaneOffset(outCoord, outEW, lw, w, outProbe)
	for _, point := range QuarterTurnPath(center, in, out, 3.5*lw, outOffset, math.Max(100, lw*2)) {
		if len(points) == 0 || point.Sub(points[len(points)-1]).Dot(in) >= -.01 {
			points = append(points, point)
		}
	}
	return points
}
