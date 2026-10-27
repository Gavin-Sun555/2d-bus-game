package sim

import "math"

// WideLeftTurnPath joins the approach directly to the outgoing travel lane.
// Complete the lateral transition inside the junction, rather than exiting in
// the median lane and then cutting across a queue outside the junction.
func WideLeftTurnPath(center, in, out Vec2, inOffset, outOffset float64, geometry JunctionGeometry) []Vec2 {
	rightIn := Vec2{X: -in.Y, Y: in.X}
	rightOut := Vec2{X: -out.Y, Y: out.X}
	entry := math.Abs(in.X)*geometry.HalfX + math.Abs(in.Y)*geometry.HalfY
	exit := math.Abs(out.X)*geometry.HalfX + math.Abs(out.Y)*geometry.HalfY
	p0 := center.Sub(in.Mul(entry)).Add(rightIn.Mul(inOffset))
	p3 := center.Add(out.Mul(exit)).Add(rightOut.Mul(outOffset))
	// Shorter handles keep the middle of the curve on its own side of the
	// opposing-left-turn diagonal. A quarter circle between curb lanes cuts
	// across that diagonal; merely changing its exit offset is unsafe.
	handle := .7 * math.Min(entry-inOffset, exit-outOffset)
	p1 := p0.Add(in.Mul(handle))
	p2 := p3.Sub(out.Mul(handle))
	const steps = 40
	points := make([]Vec2, 0, steps+1)
	for i := 0; i <= steps; i++ {
		u := float64(i) / steps
		v := 1 - u
		points = append(points, p0.Mul(v*v*v).Add(p1.Mul(3*v*v*u)).Add(p2.Mul(3*v*u*u)).Add(p3.Mul(u*u*u)))
	}
	// A straight run-out stabilizes pursuit before road-lane policies resume.
	return append(points, p3.Add(out.Mul(100)))
}

func appendWideLeftLaneReturn(points []Vec2, center, out Vec2, from, to, available float64, geometry JunctionGeometry) []Vec2 {
	if math.Abs(to-from) < 1 {
		return points
	}
	right := Vec2{X: -out.Y, Y: out.X}
	mouth := math.Abs(out.X)*geometry.HalfX + math.Abs(out.Y)*geometry.HalfY
	// The junction reservation ends before this taper starts, allowing the two
	// left-turn lanes to clear in parallel. The controller checks crossed gaps
	// before following this return and can retain its physical lane while blocked.
	start := mouth + 240
	length := math.Min(800, available*.88-start)
	if length <= 0 {
		return points
	}
	for i := 0; i <= 16; i++ {
		u := float64(i) / 16
		offset := from + (to-from)*u*u*(3-2*u)
		points = append(points, center.Add(out.Mul(start+length*u)).Add(right.Mul(offset)))
	}
	return points
}

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
