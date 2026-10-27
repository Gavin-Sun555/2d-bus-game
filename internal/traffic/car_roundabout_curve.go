package traffic

import (
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func configureRoundaboutExit(car *sim.TrafficCar, w *sim.World, rb *sim.Roundabout, lw, heading float64) {
	forward := sim.UnitFromAngle(heading)
	car.RoundaboutExitHeading = heading
	car.RoundaboutExitEW = math.Abs(forward.X) > .5
	car.RoundaboutExitRC = rb.Center.Y
	if !car.RoundaboutExitEW {
		car.RoundaboutExitRC = rb.Center.X
	}
	lanes := w.GetRoadLanes(car.RoundaboutExitRC, car.RoundaboutExitEW, rb.Center.Add(forward.Mul(rb.Radius+160)))
	lane := math.Min(float64(car.LaneIndex), math.Max(0, float64(min(2, lanes/2-1))))
	car.RoundaboutExitOffset = (lane + .5) * lw
	car.RoundaboutExitLane = int(lane)
	car.RoundaboutExitAngle = sim.NormalizeAngle(heading + math.Asin(math.Min(.8, car.RoundaboutExitOffset/car.RoundaboutTargetR)))
}

func beginRoundaboutEntry(car *sim.TrafficCar, rb sim.Roundabout, lw float64) {
	forward := sim.UnitFromAngle(car.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	lateral := car.Pos.Sub(car.RoundaboutCenter).Dot(right)
	join := sim.RoundaboutEntryJoin(rb, forward, lateral, car.RoundaboutTargetR, lw)
	start, _ := sim.RoundaboutEntryPoint(join, 0)
	if car.Pos.Sub(start).Dot(forward) > 8 {
		beginLegacyRoundaboutEntry(car)
		return
	}
	join.StartShift = car.Pos.Sub(start)
	car.RoundaboutCurve = &sim.CarRoundaboutCurve{Entry: &join}
	measureRoundaboutCurve(car.RoundaboutCurve)

}

// Fallback for vehicles already inside the mouth and adjacent manually driven
// buses, whose future path is not a published parallel-entry corridor.
func beginLegacyRoundaboutEntry(car *sim.TrafficCar) {
	forward := sim.UnitFromAngle(car.Heading)

	theta := car.RoundaboutAngle - .85
	goal := car.RoundaboutCenter.Add(sim.UnitFromAngle(theta).Mul(car.RoundaboutTargetR))
	tangent := sim.Vec2{X: math.Sin(theta), Y: -math.Cos(theta)}
	handle := math.Min(120, math.Max(110, 25+.6*math.Abs(car.RoundaboutRadius-car.RoundaboutTargetR)))
	car.RoundaboutCurve = newRoundaboutCurve(car.Pos, car.Pos.Add(forward.Mul(handle)), goal.Sub(tangent.Mul(math.Min(60, car.RoundaboutTargetR*.3))), goal, false)
}

func beginRoundaboutExit(car *sim.TrafficCar) {
	out := sim.UnitFromAngle(car.RoundaboutExitHeading)
	right := sim.Vec2{X: -out.Y, Y: out.X}
	offset := car.RoundaboutExitOffset
	if offset == 0 {
		offset = car.RoundaboutTargetR * math.Sin(car.RoundaboutExitAngle-car.RoundaboutExitHeading)
	}
	goal := car.RoundaboutCenter.Add(out.Mul(car.RoundaboutOuterRadius + 80)).Add(right.Mul(offset))
	tangent := sim.UnitFromAngle(car.Heading)
	handle := math.Min(120, car.Pos.Distance(goal)*.4)
	car.RoundaboutCurve = newRoundaboutCurve(car.Pos, car.Pos.Add(tangent.Mul(handle)), goal.Sub(out.Mul(handle)), goal, true)
}

func newRoundaboutCurve(start, c1, c2, end sim.Vec2, exiting bool) *sim.CarRoundaboutCurve {
	curve := &sim.CarRoundaboutCurve{Start: start, Control1: c1, Control2: c2, End: end, Exiting: exiting}
	measureRoundaboutCurve(curve)
	return curve
}

func measureRoundaboutCurve(curve *sim.CarRoundaboutCurve) {
	previous, _ := roundaboutCurveAt(curve, 0)
	for i := 1; i < len(curve.ArcLengths); i++ {
		point, _ := roundaboutCurveAt(curve, float64(i)/64)
		curve.ArcLengths[i] = curve.ArcLengths[i-1] + point.Distance(previous)
		previous = point
	}
}

func roundaboutCurveAt(c *sim.CarRoundaboutCurve, u float64) (sim.Vec2, sim.Vec2) {
	if c.Entry != nil {
		return sim.RoundaboutEntryPoint(*c.Entry, u)
	}
	v := 1 - u
	point := c.Start.Mul(v * v * v).Add(c.Control1.Mul(3 * v * v * u)).Add(c.Control2.Mul(3 * v * u * u)).Add(c.End.Mul(u * u * u))
	tangent := c.Control1.Sub(c.Start).Mul(3 * v * v).Add(c.Control2.Sub(c.Control1).Mul(6 * v * u)).Add(c.End.Sub(c.Control2).Mul(3 * u * u))
	return point, tangent
}

func moveCarRoundabout(car *sim.TrafficCar, dt float64) {
	localizeRoundaboutCar(car)
	if car.RoundaboutCurve == nil {
		diff := sim.NormalizeAngle(car.RoundaboutAngle - car.RoundaboutExitAngle)
		if diff < 0 {
			diff += 2 * math.Pi
		}
		if car.RoundaboutTraveled >= 1 && diff <= .65 {
			beginRoundaboutExit(car)
		}
	}
	if car.RoundaboutCurve != nil {
		c := car.RoundaboutCurve
		total := c.ArcLengths[64]
		advance := math.Abs(car.Speed) * dt
		used := math.Min(advance, math.Max(0, total-c.Distance))
		c.Distance += used
		index := sort.Search(len(c.ArcLengths), func(i int) bool { return c.ArcLengths[i] >= c.Distance })
		index = int(sim.Clamp(float64(index), 1, 64))
		fraction := (c.Distance - c.ArcLengths[index-1]) / math.Max(1e-9, c.ArcLengths[index]-c.ArcLengths[index-1])
		point, tangent := roundaboutCurveAt(c, (float64(index-1)+fraction)/64)
		previous := car.RoundaboutAngle
		car.Pos = point
		car.Heading = math.Atan2(tangent.Y, tangent.X)
		car.RoundaboutRadius = point.Distance(car.RoundaboutCenter)
		car.RoundaboutAngle = math.Atan2(point.Y-car.RoundaboutCenter.Y, point.X-car.RoundaboutCenter.X)
		car.RoundaboutTraveled += math.Max(0, sim.NormalizeAngle(previous-car.RoundaboutAngle))
		if c.Distance < total-1e-6 {
			return
		}
		car.RoundaboutCurve = nil
		remaining := (advance - used) / math.Max(math.Abs(car.Speed), 1e-9)
		if c.Exiting {
			car.InRoundabout = false
			car.Heading = car.RoundaboutExitHeading
			car.IsEW = car.RoundaboutExitEW
			car.RoadCenter = car.RoundaboutExitRC
			car.LaneIndex = car.RoundaboutExitLane
			car.RoundaboutCooldown = 6
			car.Pos = car.Pos.Add(sim.UnitFromAngle(car.Heading).Mul(car.Speed * remaining))
			return
		}
		if remaining <= 1e-6 {
			return
		}
		dt = remaining
	}
	// Once joined, hold the circle and its tangent exactly. Radial convergence is
	// retained only for loaded/legacy states that did not start an entry curve.
	previous := car.RoundaboutAngle
	radius := car.RoundaboutRadius
	car.RoundaboutAngle = sim.NormalizeAngle(previous - car.Speed/math.Max(radius, 1)*dt)
	car.RoundaboutTraveled += car.Speed / math.Max(radius, 1) * dt
	change := sim.Clamp(car.RoundaboutTargetR-radius, -math.Abs(car.Speed)*.45*dt, math.Abs(car.Speed)*.45*dt)
	car.RoundaboutRadius += change
	car.Pos = car.RoundaboutCenter.Add(sim.UnitFromAngle(car.RoundaboutAngle).Mul(car.RoundaboutRadius))
	car.Heading = sim.RotateTowards(car.Heading, sim.NormalizeAngle(car.RoundaboutAngle-math.Pi/2), math.Min(5, math.Abs(car.Speed)/35)*dt)
}

// Collision separation changes Pos independently of the navigation cache. Keep
// the next prediction continuous from that actual position, rather than jumping
// back across a neighboring bus to the previously stored circular coordinates.
func localizeRoundaboutCar(car *sim.TrafficCar) {
	if !car.InRoundabout || car.RoundaboutCurve != nil {
		return
	}
	expected := car.RoundaboutCenter.Add(sim.UnitFromAngle(car.RoundaboutAngle).Mul(car.RoundaboutRadius))
	if expected.Distance(car.Pos) <= .1 {
		return
	}
	radial := car.Pos.Sub(car.RoundaboutCenter)
	if radius := radial.Length(); radius > 1 {
		car.RoundaboutRadius = radius
		car.RoundaboutAngle = math.Atan2(radial.Y, radial.X)
	}
}
