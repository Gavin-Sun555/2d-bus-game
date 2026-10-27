package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func parallelRoundaboutEntry(f carFrame, rb sim.Roundabout, pos sim.Vec2, heading, length, width float64, bus bool) bool {
	incoming := roundaboutApproachDirection(f.car)
	if f.car.InRoundabout {
		incoming = sim.UnitFromAngle(f.car.RoundaboutExitHeading)
	}
	a := sim.OBB{Center: f.car.Pos, Heading: f.car.Heading, HalfLen: f.car.Length / 2, HalfWid: f.car.Width / 2}
	b := sim.OBB{Center: pos, Heading: heading, HalfLen: length / 2, HalfWid: width / 2}
	return sim.SeparateRoundaboutEntries(rb, incoming, f.laneWidth, a, false, b, bus)
}

// Concentric travel does not become a conflict merely because centers are less
// than the old 60/85px thresholds apart. Exiting vehicles keep their priority.
func separateCirculatingBodies(car *sim.TrafficCar, pos sim.Vec2, heading, length, width float64, exiting bool) bool {
	if exiting || (car.RoundaboutCurve != nil && car.RoundaboutCurve.Exiting) {
		return false
	}
	radial := car.Pos.Sub(car.RoundaboutCenter)
	other := pos.Sub(car.RoundaboutCenter)
	if math.Abs(radial.Length()-car.RoundaboutTargetR) > 2 || radial.Length() < 1 || other.Length() < 1 {
		return false
	}
	ownTangent := sim.Vec2{X: radial.Y, Y: -radial.X}.Normalize()
	if ownTangent.Dot(sim.UnitFromAngle(car.Heading)) < .98 {
		return false
	}
	tangent := sim.Vec2{X: other.Y, Y: -other.X}.Normalize()
	if tangent.Dot(sim.UnitFromAngle(heading)) < .98 {
		return false
	}
	// The rectangle's radial projection includes its turning corners.
	extent := func(r sim.Vec2, h, l, w float64) float64 {
		u := r.Normalize()
		f := sim.UnitFromAngle(h)
		return math.Abs(f.Dot(u))*l/2 + math.Abs(sim.Vec2{X: -f.Y, Y: f.X}.Dot(u))*w/2
	}
	return math.Abs(radial.Length()-other.Length()) > extent(radial, car.Heading, car.Length, car.Width)+extent(other, heading, length, width)+6
}

func parallelRoundaboutBus(f carFrame, rb sim.Roundabout, bus *sim.Bus) bool {
	return bus != nil && bus.RoundaboutEntry != nil && bus.RoundaboutEntry.Remaining > 0 && bus.RoundaboutEntry.Center == rb.Center && parallelRoundaboutEntry(f, rb, bus.Pos, bus.Heading, bus.Length, bus.Width, true)
}

// Follow the actual curve, including its exit. Fixed center-distance tests
// otherwise turn two nonintersecting exit lanes into a mutual stop.
func roundaboutCarPathClear(car *sim.TrafficCar, pos sim.Vec2, heading, length, width float64) bool {
	// This is a swept-body clearance test, not a following-distance rule.
	// Expanding both bodies makes a safely separated entrance queue hold a
	// circulating car forever. Following limits below still protect leaders
	// whose bodies actually intersect the remaining path.
	other := sim.OBB{Center: pos, Heading: heading, HalfLen: length / 2, HalfWid: width / 2}
	predicted := *car
	if car.RoundaboutCurve != nil {
		curve := *car.RoundaboutCurve
		predicted.RoundaboutCurve = &curve
	}
	predicted.Speed = 90
	for distance := 0.; distance <= 120; distance += 6 {
		body := sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			return false
		}
		moveCarRoundabout(&predicted, 6.0/90)
	}
	return true
}

// A stopped articulated bus still occupies the tail's arc. A clear front
// section alone cannot exempt a circulating car from following clearance.
func roundaboutBusPathClear(car *sim.TrafficCar, bus *sim.Bus) bool {
	for _, part := range bus.BodyParts() {
		if part.HalfLen > 0 && !roundaboutCarPathClear(car, part.Center, part.Heading, part.HalfLen*2, part.HalfWid*2) {
			return false
		}
	}
	return true
}
