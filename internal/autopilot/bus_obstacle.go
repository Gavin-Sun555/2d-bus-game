package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// ObserveBus includes the trailer's complete occupied envelope in headway and
// lane reservation, while narrow-phase collision uses the two physical bodies.
func ObserveBus(b *sim.Bus) VehicleObstacle {
	b.PreparePosePredictions()
	body := b.BodyEnvelope()
	return VehicleObstacle{Bus: b, Pos: body.Center, Heading: body.Heading, Length: body.HalfLen * 2, Width: body.HalfWid * 2, Speed: b.Speed, YieldingTo: b.YieldingToBus, JunctionCorridor: b.JunctionCorridor}
}

// Predictions use the same reference point as their published path. Bus
// corridors track the powered section, whereas ObserveBus's broad-phase
// envelope is centered farther back for an articulated vehicle. Placing that
// whole envelope at the front-section center invents a forward protrusion.
func predictedObstacleBodies(ob VehicleObstacle, distance float64) [2]sim.OBB {
	if ob.Bus != nil {
		b := ob.Bus
		if distance > 0 {
			position, heading := predictedObstacleBusPose(ob, distance)
			return b.BodyPartsAtPose(position, heading)
		}
		return b.BodyParts()
	}
	body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
	if distance <= 0 {
		return [2]sim.OBB{body}
	}
	if ob.JunctionTurn != nil {
		body.Center, body.Heading = ob.JunctionTurn.Predict(distance)
	} else if predicted, ok := sim.PredictJunctionBody(ob.JunctionCorridor, distance); ok {
		body.Center, body.Heading = predicted.Center, predicted.Heading
	} else {
		body.Center = ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(distance))
	}
	return [2]sim.OBB{body}
}

func predictedObstacleBusPose(ob VehicleObstacle, distance float64) (sim.Vec2, float64) {
	b := ob.Bus
	if predicted, ok := sim.PredictJunctionBody(ob.JunctionCorridor, distance); ok {
		start, _ := sim.PredictJunctionBody(ob.JunctionCorridor, 0)
		position := predicted.Center.Add(b.Pos.Sub(start.Center))
		heading := predicted.Heading + sim.NormalizeAngle(b.Heading-start.Heading)*math.Max(0, 1-distance/b.WheelBase)
		return position, heading
	}
	return b.Pos.Add(sim.UnitFromAngle(b.Heading).Mul(distance)), b.Heading
}

// sweepBodies computes the same two-section pose once for all obstacles at
// this sample. The cache lives only for the current scan, so it cannot go stale.
func sweepBodies(bus *sim.Bus, front sim.OBB) [2]sim.OBB {
	bodies := [2]sim.OBB{front}
	if bus.IsArticulated() {
		tail := bus.TrailerBodyAtPose(front.Center, front.Heading)
		tail.HalfLen += math.Max(0, front.HalfLen-bus.Length/2)
		tail.HalfWid += math.Max(0, front.HalfWid-bus.Width/2)
		bodies[1] = tail
	}
	return bodies
}

func obbHits(a, b sim.OBB) bool {
	if a.HalfLen >= 0 && a.HalfWid >= 0 && b.HalfLen >= 0 && b.HalfWid >= 0 {
		r := a.HalfLen + a.HalfWid + b.HalfLen + b.HalfWid
		if math.Abs(a.Center.X-b.Center.X) > r || math.Abs(a.Center.Y-b.Center.Y) > r {
			return false
		}
	}
	hit, _, _ := sim.CheckOBBCollision(a, b)
	return hit
}

// Reject only geometry outside conservative bounds of BOTH the target pose
// and the source pose (articulation can reject a prediction and fall back).
// No traffic decision or articulation result is approximated or cached.
func predictedBusSeparated(bodies [2]sim.OBB, b *sim.Bus, position sim.Vec2, heading, lengthMargin, widthMargin float64) bool {
	if !(b.Length > 0 && b.Width >= 0 && b.Length <= 1e6 && b.Width <= 1e6 && math.Abs(b.TailHeading()) <= 1e6 && lengthMargin >= 0 && widthMargin >= 0 && math.Abs(heading) <= 1e6 && math.Abs(b.Heading) <= 1e6) {
		return false
	}
	padding := lengthMargin + widthMargin + 2
	frontReach := (b.Length+b.Width)/2 + padding
	hitch := position
	if b.IsArticulated() {
		hitch = position.Sub(sim.UnitFromAngle(heading).Mul(b.Length / 2))
	}
	tailReach := b.Length + b.Width/2 + padding
	for _, own := range bodies {
		if own.HalfLen <= 0 {
			continue
		}
		if !(own.HalfWid >= 0) {
			return false
		}
		reach := own.HalfLen + own.HalfWid
		separated := func(center sim.Vec2, extent float64) bool {
			d := own.Center.Sub(center)
			return math.Abs(d.X) > reach+extent || math.Abs(d.Y) > reach+extent
		}
		if !separated(position, frontReach) || (b.IsArticulated() && (!separated(hitch, tailReach) || !separated(b.Pos, 1.5*b.Length+b.Width/2+padding))) {
			return false
		}
	}
	return true
}

func sweepBodiesCollision(bodies [2]sim.OBB, ob VehicleObstacle, distance, lengthMargin, widthMargin float64) bool {
	var parts [2]sim.OBB
	if ob.Bus != nil && distance > 0 {
		position, heading := predictedObstacleBusPose(ob, distance)
		if predictedBusSeparated(bodies, ob.Bus, position, heading, lengthMargin, widthMargin) {
			return false
		}
		parts = ob.Bus.BodyPartsAtPose(position, heading)
	} else {
		parts = predictedObstacleBodies(ob, distance)
	}
	for _, part := range parts {
		if part.HalfLen <= 0 {
			continue
		}
		part.HalfLen += lengthMargin
		part.HalfWid += widthMargin
		for _, body := range bodies {
			if body.HalfLen <= 0 {
				continue
			}
			if obbHits(body, part) {
				return true
			}
		}
	}
	return false
}

type sweepSample struct {
	front, tail sim.OBB
	tailReady   bool
}

func (s *sweepSample) collides(bus *sim.Bus, obstacle VehicleObstacle, distance, lengthMargin, widthMargin float64) bool {
	if !bus.IsArticulated() {
		return sweepBodiesCollision([2]sim.OBB{s.front}, obstacle, distance, lengthMargin, widthMargin)
	}
	if obstacle.Bus != nil && distance > 0 {
		b := obstacle.Bus
		position, heading := predictedObstacleBusPose(obstacle, distance)
		bReach := b.OverallLength() - b.Length/2 + b.Width/2 + lengthMargin + widthMargin + 2
		sReach := bus.Length + s.front.HalfLen + s.front.HalfWid + bReach
		dpFront := position.Sub(s.front.Center)
		nearFront := !(math.Abs(dpFront.X) > sReach || math.Abs(dpFront.Y) > sReach)
		nearSource := false
		if !nearFront && s.mayReject(bus) {
			dpSource := position.Sub(bus.Pos)
			nearSource = !(math.Abs(dpSource.X) > sReach || math.Abs(dpSource.Y) > sReach)
		}
		if !nearFront && !nearSource {
			if !s.collidesParts(bus, b.BodyParts(), lengthMargin, widthMargin) {
				return false
			}
		}
		if s.separatedFromPredictedBusWithSource(bus, b, position, heading, lengthMargin, widthMargin) {
			return false
		}
		return s.collidesParts(bus, b.BodyPartsAtPose(position, heading), lengthMargin, widthMargin)
	}
	return s.collidesParts(bus, predictedObstacleBodies(obstacle, distance), lengthMargin, widthMargin)
}

func (s *sweepSample) separatedFromPredictedBus(b *sim.Bus, position sim.Vec2, heading, lengthMargin, widthMargin float64) bool {
	return s.separatedFromPredictedBusWithSource(nil, b, position, heading, lengthMargin, widthMargin)
}

// With both own sections known, test the other powered section and its
// conservative trailer reach before integrating its trailer. A rejected
// prediction can also put the powered section back at its starting pose.
func (s *sweepSample) separatedFromPredictedBusWithSource(bus, b *sim.Bus, position sim.Vec2, heading, lengthMargin, widthMargin float64) bool {
	if !s.tailReady && bus == nil {
		return false
	}
	if s.tailReady && predictedBusSeparated([2]sim.OBB{s.front, s.tail}, b, position, heading, lengthMargin, widthMargin) {
		return true
	}
	if !b.IsArticulated() {
		return false
	}
	target := sim.OBB{Center: position, Heading: heading, HalfLen: b.Length/2 + lengthMargin, HalfWid: b.Width/2 + widthMargin}
	if obbHits(s.front, target) {
		return false
	}
	other := sweepSample{front: target}
	if other.tailMayCollide(b, s.front) {
		return false
	}
	if s.tailReady {
		if obbHits(s.tail, target) || other.tailMayCollide(b, s.tail) {
			return false
		}
	} else {
		if s.tailMayCollide(bus, target) {
			return false
		}
		otherHitch := position.Sub(sim.UnitFromAngle(heading).Mul(b.Length / 2))
		otherTailExtent := b.Length + b.Width/2 + math.Max(lengthMargin, widthMargin) + 1
		otherTailBound := sim.OBB{Center: otherHitch, Heading: heading, HalfLen: otherTailExtent, HalfWid: otherTailExtent}
		if s.tailMayCollide(bus, otherTailBound) {
			return false
		}
	}
	if !other.mayReject(b) {
		return true
	}
	source := target
	source.Center, source.Heading = b.Pos, b.Heading
	if obbHits(s.front, source) {
		return false
	}
	if s.tailReady {
		return !obbHits(s.tail, source)
	}
	return !s.tailMayCollide(bus, source)
}

func (s *sweepSample) mayReject(bus *sim.Bus) bool {
	if s.tailReady {
		d := s.tail.Center.Sub(s.front.Center)
		limit := bus.Length + 2
		return math.Abs(d.X) > limit || math.Abs(d.Y) > limit
	}
	if !(bus.Length > 0 && math.Abs(s.front.Heading) <= 1e6 && math.Abs(bus.Heading) <= 1e6 && math.Abs(bus.TailHeading()) <= 1e6) {
		return true
	}
	initDiff := math.Abs(sim.NormalizeAngle(bus.TailHeading() - s.front.Heading))
	if initDiff > sim.MaxArticulationAngle {
		return true
	}
	u := sim.UnitFromAngle(s.front.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	delta := s.front.Center.Sub(bus.Pos)
	turnAbs := math.Abs(sim.NormalizeAngle(s.front.Heading - bus.Heading))
	if delta.X == 0 && delta.Y == 0 && turnAbs == 0 {
		return initDiff > 0.85
	}
	rot := 0.5 * bus.Length * turnAbs
	deltaU := delta.Dot(u)
	deltaR := math.Abs(delta.Dot(r))
	if !(deltaU > rot+1 && deltaU <= 4000 && deltaR <= 4000) {
		return true
	}
	moveBound := (deltaR + rot) / (deltaU - rot)
	return math.Max(initDiff, moveBound)+turnAbs > 0.85
}

func (s *sweepSample) collidesParts(bus *sim.Bus, parts [2]sim.OBB, lengthMargin, widthMargin float64) bool {
	for _, other := range parts {
		if other.HalfLen <= 0 {
			continue
		}
		other.HalfLen += lengthMargin
		other.HalfWid += widthMargin
		if obbHits(s.front, other) {
			return true
		}
		if !s.tailReady {
			if !s.tailMayCollide(bus, other) {
				continue
			}
			s.tail = sweepBodies(bus, s.front)[1]
			s.tailReady = true
		}
		if obbHits(s.tail, other) {
			return true
		}
	}
	return false
}

func (s *sweepSample) tailMayCollide(bus *sim.Bus, other sim.OBB) bool {
	reach := bus.Length + s.front.HalfLen + s.front.HalfWid + other.HalfLen + other.HalfWid + 1
	target := other.Center.Sub(s.front.Center)
	couldHitTarget := !(math.Abs(target.X) > reach || math.Abs(target.Y) > reach)
	if couldHitTarget && bus.Length > 0 && math.Abs(s.front.Heading) <= 1e6 && math.Abs(other.Heading) <= 1e6 && math.Abs(bus.Heading) <= 1e6 && math.Abs(bus.TailHeading()) <= 1e6 {
		u := sim.UnitFromAngle(s.front.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		sinAlphaMax := 0.86602541
		initDiff := math.Abs(sim.NormalizeAngle(bus.TailHeading() - s.front.Heading))
		if initDiff <= sim.MaxArticulationAngle {
			delta := s.front.Center.Sub(bus.Pos)
			turnAbs := math.Abs(sim.NormalizeAngle(s.front.Heading - bus.Heading))
			rot := 0.5 * bus.Length * turnAbs
			deltaU := delta.Dot(u)
			deltaR := math.Abs(delta.Dot(r))
			if deltaU > rot+1 && deltaU <= 4000 && deltaR <= 4000 {
				moveBound := (deltaR + rot) / (deltaU - rot)
				bound := math.Max(initDiff, moveBound) + 1e-4
				if bound < sinAlphaMax {
					sinAlphaMax = bound
				}
			} else if delta.X == 0 && delta.Y == 0 && turnAbs == 0 {
				if initDiff+1e-4 < sinAlphaMax {
					sinAlphaMax = initDiff + 1e-4
				}
			}
		}
		uOther := sim.UnitFromAngle(other.Heading)
		dotU := math.Abs(uOther.Dot(u))
		dotR := math.Abs(uOther.Dot(r))
		otherAlong := dotU*other.HalfLen + dotR*other.HalfWid
		otherAcross := dotR*other.HalfLen + dotU*other.HalfWid
		tailLatReach := s.front.HalfWid + (0.5*bus.Length+s.front.HalfLen)*sinAlphaMax + otherAcross + 1
		tailLongReach := 0.125*bus.Length + s.front.HalfLen + s.front.HalfWid*sinAlphaMax + otherAlong + 1
		if math.Abs(target.Dot(r)) > tailLatReach || math.Abs(target.Dot(u)+0.875*bus.Length) > tailLongReach {
			couldHitTarget = false
		}
	}
	if couldHitTarget {
		return true
	}
	if !s.mayReject(bus) {
		return false
	}
	source := other.Center.Sub(bus.Pos)
	if math.Abs(source.X) > reach || math.Abs(source.Y) > reach {
		return false
	}
	sourceTail := bus.TrailerBody()
	sourceTail.HalfLen += math.Max(0, s.front.HalfLen-bus.Length/2)
	sourceTail.HalfWid += math.Max(0, s.front.HalfWid-bus.Width/2)
	return obbHits(sourceTail, other)
}

func obstacleSweepCollision(bus *sim.Bus, front sim.OBB, ob VehicleObstacle, distance, lengthMargin, widthMargin float64) bool {
	return sweepBodiesCollision(sweepBodies(bus, front), ob, distance, lengthMargin, widthMargin)
}
