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
		b := *ob.Bus
		if distance > 0 {
			position := b.Pos.Add(sim.UnitFromAngle(b.Heading).Mul(distance))
			heading := b.Heading
			if predicted, ok := sim.PredictJunctionBody(ob.JunctionCorridor, distance); ok {
				start, _ := sim.PredictJunctionBody(ob.JunctionCorridor, 0)
				position = predicted.Center.Add(b.Pos.Sub(start.Center))
				heading = predicted.Heading + sim.NormalizeAngle(b.Heading-start.Heading)*math.Max(0, 1-distance/b.WheelBase)
			}
			b = b.AtPose(position, heading)
		}
		return b.BodyParts()
	}
	body := sim.OBB{Center: ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(distance)), Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
	if distance <= 0 {
		return [2]sim.OBB{body}
	}
	if predicted, ok := sim.PredictJunctionBody(ob.JunctionCorridor, distance); ok {
		body.Center, body.Heading = predicted.Center, predicted.Heading
	}
	if ob.JunctionTurn != nil {
		body.Center, body.Heading = ob.JunctionTurn.Predict(distance)
	}
	return [2]sim.OBB{body}
}

// sweepBodies computes the same two-section pose once for all obstacles at
// this sample. The cache lives only for the current scan, so it cannot go stale.
func sweepBodies(bus *sim.Bus, front sim.OBB) [2]sim.OBB {
	bodies := [2]sim.OBB{front}
	if bus.IsArticulated() {
		predicted := bus.AtPose(front.Center, front.Heading)
		tail := predicted.TrailerBody()
		tail.HalfLen += math.Max(0, front.HalfLen-bus.Length/2)
		tail.HalfWid += math.Max(0, front.HalfWid-bus.Width/2)
		bodies[1] = tail
	}
	return bodies
}

func sweepBodiesCollision(bodies [2]sim.OBB, ob VehicleObstacle, distance, lengthMargin, widthMargin float64) bool {
	for _, part := range predictedObstacleBodies(ob, distance) {
		if part.HalfLen <= 0 {
			continue
		}
		part.HalfLen += lengthMargin
		part.HalfWid += widthMargin
		for _, body := range bodies {
			if body.HalfLen <= 0 {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(body, part); hit {
				return true
			}
		}
	}
	return false
}

func obstacleSweepCollision(bus *sim.Bus, front sim.OBB, ob VehicleObstacle, distance, lengthMargin, widthMargin float64) bool {
	return sweepBodiesCollision(sweepBodies(bus, front), ob, distance, lengthMargin, widthMargin)
}
