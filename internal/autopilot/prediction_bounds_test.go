package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand/v2"
	"testing"
)

func TestPredictionBoundsPreserveCollisionAndRejectedPoses(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 27))
	rejected, separated, narrowSeparated := 0, 0, 0
	for i := 0; i < 12000; i++ {
		b := sim.NewBus(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*2*math.Pi-math.Pi)
		b.SetModel(sim.BusArticulated)
		if i%4 == 1 {
			b.SetModel(sim.BusStandard)
		}
		if i%4 == 2 {
			b.SetModel(sim.BusDoubleDecker)
		}
		b.TrailerHeading = b.Heading + (rng.Float64()*2-1)*sim.MaxArticulationAngle
		if i%3 == 0 {
			b.JunctionCorridor = &sim.JunctionCorridor{Remaining: 1, Bodies: []sim.OBB{
				{Center: b.Pos, Heading: b.Heading},
				{Center: b.Pos.Add(sim.UnitFromAngle(b.Heading).Mul(30)), Heading: b.Heading},
				{Center: b.Pos.Add(sim.UnitFromAngle(b.Heading + math.Pi/2).Mul(80)), Heading: b.Heading + math.Pi/2},
			}}
		}
		ob := ObserveBus(b)
		ob.Speed = rng.Float64() * 1000
		distance := rng.Float64() * 700
		own := [2]sim.OBB{{Center: sim.Vec2{X: rng.Float64()*1400 - 700, Y: rng.Float64()*1400 - 700}, Heading: rng.Float64() * 2 * math.Pi, HalfLen: 48, HalfWid: 14}}
		if i%2 == 0 {
			own[1] = own[0]
			own[1].Center.X -= 96
		}
		lm, wm := rng.Float64()*12, rng.Float64()*5
		want := false
		for _, part := range predictedObstacleBodies(ob, distance) {
			if part.HalfLen <= 0 {
				continue
			}
			part.HalfLen += lm
			part.HalfWid += wm
			for _, body := range own {
				if body.HalfLen > 0 && obbHits(body, part) {
					want = true
				}
			}
		}
		p, h := predictedObstacleBusPose(ob, distance)
		pose := b.AtPose(p, h)
		if pose.Pos == b.Pos && p != b.Pos {
			rejected++
		}
		if predictedBusSeparated(own, b, p, h, lm, wm) {
			separated++
		}
		if own[1].HalfLen > 0 {
			sample := sweepSample{front: own[0], tail: own[1], tailReady: true}
			if sample.separatedFromPredictedBus(b, p, h, lm, wm) {
				if want {
					t.Fatalf("case %d: section bounds culled an actual collision", i)
				}
				if !predictedBusSeparated(own, b, p, h, lm, wm) {
					narrowSeparated++
				}
			}
		}
		if got := sweepBodiesCollision(own, ob, distance, lm, wm); got != want {
			t.Fatalf("case %d: collision %v, want %v", i, got, want)
		}
	}
	if rejected == 0 || separated == 0 || narrowSeparated == 0 {
		t.Fatalf("missing coverage: rejected %d separated %d narrow bounds %d", rejected, separated, narrowSeparated)
	}
}
