package autopilot

import (
	"math"
	"math/rand"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Deliberately recompute the complete geometry for every obstacle/sample.
// These oracles retain the old motion assumptions and collision entry points,
// independent of the controller's newly shared sample/BodyParts storage.
func fullControlPrediction(f autopilotFrame, steer float64) float64 {
	speed := math.Max(20, f.speedAbs)
	horizon := math.Min(1.2, .20+speed/180)
	limit := math.Inf(1)
	for _, ob := range f.obstacles {
		reach := speed*horizon + math.Abs(ob.Speed)*horizon + (f.bus.Length+ob.Length)/2 + 20
		d := ob.Pos.Sub(f.bus.Pos)
		if d.Dot(d) > reach*reach {
			continue
		}
		pos, heading, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
		for elapsed := .04; elapsed <= horizon; elapsed += .04 {
			rate := f.bus.SteerTurnSpeed
			if steer == 0 {
				rate = f.bus.SteerReturnSpd
			}
			steering += sim.Clamp(steer*f.bus.MaxSteerAngle-steering, -rate*.04, rate*.04)
			heading += speed / f.bus.WheelBase * math.Tan(steering) * .04
			pos = pos.Add(sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}.Mul(speed * .04))
			front := sim.OBB{Center: pos, Heading: heading, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
			hit := false
			for _, body := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen++
				body.HalfWid++
				if h, _, _ := sim.CheckBusAtBody(f.bus, front, body); h {
					hit = true
					break
				}
			}
			if hit {
				limit = math.Min(limit, math.Sqrt(2*180*math.Max(0, speed*elapsed-18)))
				break
			}
		}
	}
	return limit
}

func fullArticulatedPrediction(f autopilotFrame, steer float64, straight bool) float64 {
	if !f.bus.IsArticulated() {
		return math.Inf(1)
	}
	copy := *f.bus
	speed := math.Max(16, f.speedAbs)
	for tick := 1; tick <= 16; tick++ {
		elapsed := float64(tick) * .05
		wheel := copy.SteeringAngle + sim.Clamp(steer*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
		angle := copy.Heading + speed/copy.WheelBase*math.Tan(wheel)*.05
		pos := copy.Pos.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(speed * .05))
		copy = copy.AtPose(pos, angle)
		copy.SteeringAngle = wheel
		collision := false
		if tick%4 == 0 && (straight || math.Abs(wheel) > .02 || math.Abs(sim.NormalizeAngle(copy.Heading-copy.TailHeading())) > .03) {
			collision = copy.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier
		}
		for _, ob := range f.obstacles {
			delta := ob.Pos.Sub(copy.Pos)
			reach := copy.OverallLength() + ob.Length + 100
			if delta.Dot(delta) > reach*reach {
				continue
			}
			hit := false
			for _, body := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen++
				body.HalfWid++
				if touching, _, _ := copy.CollidesBody(body); touching {
					hit = true
					break
				}
			}
			if hit {
				collision = true
				break
			}
		}
		if collision {
			return math.Sqrt(2 * 180 * math.Max(0, speed*elapsed-20))
		}
	}
	return math.Inf(1)
}

func TestReusedPredictionsMatchFullObstacleScans(t *testing.T) {
	rng := rand.New(rand.NewSource(41509))
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -10000}, P2: sim.Vec2{X: 10000}, Lanes: 8}}}
	ad := NewAutopilotDriver()
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
		for i := 0; i < 100; i++ {
			bus := sim.NewBus(rng.Float64()*100-50, rng.Float64()*100-50, rng.Float64()*math.Pi*2)
			bus.SetModel(model)
			bus.TrailerHeading = bus.Heading + rng.Float64() - .5
			bus.Speed = rng.Float64() * 350
			bus.SteeringAngle = rng.Float64()*.4 - .2
			bus.PreparePosePredictions()
			f := autopilotFrame{bus: bus, world: w, speedAbs: bus.Speed, laneWidth: 52}
			for j := 0; j < 20; j++ {
				p := bus.Pos.Add(sim.Vec2{X: rng.Float64()*700 - 350, Y: rng.Float64()*700 - 350})
				if j%5 == 0 {
					other := sim.NewBus(p.X, p.Y, rng.Float64()*math.Pi*2)
					other.SetModel(sim.BusArticulated)
					other.Speed = rng.Float64() * 100
					other.TrailerHeading = other.Heading - .3
					f.obstacles = append(f.obstacles, ObserveBus(other))
				} else {
					f.obstacles = append(f.obstacles, VehicleObstacle{Pos: p, Heading: rng.Float64() * math.Pi * 2, Speed: rng.Float64()*200 - 20, Length: 46, Width: 22})
				}
			}
			steer := rng.Float64()*2 - 1
			if i%3 == 0 {
				steer = 0
			}
			want := fullControlPrediction(f, steer)
			if got := ad.controlSweptSpeed(f, steer); got != want {
				t.Fatalf("model%d sample%d control got%g want%g", model, i, got, want)
			}
			for _, straight := range []bool{false, true} {
				want := fullArticulatedPrediction(f, steer, straight)
				if got := ad.articulatedSweptSpeed(f, steer, straight); got != want {
					t.Fatalf("model%d sample%d articulated got%g want%g", model, i, got, want)
				}
			}
		}
	}
}
