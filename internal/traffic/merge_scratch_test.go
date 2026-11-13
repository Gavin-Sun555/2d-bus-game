package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand/v2"
	"reflect"
	"testing"
)

func TestLaneMergeScratchMatchesUncachedAndKeepsPublishedPaths(t *testing.T) {
	rng := rand.New(rand.NewPCG(18, 93))
	tm := &TrafficManager{}
	for i := 0; i < 800; i++ {
		car := &sim.TrafficCar{Pos: sim.Vec2{X: float64(i) * 11, Y: 26}, Length: 48, Width: 22, Speed: 80, Heading: 0}
		other := &sim.TrafficCar{Pos: car.Pos.Add(sim.Vec2{X: 80 + rng.Float64()*400, Y: 52 + rng.Float64()*100}), Length: 48, Width: 22, Speed: 70, Heading: 0}
		turn := &sim.CarJunctionTurn{LaneChange: true, ExitHeading: 0, Points: []sim.Vec2{other.Pos, other.Pos.Add(sim.Vec2{X: 100, Y: -52}), other.Pos.Add(sim.Vec2{X: 400, Y: -52})}}
		other.JunctionTurn = turn
		original := append([]sim.Vec2(nil), turn.Points...)
		tm.Cars = []*sim.TrafficCar{car, other}
		f := carFrame{car: car, laneWidth: 52, activeCount: 2}
		direction := 1.
		if i%2 == 0 {
			direction = -1
		}
		want := tm.laneMergeClearUncached(f, sim.Vec2{X: 1}, direction)
		got := tm.laneMergeClear(f, sim.Vec2{X: 1}, direction)
		if got != want {
			t.Fatalf("case %d: cached %v uncached %v", i, got, want)
		}
		if !reflect.DeepEqual(original, turn.Points) {
			t.Fatal("published turn points modified")
		}
	}
	if cap(tm.mergeOwnBodies) == 0 || cap(tm.mergeOtherBodies) == 0 {
		t.Fatal("scratch path was not exercised")
	}
}

// Frozen allocation-heavy implementation is an independent geometry oracle.
func (tm *TrafficManager) laneMergeClearUncached(f carFrame, forward sim.Vec2, direction float64) bool {
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	target := f.car.Pos.Add(right.Mul(f.laneWidth * direction))
	clear := func(pos sim.Vec2, length, width, speed float64) bool {
		delta := pos.Sub(target)
		along := delta.Dot(forward)
		return math.Abs(delta.Dot(right)) > (width+f.car.Width)/2+4 || along > (length+f.car.Length)/2+math.Max(35, f.car.Speed*.8) || along < -(length+f.car.Length)/2-math.Max(40, speed*1.2)
	}
	active := tm.carsForFrame(f)
	var ours []sim.OBB
	for _, other := range active {
		if other != f.car && !clear(other.Pos, other.Length, other.Width, other.Speed) {
			return false
		}
		if other == f.car || other.JunctionTurn == nil || !other.JunctionTurn.LaneChange {
			continue
		}
		if other.Pos.Distance(f.car.Pos) > 600 {
			continue
		}
		// A follower paused behind us must not reserve our escape into the
		// empty lane ahead. Its normal headway keeps it behind our rear bumper.
		if math.Cos(other.JunctionTurn.ExitHeading-f.car.Heading) > .98 && other.Pos.Sub(f.car.Pos).Dot(forward) < -(f.car.Length+other.Length)/2-12 {
			continue
		}
		if ours == nil {
			length := carLaneChangeLength(f.car, f.laneWidth)
			points := make([]sim.Vec2, 33)
			for i := range points {
				t := float64(i) / 32
				ease := t * t * t * (10 - 15*t + 6*t*t)
				points[i] = f.car.Pos.Add(forward.Mul(length * t)).Add(right.Mul(f.laneWidth * direction * ease))
			}
			ours = sim.AppendPathBodies(nil, points, f.car.Length, f.car.Width)
		}
		turn := other.JunctionTurn
		remaining := append([]sim.Vec2{other.Pos}, turn.Points[turn.Segment+1:]...)
		theirs := sim.AppendPathBodies(nil, remaining, other.Length, other.Width)
		if !sim.CorridorsSeparated(ours, theirs) {
			return false
		}

	}
	for _, bus := range f.buses {
		if bus != nil && !clear(bus.Pos, bus.Length, bus.Width, bus.Speed) {
			return false
		}
	}
	return true
}
