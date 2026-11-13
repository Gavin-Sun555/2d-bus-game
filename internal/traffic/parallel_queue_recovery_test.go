package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestParallelStoppedCarsAreNotDeadlocked(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, stagger := range []float64{-20, 0, 20} {
			u := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -u.Y, Y: u.X}
			a := &sim.TrafficCar{ID: 1, Heading: heading, Length: 46, Width: 22}
			b := &sim.TrafficCar{ID: 2, Pos: right.Mul(52).Add(u.Mul(stagger)), Heading: heading, Length: 46, Width: 22}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{a, b}}
			tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { t.Fatal("reset clean parallel queue") }
			for tick := 0; tick < 200; tick++ {
				tm.ResolveDeadlocks(1, nil, 2, 52, nil)
			}
			if len(tm.Cars) != 2 || a.DeadlockTimer != 0 || b.DeadlockTimer != 0 {
				t.Fatalf("parallel queue counted as jam heading%.2f stagger%.0f", heading, stagger)
			}
		}
	}
}

func TestCrossingStoppedCarsStillRecover(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		a := &sim.TrafficCar{ID: 1, Heading: heading, Length: 46, Width: 22}
		b := &sim.TrafficCar{ID: 2, Pos: sim.UnitFromAngle(heading).Mul(30), Heading: heading + math.Pi/2, Length: 46, Width: 22}
		tm := &TrafficManager{Cars: []*sim.TrafficCar{a, b}}
		called := false
		tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { called = true }
		for tick := 0; tick < 100 && !called; tick++ {
			tm.ResolveDeadlocks(1, nil, len(tm.Cars), 52, nil)
		}
		if !called {
			t.Fatal("crossing wedge never recovered")
		}
	}
}

func TestAlignedStoppedCarQueueIsNotReset(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, count := range []int{5, 80} {
			tm := &TrafficManager{}
			for i := 0; i < count; i++ {
				tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i + 1, Pos: sim.UnitFromAngle(heading).Mul(float64(i) * 64), Heading: heading, Length: 46, Width: 22, DeadlockTimer: 73})
			}
			tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { t.Fatal("reset a clean stationary follower") }
			for tick := 0; tick < 90; tick++ {
				tm.ResolveDeadlocks(1, nil, count, 52, nil)
			}
			for _, c := range tm.Cars {
				if c.DeadlockTimer != 0 {
					t.Fatal("counted normal queue time as deadlock")
				}
			}
		}
	}
}

// Captured car147886 was reset because a safe rear follower was only about
// 4.6px behind its bumper. A queue must be recognized in both directions.
func TestTightAlignedQueueDoesNotResetLeadingOrTrailingCar(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, gap := range []float64{1, 4.6, 8, 17} {
			u := sim.UnitFromAngle(h)
			a := &sim.TrafficCar{ID: 1, Heading: h, Length: 46, Width: 22, DeadlockTimer: 74, IsBraking: true}
			b := &sim.TrafficCar{ID: 2, Pos: u.Mul(-(46 + gap)), Heading: h, Length: 46, Width: 22, DeadlockTimer: 74, IsBraking: true}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{a, b}}
			tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { t.Fatal("reset a physically separated queue") }
			tm.ResolveDeadlocks(1./60, nil, 2, 52, nil)
			if a.DeadlockTimer != 0 || b.DeadlockTimer != 0 {
				t.Fatal("retained false deadlock age")
			}
		}
	}
}

func TestAlignedPhysicalOverlapStillCountsAsDeadlock(t *testing.T) {
	a := &sim.TrafficCar{Length: 46, Width: 22}
	b := &sim.TrafficCar{Pos: sim.Vec2{X: 45}, Length: 46, Width: 22}
	if !carDeadlockCorridor(a, b) || !carDeadlockCorridor(b, a) {
		t.Fatal("actual overlap ignored")
	}
}
