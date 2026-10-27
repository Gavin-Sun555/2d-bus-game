package traffic

import (
	"fmt"
	"math"
	"math/rand"
	"reflect"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func copyRecoveryCars(cars []*sim.TrafficCar) []*sim.TrafficCar {
	if cars == nil {
		return nil
	}
	out := make([]*sim.TrafficCar, len(cars))
	for i, c := range cars {
		v := *c
		out[i] = &v
	}
	return out
}

// The reference retains the pre-index scan algorithm, with the same corrected
// aligned-queue policy as production. Intentional policy changes are covered by
// TestObliqueBusDoesNotExemptCarFromDeadlockRecovery separately.
func TestRecoveryQueriesMatchFrozenOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(20261026))
	w := &sim.World{}
	tls := &sim.TrafficLightSystem{}
	for i := 0; i < 80; i++ {
		p := sim.Vec2{X: rng.Float64()*10000 - 5000, Y: rng.Float64()*10000 - 5000}
		w.Intersections = append(w.Intersections, p)
		tls.Intersections = append(tls.Intersections, &sim.TrafficIntersection{Center: p, IsMajor: i%2 == 0, GreenDuration: 4 + rng.Float64()*10, YellowDuration: 2, LeftGreenDuration: 3})
	}
	player := sim.NewBus(0, 0, 0)
	player.SetModel(sim.BusArticulated)
	follower := sim.NewBus(400, 400, math.Pi/2)
	follower.SetModel(sim.BusDoubleDecker)
	yielding := sim.NewBus(0, 0, 0)
	yielding.StationMergeRequest = &sim.BusStationMergeRequest{Remaining: 1}
	removed := 0
	for epoch := 0; epoch < 32; epoch++ {
		n := []int{0, 32, 65, 300, 864, 1152}[epoch%6]
		a := &TrafficManager{AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: follower}}}}
		for i := 0; i < n; i++ {
			c := &sim.TrafficCar{ID: n - i, Pos: sim.Vec2{X: float64(rng.Intn(60)-30)*90 + float64(i%3), Y: float64(rng.Intn(60)-30) * 90}, Length: 20 + rng.Float64()*220, Width: 22, Heading: rng.Float64() * 6.28, Speed: float64(rng.Intn(5)), DeadlockTimer: float64(rng.Intn(100))}
			c.DeadlockTracker.Update(0, 0, c.Pos, 0, true)
			if i%13 == 0 {
				c.YieldingToBus = yielding
			}
			a.Cars = append(a.Cars, c)
		}
		b := &TrafficManager{Cars: copyRecoveryCars(a.Cars), AIBusMgr: a.AIBusMgr}
		for frame := 0; frame < 12; frame++ {
			// Public edits, population changes and independently invoked detection must
			// never borrow a stale index from a previous update or different slot order.
			if frame == 3 {
				w.Intersections[0] = sim.Vec2{X: 512, Y: 512}
				tls.Intersections[0].GreenDuration++
			}
			if frame == 4 {
				tls.Intersections[1].Center = tls.Intersections[0].Center
			}
			if frame == 5 {
				tls.Intersections[0], tls.Intersections[1] = tls.Intersections[1], tls.Intersections[0]
			}
			if frame == 6 && len(a.Cars) > 70 {
				a.Cars = a.Cars[:70]
				b.Cars = b.Cars[:70]
			}
			if frame == 7 {
				tls.Intersections[0].ApproachIs4Lane[0] = true
			}
			if frame == 8 && len(a.Cars) > 0 {
				a.Cars[0].Pos.X += 513
				b.Cars[0].Pos.X += 513
			}
			active := len(a.Cars)
			if frame%3 == 0 {
				active /= 2
			}
			before := len(a.Cars)
			a.ResolveDeadlocks(1, player, active, 52, w, tls)
			removed += before - len(a.Cars)
			b.resolveDeadlocksOracle(1, player, active, 52, w, tls)
			if !reflect.DeepEqual(a.Cars, b.Cars) {
				t.Fatalf("epoch%d frame%d diverged", epoch, frame)
			}
		}
	}
	if removed == 0 {
		t.Fatal("oracle never exercised despawning")
	}
}

func TestRecoveryQueryBoundsAndTimeoutInvalidation(t *testing.T) {
	tm := &TrafficManager{}
	for i := 0; i < 100; i++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: sim.Vec2{X: float64(i) * 512}, Length: 48})
	}
	w := &sim.World{Intersections: []sim.Vec2{{X: 130}, {X: 512, Y: 130 - math.SmallestNonzeroFloat64}}}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{{Center: sim.Vec2{X: -1}, GreenDuration: 4}, {Center: sim.Vec2{X: 1}, IsMajor: true, GreenDuration: 12}}}
	if !tm.prepareRecoveryQueries(100, w, tls, false) {
		t.Fatal("index not enabled")
	}
	ids, ok := tm.recoveryCandidates(tm.Cars[0], true)
	if !ok || len(ids) >= 100 {
		t.Fatal("no candidate reduction")
	}
	if tm.recoveryQueries.nearIntersection(sim.Vec2{}) {
		t.Fatal("strict 130 boundary changed")
	}
	tm.Cars[0].Length = 1400
	if !tm.prepareRecoveryQueries(100, w, tls, false) {
		t.Fatal("large valid car fallback")
	}
	ids, _ = tm.recoveryCandidates(tm.Cars[1], true)
	found := false
	for _, id := range ids {
		if id == 0 {
			found = true
		}
	}
	if !found {
		t.Fatal("long car missed")
	}
	for _, count := range []int{100, 65, 100} {
		tls.Intersections[0].GreenDuration++
		tm.prepareRecoveryQueries(count, w, tls, false)
		for i := 0; i < count; i++ {
			p := tm.Cars[i].Pos
			if got, want := tm.recoveryQueries.timeoutAt(i, p), tls.DeadlockTimeoutAt(p); got != want {
				t.Fatalf("slot%d timeout %v/%v", i, got, want)
			}
		}
	}
	tls.Intersections[0], tls.Intersections[1] = tls.Intersections[1], tls.Intersections[0]
	tm.prepareRecoveryQueries(100, w, tls, false)
	if got, want := tm.recoveryQueries.timeoutAt(0, sim.Vec2{}), tls.DeadlockTimeoutAt(sim.Vec2{}); got != want {
		t.Fatal("nearest tie order changed")
	}
	rng := rand.New(rand.NewSource(11))
	for i := 0; i < 1000; i++ {
		pos := sim.Vec2{X: rng.Float64()*20000 - 10000, Y: rng.Float64()*20000 - 10000}
		if got, want := tm.recoveryQueries.timeoutAt(i%100, pos), tls.DeadlockTimeoutAt(pos); got != want {
			t.Fatal("nearest result changed")
		}
	}
	// A malformed public signal slice can fall back partway through a refresh.
	tm.recoveryQueries.timeoutAt(0, sim.Vec2{})
	original := tls.Intersections[1]
	tls.Intersections[0].GreenDuration += 1
	tls.Intersections[1] = nil
	if tm.prepareRecoveryQueries(100, w, tls, false) {
		t.Fatal("nil signal did not fall back")
	}
	tls.Intersections[1] = original
	tm.prepareRecoveryQueries(100, w, tls, false)
	if got, want := tm.recoveryQueries.timeoutAt(0, sim.Vec2{}), tls.DeadlockTimeoutAt(sim.Vec2{}); got != want {
		t.Fatal("partial refresh retained stale timeout")
	}
	tm.prepareRecoveryQueries(100, w, nil, false)
	if got := tm.recoveryQueries.timeoutAt(0, sim.Vec2{}); got != 2*sim.DefaultSignalCycle {
		t.Fatal("removed signal system reused timeout")
	}
	tm.Cars[0].Length = math.Inf(1)
	if tm.prepareRecoveryQueries(100, w, tls, false) {
		t.Fatal("invalid length did not fall back")
	}
	tm.Cars[0].Length = 48
	tm.Cars[0].Pos.X = math.NaN()
	if tm.prepareRecoveryQueries(100, w, tls, false) {
		t.Fatal("invalid position did not fall back")
	}
}

func (tm *TrafficManager) resolveDeadlocksOracle(dt float64, bus *sim.Bus, activeCount int, laneWidth float64, w *sim.World, signals ...*sim.TrafficLightSystem) {
	var tls *sim.TrafficLightSystem
	if len(signals) > 0 {
		tls = signals[0]
	}
	if tm == nil || len(tm.Cars) == 0 {
		return
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]

		// 1. Moving vehicles are never deadlocked
		if math.Abs(car.Speed) >= 3.0 {
			car.DeadlockTimer = 0
			continue
		}
		if yielding := car.YieldingToBus; yielding != nil && ((yielding.StationMergeRequest != nil && yielding.StationMergeRequest.Remaining > 0) || (yielding.LaneMergeRequest != nil && yielding.LaneMergeRequest.Remaining > 0)) {
			car.DeadlockTimer = 0
			continue
		}

		// Detect if car is near an intersection (within 130px of intersection center)
		isNearIntersection := false
		if w != nil {
			for _, inter := range w.Intersections {
				if car.Pos.Distance(inter) < 130.0 {
					isNearIntersection = true
					break
				}
			}
		}

		isJammed := false

		// 2. Check relation to player bus
		if bus != nil && math.Abs(bus.Speed) < 8.0 {
			distToBus := car.Pos.Distance(bus.Pos)
			if distToBus < 180.0 {
				toBus := bus.Pos.Sub(car.Pos)
				cosH := math.Cos(car.Heading)
				sinH := math.Sin(car.Heading)
				fwdBus := toBus.X*cosH + toBus.Y*sinH
				latBus := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
				headDiff := math.Cos(car.Heading - bus.Heading)
				// If car is queued behind player bus heading in the same direction on open road:
				if !isNearIntersection && headDiff > 0.98 && fwdBus > 10.0 && latBus < laneWidth*1.5 {
					car.DeadlockTimer = 0
					continue
				}
				// If car is jammed against player bus (crosswise, oncoming, turning, or trapped against front bumper):
				maxJamDist := (car.Length+bus.Length)*0.5 + 40.0
				if distToBus < maxJamDist && (fwdBus > -15.0 || distToBus < (car.Length+bus.Length)*0.5+15.0) {
					isJammed = true
				}
			}
		}

		// 3. Check against other civilian AI cars
		if !isJammed {
			for j := 0; j < activeCount; j++ {
				if j == i {
					continue
				}
				other := tm.Cars[j]
				if math.Abs(other.Speed) >= 3.0 {
					continue
				}
				dist := car.Pos.Distance(other.Pos)
				maxJamDist := (car.Length+other.Length)*0.5 + 35.0
				if dist < maxJamDist {
					toOther := other.Pos.Sub(car.Pos)
					cosH := math.Cos(car.Heading)
					sinH := math.Sin(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					if fwd > -15.0 || dist < (car.Length+other.Length)*0.5+10.0 {
						isJammed = true
						break
					}
				}
			}
		}

		// 4. Check against AI route buses
		if !isJammed && tm.AIBusMgr != nil {
			for _, ab := range tm.AIBusMgr.Buses {
				if ab.Bus == nil || math.Abs(ab.Bus.Speed) >= 3.0 {
					continue
				}
				dist := car.Pos.Distance(ab.Bus.Pos)
				maxJamDist := (car.Length+ab.Bus.Length)*0.5 + 40.0
				if dist < maxJamDist {
					toOther := ab.Bus.Pos.Sub(car.Pos)
					cosH := math.Cos(car.Heading)
					sinH := math.Sin(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					lat := math.Abs(-toOther.X*sinH + toOther.Y*cosH)
					headDiff := math.Cos(car.Heading - ab.Bus.Heading)
					if !isNearIntersection && headDiff > 0.98 && fwd > 10.0 && lat < laneWidth*1.5 {
						continue
					}
					if fwd > -15.0 || dist < (car.Length+ab.Bus.Length)*0.5+15.0 {
						isJammed = true
						break
					}
				}
			}
		}

		car.DeadlockTimer = car.DeadlockTracker.Update(car.DeadlockTimer, dt, car.Pos, car.Speed, isJammed)
		if car.DeadlockTimer >= tls.DeadlockTimeoutAt(car.Pos) {
			tm.DespawnCar(i)
			break
		}
	}
}

func TestRecoveryFullTrafficMatchesFullScans(t *testing.T) {
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities, sim.MapThemeCustom} {
		for _, density := range []int{1, 4} {
			t.Run(fmt.Sprintf("map%d/density%d", theme, density), func(t *testing.T) {
				sim.SeedSimulation(20261025)
				w := sim.NewWorld(52, theme)
				a := NewTrafficManager(52)
				a.BuildForWorld(w, MaxTrafficCars, 52)
				b := &TrafficManager{disableRecoveryQueries: true, CarSpeedRange: a.CarSpeedRange}
				for _, c := range a.Cars {
					copy := *c
					b.Cars = append(b.Cars, &copy)
				}
				b.RestorePopulation(a.PopulationSnapshot())
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				st := w.Stops[0]
				bus := sim.NewBus(st.Pos.X, st.Pos.Y, st.Heading)
				bus.SetModel(sim.BusArticulated)
				a.laneOrder.build(a.GetActiveCars(density), 52)
				if !a.laneOrder.ready {
					t.Fatal("real map population did not exercise the lane index")
				}
				for frame := 0; frame < 180; frame++ {
					tls.Update(1. / 60)
					sim.SeedSimulation(int64(9000 + frame))
					a.Update(1./60, bus, tls, density, 52, w)
					if frame == 0 && a.recoveryQueries.cells == nil {
						t.Fatal("recovery index not exercised")
					}
					sim.SeedSimulation(int64(9000 + frame))
					b.Update(1./60, bus, tls, density, 52, w)
					if !reflect.DeepEqual(a.Cars, b.Cars) {
						t.Fatalf("frame%d changed live vehicle state", frame)
					}
					a.ResolveCarCollisions(density)
					b.ResolveCarCollisions(density)
					if !reflect.DeepEqual(a.Cars, b.Cars) {
						t.Fatalf("frame%d changed collision result", frame)
					}
				}
			})
		}
	}
}
