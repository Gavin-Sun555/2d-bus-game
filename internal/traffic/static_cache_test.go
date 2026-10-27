package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"reflect"
	"testing"
)

func TestStaticCachesPreserveTrafficFrames(t *testing.T) {
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities, sim.MapThemeCustom} {
		t.Run(fmt.Sprintf("map%d", theme), func(t *testing.T) {
			sim.SeedSimulation(907)
			w := sim.NewWorld(52, theme)
			a := NewTrafficManager(52)
			a.BuildForWorld(w, MaxTrafficCars, 52)
			b := &TrafficManager{disableJunctionIndex: true, disableJunctionReuse: true, disableJunctionBroadPhase: true, CarSpeedRange: a.CarSpeedRange}
			for _, car := range a.Cars {
				copy := *car
				b.Cars = append(b.Cars, &copy)
			}
			b.RestorePopulation(a.PopulationSnapshot())
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			st := w.Stops[0]
			player := sim.NewBus(st.Pos.X, st.Pos.Y, st.Heading)
			fleet := NewAIBusManager()
			fleet.BuildFleet(w, 8, 52, player)
			a.AIBusMgr = fleet
			b.AIBusMgr = fleet
			frames := 600
			if testing.Short() {
				frames = 30
			}
			for frame := 0; frame < frames; frame++ {
				tls.Update(1. / 60)
				sim.SeedSimulation(int64(1000 + frame))
				a.Update(1./60, player, tls, 4, 52, w)
				sim.SeedSimulation(int64(1000 + frame))
				b.Update(1./60, player, tls, 4, 52, w)
				if len(a.Cars) != len(b.Cars) {
					t.Fatal("population changed")
				}
				for i := range a.Cars {
					if !reflect.DeepEqual(a.Cars[i], b.Cars[i]) {
						t.Fatalf("frame%d car%d cached pos%v speed%.8f full pos%v speed%.8f", frame, i, a.Cars[i].Pos, a.Cars[i].Speed, b.Cars[i].Pos, b.Cars[i].Speed)
					}
				}
			}
		})
	}
}

func TestCarPlanCacheIsImmutableAndInvalidates(t *testing.T) {
	w := sim.NewWorld(52)
	w.PrepareJunctionGeometry(52)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	inter := tls.Intersections[0]
	car := &sim.TrafficCar{Pos: inter.Center.Add(sim.Vec2{X: -500, Y: 26}), Heading: 0, IsEW: true, RoadCenter: inter.Center.Y, LaneIndex: 0, Length: 48, Width: 22}
	scratch := carJunctionScratch{owner: car}
	f := carFrame{car: car, world: w, tls: tls, laneWidth: 52, scratch: &scratch}
	p := planCarJunctionTurn(f, inter)
	if p == nil {
		t.Fatal("missing turn")
	}
	original := append([]sim.Vec2(nil), p.Points...)
	car.Pos.X += 10
	if planCarJunctionTurn(f, inter) != p {
		t.Fatal("rebuilt unchanged turn geometry")
	}
	car.LaneIndex = 1
	changed := planCarJunctionTurn(f, inter)
	if changed == p {
		t.Fatal("lane edit reused old turn")
	}
	if !reflect.DeepEqual(p.Points, original) {
		t.Fatal("cache modified previous plan")
	}
	w.Roads[0].Lanes += 2
	w.PrepareJunctionGeometry(52)
	if planCarJunctionTurn(f, inter) == changed {
		t.Fatal("road edit reused old plan")
	}
	f.laneWidth = 78
	prev := scratch.plan
	if planCarJunctionTurn(f, inter) == prev {
		t.Fatal("width edit reused old plan")
	}
}

func TestCachedTurnSamplesExactlyMatchFullSweep(t *testing.T) {
	for _, lw := range []float64{20, 42, 52, 78} {
		for _, heading := range []float64{0, .37, 1.5707963267948966, 3.141592653589793} {
			for _, length := range []float64{46, 96} {
				turn := &sim.CarJunctionTurn{Points: sim.LeftTurnPath(sim.Vec2{}, heading, lw, 8*lw)}
				scratch := carJunctionScratch{}
				for segment := 0; segment < len(turn.Points)-1; segment++ {
					start := turn.Points[segment].Add(turn.Points[segment+1].Sub(turn.Points[segment]).Mul(.37))
					points := append([]sim.Vec2{start}, turn.Points[segment+1:]...)
					want := sim.AppendPathBodies(nil, points, length, 22)
					got := scratch.appendBodies(nil, points, turn, length, 22)
					if !reflect.DeepEqual(got, want) {
						t.Fatalf("cached samples changed width%.0f heading%.2f segment%d", lw, heading, segment)
					}
				}
				// In-place path edits and dimensions cannot reuse an obsolete sweep.
				turn.Points[5].X += 3
				if got, want := scratch.appendBodies(nil, turn.Points, turn, length+5, 24), sim.AppendPathBodies(nil, turn.Points, length+5, 24); !reflect.DeepEqual(got, want) {
					t.Fatal("edited sweep reused stale samples")
				}
			}
		}
	}
}

func TestJunctionIndexRetainsDistantCommittedTurn(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 8}, {P1: sim.Vec2{Y: -5000}, P2: sim.Vec2{Y: 5000}, Lanes: 8}}}
	w.RebuildIntersections()
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{{Center: sim.Vec2{}, Phase: sim.PhaseEWGreen}}}
	turn := &sim.CarJunctionTurn{Center: sim.Vec2{}, Points: []sim.Vec2{{X: 1990, Y: 78}, {X: 2100, Y: 78}}, Progress: 10, ExitEW: true, ExitLane: 1}
	car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: 2000, Y: 78}, Heading: 0, IsEW: true, Speed: 30, TargetSpeed: 30, Length: 48, Width: 22, LaneIndex: 1, JunctionTurn: turn}
	other := *car
	copyTurn := *turn
	other.JunctionTurn = &copyTurn
	a := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	b := &TrafficManager{Cars: []*sim.TrafficCar{&other}, disableJunctionIndex: true, disableJunctionReuse: true}
	a.Update(1./60, nil, tls, 4, 52, w)
	b.Update(1./60, nil, tls, 4, 52, w)
	if car.JunctionCorridor == nil || !reflect.DeepEqual(car, &other) {
		t.Fatal("lost committed turn reservation beyond ordinary query radius")
	}
}

func TestCommittedTurnDoesNotMutateCachedTemplate(t *testing.T) {
	w, tls := forcedLeftWorld(6, 0)
	inter := tls.Intersections[0]
	inter.Phase = sim.PhaseEWLeftGreen
	car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -1000, Y: 26}, Heading: 0, IsEW: true, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, LaneIndex: 0}
	scratch := carJunctionScratch{owner: car}
	frame := carFrame{car: car, world: w, tls: tls, laneWidth: 52, scratch: &scratch}
	template := planCarJunctionTurn(frame, inter)
	if template == nil {
		t.Fatal("missing turn")
	}
	original := append([]sim.Vec2(nil), template.Points...)
	car.Pos = template.Points[0]
	tm := &TrafficManager{}
	tm.turnCarAtIntersection(frame)
	if car.JunctionTurn == nil || car.JunctionTurn == template {
		t.Fatal("motion did not commit an independent turn")
	}
	moveCarThroughJunction(car, .5)
	if template.Segment != 0 || template.Progress != 0 || !reflect.DeepEqual(template.Points, original) {
		t.Fatal("advancing motion mutated future route template")
	}
}
