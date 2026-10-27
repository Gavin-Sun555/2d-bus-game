package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// The cross street ends at this boundary. Turning left is an outward map
// exit even though only the inward half of that street is stored in Roads.
func boundaryTurnWorld(lanes int, h float64, internal bool) (*sim.World, *sim.TrafficLightSystem) {
	u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: lanes}, {P1: sim.Vec2{}, P2: r.Mul(2000), Lanes: 4}}}
	if internal {
		w.Roads = append(w.Roads, sim.RoadSegment{P1: u.Mul(3000).Sub(r.Mul(2000)), P2: u.Mul(4000).Sub(r.Mul(2000)), Lanes: 2})
	}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	return w, tls
}

func TestBoundaryLeftLaneTurnsOutOnLeftArrow(t *testing.T) {
	for _, lw := range []float64{42, 52, 64, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/lanes%d/h%.2f", lw, lanes, h), func(t *testing.T) {
					w, tls := boundaryTurnWorld(lanes, h, false)
					inter := tls.Intersections[0]
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					leftGreen, throughGreen := sim.PhaseEWLeftGreen, sim.PhaseEWGreen
					if math.Abs(u.Y) > .5 {
						leftGreen, throughGreen = sim.PhaseNSLeftGreen, sim.PhaseNSGreen
					}
					line := inter.GetStopLineDistForHeading(h, lw)
					car := &sim.TrafficCar{ID: 0, Pos: u.Mul(-line - 35).Add(r.Mul(lw / 2)), Heading: h, Speed: 0, TargetSpeed: 100, Length: 46, Width: 22, LaneIndex: 0, IsEW: math.Abs(u.X) > .5}
					f := carFrame{car: car, world: w, tls: tls, laneWidth: lw, activeCount: 1}
					if left, right := carIntersectionTurns(f, inter); !left || right {
						t.Fatalf("outward left intent lost: left=%v right=%v", left, right)
					}
					plan := planCarJunctionTurn(f, inter)
					if plan == nil || plan.Straight || math.Abs(sim.NormalizeAngle(plan.ExitHeading-h+math.Pi/2)) > .001 {
						t.Fatalf("left lane planned a through movement: %+v", plan)
					}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					inter.Phase = throughGreen
					for i := 0; i < 300; i++ {
						tm.Update(1./60, nil, tls, 1, lw, w)
					}
					if car.JunctionTurn != nil || car.Speed > 1 || -car.Pos.Dot(u)-car.Length/2 < line {
						t.Fatal("left lane entered on through-only green")
					}
					inter.Phase = leftGreen
					// The ordinary through lane must remain stopped while the arrow is green.
					through := *car
					through.Pos = car.Pos.Add(r.Mul(lw))
					through.LaneIndex = 1
					other := f
					other.car = &through
					if speed := tm.signalSpeed(other, 100); speed != 0 {
						t.Fatalf("through lane accepted left arrow: %.1f", speed)
					}
					turned, removed := false, false
					for i := 0; i < 1600; i++ {
						tm.Update(1./60, nil, tls, 1, lw, w)
						if turn := car.JunctionTurn; turn != nil {
							if turn.Straight {
								t.Fatal("left arrow started straight corridor")
							}
							turned = true
						}
						present := false
						for _, c := range tm.Cars {
							present = present || c == car
						}
						if !present {
							if !turned || math.Sin(h-car.Heading) < .2 {
								t.Fatal("removed without following outward left turn")
							}
							half := (math.Abs(math.Sin(car.Heading-h))*car.Length + math.Abs(math.Cos(car.Heading-h))*car.Width) / 2
							if car.Pos.Dot(r) > -half {
								t.Fatal("removed before rear cleared boundary")
							}
							removed = true
							break
						}
					}
					if !removed {
						t.Fatalf("outward left car stuck: pos%v heading%.2f speed%.1f", car.Pos, car.Heading, car.Speed)
					}
				})
			}
		}
	}
}

func TestUnavailableInternalLeftCannotBorrowArrowForStraight(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, tls := boundaryTurnWorld(4, h, true)
		var inter *sim.TrafficIntersection
		for _, it := range tls.Intersections {
			if it.Center.Length() < 1 {
				inter = it
			}
		}
		u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
		r := sim.Vec2{X: -u.Y, Y: u.X}
		line := inter.GetStopLineDistForHeading(h, 52)
		car := &sim.TrafficCar{Pos: u.Mul(-line - 35).Add(r.Mul(26)), Heading: h, Length: 46, Width: 22, LaneIndex: 0, IsEW: math.Abs(u.X) > .5}
		inter.Phase = sim.PhaseEWLeftGreen
		if math.Abs(u.Y) > .5 {
			inter.Phase = sim.PhaseNSLeftGreen
		}
		f := carFrame{car: car, world: w, tls: tls, laneWidth: 52}
		if left, _ := carIntersectionTurns(f, inter); left {
			t.Fatal("invented an internal map exit")
		}
		if speed := (&TrafficManager{}).signalSpeed(f, 100); speed != 0 {
			t.Fatalf("straight path borrowed the left arrow: h%.2f speed%.1f", h, speed)
		}
	}
}

func TestRealMapBoundaryLeftSignalMatchesTurnPath(t *testing.T) {
	checked := 0
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		w := sim.NewWorld(52, theme)
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)
		for _, inter := range tls.Intersections {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				if w.IsAtRoundabout(inter.Center) || !inter.ApproachHasDedicatedLanesForHeading(h) || !hasRoadInDirection(w, inter.Center, h+math.Pi) || hasRoadInDirection(w, inter.Center, h-math.Pi/2) || !isMapExitAhead(w, inter.Center, h-math.Pi/2) {
					continue
				}
				checked++
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				p := inter.Center.Sub(u.Mul(inter.GetStopLineDistForHeading(h, 52) + 35)).Add(r.Mul(26))
				car := &sim.TrafficCar{Pos: p, Heading: h, Length: 46, Width: 22, LaneIndex: 0, IsEW: math.Abs(u.X) > .5}
				if car.IsEW {
					car.RoadCenter = inter.Center.Y
				} else {
					car.RoadCenter = inter.Center.X
				}
				f := carFrame{car: car, world: w, tls: tls, laneWidth: 52}
				plan := planCarJunctionTurn(f, inter)
				if plan == nil || plan.Straight || math.Abs(sim.NormalizeAngle(plan.ExitHeading-h+math.Pi/2)) > .001 {
					t.Fatalf("map%d edge%v h%.2f lost left exit", theme, inter.Center, h)
				}
				inter.Phase = sim.PhaseEWLeftGreen
				if math.Abs(u.Y) > .5 {
					inter.Phase = sim.PhaseNSLeftGreen
				}
				if limit := (&TrafficManager{}).signalSpeed(f, 100); limit != 100 {
					t.Fatalf("map%d edge%v blocked valid left arrow: %.1f", theme, inter.Center, limit)
				}
			}
		}
	}
	if checked == 0 {
		t.Fatal("no real-map boundary scenarios checked")
	}
	t.Logf("checked %d real-map boundary approaches", checked)
}
