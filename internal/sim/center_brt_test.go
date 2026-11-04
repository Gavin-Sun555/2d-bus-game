package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestCenterBRTMapGeometry(t *testing.T) {
	for _, large := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 64} {
			t.Run(fmt.Sprintf("large%v/width%.0f", large, lw), func(t *testing.T) {
				w := &World{CenterBRT: true, LargeGrid: large}
				w.BuildMap(MapThemeMetropolis, lw)
				roads, stops := 0, 0
				for _, r := range w.Roads {
					eligible := r.LaneCount() == 8 && r.HasBusLane()
					if r.HasMedianBusLane() != eligible {
						t.Fatal("wrong BRT road eligibility", r)
					}
					if eligible {
						roads++
						if r.BusLaneIndex() != 0 || r.FirstGeneralTrafficLane() != 1 || r.GeneralTrafficLanes() != 3 {
							t.Fatal("wrong lane allocation")
						}
					}
				}
				for _, s := range w.Stops {
					ew := math.Abs(math.Cos(s.Heading)) > .5
					coord := s.RoadCenter.X
					if ew {
						coord = s.RoadCenter.Y
					}
					r, ok := w.RoadAt(coord, ew, s.RoadCenter)
					if !ok {
						t.Fatalf("stop %d missing road", s.ID)
					}
					if s.CenterBRT != r.HasMedianBusLane() {
						t.Fatalf("stop %d mode does not match road", s.ID)
					}
					if !s.CenterBRT {
						continue
					}
					stops++
					right := Vec2{X: -math.Sin(s.Heading), Y: math.Cos(s.Heading)}
					if math.Abs(s.Pos.Sub(s.RoadCenter).Dot(right)-lw*.5) > 1e-7 {
						t.Fatal("BRT bay outside central lane")
					}
					if math.Abs(s.ShelterPos.Sub(s.RoadCenter).Dot(right)) > 1e-7 {
						t.Fatal("platform not on median")
					}
					if w.InsideJunction(s.Pos, lw, s.Length/2+30) {
						t.Fatalf("stop %d overlaps junction", s.ID)
					}
					b := NewBus(s.FrontBerthPosition().X, s.FrontBerthPosition().Y, s.Heading)
					if !s.ContainsBus(b) {
						t.Fatal("central station cannot board")
					}
					b.Pos = b.Pos.Add(right.Mul(lw * 3))
					if s.ContainsBusInOuterLane(b, lw) {
						t.Fatal("central BRT permits curb boarding")
					}
					if offset := RoadLaneOffset(coord, ew, lw, w, s.RoadCenter); math.Abs(offset-lw*.5) > 1e-7 {
						t.Fatal("bus route does not use central lane", offset)
					}
				}
				if roads == 0 || stops == 0 {
					t.Fatal("no central BRT roads/stops")
				}
				w.UpdateLaneWidth(lw + 8)
				for _, s := range w.Stops {
					if s.CenterBRT && math.Abs(s.Pos.Distance(s.RoadCenter)-(lw+8)*.5) > 1e-7 {
						t.Fatal("lane width update lost BRT geometry")
					}
				}
			})
		}
	}
}

func TestCenterBRTOnlyEligibleMapsAndOptIn(t *testing.T) {
	for _, theme := range SelectableMapThemes() {
		w := &World{CenterBRT: true}
		w.BuildMap(theme, 52)
		if w.CenterBRT != w.HasBRTEligibleRoads() {
			t.Fatal("BRT mode does not match road eligibility")
		}
		if !w.HasBRTEligibleRoads() {
			for _, s := range w.Stops {
				if s.CenterBRT {
					t.Fatal("central stop on unsupported map")
				}
			}
		}
	}
	w := NewWorld(52)
	for _, r := range w.Roads {
		if r.HasMedianBusLane() {
			t.Fatal("BRT enabled by default")
		}
	}
}

func TestCenterBRTSignalsAndPlayerStraightPassage(t *testing.T) {
	const lw = 52.
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := UnitFromAngle(h)
		right := Vec2{X: -u.Y, Y: u.X}
		w := &World{Roads: []RoadSegment{
			{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: RoadBusBoulevard8Lane, MedianBusLane: true},
			{P1: right.Mul(-3000), P2: right.Mul(3000), Lanes: 4},
		}}
		w.RebuildIntersections()
		tls := NewTrafficLightSystem()
		tls.BuildForWorld(w)
		it := tls.Intersections[0]
		it.Phase = PhaseEWGreen
		if math.Abs(u.Y) > .5 {
			it.Phase = PhaseNSGreen
		}
		if !it.ApproachMedianBusLane[it.GetApproachIndexForHeading(h)] {
			t.Fatal("signal lost BRT metadata")
		}
		line := it.GetStopLineDistForHeading(h, lw)
		p := u.Mul(-line - 35).Add(right.Mul(lw * .5))
		b := NewBus(p.X, p.Y, h)
		b.Speed = 35
		if violation, notice := tls.CheckPlayerViolation(b, lw); violation {
			t.Fatal(notice)
		}
		if !it.PlayerEntryBusLane || it.PlayerEntryLane != 1 || it.PlayerCrossedOnRed {
			t.Fatal("central BRT through movement interpreted as left-only")
		}
		for s := -line - 30; s < line+b.Length+10; s += 5 {
			b.Pos = u.Mul(s).Add(right.Mul(lw * .5))
			if violation, notice := tls.CheckPlayerViolation(b, lw); violation {
				t.Fatal(notice)
			}
		}
	}
}
