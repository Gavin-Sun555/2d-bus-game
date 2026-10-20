package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestEightLaneRoadTypesAndCustomMap(t *testing.T) {
	for _, busLane := range []bool{false, true} {
		data := GetDefaultCustomMapData()
		data.Roads[0].Lanes = 8
		data.Roads[0].BusLane = busLane
		encoded, err := json.Marshal(data)
		if err != nil {
			t.Fatal(err)
		}
		var restored CustomMapData
		if err = json.Unmarshal(encoded, &restored); err != nil {
			t.Fatal(err)
		}
		if err = ValidateCustomMapData(&restored); err != nil {
			t.Fatal(err)
		}
		world := &World{}
		ApplyCustomMapToWorld(world, &restored, 52)
		r := world.Roads[0]
		if r.LaneCount() != 8 || r.HasBusLane() != busLane || r.GetWidth(52) != 8*52+24 {
			t.Fatalf("incorrect road: %+v", r)
		}
		want := 4
		if busLane {
			want = 3
		}
		if r.GeneralTrafficLanes() != want {
			t.Fatalf("general lanes: %d", r.GeneralTrafficLanes())
		}
	}
	data := GetDefaultCustomMapData()
	data.Roads[0].Lanes = 4
	data.Roads[0].BusLane = true
	if ValidateCustomMapData(data) == nil {
		t.Fatal("reserved lane on a non-eight-lane road accepted")
	}
}

func TestEightLaneRoadJunctionGeometry(t *testing.T) {
	for _, width := range []float64{42, 52, 78} {
		w := &World{Roads: []RoadSegment{{P1: Vec2{X: -1800}, P2: Vec2{X: 1800}, Lanes: 8}, {P1: Vec2{Y: -1800}, P2: Vec2{Y: 1800}, Lanes: 8}}}
		w.RebuildIntersections()
		inter := &TrafficIntersection{Center: Vec2{}}
		inter.SetupApproachesFromWorld(w)
		g := w.JunctionAt(Vec2{}, width)
		ew, _, ns, _ := inter.GetApproachDimensions(width)
		if ew != g.HalfX+JunctionStopInset || ns != g.HalfY+JunctionStopInset {
			t.Fatalf("signal geometry %v/%v does not match road %+v", ew, ns, g)
		}
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			in := Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			out := Vec2{X: -in.Y, Y: in.X}
			path := appendEightLaneRightTurn(nil, w, Vec2{}, in, out, 0, 0, math.Abs(in.X) > .5, math.Abs(out.X) > .5, in.Mul(-600), out.Mul(600), width)
			right := Vec2{X: -in.Y, Y: in.X}
			aligned := false
			for i, p := range path {
				if -p.Dot(in) >= ew+60 && math.Abs(p.Dot(right)-3.5*width) < .01 {
					aligned = true
				}
				if i > 0 && p.Sub(path[i-1]).Dot(in) < -.01 {
					t.Fatal("right turn path reverses")
				}
			}
			if !aligned {
				t.Fatalf("right lane not reached before stop line for heading %v", heading)
			}
			outRight := Vec2{X: -out.Y, Y: out.X}
			if math.Abs(path[len(path)-1].Dot(outRight)-width*2.5) > .01 {
				t.Fatal("wrong exit lane")
			}
		}
	}
}

func TestMetropolisIncludesBothEightLaneRoads(t *testing.T) {
	w := NewWorld(52, MapThemeMetropolis)
	normal, reserved := false, false
	for _, r := range w.Roads {
		normal = normal || r.Type == RoadBoulevard8Lane
		reserved = reserved || r.HasBusLane()
	}
	if !normal || !reserved {
		t.Fatal("both road variants must be playable")
	}
}

func TestOpposingCurbBusLeftTurnCorridorsDoNotIntersect(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		g := NewJunctionGeometry(4*lw+12, 4*lw+12, lw)
		east, north := Vec2{X: 1}, Vec2{Y: -1}
		a := QuarterTurnPath(Vec2{}, east, north, 3.5*lw, .5*lw, g.TurnRadius)
		b := QuarterTurnPath(Vec2{}, east.Mul(-1), north.Mul(-1), 3.5*lw, .5*lw, g.TurnRadius)
		body := func(path []Vec2, i int) OBB {
			delta := path[min(i+1, len(path)-1)].Sub(path[max(0, i-1)])
			return OBB{Center: path[i], Heading: math.Atan2(delta.Y, delta.X), HalfLen: 50, HalfWid: 20}
		}
		for i := range a {
			for j := range b {
				if hit, _, _ := CheckOBBCollision(body(a, i), body(b, j)); hit {
					t.Fatalf("width%.0f opposing curb bus left-turn bodies overlap at%d/%d", lw, i, j)
				}
			}
		}
	}
}
