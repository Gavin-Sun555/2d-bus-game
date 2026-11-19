package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"reflect"
	"testing"
)

func TestTurnExitGeometryMemoMatchesOriginalSweep(t *testing.T) {
	var memo turnExitGeometry
	points := []sim.Vec2{{X: 0, Y: 0}, {X: 100, Y: 0}, {X: 150, Y: -50}, {X: 150, Y: -180}}
	for _, change := range []string{"initial", "same", "coordinate", "signed zero", "length", "width", "shorter"} {
		length, width := 96., 36.
		switch change {
		case "coordinate":
			points[2].X++
		case "signed zero":
			points[0].Y = math.Copysign(0, -1)
		case "length":
			length = 98
		case "width":
			width = 38
		case "shorter":
			points = points[:3]
		}
		want := sim.AppendPathBodies(nil, points, length, width)
		got := memo.build(points, length, width)
		if !reflect.DeepEqual(got, want) {
			t.Fatal("memo changed swept bodies", change)
		}
		if !memo.matches(points, length, width) {
			t.Fatal("memo did not retain complete key", change)
		}
	}
	if n := testing.AllocsPerRun(100, func() { memo.build(points, 96, 36) }); n != 0 {
		t.Fatal("repeated geometry allocates", n)
	}
}
func TestTurnExitExtendedGeometryMemoKeepsIndependentCandidates(t *testing.T) {
	var scratch turnExitGeometryScratch
	paths := [][]sim.Vec2{{{}, {X: 60, Y: 20}, {X: 100, Y: 100}}, {{}, {X: 80, Y: 30}, {X: 120, Y: 100}}}
	saved := []sim.OBB(nil)
	for i := 0; i < 10; i++ {
		for j, path := range paths {
			end := sim.Vec2{X: 130 + float64(j)*20, Y: 200}
			points := append([]sim.Vec2(nil), path...)
			points[len(points)-1] = end
			got := scratch.extendedBodies(path, end, 100, 40)
			if !reflect.DeepEqual(got, sim.AppendPathBodies(nil, points, 100, 40)) {
				t.Fatal("candidate cache changed geometry")
			}
			if i == 0 && j == 0 {
				saved = got
			}
		}
	}
	points := append([]sim.Vec2(nil), paths[0]...)
	points[len(points)-1] = sim.Vec2{X: 130, Y: 200}
	if !reflect.DeepEqual(saved, sim.AppendPathBodies(nil, points, 100, 40)) {
		t.Fatal("second candidate overwrote first")
	}
}
