package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestCloseStationLeftDepartureHasForwardTangents(t *testing.T) {
	for _, lw := range []float64{42, 52} {
		t.Run(fmt.Sprint(lw), func(t *testing.T) {
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			from := *w.Stops[7]
			from.RoadCenter.Y = 900 * sim.MapGridScale
			from.UpdateGeometry(lw)
			path := BuildAutopilotDrivePath(w, &from, w.Stops[9], lw)
			forward := sim.Vec2{Y: 1}
			right := sim.Vec2{X: -1}
			first := path[1].Sub(path[0]).Normalize()
			if first.Dot(forward) < .99 {
				t.Fatalf("departure tangent sideways: %v", first)
			}
			for i := 1; i < len(path); i++ {
				delta := path[i].Sub(path[i-1])
				along := delta.Dot(forward)
				if path[i].Y > 3200 {
					break
				}
				if along < -.01 || math.Abs(delta.Dot(right)) > along+.01 {
					t.Fatalf("abrupt departure segment%d: %v", i, delta)
				}
			}
		})
	}
}

func TestUniversityStationLeavesRoomBeforeLeftMerge(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		w := sim.NewWorld(lw, sim.MapThemeMetropolis)
		from := w.Stops[7]
		junction := sim.Vec2{X: from.RoadCenter.X, Y: 1200 * sim.MapGridScale}
		g := w.JunctionAt(junction, lw)
		// Bay exit plus one bus length of settling for each lane crossed, before
		// the left turn's stop line. This is distinct from mere platform clearance.
		available := junction.Y - from.RoadCenter.Y - g.HalfY - sim.JunctionStopInset - 48
		required := 300 + 3*96 + 5*lw
		if available < required {
			t.Fatalf("width%.0f: departure runway%.0f needs%.0f", lw, available, required)
		}
	}
}
